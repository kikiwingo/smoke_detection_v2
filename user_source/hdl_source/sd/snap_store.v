// ============================================================
// snap_store.v — TF卡告警快照存储 (缩略图 + 二进制日志, 环形裸扇区)
//
// 位置: user_source/hdl_source/sd/snap_store.v
// 时钟: S_csi_rx_clk (103.5MHz, 与 ISP 输出流同域; SPI 时钟由 sd_card_top 内分频)
//
// 功能 (2026-10-18 新增, 对齐项目介绍"TF卡环形裸扇区存储/缩略图/日志"):
//   1. 告警上升沿 (I_snap_trig 经两级同步) 触发采集当前帧
//   2. 视频流 (I_tdata 96bit=4像素RGB888) 灰度化 + 网格采样
//      (每 32 列 x 每 32 行取 1 像素 → 40x22=880 像素 8bit 缩略图)
//   3. 采集完成后依次写 3 个扇区 (512B):
//      扇区0: 二进制日志 (魔数/帧号/面积/包围盒/标志/时间戳)
//      扇区1-2: 缩略图 (1024B, 后 144B 补 0xFF)
//   4. 环形扇区地址: LOG_BASE 起, 每告警 +3, 回绕于 RING_SECTORS
//
// 与 sd_card_top 接口 (官方例程原样):
//   O_sec_write / O_sec_addr / O_sec_data / I_data_req / I_write_end
//
// 风险控制: 本模块完全旁路, 不插入视频数据通路; SD 卡异常只影响快照,
//   不影响实时显示/检测/告警主功能。缩略图采用组合读 RAM (1024x8 分布RAM)。
// ============================================================
module snap_store #(
    parameter [9:0]  THUMB_W      = 10'd40,    // 缩略图宽 (1280/32)
    parameter [9:0]  THUMB_H      = 10'd22,    // 缩略图高 (720/32 取整)
    parameter [31:0] LOG_BASE     = 32'd8,     // 日志扇区起始 (避开低区)
    parameter [31:0] RING_SECTORS = 32'd96     // 环形区扇区数 (3扇区x32次告警)
)(
    input  wire         I_clk,          // S_csi_rx_clk
    input  wire         I_rst_n,
    // ---- 视频流 (ISP 输出, 与 awb 同像素位序: [23:0]=像素0) ----
    input  wire         I_tuser,        // 帧开始
    input  wire         I_tvalid,       // 像素有效
    input  wire [95:0]  I_tdata,        // 4像素 RGB888
    // ---- 告警触发 (HDMI域脉冲, 内部两级同步) ----
    input  wire         I_snap_trig,    // 总告警上升沿 (alarm_module 输出)
    // ---- 检测状态 (日志内容, csi域帧级信号) ----
    input  wire         I_fire_valid,
    input  wire         I_cig_valid,
    input  wire [19:0]  I_fire_area,    // 明火连通域面积
    input  wire [19:0]  I_cig_count,    // 烟头亮点计数
    input  wire [10:0]  I_fxmin, I_fxmax,
    input  wire [9:0]   I_fymin, I_fymax,
    input  wire [10:0]  I_cxmin, I_cxmax,
    input  wire [9:0]   I_cymin, I_cymax,
    // ---- SD 接口 (到 sd_card_top) ----
    output reg          O_sec_write,
    output reg  [31:0]  O_sec_addr,
    output reg  [7:0]   O_sec_data,
    input  wire         I_data_req,     // sd_card_top 请求下一字节
    input  wire         I_write_end     // 单扇区写完成
);

    // ================= 告警触发 CDC (HDMI域→csi域) =================
    reg snap_s1, snap_s2, snap_s2d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            snap_s1 <= 1'b0; snap_s2 <= 1'b0; snap_s2d <= 1'b0;
        end else begin
            snap_s1  <= I_snap_trig;
            snap_s2  <= snap_s1;
            snap_s2d <= snap_s2;
        end
    end
    wire snap_rise = snap_s2 && !snap_s2d;

    // ================= csi 域帧计数 (日志时间戳) =================
    reg [31:0] frame_cnt;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) frame_cnt <= 32'd0;
        else if(I_tuser) frame_cnt <= frame_cnt + 32'd1;
    end

    // ================= 缩略图 RAM (1024x8, 组合读) =================
    reg [7:0] thumb_ram [0:1023];
    reg [9:0] thumb_waddr;   // 采集写地址
    // (缩略图组合读 ram_rd_addr/thumb_byte 声明于 byte_idx 声明之后)

    // ================= 触发信息锁存 (日志) =================
    reg [31:0] log_frame;     // 采集帧号
    reg [19:0] log_fire_area, log_cig_count;
    reg [10:0] log_fxmin, log_fxmax, log_cxmin, log_cxmax;
    reg [9:0]  log_fymin, log_fymax, log_cymin, log_cymax;
    reg        log_fire_v, log_cig_v;

    // ================= 主状态机 =================
    localparam S_IDLE       = 4'd0;   // 等告警
    localparam S_WAIT_FRAME = 4'd1;   // 等下一帧 (tuser)
    localparam S_CAPTURE    = 4'd2;   // 采集缩略图
    localparam S_WR_LOG     = 4'd3;   // 写日志扇区
    localparam S_WR_THUMB0  = 4'd4;   // 写缩略图扇区0
    localparam S_WR_THUMB1  = 4'd5;   // 写缩略图扇区1
    localparam S_DONE       = 4'd6;

    reg [3:0]  state;
    // 像素坐标计数 (仅采集期间推进, 与视频流同拍)
    reg [11:0] px_cnt;       // 列 0..1279
    reg [9:0]  py_cnt;       // 行 0..719
    reg [9:0]  thumb_cnt;    // 已采像素数 0..879
    reg [9:0]  byte_idx;     // 扇区写字节索引 0..511
    reg [31:0] ring_ptr;     // 环形扇区指针 (LOG_BASE 起, 步进 3)
    reg        busy;         // 采集/写卡忙 (供观察)

    // 采样判定: 每 32 列 x 每 32 行取 1 像素 (行/列从 0 起)
    localparam [9:0] THUMB_TOTAL = THUMB_W * THUMB_H;   // 40*22=880
    // 扇区2有效缩略图字节数: 880-512=368, 其后 144B 补 0xFF
    // (修复: 原 S_WR_THUMB1 复用 byte_idx 直读 RAM[0:511], 导致扇区2重复上半部)
    localparam [9:0] THUMB_SEC1_VALID = THUMB_TOTAL - 10'd512;
    wire samp_x = (px_cnt[4:0] == 5'd0);
    wire samp_y = (py_cnt[4:0] == 5'd0);
    wire capture_done = (thumb_cnt >= THUMB_TOTAL);      // 采满 880 像素

    // 缩略图组合读 (byte_idx 已声明; RAM 组合异步读, 写扇区时逐字节取)
    // 扇区1读 RAM[0:511]; 扇区2读 RAM[512:1023], 超出880像素部分补 0xFF
    wire [9:0] ram_rd_addr = (state == S_WR_THUMB1) ? (byte_idx + 10'd512) : byte_idx[9:0];
    wire       thumb_pad   = (state == S_WR_THUMB1) && (byte_idx >= THUMB_SEC1_VALID);
    wire [7:0] thumb_byte  = thumb_pad ? 8'hFF : thumb_ram[ram_rd_addr];
    // 灰度: (R+G+B)>>2, 整数近似亮度
    wire [9:0] gray_sum = {2'b0,I_tdata[23:16]} + {2'b0,I_tdata[15:8]} + {2'b0,I_tdata[7:0]};
    wire [7:0] gray_byte = gray_sum[9:2];

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            state       <= S_IDLE;
            px_cnt      <= 12'd0;
            py_cnt      <= 10'd0;
            thumb_cnt   <= 10'd0;
            thumb_waddr <= 10'd0;
            byte_idx    <= 10'd0;
            ring_ptr    <= LOG_BASE;
            O_sec_write <= 1'b0;
            O_sec_addr  <= 32'd0;
            O_sec_data  <= 8'hFF;
            busy        <= 1'b0;
            log_frame   <= 32'd0;
            log_fire_area <= 20'd0; log_cig_count <= 20'd0;
            log_fxmin <= 11'd0; log_fxmax <= 11'd0;
            log_fymin <= 10'd0; log_fymax <= 10'd0;
            log_cxmin <= 11'd0; log_cxmax <= 11'd0;
            log_cymin <= 10'd0; log_cymax <= 10'd0;
            log_fire_v <= 1'b0; log_cig_v <= 1'b0;
        end else begin
            case(state)
                // ---- 等告警: 锁存触发信息 ----
                S_IDLE: begin
                    O_sec_write <= 1'b0;
                    if(snap_rise) begin
                        log_frame      <= frame_cnt;
                        log_fire_area  <= I_fire_area;
                        log_cig_count  <= I_cig_count;
                        log_fxmin <= I_fxmin; log_fxmax <= I_fxmax;
                        log_fymin <= I_fymin; log_fymax <= I_fymax;
                        log_cxmin <= I_cxmin; log_cxmax <= I_cxmax;
                        log_cymin <= I_cymin; log_cymax <= I_cymax;
                        log_fire_v <= I_fire_valid;
                        log_cig_v  <= I_cig_valid;
                        busy      <= 1'b1;
                        state     <= S_WAIT_FRAME;
                    end
                end
                // ---- 等下一帧帧头 (确保从完整帧开始采集) ----
                S_WAIT_FRAME: begin
                    if(I_tuser) begin
                        px_cnt      <= 12'd0;
                        py_cnt      <= 10'd0;
                        thumb_cnt   <= 10'd0;
                        thumb_waddr <= 10'd0;
                        state       <= S_CAPTURE;
                    end
                end
                // ---- 采集: 网格采样写 RAM ----
                S_CAPTURE: begin
                    if(I_tvalid) begin
                        // 坐标推进
                        if(I_tuser) begin
                            px_cnt <= 12'd0;
                            py_cnt <= 10'd0;
                        end else if(px_cnt == 12'd1279) begin
                            px_cnt <= 12'd0;
                            if(py_cnt == 10'd719) py_cnt <= 10'd0;
                            else py_cnt <= py_cnt + 10'd1;
                        end else begin
                            px_cnt <= px_cnt + 12'd1;
                        end
                        // 采样点写 RAM
                        if(samp_x && samp_y && !capture_done) begin
                            thumb_ram[thumb_waddr] <= gray_byte;
                            thumb_waddr <= thumb_waddr + 10'd1;
                            thumb_cnt   <= thumb_cnt + 10'd1;
                        end
                        // 采满 -> 转写卡
                        if(capture_done) begin
                            byte_idx <= 10'd0;
                            state    <= S_WR_LOG;
                        end
                    end
                end
                // ---- 写日志扇区 (512B) ----
                S_WR_LOG: begin
                    O_sec_write <= 1'b1;
                    O_sec_addr  <= ring_ptr;
                    if(I_data_req) begin
                        O_sec_data <= log_byte(byte_idx, log_frame, log_fire_area,
                                               log_cig_count, log_fxmin, log_fxmax,
                                               log_fymin, log_fymax, log_cxmin, log_cxmax,
                                               log_cymin, log_cymax, log_fire_v, log_cig_v);
                        if(byte_idx == 10'd511)
                            byte_idx <= 10'd0;
                        else
                            byte_idx <= byte_idx + 10'd1;
                    end
                    if(I_write_end) begin
                        O_sec_write <= 1'b0;
                        byte_idx    <= 10'd0;
                        state       <= S_WR_THUMB0;
                    end
                end
                // ---- 写缩略图扇区0 (RAM[0:511]) ----
                S_WR_THUMB0: begin
                    O_sec_write <= 1'b1;
                    O_sec_addr  <= ring_ptr + 32'd1;
                    if(I_data_req) begin
                        O_sec_data <= thumb_byte;
                        if(byte_idx == 10'd511)
                            byte_idx <= 10'd0;
                        else
                            byte_idx <= byte_idx + 10'd1;
                    end
                    if(I_write_end) begin
                        O_sec_write <= 1'b0;
                        byte_idx    <= 10'd0;
                        state       <= S_WR_THUMB1;
                    end
                end
                // ---- 写缩略图扇区1 (RAM[512:1023], 后 144B 空闲补 0xFF) ----
                S_WR_THUMB1: begin
                    O_sec_write <= 1'b1;
                    O_sec_addr  <= ring_ptr + 32'd2;
                    if(I_data_req) begin
                        O_sec_data <= thumb_byte;
                        if(byte_idx == 10'd511)
                            byte_idx <= 10'd0;
                        else
                            byte_idx <= byte_idx + 10'd1;
                    end
                    if(I_write_end) begin
                        O_sec_write <= 1'b0;
                        // 环形推进: +3 扇区, 回绕于 LOG_BASE+RING_SECTORS
                        if(ring_ptr + 32'd3 >= LOG_BASE + RING_SECTORS)
                            ring_ptr <= LOG_BASE;
                        else
                            ring_ptr <= ring_ptr + 32'd3;
                        busy  <= 1'b0;
                        state <= S_DONE;
                    end
                end
                // ---- 完成 ----
                S_DONE: begin
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    // ================= 日志扇区字节组合输出 =================
    // 512B 二进制: 魔数 + 帧号 + 面积 + 包围盒 + 标志 + 保留
    function [7:0] log_byte;
        input [9:0]  idx;
        input [31:0] frm;
        input [19:0] fa, ca;
        input [10:0] fx0, fx1, cx0, cx1;
        input [9:0]  fy0, fy1, cy0, cy1;
        input fv, cv;
        begin
            case(idx)
                10'd0: log_byte = 8'hA5;
                10'd1: log_byte = 8'hA5;
                10'd2: log_byte = 8'hA5;
                10'd3: log_byte = 8'hA5;
                10'd4: log_byte = frm[7:0];
                10'd5: log_byte = frm[15:8];
                10'd6: log_byte = frm[23:16];
                10'd7: log_byte = frm[31:24];
                10'd8:  log_byte = fa[7:0];
                10'd9:  log_byte = fa[15:8];
                10'd10: log_byte = fa[19:16];
                10'd11: log_byte = 8'd0;
                10'd12: log_byte = ca[7:0];
                10'd13: log_byte = ca[15:8];
                10'd14: log_byte = ca[19:16];
                10'd15: log_byte = 8'd0;
                10'd16: log_byte = fx0[7:0];
                10'd17: log_byte = {5'd0, fx0[10:8]};
                10'd18: log_byte = fx1[7:0];
                10'd19: log_byte = {5'd0, fx1[10:8]};
                10'd20: log_byte = fy0[7:0];
                10'd21: log_byte = {6'd0, fy0[9:8]};
                10'd22: log_byte = fy1[7:0];
                10'd23: log_byte = {6'd0, fy1[9:8]};
                10'd24: log_byte = cx0[7:0];
                10'd25: log_byte = {5'd0, cx0[10:8]};
                10'd26: log_byte = cx1[7:0];
                10'd27: log_byte = {5'd0, cx1[10:8]};
                10'd28: log_byte = cy0[7:0];
                10'd29: log_byte = {6'd0, cy0[9:8]};
                10'd30: log_byte = cy1[7:0];
                10'd31: log_byte = {6'd0, cy1[9:8]};
                10'd32: log_byte = {6'd0, cv, fv};   // bit0=fire, bit1=cig (与 tf_card_decode.py 位序一致)
                10'd33: log_byte = 8'd0;
                10'd34: log_byte = frm[7:0];          // 时间戳低32=帧号
                10'd35: log_byte = frm[15:8];
                10'd36: log_byte = frm[23:16];
                10'd37: log_byte = frm[31:24];
                default: log_byte = 8'hFF;
            endcase
        end
    endfunction

endmodule
