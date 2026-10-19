// ============================================================
// target_box_osd_4lane.v - 多目标正方形锁定框叠加
// (纯PL像素流水线, 四路并行, 无帧缓存/不访问DDR/零丢帧)
//
// 功能 (门禁人脸识别式正方形实线框):
//   1. 每个有效目标绘制一个空心红色实线正方形框: 以目标外接框中心为
//      中心, 边长 = 外接框长边外扩 PAD (默认1.5倍), 完整包裹目标;
//      细长条状目标同样框成正方形, 不做长条外接框
//   2. 多目标: 每个有效目标各画一个框, 互不影响 (重叠区域同色)
//   3. 场消隐期电平锁存各目标坐标, 消隐末拍 IIR 平滑一拍生效,
//      帧内坐标固定无撕裂; 目标移动逐帧平滑跟随; 首次有效帧直接装载
//   4. target_valid=0: 全部通道 osd_en=0, 视频直通不叠加
//
// 正方形几何 (逐目标, 组合逻辑):
//   w=xmax-xmin, h=ymax-ymin, D=max(w,h)+1      (外接框长边)
//   side = D + 2*(D>>PAD_SHIFT)                 (默认PAD_SHIFT=2 → 1.5倍)
//   cx = xmin+w/2, cy = ymin+h/2                (外接框中心)
//   方框 = [cx-side/2, cy-side/2] ~ [cx+side/2, cy+side/2]
//   超出画面的部分裁剪到帧边界 (贴边目标只显示画面内部分)
//   IIR 对方框四个边界分别平滑: 每帧各边界序列宽=高=side-1,
//   凸组合保持正方形性 → 跟踪滑动过程中框始终是正方形
//
// 已知取舍:
//   - IIR 算术右移截断, 静止目标残差 < 2^IIR_SHIFT (默认<=3像素, 视觉无感)
//   - 框显示上一帧检测结果 (帧锁存固有1帧延迟, 与 osd_box.v 一致)
//   - 目标贴画面边缘时框被裁剪, 画面内可见部分可能不再是正方形
//   - PAD_SHIFT 建议取值 1~3 (0 时 side 可达3倍, 内部位宽不保证)
//
// 输入约定:
//   I_x/I_y = 本拍首像素(通道0)坐标, 通道i对应像素列 x+i;
//   I_xmin[MAX_BOX*11-1:0] 等为打包总线, [i*11+:11] = 目标i;
//   建议按面积降序接入 (detect_ccl_96 输出即降序), box0=最大目标
// ============================================================
module target_box_osd_4lane #(
    parameter        MAX_BOX    = 4,           // 最大目标框数
    parameter        LANES      = 4,           // 每拍并行像素数
    parameter [11:0] IMG_WIDTH  = 12'd1280,    // 帧宽 (裁剪边界)
    parameter [10:0] IMG_HEIGHT = 11'd720,     // 帧高
    parameter [3:0]  BORDER_T   = 4'd3,        // 边框线宽(单边) → 总宽 2T+1
    parameter [3:0]  IIR_SHIFT  = 4'd2,        // IIR 系数 1/2^SHIFT (2 → 1/4)
    parameter [3:0]  PAD_SHIFT  = 4'd2,        // 方框外扩: side≈max(w,h)*(1+2/2^PAD), 建议1~3
    parameter [23:0] OSD_COLOR  = 24'hFF0000   // 叠加色 RGB888 (红)
)(
    input  wire                        I_clk,
    input  wire                        I_rst_n,
    // ---- 像素流水接口 (每拍LANES像素, 与本拍数据同拍) ----
    input  wire                        I_hs,           // 行消隐标志 (高=行消隐期; 无此信号接0)
    input  wire                        I_vs,           // 场消隐标志 (高=场消隐期, 含场同步脉冲)
    input  wire [10:0]                 I_x,            // 本拍首像素列坐标 (0..1279)
    input  wire [9:0]                  I_y,            // 行坐标 (0..719)
    // ---- 多目标坐标 (上游检测模块输出, 帧级更新, 打包总线) ----
    input  wire [MAX_BOX-1:0]          I_target_valid, // bit i = 目标i有效
    input  wire [MAX_BOX*11-1:0]       I_xmin,         // [i*11+:11] = 目标i左边界
    input  wire [MAX_BOX*11-1:0]       I_xmax,
    input  wire [MAX_BOX*10-1:0]       I_ymin,
    input  wire [MAX_BOX*10-1:0]       I_ymax,
    // ---- OSD 输出 (组合, 与像素流同拍零延迟) ----
    output wire [LANES-1:0]            O_osd_en,       // 逐通道叠加使能 (bit0=首像素)
    output wire [23:0]                 O_osd_rgb       // 叠加色 (恒为 OSD_COLOR)
);

    localparam [10:0] W_M1 = IMG_WIDTH  - 12'd1;
    localparam [9:0]  H_M1 = IMG_HEIGHT - 11'd1;
    localparam signed [11:0] BT_X = {8'd0, BORDER_T};
    localparam signed [10:0] BT_Y = {7'd0, BORDER_T};

    // ---- 场消隐期电平锁存: 消隐内每拍更新, 消隐末拍锁定最终值 ----
    // (上游坐标在消隐期内才完成帧级更新时, 电平锁存比边沿锁存拿到更新的值)
    reg [10:0] xmin_l [0:MAX_BOX-1];
    reg [10:0] xmax_l [0:MAX_BOX-1];
    reg [9:0]  ymin_l [0:MAX_BOX-1];
    reg [9:0]  ymax_l [0:MAX_BOX-1];
    reg        tv_l   [0:MAX_BOX-1];
    integer bii;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            for(bii = 0; bii < MAX_BOX; bii = bii + 1) begin
                xmin_l[bii] <= 11'd0; xmax_l[bii] <= 11'd0;
                ymin_l[bii] <= 10'd0; ymax_l[bii] <= 10'd0;
                tv_l[bii]   <= 1'b0;
            end
        end else if(I_vs) begin
            for(bii = 0; bii < MAX_BOX; bii = bii + 1) begin
                xmin_l[bii] <= I_xmin[bii*11 +: 11];
                xmax_l[bii] <= I_xmax[bii*11 +: 11];
                ymin_l[bii] <= I_ymin[bii*10 +: 10];
                ymax_l[bii] <= I_ymax[bii*10 +: 10];
                tv_l[bii]   <= I_target_valid[bii];
            end
        end
    end

    // ---- 场消隐下降沿 (消隐末拍): IIR 一拍更新, 帧内坐标固定 ----
    reg vs_d1;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) vs_d1 <= 1'b0;
        else         vs_d1 <= I_vs;
    end
    wire vs_fall = vs_d1 && !I_vs;

    // ---- 平滑状态: 正方形四边界 (各目标独立 IIR) ----
    reg signed [11:0] sq_xmin_s [0:MAX_BOX-1];   // 12位有符号, 取值 0..1279
    reg signed [11:0] sq_xmax_s [0:MAX_BOX-1];
    reg signed [10:0] sq_ymin_s [0:MAX_BOX-1];
    reg signed [10:0] sq_ymax_s [0:MAX_BOX-1];
    reg               box_valid [0:MAX_BOX-1];   // 本帧绘制使能 (帧内稳定)
    reg               box_init  [0:MAX_BOX-1];   // 已装载过有效坐标

    // ---- 命中判定网络: hit_lane[通道gi][目标bi] ----
    wire [MAX_BOX-1:0] hit_lane [0:LANES-1];
    wire pixel_valid = !I_vs && !I_hs;

    genvar bi, gi;
    generate
        for(bi = 0; bi < MAX_BOX; bi = bi + 1) begin : GEN_BOX
            // ===== 正方形几何 (组合, 由锁存外接框计算) =====
            wire [10:0] wb = xmax_l[bi] - xmin_l[bi];
            wire [9:0]  hb = ymax_l[bi] - ymin_l[bi];
            wire [10:0] Dm1 = (wb >= {1'b0, hb}) ? wb : {1'b0, hb};
            wire [11:0] D    = Dm1 + 12'd1;                     // 外接框长边
            wire [12:0] side = D + ((D >> PAD_SHIFT) << 1);     // 方框边长
            wire [11:0] half = side[12:1];                      // 半边
            wire [10:0] cx = xmin_l[bi] + (wb >> 1);            // 外接框中心
            wire [9:0]  cy = ymin_l[bi] + (hb >> 1);
            wire signed [11:0] sq_xmin_raw = $signed({2'b0, cx}) - $signed({1'b0, half});
            wire signed [13:0] sq_xmax_raw = sq_xmin_raw + $signed({1'b0, side}) - 14'sd1;
            wire signed [11:0] sq_ymin_raw = $signed({2'b0, cy}) - $signed({1'b0, half});
            wire signed [13:0] sq_ymax_raw = sq_ymin_raw + $signed({1'b0, side}) - 14'sd1;
            // 裁剪到画面内 (平滑对象取裁剪后边界, 位宽恒定)
            wire [10:0] sq_xmin_c = sq_xmin_raw[11] ? 11'd0 : sq_xmin_raw[10:0];
            wire [10:0] sq_xmax_c = (sq_xmax_raw[12:0] > {2'b0, W_M1}) ? W_M1 : sq_xmax_raw[10:0];
            wire [9:0]  sq_ymin_c = sq_ymin_raw[11] ? 10'd0 : sq_ymin_raw[9:0];
            wire [9:0]  sq_ymax_c = (sq_ymax_raw[12:0] > {3'b0, H_M1}) ? H_M1 : sq_ymax_raw[9:0];

            // ===== 一阶 IIR: sq <= sq + (new - sq) >> IIR_SHIFT =====
            // 目标移动 → 逐帧逼近; 抖动 → 被衰减; 首次有效帧直接装载(无滑入过程)
            wire signed [11:0] diff_xmin = {1'b0, sq_xmin_c} - sq_xmin_s[bi];
            wire signed [11:0] diff_xmax = {1'b0, sq_xmax_c} - sq_xmax_s[bi];
            wire signed [10:0] diff_ymin = {1'b0, sq_ymin_c} - sq_ymin_s[bi];
            wire signed [10:0] diff_ymax = {1'b0, sq_ymax_c} - sq_ymax_s[bi];
            always @(posedge I_clk or negedge I_rst_n) begin
                if(!I_rst_n) begin
                    sq_xmin_s[bi] <= 12'd0; sq_xmax_s[bi] <= 12'd0;
                    sq_ymin_s[bi] <= 11'd0; sq_ymax_s[bi] <= 11'd0;
                    box_valid[bi] <= 1'b0; box_init[bi] <= 1'b0;
                end else if(vs_fall) begin
                    if(tv_l[bi] && !box_init[bi]) begin        // 首次有效: 直接装载
                        sq_xmin_s[bi] <= {1'b0, sq_xmin_c};
                        sq_xmax_s[bi] <= {1'b0, sq_xmax_c};
                        sq_ymin_s[bi] <= {1'b0, sq_ymin_c};
                        sq_ymax_s[bi] <= {1'b0, sq_ymax_c};
                        box_init[bi] <= 1'b1;
                    end else if(tv_l[bi]) begin                // 平滑跟踪
                        sq_xmin_s[bi] <= sq_xmin_s[bi] + (diff_xmin >>> IIR_SHIFT);
                        sq_xmax_s[bi] <= sq_xmax_s[bi] + (diff_xmax >>> IIR_SHIFT);
                        sq_ymin_s[bi] <= sq_ymin_s[bi] + (diff_ymin >>> IIR_SHIFT);
                        sq_ymax_s[bi] <= sq_ymax_s[bi] + (diff_ymax >>> IIR_SHIFT);
                    end
                    // 目标有效且坐标合法才绘制; 无效帧不画且保持平滑状态
                    box_valid[bi] <= tv_l[bi] && (xmax_l[bi] >= xmin_l[bi]) && (ymax_l[bi] >= ymin_l[bi]);
                end
            end

            // ===== 空心框命中 (组合, 有符号差分, 无回绕) =====
            // 左右边带: x 贴近 xmin/xmax 且 y 在框竖向范围内
            // 上下边带: y 贴近 ymin/ymax 且 x 在框横向范围内 (四角属边带)
            wire signed [11:0] dxb  = {1'b0, I_x} - sq_xmin_s[bi];
            wire signed [11:0] dxmb = {1'b0, I_x} - sq_xmax_s[bi];
            wire signed [10:0] dyb  = {1'b0, I_y} - sq_ymin_s[bi];
            wire signed [10:0] dymb = {1'b0, I_y} - sq_ymax_s[bi];
            wire near_tb = (dyb  >= -BT_Y) && (dyb  <= BT_Y); // 贴近上边
            wire near_bb = (dymb >= -BT_Y) && (dymb <= BT_Y); // 贴近下边
            wire in_yb   = (dyb  >= -BT_Y) && (dymb <= BT_Y); // y 在框竖向范围
            for(gi = 0; gi < LANES; gi = gi + 1) begin : GEN_LANE
                wire signed [11:0] dxi  = dxb  + gi;          // 通道i: x+i - xmin
                wire signed [11:0] dxxm = dxmb + gi;          // 通道i: x+i - xmax
                assign hit_lane[gi][bi] = box_valid[bi] &&
                    ((in_yb && (((dxi >= -BT_X) && (dxi <= BT_X)) ||
                                ((dxxm >= -BT_X) && (dxxm <= BT_X)))) ||
                     ((((dxi >= -BT_X) && (dxxm <= BT_X))) && (near_tb || near_bb)));
            end
        end
    endgenerate

    // ---- 多目标叠加: 任一目标命中即绘制 ----
    generate
        for(gi = 0; gi < LANES; gi = gi + 1) begin : GEN_LANE_OR
            assign O_osd_en[gi] = pixel_valid && |hit_lane[gi];
        end
    endgenerate

    assign O_osd_rgb = OSD_COLOR;

endmodule
