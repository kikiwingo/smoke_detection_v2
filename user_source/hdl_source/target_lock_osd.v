// ============================================================
// target_lock_osd.v - 红色光圈/光斑目标锁定框 OSD (纯PL, 像素时钟域)
// 安路HX1P35A / TD 6.x 可综合; 无帧缓存/不访问DDR, 组合输出0拍零丢帧
//
// 目标定义: 前置色彩二值掩码模块(color_mask_96红色启发式)检出的
//   红色封闭光圈/红色灯光光斑; target_valid 仅由红色目标拉高,
//   绿色像素/绿色区域不进入本信号, 无法触发框绘制
//
// 功能:
//  1. target_valid=1: 绘制紧贴目标外接轮廓的空心细红色矩形锁定框
//     (人脸识别框风格), 目标移动时红框逐帧跟随更新
//  2. 场消隐阶段(vs上升沿)锁存目标坐标, 单帧内坐标固定防止画面撕裂;
//     坐标经一阶IIR低通平滑(每帧步进1/4, 四舍五入), 抑制框剧烈抖动;
//     首次检出直接锁定(无收敛迟滞), 目标消失后平滑值复位(重现零迟滞)
//  3. target_valid=0: O_osd_en 恒0, 下游透传原图, 不绘制任何框
//
// 时序: 内部坐标/标志全部为寄存器; O_osd_en/O_osd_rgb 组合输出(0拍),
//   与输入像素同拍; 下游若需寄存输出, 请将视频同步延迟1拍配平
// 同步极性: I_hs/I_vs 高有效(高=消隐期), 双低区间=有效显示区
// 适配: 挂在视图多路复用之后的显示链路上, 与几路并行视图无关
//
// 下游混合示例:
//   替换式:           disp = O_osd_en ? O_osd_rgb : video_rgb;
//   半透明(alpha=8):  disp = (video*(16-8*O_osd_en) + O_osd_rgb*8*O_osd_en + 8) >> 4;
// ============================================================
module target_lock_osd #(
    parameter [10:0] FRAME_W   = 11'd1280,   // 帧宽(像素)
    parameter [9:0]  FRAME_H   = 10'd720,    // 帧高(行)
    parameter [3:0]  BOX_T     = 4'd2,       // 边框线宽(细框2px)
    parameter [3:0]  IIR_SHIFT = 4'd2        // IIR平滑系数: 每帧步进1/2^SHIFT (>=1)
)(
    input             I_clk,          // 像素时钟
    input             I_rst_n,        // 异步复位(低有效)
    input             I_hs,           // 行同步(高=行消隐)
    input             I_vs,           // 场同步(高=场消隐)
    input  [10:0]     I_x,            // 当前像素x坐标(0..FRAME_W-1)
    input  [9:0]      I_y,            // 当前像素y坐标(0..FRAME_H-1)
    input             I_target_valid, // 目标有效(帧级, 场消隐期稳定; 仅红色光圈/光斑拉高)
    input  [10:0]     I_xmin,         // 目标外接框左边界
    input  [10:0]     I_xmax,         // 目标外接框右边界
    input  [9:0]      I_ymin,         // 目标外接框上边界
    input  [9:0]      I_ymax,         // 目标外接框下边界
    output            O_osd_en,       // 该像素属于锁定框时=1, 否则=0
    output [23:0]     O_osd_rgb       // OSD叠加色(红色FF0000)
);

    // ---- 场消隐起始脉冲 (vs上升沿, 此刻锁存本帧目标坐标) ----
    reg vs_d1;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            vs_d1 <= 1'b0;
        else
            vs_d1 <= I_vs;
    end
    wire frame_start = I_vs && !vs_d1;

    // ---- IIR一步平滑: s' = s + (x - s + HALF) >> SHIFT, 结果钳位[0,MAX] ----
    // 一阶低通: 每帧向新坐标移动差值的1/4(四舍五入), 平滑跟随抑制框抖动
    localparam signed [12:0] IIR_HALF = (13'sd1 << (IIR_SHIFT - 1'b1));
    function [10:0] iir_step;
        input [10:0] s;      // 当前平滑值
        input [10:0] x;      // 本帧原始锁存值
        input [10:0] maxv;   // 坐标上限
        reg signed [12:0] diff;
        reg signed [12:0] ns;
        begin
            diff = {2'b00, x} - {2'b00, s};                    // 13位有符号差(补码)
            ns   = {2'b00, s} + ((diff + IIR_HALF) >>> IIR_SHIFT);
            if(ns[12])                  iir_step = 11'd0;      // 负: 钳0
            else if(ns[10:0] > maxv)    iir_step = maxv;       // 超上限: 钳上限
            else                        iir_step = ns[10:0];
        end
    endfunction

    // ---- 场消隐锁存 + IIR平滑 (单帧内坐标固定防撕裂, 新一帧更新, 逐帧跟随) ----
    reg        r_valid;                // 本帧目标有效(帧内稳定)
    reg [10:0] s_xmin, s_xmax;         // 平滑后外接框
    reg [9:0]  s_ymin, s_ymax;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_valid <= 1'b0;
            s_xmin  <= 11'd0; s_xmax <= 11'd0;
            s_ymin  <= 10'd0; s_ymax <= 10'd0;
        end else if(frame_start) begin
            r_valid <= I_target_valid;
            if(!I_target_valid || !r_valid) begin
                // 无目标帧或本帧首次检出: 平滑值直接取原始值——
                // 目标重现瞬间锁定无IIR迟滞, 避免从(0,0)缓慢收敛
                s_xmin <= I_xmin; s_xmax <= I_xmax;
                s_ymin <= I_ymin; s_ymax <= I_ymax;
            end else begin
                // 连续检出: 逐帧IIR平滑, 目标移动时红框平滑跟随
                s_xmin <= iir_step(s_xmin, I_xmin, FRAME_W - 1'b1);
                s_xmax <= iir_step(s_xmax, I_xmax, FRAME_W - 1'b1);
                s_ymin <= iir_step({1'b0,s_ymin}, {1'b0,I_ymin}, {1'b0,FRAME_H - 1'b1})[9:0];
                s_ymax <= iir_step({1'b0,s_ymax}, {1'b0,I_ymax}, {1'b0,FRAME_H - 1'b1})[9:0];
            end
        end
    end

    // ---- 空心细红框判定 (矩形内∧距边<BOX_T, 短路保证减法非负, 仅边框不填充) ----
    wire box_region = (I_x >= s_xmin) && (I_x <= s_xmax) &&
                      (I_y >= s_ymin) && (I_y <= s_ymax);
    wire box_on = box_region &&
                  ( ((I_x - s_xmin) < BOX_T) ||
                    ((s_xmax - I_x) < BOX_T) ||
                    ((I_y - s_ymin) < BOX_T) ||
                    ((s_ymax - I_y) < BOX_T) );

    // ---- 输出 (组合0拍, 与输入像素同拍; 消隐期强制无效) ----
    // 仅 target_valid(r_valid) 有效时绘制; 无目标 -> O_osd_en 恒0 -> 透传原图
    wire disp_active = !I_hs && !I_vs;                  // 有效显示区(行场同步双低)
    assign O_osd_en  = r_valid && disp_active && box_on;
    assign O_osd_rgb = 24'hFF0000;                      // 红色锁定框

endmodule
