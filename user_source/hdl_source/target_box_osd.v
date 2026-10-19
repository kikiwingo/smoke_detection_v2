// ============================================================================
// target_box_osd.v  安路 HX1P35A  纯PL流水线 OSD 空心红框 (单目标, 1像素/拍)
// ----------------------------------------------------------------------------
// 功能: 在视频流上叠加 1px 红色空心矩形框 (目标外接框), 只描边、不填充。
//       对应 Python 模型 sim_osd_box.py (151 项检查全 PASS), 逐拍一致。
//
// 接口 (需求端口名 -> 本模块名):
//   hs,vs        -> I_hs/I_vs       行/场同步, 高电平=消隐期
//   x,y          -> I_x/I_y         当前像素坐标 (I_x[10:0]=0..1279, I_y[9:0]=0..719)
//   xmin,ymin,
//   xmax,ymax    -> I_xmin..I_ymax  目标外接框, 帧内稳定 (bbox 帧头快照协议)
//   target_valid -> I_target_valid  目标有效
//   osd_en       -> O_osd_en        1=该像素输出 O_osd_rgb(纯红), 0=下游透传原图
//   osd_rgb      -> O_osd_rgb       24'hFF0000
//
// 逻辑 (按需求逐条):
//   1. 校验: 仅当 xmax>xmin 且 ymax>ymin (严格不等, 宽高均>=2px) 才允许画框,
//      坐标异常(等宽/等高/反转/越界)一律不绘制 — 杜绝 1px 高框整行填充红线。
//   2. 命中 (1px 空心边, 精确相等, 无厚度):
//        ((x==xmin || x==xmax) && (ymin<=y<=ymax))   // 左右竖边
//     || ((y==ymin || y==ymax) && (xmin<=x<=xmax))   // 上下横边
//   3. 场消隐期(vs=1)每拍电平锁存输入坐标, 单帧内坐标固定;
//      vs 下降沿一拍处理: 首帧有效直接装载, 之后逐坐标 IIR 平滑
//      (floor 截断, 收敛残差<=3px, 抑制框抖动); 无效帧冻结平滑态且不绘制。
//   4. target_valid=0 -> box_valid=0, osd_en=0 全透传原图。
//
// 约束: 纯组合+寄存器流水, 无帧缓存/不读DDR/零丢帧, 全帧每拍实时输出,
//       输入输出同拍对齐, 无流水延迟。
//
// 边界保护: 锁存坐标先裁剪到 [0,W-1]x[0,H-1] 再校验 (输入无符号, 只需
//       上限裁剪); IIR 为凸组合, 平滑态永不越出帧界、永不越过目标;
//       绘制前对平滑态再做一次严格校验, 运动瞬态下亦不可能整行/整列填充。
//
// 取舍: 边坐标独立 IIR 在目标快速移动时残差<=3px; 平滑态严格校验使
//       极窄目标(2~3px)快速移动时可能瞬态丢框, 静止目标无影响;
//       框宽/高<2px 的目标按需求视为异常坐标, 不绘制。
//
// 集成: 本模块需外部提供与 1px/拍视频流对齐的 I_x/I_y 像素计数器
//       (现有 osd_box.v 是内部计数, wrapper 的 hdmi 域当前没有现成
//       x/y 信号, 需另加 11/10 位计数器或从时序模块引出);
//       O_osd_en/O_osd_rgb 接下游 alpha 混合 (公式参考 osd_box.v)。
// ============================================================================
module target_box_osd #(
    parameter [11:0] IMG_WIDTH  = 12'd1280,   // 有效行宽
    parameter [10:0] IMG_HEIGHT = 11'd720,    // 有效行高
    parameter [3:0]  IIR_SHIFT  = 4'd2,       // IIR 系数 1/2^SHIFT
    parameter [23:0] OSD_COLOR  = 24'hFF0000  // 纯红
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_hs,                  // 行同步, 高=行消隐
    input  wire        I_vs,                  // 场同步, 高=场消隐
    input  wire [10:0] I_x,                   // 当前像素列
    input  wire [9:0]  I_y,                   // 当前像素行
    input  wire [10:0] I_xmin,                // 目标外接框 (帧内稳定)
    input  wire [9:0]  I_ymin,
    input  wire [10:0] I_xmax,
    input  wire [9:0]  I_ymax,
    input  wire        I_target_valid,        // 目标有效
    output wire        O_osd_en,              // 1=该像素叠加 OSD 颜色
    output wire [23:0] O_osd_rgb              // OSD 颜色 (纯红)
);

    localparam [10:0] W_M1 = IMG_WIDTH  - 12'd1;
    localparam [9:0]  H_M1 = IMG_HEIGHT - 11'd1;

    // ------------------------------------------------------------------------
    // 场消隐期电平锁存: vs=1 期间每拍锁存, 消隐期末拿到最新坐标
    // 单帧内坐标固定 (bbox 帧头快照协议下输入本已帧内稳定, 锁存兜底)
    // ------------------------------------------------------------------------
    reg        tv_l;
    reg [10:0] xmin_l, xmax_l;
    reg [9:0]  ymin_l, ymax_l;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            tv_l   <= 1'b0;
            xmin_l <= 11'd0;
            ymin_l <= 10'd0;
            xmax_l <= 11'd0;
            ymax_l <= 10'd0;
        end else if (I_vs) begin
            tv_l   <= I_target_valid;
            xmin_l <= I_xmin;
            ymin_l <= I_ymin;
            xmax_l <= I_xmax;
            ymax_l <= I_ymax;
        end
    end

    // ------------------------------------------------------------------------
    // 边界保护: 锁存坐标裁剪到帧内 (输入无符号, 只需上限裁剪)
    // 越界退化 (如整框在帧外被裁成 0 宽) 由下方严格校验排除
    // ------------------------------------------------------------------------
    wire [10:0] xmin_c = (xmin_l <= W_M1) ? xmin_l : W_M1;
    wire [10:0] xmax_c = (xmax_l <= W_M1) ? xmax_l : W_M1;
    wire [9:0]  ymin_c = (ymin_l <= H_M1) ? ymin_l : H_M1;
    wire [9:0]  ymax_c = (ymax_l <= H_M1) ? ymax_l : H_M1;

    // 需求 1: 严格校验, 宽高均>=2px 才允许画框
    wire valid_l = tv_l && (xmax_c > xmin_c) && (ymax_c > ymin_c);

    // ------------------------------------------------------------------------
    // vs 下降沿一拍处理: 首帧有效直接装载 / 有效帧 IIR / 无效帧冻结
    // ------------------------------------------------------------------------
    reg vs_d1;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            vs_d1 <= 1'b0;
        else
            vs_d1 <= I_vs;
    end
    wire vs_fall = vs_d1 && !I_vs;

    reg             box_init;                   // 已收到过有效帧
    reg             box_valid;                  // 本帧目标有效且坐标合法
    reg signed [11:0] sq_xmin, sq_xmax;         // 平滑态, 值域 [0,1279]
    reg signed [10:0] sq_ymin, sq_ymax;         // 值域 [0,719]

    // IIR 差值: 目标(已裁剪) - 平滑态, 位宽恰好覆盖值域差, 无溢出
    wire signed [11:0] d_xmin = {1'b0, xmin_c} - sq_xmin;
    wire signed [11:0] d_xmax = {1'b0, xmax_c} - sq_xmax;
    wire signed [10:0] d_ymin = {1'b0, ymin_c} - sq_ymin;
    wire signed [10:0] d_ymax = {1'b0, ymax_c} - sq_ymax;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            box_init  <= 1'b0;
            box_valid <= 1'b0;
            sq_xmin <= 12'd0;
            sq_xmax <= 12'd0;
            sq_ymin <= 11'd0;
            sq_ymax <= 11'd0;
        end else if (vs_fall) begin
            if (tv_l && valid_l && !box_init) begin
                // 首帧有效: 直接装载, 无收敛延迟
                sq_xmin <= $signed({1'b0, xmin_c});
                sq_xmax <= $signed({1'b0, xmax_c});
                sq_ymin <= $signed({1'b0, ymin_c});
                sq_ymax <= $signed({1'b0, ymax_c});
                box_init <= 1'b1;
            end else if (tv_l && valid_l) begin
                // IIR: sq <= sq + (t - sq) >> SHIFT, 算术右移=floor 截断
                // 凸组合保证平滑态不越出帧界; 步进不翻转符号 => 永不越过目标
                sq_xmin <= sq_xmin + (d_xmin >>> IIR_SHIFT);
                sq_xmax <= sq_xmax + (d_xmax >>> IIR_SHIFT);
                sq_ymin <= sq_ymin + (d_ymin >>> IIR_SHIFT);
                sq_ymax <= sq_ymax + (d_ymax >>> IIR_SHIFT);
            end
            // 无效帧(坐标异常/tv=0): 冻结平滑态, box_init 保持
            box_valid <= valid_l;
        end
    end

    // ------------------------------------------------------------------------
    // 绘制: 仅有效像素区, 平滑态再校验后按需求公式描 1px 空心边
    // ------------------------------------------------------------------------
    wire       pixel_valid = !I_vs && !I_hs;    // 消隐期不绘制, 防消隐期
                                                // x/y 冻结值误命中边沿
    wire [10:0] xmin_d = sq_xmin[10:0];         // 平滑态值域在帧内, 低11位即值
    wire [10:0] xmax_d = sq_xmax[10:0];
    wire [9:0]  ymin_d = sq_ymin[9:0];
    wire [9:0]  ymax_d = sq_ymax[9:0];

    // 平滑态再校验: 运动瞬态下若状态退化(宽/高<2px)立即禁画,
    // 从机制上保证永不出现整行/整列填充红线
    wire box_ok = box_valid && (xmax_d > xmin_d) && (ymax_d > ymin_d);

    wire on_v  = (I_x == xmin_d) || (I_x == xmax_d);
    wire on_h  = (I_y == ymin_d) || (I_y == ymax_d);
    wire in_vy = (I_y >= ymin_d) && (I_y <= ymax_d);
    wire in_hx = (I_x >= xmin_d) && (I_x <= xmax_d);

    // 需求 2 命中公式, 精确相等 => 1px 空心边, 无厚度无填充
    assign O_osd_en  = pixel_valid && box_ok && ((on_v && in_vy) || (on_h && in_hx));
    assign O_osd_rgb = OSD_COLOR;

endmodule
