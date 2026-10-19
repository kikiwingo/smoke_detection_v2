// ============================================================
// sobel_disp.v - 显示域 Sobel 边缘检测 (3x3窗口, 行缓存架构)
// 移植自历史工程 sobel_edge.v (模块/端口改名统一, 逻辑不变)
// 输入: 亮度 Y=(77R+150G+29B)>>8, de/hs/vs (原图显示流)
// 输出: 二值边缘图 (|Gx|+|Gy| > EDGE_TH, 白=边缘, 黑=非边缘)
// 时序: 数据路径2拍(窗口1拍+输出1拍), de/hs/vs 同延迟2拍
// 已知小瑕疵: 行首2列窗口混入上一行末列 (沿用历史实现, 不影响观感)
// ============================================================
module sobel_disp #(
    parameter IMG_WIDTH = 1280,
    parameter [12:0] EDGE_TH = 13'd150   // 边缘阈值 |Gx|+|Gy|, 最大1020
)(
    input               I_clk,
    input               I_rst_n,
    input       [7:0]   I_din,
    input               I_de, I_hsync, I_vsync,
    output reg  [7:0]   O_dout,
    output reg          O_de_out, O_hsync_out, O_vsync_out
);
    // 行缓存
    reg [7:0] line0 [0:IMG_WIDTH-1];
    reg [7:0] line1 [0:IMG_WIDTH-1];
    reg [10:0] wr_ptr;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) wr_ptr <= 11'd0;
        else if(I_de) begin
            if(wr_ptr == IMG_WIDTH-1) wr_ptr <= 11'd0;
            else wr_ptr <= wr_ptr + 11'd1;
        end
    end

    always @(posedge I_clk) begin
        if(I_de) begin
            line0[wr_ptr] <= line1[wr_ptr];
            line1[wr_ptr] <= I_din;
        end
    end

    reg [7:0] p00,p01,p02,p10,p11,p12,p20,p21,p22;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            {p00,p01,p02} <= 24'd0;
            {p10,p11,p12} <= 24'd0;
            {p20,p21,p22} <= 24'd0;
        end else if(I_de) begin
            p00<=p01; p01<=p02; p02<=line0[wr_ptr];
            p10<=p11; p11<=p12; p12<=line1[wr_ptr];
            p20<=p21; p21<=p22; p22<=I_din;
        end
    end

    // 有符号Sobel: 必须先符号扩展为9位有符号再相减,
    // 否则8位无符号减法会回绕, 负梯度变成大数
    wire signed [8:0] sx00 = $signed({1'b0,p00});
    wire signed [8:0] sx01 = $signed({1'b0,p01});
    wire signed [8:0] sx02 = $signed({1'b0,p02});
    wire signed [8:0] sx10 = $signed({1'b0,p10});
    wire signed [8:0] sx12 = $signed({1'b0,p12});
    wire signed [8:0] sx20 = $signed({1'b0,p20});
    wire signed [8:0] sx21 = $signed({1'b0,p21});
    wire signed [8:0] sx22 = $signed({1'b0,p22});

    wire signed [11:0] gx = (sx02 - sx00) + ((sx12 - sx10) <<< 1) + (sx22 - sx20);
    wire signed [11:0] gy = (sx00 - sx20) + ((sx01 - sx21) <<< 1) + (sx02 - sx22);
    wire [11:0] abs_gx = gx[11] ? -gx : gx;
    wire [11:0] abs_gy = gy[11] ? -gy : gy;
    wire [12:0] g_sum = abs_gx + abs_gy;

    reg de_d1, hs_d1, vs_d1;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_dout <= 8'd0;
            de_d1 <= 1'b0; hs_d1 <= 1'b0; vs_d1 <= 1'b0;
            O_de_out <= 1'b0; O_hsync_out <= 1'b0; O_vsync_out <= 1'b0;
        end else begin
            O_dout <= (g_sum > EDGE_TH) ? 8'hFF : 8'h00;
            de_d1 <= I_de; hs_d1 <= I_hsync; vs_d1 <= I_vsync;
            O_de_out <= de_d1; O_hsync_out <= hs_d1; O_vsync_out <= vs_d1;
        end
    end

endmodule
