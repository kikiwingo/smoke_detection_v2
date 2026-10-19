// ============================================================
// mask_disp.v - 显示域红橙色彩掩码视图 (与 color_mask_96 同判据)
// 判据: r>=160 && r>g+64 && r>b+48 && r+g+b>=210
// 输出: 候选像素绿色高亮 (0,255,0), 其余黑色背景
// 时序: 1拍寄存输出, de/hs/vs 同延迟1拍
// ============================================================
module mask_disp (
    input               I_clk,
    input               I_rst_n,
    input       [23:0]  I_din,
    input               I_de, I_hsync, I_vsync,
    output reg  [23:0]  O_dout,
    output reg          O_de_out, O_hsync_out, O_vsync_out
);
    wire [7:0] r = I_din[23:16];
    wire [7:0] g = I_din[15:8];
    wire [7:0] b = I_din[7:0];
    // 无符号比较, 显式扩位防截断
    wire cand = (r >= 8'd160) &&
                ({1'b0,r} > {1'b0,g} + 9'd64) &&
                ({1'b0,r} > {1'b0,b} + 9'd48) &&
                ({1'b0,r} + {1'b0,g} + {1'b0,b} >= 9'd210);

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_dout <= 24'd0;
            O_de_out <= 1'b0; O_hsync_out <= 1'b0; O_vsync_out <= 1'b0;
        end else begin
            O_dout <= cand ? 24'h00FF00 : 24'h000000;
            O_de_out <= I_de; O_hsync_out <= I_hsync; O_vsync_out <= I_vsync;
        end
    end

endmodule
