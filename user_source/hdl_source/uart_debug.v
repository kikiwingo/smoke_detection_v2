// ============================================================
// uart_debug.v - 检测状态 UART 调试输出 (每秒一行)
// 时钟: hdmi 像素时钟域 (74.25MHz)
// 格式 (12字节, 每秒一次): "F0 C0 T150\r\n"
//   F<0/1> = 明火有效   C<0/1> = 烟头有效   T<3位十进制> = 烟头阈值 LUM_TH
// 复用官方 uart_tx.v (115200, 8N1), 输出接板子 UART TX (H1)。
// 用途: 上位机/串口终端实时监控检测状态, 便于场景标定与记录。
// ============================================================
module uart_debug (
    input  wire        I_clk,          // 74.25MHz
    input  wire        I_rst_n,
    input  wire        I_frame_tick,   // 每帧一个脉冲
    input  wire        I_fire_valid,   // 明火有效
    input  wire        I_cig_valid,    // 烟头有效
    input  wire [7:0]  I_lum_th,       // 烟头阈值
    output wire        O_tx_pin
);

    // ---- 每秒触发一次 (60帧 @60fps) ----
    reg [5:0] frame_cnt;
    reg       send_trig;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            frame_cnt <= 6'd0;
            send_trig <= 1'b0;
        end else begin
            send_trig <= 1'b0;
            if(I_frame_tick) begin
                if(frame_cnt == 6'd59) begin
                    frame_cnt <= 6'd0;
                    send_trig <= 1'b1;
                end else
                    frame_cnt <= frame_cnt + 6'd1;
            end
        end
    end

    // ---- 阈值 BCD (8bit -> 3位十进制, 组合 double-dabble) ----
    function [11:0] bin8_bcd;
        input [7:0] bin;
        reg [19:0] sh;
        integer i;
        begin
            sh = {12'b0, bin};
            for(i=0; i<8; i=i+1) begin
                if(sh[19:16] >= 5) sh[19:16] = sh[19:16] + 4'd3;
                if(sh[15:12] >= 5) sh[15:12] = sh[15:12] + 4'd3;
                if(sh[11:8]  >= 5) sh[11:8]  = sh[11:8]  + 4'd3;
                sh = sh << 1;
            end
            bin8_bcd = sh[19:8];
        end
    endfunction
    wire [11:0] th_bcd = bin8_bcd(I_lum_th);

    // ---- 当前发送字节内容 (组合) ----
    reg [3:0] byte_idx;
    reg       sending;
    reg       tx_valid;
    wire      tx_ready;
    reg [7:0] tx_data;
    always @* begin
        tx_data = 8'h00;
        case(byte_idx)
            4'd0:  tx_data = 8'h46;                    // 'F'
            4'd1:  tx_data = 8'h30 + I_fire_valid;     // 明火 0/1
            4'd2:  tx_data = 8'h20;                    // ' '
            4'd3:  tx_data = 8'h43;                    // 'C'
            4'd4:  tx_data = 8'h30 + I_cig_valid;      // 烟头 0/1
            4'd5:  tx_data = 8'h20;                    // ' '
            4'd6:  tx_data = 8'h54;                    // 'T'
            4'd7:  tx_data = 8'h30 + th_bcd[11:8];     // 阈值百位
            4'd8:  tx_data = 8'h30 + th_bcd[7:4];      // 十位
            4'd9:  tx_data = 8'h30 + th_bcd[3:0];      // 个位
            4'd10: tx_data = 8'h0D;                    // '\r'
            4'd11: tx_data = 8'h0A;                    // '\n'
            default: tx_data = 8'h00;
        endcase
    end

    // ---- 发送状态机 (字节被 uart_tx 接受后推进) ----
    wire byte_accepted = tx_valid && tx_ready;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            byte_idx <= 4'd0;
            sending  <= 1'b0;
            tx_valid <= 1'b0;
        end else begin
            if(send_trig && !sending) begin
                sending  <= 1'b1;
                byte_idx <= 4'd0;
                tx_valid <= 1'b1;
            end else if(sending && byte_accepted) begin
                if(byte_idx == 4'd11) begin
                    sending  <= 1'b0;
                    tx_valid <= 1'b0;
                end else begin
                    byte_idx <= byte_idx + 4'd1;   // tx_valid 保持 1
                end
            end
        end
    end

    // ---- 官方 UART TX (74.25MHz 近似取 74, 误差 0.4% 在 UART 容忍内) ----
    uart_tx #(
        .CLK_FRE   (74),
        .BAUD_RATE (115200)
    ) u_uart_tx(
        .clk          (I_clk),
        .rst_n        (I_rst_n),
        .tx_data      (tx_data),
        .tx_data_valid(tx_valid),
        .tx_data_ready(tx_ready),
        .tx_pin       (O_tx_pin)
    );

endmodule
