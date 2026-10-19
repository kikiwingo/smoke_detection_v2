// Four-pixel RGB888 color candidate mask and per-frame candidate counter.
// The input word follows isp_top's 4-pixel/96-bit convention: pixel 0 is [95:72].
module color_mask_96 #(
    parameter [7:0] RED_MIN       = 8'd160,
    parameter [7:0] GREEN_MIN     = 8'd0,
    parameter [7:0] RED_GREEN_GAP = 8'd64,
    parameter [7:0] RED_BLUE_GAP  = 8'd48,
    parameter [9:0] LUMA_SUM_MIN = 10'd210
)(
    input wire        I_clk,
    input wire        I_rst_n,
    input wire        I_tlast,
    input wire        I_tuser,
    input wire [95:0] I_tdata,
    input wire        I_tvalid,
    output wire       O_tlast,
    output wire       O_tuser,
    output wire [95:0] O_tdata,
    output wire       O_tvalid,
    output reg [19:0] O_frame_count,
    output wire [3:0] O_candidate
    // 外接框/告警判定由 detect_ccl_96 (连通域检测) 输出, 本模块只出候选与计数
);
    wire [23:0] pixel0 = I_tdata[95:72];
    wire [23:0] pixel1 = I_tdata[71:48];
    wire [23:0] pixel2 = I_tdata[47:24];
    wire [23:0] pixel3 = I_tdata[23:0];

    function is_candidate;
        input [23:0] pixel;
        reg [7:0] r, g, b;
        reg [8:0] g_gap, b_gap;
        reg [9:0] luma_sum;
        begin
            r = pixel[23:16];
            g = pixel[15:8];
            b = pixel[7:0];
            g_gap = {1'b0,g} + {1'b0,RED_GREEN_GAP};
            b_gap = {1'b0,b} + {1'b0,RED_BLUE_GAP};
            luma_sum = {2'b0,r} + {2'b0,g} + {2'b0,b};
            // Strict bright-red candidate filter: reject weak red casts and
            // most skin tones by requiring both high R and strong R dominance.
            is_candidate = (r >= RED_MIN) && (g >= GREEN_MIN) &&
                           ({1'b0,r} > g_gap) && ({1'b0,r} > b_gap) &&
                           (luma_sum >= LUMA_SUM_MIN);
        end
    endfunction

    assign O_candidate[0] = is_candidate(pixel0);
    assign O_candidate[1] = is_candidate(pixel1);
    assign O_candidate[2] = is_candidate(pixel2);
    assign O_candidate[3] = is_candidate(pixel3);

    // Green marks candidate pixels; black is the background for easy threshold tuning.
    assign O_tdata[95:72] = O_candidate[0] ? 24'h00ff00 : 24'h000000;
    assign O_tdata[71:48] = O_candidate[1] ? 24'h00ff00 : 24'h000000;
    assign O_tdata[47:24] = O_candidate[2] ? 24'h00ff00 : 24'h000000;
    assign O_tdata[23:0]  = O_candidate[3] ? 24'h00ff00 : 24'h000000;
    assign O_tlast = I_tlast;
    assign O_tuser = I_tuser;
    assign O_tvalid = I_tvalid;

    reg [19:0] count_accum;
    wire [2:0] hit_count = {2'b0,O_candidate[0]} + {2'b0,O_candidate[1]} +
                           {2'b0,O_candidate[2]} + {2'b0,O_candidate[3]};

    // ---- 帧级候选像素计数 (外接框跟踪已迁移至 detect_ccl_96 连通域检测) ----
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            count_accum   <= 20'd0;
            O_frame_count <= 20'd0;
        end else if(I_tuser) begin
            // 帧头: 快照上一帧的计数, 复位开始新帧 (tuser与首字同拍: 立即计入首字)
            O_frame_count <= count_accum;
            count_accum   <= I_tvalid ? {{17{1'b0}},hit_count} : 20'd0;
        end else if(I_tvalid) begin
            count_accum <= count_accum + hit_count;
        end
    end
endmodule
