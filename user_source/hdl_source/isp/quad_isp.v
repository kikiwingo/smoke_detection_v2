// Four-view mosaic stream generator.
// Input: four RGB pixels per 96-bit beat, 320 valid beats per 1280-pixel line.
// Output: sequential 128-bit RGB stream for quad_writer.
// Each 2x2-downsampled view is 640x360; two 640-pixel lines are packed into
// one 1280-pixel mosaic line. Each camera frame contributes 360 mosaic lines.
module quad_isp (
    input              I_clk,
    input              I_rst_n,
    input              I_tlast,
    input              I_tuser,
    input              I_tvalid,
    input      [95:0]  I_orig_tdata,
    input      [95:0]  I_edge_tdata,
    input      [95:0]  I_mask_tdata,
    output reg         O_tvalid,
    output reg [127:0] O_tdata
);
    reg [8:0] word_cnt;
    reg [9:0] line_cnt;
    reg       frame_par;

    // AXI video markers are used as row/frame boundaries. The marker beat
    // carries the first pixel of the frame, so select the new parity on tuser.
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            word_cnt <= 9'd0;
            line_cnt <= 10'd0;
            frame_par <= 1'b0;
        end else begin
            if (I_tuser) begin
                line_cnt <= 10'd0;
                frame_par <= ~frame_par;
            end else if (I_tlast) begin
                line_cnt <= line_cnt + 10'd1;
            end

            if (I_tlast)
                word_cnt <= 9'd0;
            else if (I_tvalid)
                word_cnt <= word_cnt + 9'd1;
        end
    end

    wire frame_eff = I_tuser ? ~frame_par : frame_par;
    wire [9:0] line_eff = I_tuser ? 10'd0 : line_cnt;

    wire [47:0] orig_pair;
    wire [47:0] edge_pair;
    wire [47:0] mask_pair;

    quad_hv_avg u_hv_orig (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_tvalid(I_tvalid),
        .I_tlast(I_tlast), .I_tuser(I_tuser), .I_word_cnt(word_cnt),
        .I_tdata(I_orig_tdata), .O_v_pair(orig_pair)
    );
    quad_hv_avg u_hv_edge (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_tvalid(I_tvalid),
        .I_tlast(I_tlast), .I_tuser(I_tuser), .I_word_cnt(word_cnt),
        .I_tdata(I_edge_tdata), .O_v_pair(edge_pair)
    );
    quad_hv_avg u_hv_mask (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_tvalid(I_tvalid),
        .I_tlast(I_tlast), .I_tuser(I_tuser), .I_word_cnt(word_cnt),
        .I_tdata(I_mask_tdata), .O_v_pair(mask_pair)
    );

    // Upper half alternates original/edge by source-line parity; lower half
    // alternates mask/original. This creates the same four-view arrangement.
    wire [47:0] pair_data = !frame_eff ?
                            (line_eff[0] ? edge_pair : orig_pair) :
                            (line_eff[0] ? orig_pair : mask_pair);

    // Eight 48-bit beats form exactly 384 bits, or three 128-bit words.
    // Packing advances only on valid beats. Unlike the old gap-flush logic,
    // blanking or an isolated valid bubble cannot emit stale/partial words.
    reg [383:0] sh;
    reg [2:0]   pack_count;
    wire [383:0] sh_next = {sh[335:0], pair_data};
    wire group_done = I_tvalid && (pack_count == 3'd7);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            sh <= 384'd0;
            pack_count <= 3'd0;
        end else if (I_tvalid) begin
            sh <= sh_next;
            pack_count <= group_done ? 3'd0 : pack_count + 3'd1;
        end
    end

    // Emit word A at group completion, then B and C on the next two clocks.
    // This small drain state runs even when input tvalid is low, so every
    // complete group is delivered while the next group is being collected.
    reg [127:0] word_b;
    reg [127:0] word_c;
    reg [1:0]   drain_state;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            word_b <= 128'd0;
            word_c <= 128'd0;
            drain_state <= 2'd0;
            O_tvalid <= 1'b0;
            O_tdata <= 128'd0;
        end else begin
            O_tvalid <= 1'b0;

            if (group_done) begin
                O_tvalid <= 1'b1;
                O_tdata <= sh_next[383:256];
                word_b <= sh_next[255:128];
                word_c <= sh_next[127:0];
                drain_state <= 2'd1;
            end else if (drain_state == 2'd1) begin
                O_tvalid <= 1'b1;
                O_tdata <= word_b;
                drain_state <= 2'd2;
            end else if (drain_state == 2'd2) begin
                O_tvalid <= 1'b1;
                O_tdata <= word_c;
                drain_state <= 2'd0;
            end
        end
    end
endmodule
