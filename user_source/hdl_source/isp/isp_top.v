module isp_top (
    input 			axi4s_video_aclk,
    input 			I_rst_n			,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    input 			O_tready		,
	input [19:0]    I_min_area,
    input 			I_edge_key		,	// KEY1: 边缘检测切换按键(低有效, 直连板级I_button[1])
    output [127:0] 	O_tdata			,
    output [19:0]   O_candidate_count,
    // 四分屏拼接帧 (quad_isp: 三路2x2平均 -> 1280x720拼接位流, 每2源帧1拼接帧)
    // 旁路自消费: 不参与 tready 流控/不插入数据通路, 检测链与主通路不受影响
    output [127:0]  O_quad_tdata,
    output          O_quad_tvalid,
    // 多目标外接框 (detect_ccl_96 帧头快照, 按面积降序, 帧内稳定) -> OSD框选
    output [21:0]   O_bbox_xmin,     // [i*11+:11] = 第i大目标 (MAX_OBJ=2)
    output [21:0]   O_bbox_xmax,
    output [19:0]   O_bbox_ymin,     // [i*10+:10] = 第i大目标
    output [19:0]   O_bbox_ymax,
    output [1:0]    O_bbox_valid,    // bit i = 第i大目标有效
    output [39:0]   O_area,          // [i*20+:20] = 第i大目标面积 (标定观察)
    output 			O_tlast			,
    output 			O_tuser			,
    output 			O_tvalid		,
    output 			I_tready
);

wire         m_aixs_tvalid;  //synthesis keep
  wire [127:0] m_aixs_tdata;  //synthesis keep
  wire         m_aixs_tuser;  //synthesis keep
  wire         m_aixs_tlast;  //synthesis keep
  wire         m_aixs_tready;  //synthesis keep

  wire [ 95:0] m_aixs_tdata_96;  //synthesis keep


  wire         awb_O_tlast;  //synthesis keep
  wire         awb_O_tuser;  //synthesis keep
  wire [ 95:0] awb_O_tdata;  //synthesis keep
  wire         awb_O_tvalid;  //synthesis keep
  wire         awb_O_tready;  //synthesis keep

  wire         edge_O_tlast;  //synthesis keep
  wire         edge_O_tuser;  //synthesis keep
  wire [ 95:0] edge_O_tdata;  //synthesis keep
  wire         edge_O_tvalid;  //synthesis keep
  wire         edge_O_tready;  //synthesis keep  // 仅承接 edge 组合直通输出, 不再回接(防组合环)

  wire         mask_O_tlast;
  wire         mask_O_tuser;
  wire [95:0]   mask_O_tdata;
  wire         mask_O_tvalid;
  wire [3:0]    mask_candidate;

  wire         awb_O_tuser_128;  //synthesis keep
  wire         awb_O_tvalid_128;  //synthesis keep
  wire [127:0] awb_O_tdata_128;  //synthesis keep

  // KEY1 cycles continuously running views: raw -> edge -> color mask -> raw.
  wire S_edge_key_trig;
  reg [1:0] view_mode;  // 0 raw, 1 edge, 2 color mask
  key_remove_shakes u_edge_key_shakes(
      .I_clk          (axi4s_video_aclk),
      .I_rst_n        (I_rst_n),
      .I_key_in       (~I_edge_key),
      .O_key_trig_out (S_edge_key_trig)
  );
  always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
      if(!I_rst_n) view_mode <= 2'd0;
      else if(S_edge_key_trig) begin
          if(view_mode == 2'd2) view_mode <= 2'd0;
          else view_mode <= view_mode + 2'd1;
      end
  end

  wire         view_tuser;
  wire         view_tvalid;
  wire [95:0]  view_tdata;
  assign view_tuser  = (view_mode == 2'd1) ? edge_O_tuser  :
                       (view_mode == 2'd2) ? mask_O_tuser  : awb_O_tuser;
  assign view_tvalid = (view_mode == 2'd1) ? edge_O_tvalid :
                       (view_mode == 2'd2) ? mask_O_tvalid : awb_O_tvalid;
  assign view_tdata  = (view_mode == 2'd1) ? edge_O_tdata  :
                       (view_mode == 2'd2) ? mask_O_tdata  : awb_O_tdata;
  // ===================== 改动1: tready 跨时钟域同步 + 组合环修复 =====================
  // 原实现 (组合环根因, TD6.2 TMR-2509):
  //   assign awb_O_tready = edge_O_tready;
  //   assign edge_O_tready = O_tready;
  // 1) edge_O_tready 同时被 assign 与 edge_detect_96 的 O_tready 输出端口驱动
  //    -> 多驱动网表, SYN-5066 断开其中一个驱动;
  // 2) edge_detect_96 内部 assign O_tready = I_tready 纯组合直通, 其 I_tready 又接
  //    awb_O_tready, 与 assign 别名构成纯组合自反馈环:
  //    awb_O_tready -> edge.I_tready -> edge.O_tready -> edge_O_tready -> awb_O_tready
  // 修正: O_tready(下游就绪, 跨时钟域输入) 经双触发器同步到 axi4s_video_aclk 域,
  //       D触发器打断组合环; 同步后的 O_tready_syn 直通 awb 输出流控。
  //       edge_O_tready 不再被 assign 驱动, 仅承接 edge 端口输出且不使用。
  // AXI-Stream 语义: tvalid 不依赖 tready; 就绪允许寄存(协议允许), 2 拍建立
  //       延迟由上游 FIFO 裕量吸收, 不丢数据; 复位期 tready=0 保持握手持稳。
  reg O_tready_sync1;
  reg O_tready_sync2;
  always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
      if (!I_rst_n) begin
          O_tready_sync1 <= 1'b0;
          O_tready_sync2 <= 1'b0;
      end else begin
          O_tready_sync1 <= O_tready;       // 第1级: 异步输入亚稳态隔离
          O_tready_sync2 <= O_tready_sync1; // 第2级: 稳定采样
      end
  end
  wire O_tready_syn = O_tready_sync2;

  assign awb_O_tready = O_tready_syn;   // 下游就绪(已同步) -> awb 输出流控
  // edge_O_tready 不再被 assign 驱动, 仅由 edge_detect_96 输出端口驱动且不回接

 
    assign O_tdata  = awb_O_tdata_128;
  assign O_tlast  = awb_O_tlast;
  assign O_tuser  = awb_O_tuser_128;
  assign O_tvalid = awb_O_tvalid_128;

  //assign m_aixs_tready = O_tready;
  //assign O_tdata  = m_aixs_tdata;
  //assign O_tlast  = m_aixs_tlast    ;
  //assign O_tuser  = m_aixs_tuser    ;
  //assign O_tvalid = m_aixs_tvalid   ;


demosaic #(
    .IMG_HEIGHT          (720 ), // 图像高度
    .IMG_WIDTH           (1280), // 图像宽度
    .data_complete_delay (50  ),
    .BAYER_MODE          ("BGGR")
)
u_demosaic
(
    .I_clk   			(axi4s_video_aclk)	,   // 时钟信号
    .I_rst_n 			(I_rst_n)	,   // 复位信号，低有效
    .axi4s_video_tdata 	(I_tdata)	,  // AXI4-Stream视频数据
    .axi4s_video_tdest 	(I_tdest)	,
    .axi4s_video_tlast 	(I_tlast)	,  // 行结束信号
    .axi4s_video_tvalid	(I_tvalid)	,  // 数据有效信号
    .axi4s_video_tuser 	(I_tuser)	,  // 帧开始信号
    .axi4s_video_tready	(I_tready)	,  // 从模块准备好接受数据
    .O_tlast  			(m_aixs_tlast)	,  // 输出行结束信号
    .O_tuser  			(m_aixs_tuser)	,  // 输出帧开始信号
    .O_tdata  			(m_aixs_tdata)	,  // 输出数据
    .O_tvalid 			(m_aixs_tvalid)	,  // 输出数据有效信号
    .O_tready   		(m_aixs_tready)	   // 输出数据准备好信号
);

data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(m_aixs_tdata_96)
);

awb #(
    .IMG_HEIGHT(720 ),
    .IMG_WIDTH (1280)
) u_awb (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (m_aixs_tlast),
    .I_tuser (m_aixs_tuser),
    .I_tdata (m_aixs_tdata_96),
    .I_tvalid(m_aixs_tvalid),
    .I_tready(m_aixs_tready),
    .O_tlast (awb_O_tlast ),
    .O_tuser (awb_O_tuser ),
    .O_tdata (awb_O_tdata ),
    .O_tvalid(awb_O_tvalid),
    .O_tready(awb_O_tready)
);

// ---- Red/orange candidate mask and full-frame candidate-pixel counter ----
color_mask_96 #(
    .RED_MIN       (8'd160),
    .GREEN_MIN     (8'd0),
    .RED_GREEN_GAP (8'd64),
    .RED_BLUE_GAP  (8'd48),
    .LUMA_SUM_MIN  (10'd210)
) u_color_mask_96 (
    .I_clk          (axi4s_video_aclk),
    .I_rst_n        (I_rst_n),
    .I_tlast        (awb_O_tlast),
    .I_tuser        (awb_O_tuser),
    .I_tdata        (awb_O_tdata),
    .I_tvalid       (awb_O_tvalid),
    .O_tlast        (mask_O_tlast),
    .O_tuser        (mask_O_tuser),
    .O_tdata        (mask_O_tdata),
    .O_tvalid       (mask_O_tvalid),
    .O_frame_count  (O_candidate_count),
    .O_candidate    (mask_candidate)
);
// ---- 连通域检测: 红色候选 → 多目标外接框+面积 ----
// 4邻域 union-find 合并, 按面积降序输出全部达标连通域 (MAX_OBJ=2);
// O_bbox_valid[i]=第i大连通域面积>=帧锁存的 I_min_area (复位初值64)
// (绿色像素/掩码显示色不产生候选, 不触发)
// Latch the selected threshold at frame start so it stays constant throughout
// each frame and cannot change component decisions mid-frame.
reg [19:0] S_min_area_frame;
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if(!I_rst_n)
        S_min_area_frame <= 20'd64;
    else if(awb_O_tuser)
        S_min_area_frame <= I_min_area;
end

detect_ccl_96 #(
    .IMG_WIDTH (1280),
    .IMG_HEIGHT(720)
) u_detect_ccl_96 (
    .I_clk        (axi4s_video_aclk),
    .I_rst_n      (I_rst_n),
    .I_candidate  (mask_candidate),
    .I_tuser      (awb_O_tuser),
    .I_tlast      (awb_O_tlast),
    .I_tvalid     (awb_O_tvalid),
    .I_min_area   (S_min_area_frame),
    .O_bbox_xmin  (O_bbox_xmin),
    .O_bbox_xmax  (O_bbox_xmax),
    .O_bbox_ymin  (O_bbox_ymin),
    .O_bbox_ymax  (O_bbox_ymax),
    .O_bbox_valid (O_bbox_valid),
    .O_area       (O_area)
);
// ---- 边缘检测 (lab_ex2功能, 与原图通路并行运行) ----
edge_detect_96 #(
    .IMG_WIDTH (1280),
    .IMG_HEIGHT(720),
    .EDGE_TH   (11'd80)
) u_edge_detect_96 (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (awb_O_tlast),
    .I_tuser (awb_O_tuser),
    .I_tdata (awb_O_tdata),
    .I_tvalid(awb_O_tvalid),
    .I_tready(awb_O_tready),
    .O_tlast (edge_O_tlast),
    .O_tuser (edge_O_tuser),
    .O_tdata (edge_O_tdata),
    .O_tvalid(edge_O_tvalid),
    .O_tready(edge_O_tready)
);

    data_96bit_to_128bit u_data_96bit_to_128bit(
        .I_clk              ( axi4s_video_aclk  ),
        .I_rst_n            ( I_rst_n           ),

        .I_96b_frame_start  ( view_tuser	    ),
        .I_96b_valid        ( view_tvalid 		),
        .I_96b_data         ( view_tdata		),

        .O_128b_frame_start ( awb_O_tuser_128  	),
        .O_128b_valid       ( awb_O_tvalid_128 	),
        .O_128b_data        ( awb_O_tdata_128  	)
    );

    // ---- 四分屏拼接帧 (三路常驻并行流的 2x2 平均拼接) ----
    // 控制时序取自 awb (三路同源); 内容取自 awb/edge/mask 三路输出
    // (edge/mask 相对 awb 有常数拍延迟, 仅表现为象限内容水平平移, 不可见)
    quad_isp u_quad_isp(
        .I_clk        (axi4s_video_aclk),
        .I_rst_n      (I_rst_n),
        .I_tlast      (awb_O_tlast),
        .I_tuser      (awb_O_tuser),
        .I_tvalid     (awb_O_tvalid),
        .I_orig_tdata (awb_O_tdata),
        .I_edge_tdata (edge_O_tdata),
        .I_mask_tdata (mask_O_tdata),
        .O_tdata      (O_quad_tdata),
        .O_tvalid     (O_quad_tvalid)
    );



//data96_128 u_data96_128 (
//    .I_tdata(awb_O_tdata),
//    .O_tdata(awb_O_tdata_128)
//);

endmodule
