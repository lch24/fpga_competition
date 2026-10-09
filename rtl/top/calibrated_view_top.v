`timescale 1ns/1ps
module calibrated_view_top#(
	parameter FIXED_BILINEAR = 1, parameter FIXED_ACCUM=1,
    parameter CAMERA_SELECT = 2,
    parameter CAPTURE_WAIT_CYCLES = 625000000,
    parameter [63:0] SQUARE_SIZE_FP64 = 64'h3ff0000000000000,
    parameter [31:0] CANDIDATE_BASE = 32'h03200000,
    parameter MEM_ROW_ADDR_WIDTH   = 15         ,
	parameter MEM_COL_ADDR_WIDTH   = 10         ,
	parameter MEM_BADDR_WIDTH      = 3          ,
	parameter MEM_DQ_WIDTH         =  32        ,
	parameter MEM_DQS_WIDTH        = 32/8
)(
	input                                sys_clk              ,// Original PLL configuration expects approximately 27 MHz
    input                                clk_p ,
    input                                clk_n ,
//OV5640
    output  [1:0]                        cmos_init_done       ,//OV5640寄存器初始化完成
    //coms1	
    inout                                cmos1_scl            ,//cmos1 i2c 
    inout                                cmos1_sda            ,//cmos1 i2c 
    input                                cmos1_vsync          ,//cmos1 vsync
    input                                cmos1_href           ,//cmos1 hsync refrence,data valid
    input                                cmos1_pclk           ,//cmos1 pxiel clock
    input   [7:0]                        cmos1_data           ,//cmos1 data
    output                               cmos1_reset          ,//cmos1 reset
    //coms2
    inout                                cmos2_scl            ,//cmos2 i2c 
    inout                                cmos2_sda            ,//cmos2 i2c 
    input                                cmos2_vsync          ,//cmos2 vsync
    input                                cmos2_href           ,//cmos2 hsync refrence,data valid
    input                                cmos2_pclk           ,//cmos2 pxiel clock
    input   [7:0]                        cmos2_data           ,//cmos2 data
    output                               cmos2_reset          ,//cmos2 reset
//DDR
    output                               mem_rst_n                 ,
    output                               mem_ck                    ,
    output                               mem_ck_n                  ,
    output                               mem_cke                   ,
    output                               mem_cs_n                  ,
    output                               mem_ras_n                 ,
    output                               mem_cas_n                 ,
    output                               mem_we_n                  ,
    output                               mem_odt                   ,
    output      [MEM_ROW_ADDR_WIDTH-1:0] mem_a                     ,
    output      [MEM_BADDR_WIDTH-1:0]    mem_ba                    ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs                   ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs_n                 ,
    inout       [MEM_DQ_WIDTH-1:0]       mem_dq                    ,
    output      [MEM_DQ_WIDTH/8-1:0]     mem_dm                    ,
    output reg                           heart_beat_led            ,
    output                               ddr_init_done             ,
//MS72xx       
    output                               rstn_out                  ,
    output                               iic_tx_scl                ,
    inout                                iic_tx_sda                ,
    output                               hdmi_int_led              ,//HDMI_OUT初始化完成
//HDMI_OUT
    output                               pix_clk                   ,//pixclk                           
    output     reg                       vs_out                    , 
    output     reg                       hs_out                    , 
    output     reg                       de_out                    ,
    output     reg[7:0]                  r_out                     , 
    output     reg[7:0]                  g_out                     , 
    output     reg[7:0]                  b_out         
);

localparam WIDTH=1280,HEIGHT=720;
localparam [31:0] RAW_BASE=0,DST_BASE=32'h01000000;
localparam TH_1S=33000000;
wire cfg_clk,clk_25M,locked,init_over_rx,initial_en,core_clk,clk_125Mhz;
wire pll_lock,phy_pll_lock,gpll_lock,rst_gpll_lock,ddrphy_cpd_lock;
reg [15:0] rstn_1ms=0;reg [26:0] cnt;
wire board_rst_n,core_rst_n,pixel_rst_n,camera_rst_n;
wire algorithm_clk,algorithm_locked,algorithm_rst_n;
algorithm_clock algorithm_clock_gen(.clkin(clk_125Mhz),.clkout(algorithm_clk),.locked(algorithm_locked));
reset_sync ar(.clk(algorithm_clk),.arst_n(rstn_out&&ddr_init_done&&algorithm_locked),.rst_n(algorithm_rst_n));
reset_sync br(.clk(sys_clk),.arst_n(locked),.rst_n(board_rst_n));
reset_sync cr(.clk(core_clk),.arst_n(rstn_out&&ddr_init_done&&algorithm_locked),.rst_n(core_rst_n));
reset_sync pr(.clk(pix_clk),.arst_n(rstn_out&&ddr_init_done&&algorithm_locked),.rst_n(pixel_rst_n));
wire cam_clk=CAMERA_SELECT==1?cmos1_pclk:cmos2_pclk;
wire cam_vs=CAMERA_SELECT==1?cmos1_vsync:cmos2_vsync;
wire cam_href=CAMERA_SELECT==1?cmos1_href:cmos2_href;
wire [7:0] cam_data=CAMERA_SELECT==1?cmos1_data:cmos2_data;
wire cam_configured=CAMERA_SELECT==1?cmos_init_done[0]:cmos_init_done[1];
reset_sync camr(.clk(cam_clk),.arst_n(rstn_out&&ddr_init_done&&algorithm_locked),.rst_n(camera_rst_n));
reg [1:0] ready_sync /* synthesis PAP_ASYNC_REG=1 */;
always @(posedge core_clk or negedge core_rst_n)
 if(!core_rst_n)ready_sync<=0;else ready_sync<={ready_sync[0],cam_configured};
//PLL

    pll u_pll (
        .clkin1   (  sys_clk    ),//50MHz
        .clkout0  (  pix_clk    ),//37.125M 720P30
        .clkout1  (  cfg_clk    ),//10MHz
        .clkout2  (  clk_25M    ),//25M
        .lock (  locked     )
    );

//配置7210
wire init_over_tx;
    board_ms72xx_ctl ms72xx_ctl(
        .clk             (  cfg_clk        ), //input       clk,
        .rst_n           (  rstn_out       ), //input       rstn,
        .init_over    (  init_over_tx   ), //output      init_over,                                
        .init_over_rx    (  init_over_rx   ), //output      init_over,
//        .iic_tx_scl      (  iic_tx_scl     ), //output      iic_scl,
//        .iic_tx_sda      (  iic_tx_sda     ), //inout       iic_sda
        .iic_scl         (  iic_tx_scl        ), //output      iic_scl,
        .iic_sda         (  iic_tx_sda        )  //inout       iic_sda
    );
//   assign    hdmi_int_led    =    init_over_tx; 
    
    always @(posedge cfg_clk)
    begin
    	if(!locked)
    	    rstn_1ms <= 16'd0;
    	else
    	begin
    		if(rstn_1ms == 16'h2710)
    		    rstn_1ms <= rstn_1ms;
    		else
    		    rstn_1ms <= rstn_1ms + 1'b1;
    	end
    end
    
    assign rstn_out = (rstn_1ms == 16'h2710);

//配置CMOS///////////////////////////////////////////////////////////////////////////////////
//OV5640 register configure enable    
    board_power_on_delay	power_on_delay_inst(
    	.clk_50M                 (sys_clk        ),//input
    	.reset_n                 (board_rst_n    ),//input	
    	.camera1_rstn            (cmos1_reset    ),//output
    	.camera2_rstn            (cmos2_reset    ),//output	
    	.camera_pwnd             (               ),//output
    	.initial_en              (initial_en     ) //output		
    );
//CMOS1 Camera 
    reg_config	comos1_reg_config(
    	.clk_25M                 (clk_25M            ),//input
    	.camera_rstn             (cmos1_reset        ),//input
    	.initial_en              (initial_en         ),//input		
    	.i2c_sclk                (cmos1_scl          ),//output
    	.i2c_sdat                (cmos1_sda          ),//inout
    	.reg_conf_done           (cmos_init_done[0]  ),//output config_finished
    	.reg_index               (                   ),//output reg [8:0]
    	.clock_20k               (                   ) //output reg
    );

//CMOS2 Camera 
    reg_config	comos2_reg_config(
    	.clk_25M                 (clk_25M            ),//input
    	.camera_rstn             (cmos2_reset        ),//input
    	.initial_en              (initial_en         ),//input		
    	.i2c_sclk                (cmos2_scl          ),//output
    	.i2c_sdat                (cmos2_sda          ),//inout
    	.reg_conf_done           (cmos_init_done[1]  ),//output config_finished
    	.reg_index               (                   ),//output reg [8:0]
    	.clock_20k               (                   ) //output reg
    );

wire rd_valid;
wire rd_ready;
wire [31:0] rd_addr;
wire [31:0] rd_len;
wire [15:0] rd_tag;
wire r_valid;
wire r_ready;
wire [31:0] r_data;
wire [3:0] r_keep;
wire [15:0] r_tag;
wire r_last;
wire r_error;
wire wr_valid;
wire wr_ready;
wire [31:0] wr_addr;
wire [31:0] wr_len;
wire [15:0] wr_tag;
wire w_valid;
wire w_ready;
wire [31:0] w_data;
wire [3:0] w_keep;
wire w_last;
wire b_valid;
wire b_ready;
wire [15:0] b_tag;
wire b_error;
wire [27:0] axi_araddr,alg_araddr;
wire [3:0] axi_aruser_id,alg_aruser_id;
wire [3:0] axi_arlen,alg_arlen;
wire axi_aruser_ap,alg_aruser_ap;
wire axi_arvalid,alg_arvalid;
wire [27:0] axi_awaddr,alg_awaddr;
wire [3:0] axi_awuser_id,alg_awuser_id;
wire [3:0] axi_awlen,alg_awlen;
wire axi_awuser_ap,alg_awuser_ap;
wire axi_awvalid,alg_awvalid;
wire [255:0] axi_wdata,alg_wdata;
wire [31:0] axi_wstrb,alg_wstrb;
wire axi_awready,axi_wready,axi_wusero_last,axi_arready,axi_rvalid,axi_rlast;
wire [3:0] axi_wusero_id,axi_rid;wire [255:0] axi_rdata;
wire start_valid,start_ready,frame_ready,frame_valid,release_valid,release_ready;
wire cap_cmd_valid,cap_cmd_ready,cap_rsp_valid,cap_rsp_ready,capture_owner,display_enable;
wire rsp_valid,rsp_ready,waiting;wire [7:0] cap_status,frame_status,rsp_status,status;
wire [3:0] phase;
wire [7:0] debug_view /* synthesis PAP_MARK_DEBUG="1" */;
wire [5:0] debug_phase /* synthesis PAP_MARK_DEBUG="1" */;
wire [7:0] debug_status /* synthesis PAP_MARK_DEBUG="1" */;
assign debug_status=status;
board_flow #(.CAPTURE_WAIT_CYCLES(CAPTURE_WAIT_CYCLES)) flow(
 .clk(core_clk),.rst_n(core_rst_n),.camera_ready(ready_sync[1]),
 .start_valid(start_valid),.start_ready(start_ready),.frame_ready(frame_ready),.frame_valid(frame_valid),.frame_status(frame_status),
 .frame_release_valid(release_valid),.frame_release_ready(release_ready),
 .cap_cmd_valid(cap_cmd_valid),.cap_cmd_ready(cap_cmd_ready),.cap_rsp_valid(cap_rsp_valid),.cap_rsp_ready(cap_rsp_ready),.cap_status(cap_status),
 .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.capture_owner(capture_owner),.display_enable(display_enable),
 .status(status),.waiting(waiting),.phase(phase));
wire cam_pixel_valid;wire [15:0] cam_pixel;
camera_byte_unpack unpack(.pclk(cam_clk),.rst_n(camera_rst_n),.vsync(cam_vs),.href(cam_href),.data(cam_data),.pixel_valid(cam_pixel_valid),.pixel(cam_pixel));
wire [27:0] cap_awaddr;wire [3:0] cap_awlen,cap_awid;wire cap_awvalid;
wire [255:0] cap_wdata;wire [31:0] cap_wstrb;
board_capture_hmic #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.BASE(RAW_BASE)) capture(
 .clk(core_clk),.rst_n(core_rst_n),.pclk(cam_clk),.prst_n(camera_rst_n),.vsync(cam_vs),.pixel_valid(cam_pixel_valid),.pixel(cam_pixel),
 .cmd_valid(cap_cmd_valid),.cmd_ready(cap_cmd_ready),.rsp_valid(cap_rsp_valid),.rsp_ready(cap_rsp_ready),.rsp_status(cap_status),
 .axi_awaddr(cap_awaddr),.axi_awlen(cap_awlen),.axi_awuser_id(cap_awid),.axi_awvalid(cap_awvalid),.axi_awready(axi_awready&&capture_owner),
 .axi_wdata(cap_wdata),.axi_wstrb(cap_wstrb),.axi_wready(axi_wready&&capture_owner),.axi_wusero_last(axi_wusero_last&&capture_owner),.axi_wusero_id(axi_wusero_id));
// Independent 40 MHz algorithm domain; board/DDR remain at 125 MHz.
wire a_start_valid;
wire a_start_ready;
wire a_frame_valid;
wire a_frame_ready;
wire [7:0] a_frame_status;
wire a_release_valid;
wire a_release_ready;
wire a_rsp_valid;
wire a_rsp_ready;
wire [7:0] a_rsp_status;
wire a_rd_valid;
wire a_rd_ready;
wire [31:0] a_rd_addr;
wire [31:0] a_rd_len;
wire [15:0] a_rd_tag;
wire a_r_valid;
wire a_r_ready;
wire [31:0] a_r_data;
wire [3:0] a_r_keep;
wire [15:0] a_r_tag;
wire a_r_last;
wire a_r_error;
wire a_wr_valid;
wire a_wr_ready;
wire [31:0] a_wr_addr;
wire [31:0] a_wr_len;
wire [15:0] a_wr_tag;
wire a_w_valid;
wire a_w_ready;
wire [31:0] a_w_data;
wire [3:0] a_w_keep;
wire a_w_last;
wire a_b_valid;
wire a_b_ready;
wire [15:0] a_b_tag;
wire a_b_error;
vision_clock_bridge algorithm_cdc(.core_clk(core_clk),.core_rst_n(core_rst_n),
 .algorithm_clk(algorithm_clk),.algorithm_rst_n(algorithm_rst_n),
 .a_start_valid(a_start_valid),
 .start_valid(start_valid),
 .a_start_ready(a_start_ready),
 .start_ready(start_ready),
 .a_frame_valid(a_frame_valid),
 .frame_valid(frame_valid),
 .a_frame_ready(a_frame_ready),
 .frame_ready(frame_ready),
 .a_frame_status(a_frame_status),
 .frame_status(frame_status),
 .a_release_valid(a_release_valid),
 .release_valid(release_valid),
 .a_release_ready(a_release_ready),
 .release_ready(release_ready),
 .a_rsp_valid(a_rsp_valid),
 .rsp_valid(rsp_valid),
 .a_rsp_ready(a_rsp_ready),
 .rsp_ready(rsp_ready),
 .a_rsp_status(a_rsp_status),
 .rsp_status(rsp_status),
 .a_rd_valid(a_rd_valid),
 .rd_valid(rd_valid),
 .a_rd_ready(a_rd_ready),
 .rd_ready(rd_ready),
 .a_rd_addr(a_rd_addr),
 .rd_addr(rd_addr),
 .a_rd_len(a_rd_len),
 .rd_len(rd_len),
 .a_rd_tag(a_rd_tag),
 .rd_tag(rd_tag),
 .a_r_valid(a_r_valid),
 .r_valid(r_valid),
 .a_r_ready(a_r_ready),
 .r_ready(r_ready),
 .a_r_data(a_r_data),
 .r_data(r_data),
 .a_r_keep(a_r_keep),
 .r_keep(r_keep),
 .a_r_tag(a_r_tag),
 .r_tag(r_tag),
 .a_r_last(a_r_last),
 .r_last(r_last),
 .a_r_error(a_r_error),
 .r_error(r_error),
 .a_wr_valid(a_wr_valid),
 .wr_valid(wr_valid),
 .a_wr_ready(a_wr_ready),
 .wr_ready(wr_ready),
 .a_wr_addr(a_wr_addr),
 .wr_addr(wr_addr),
 .a_wr_len(a_wr_len),
 .wr_len(wr_len),
 .a_wr_tag(a_wr_tag),
 .wr_tag(wr_tag),
 .a_w_valid(a_w_valid),
 .w_valid(w_valid),
 .a_w_ready(a_w_ready),
 .w_ready(w_ready),
 .a_w_data(a_w_data),
 .w_data(w_data),
 .a_w_keep(a_w_keep),
 .w_keep(w_keep),
 .a_w_last(a_w_last),
 .w_last(w_last),
 .a_b_valid(a_b_valid),
 .b_valid(b_valid),
 .a_b_ready(a_b_ready),
 .b_ready(b_ready),
 .a_b_tag(a_b_tag),
 .b_tag(b_tag),
 .a_b_error(a_b_error),
 .b_error(b_error));
vision_ddr_top #(.FIXED_ACCUM(FIXED_ACCUM),.FIXED_BILINEAR(FIXED_BILINEAR),.CANDIDATE_BASE(CANDIDATE_BASE),.WIDTH(WIDTH),.HEIGHT(HEIGHT),.DST_BASE(DST_BASE)) algorithm(
 .clk(algorithm_clk),.rst_n(algorithm_rst_n),.start_valid(a_start_valid),.start_ready(a_start_ready),.square_size_fp64(SQUARE_SIZE_FP64),
 .frame_valid(a_frame_valid),.frame_ready(a_frame_ready),.frame_base(RAW_BASE),.frame_stride(32'd2560),.frame_capacity(32'd1843200),.frame_status(a_frame_status),
 .frame_release_valid(a_release_valid),.frame_release_ready(a_release_ready),
 .rsp_valid(a_rsp_valid),.rsp_ready(a_rsp_ready),.rsp_status(a_rsp_status),
 .debug_view(debug_view),.debug_phase(debug_phase),
 .result_base(),.result_stride(),.result_width(),.result_height(),
 .result_params(),.result_rms(),.debug_job(),.busy(),
 .rd_valid(a_rd_valid),
 .rd_ready(a_rd_ready),
 .rd_addr(a_rd_addr),
 .rd_len(a_rd_len),
 .rd_tag(a_rd_tag),
 .r_valid(a_r_valid),
 .r_ready(a_r_ready),
 .r_data(a_r_data),
 .r_keep(a_r_keep),
 .r_tag(a_r_tag),
 .r_last(a_r_last),
 .r_error(a_r_error),
 .wr_valid(a_wr_valid),
 .wr_ready(a_wr_ready),
 .wr_addr(a_wr_addr),
 .wr_len(a_wr_len),
 .wr_tag(a_wr_tag),
 .w_valid(a_w_valid),
 .w_ready(a_w_ready),
 .w_data(a_w_data),
 .w_keep(a_w_keep),
 .w_last(a_w_last),
 .b_valid(a_b_valid),
 .b_ready(a_b_ready),
 .b_tag(a_b_tag),
 .b_error(a_b_error));
wire algorithm_owner=!capture_owner&&!display_enable;
hmic_ddr_adapter adapter(.clk(core_clk),.rst_n(core_rst_n),.ddr_ready(ddr_init_done&&algorithm_owner),
 .rd_valid(rd_valid),
 .rd_ready(rd_ready),
 .rd_addr(rd_addr),
 .rd_len(rd_len),
 .rd_tag(rd_tag),
 .r_valid(r_valid),
 .r_ready(r_ready),
 .r_data(r_data),
 .r_keep(r_keep),
 .r_tag(r_tag),
 .r_last(r_last),
 .r_error(r_error),
 .wr_valid(wr_valid),
 .wr_ready(wr_ready),
 .wr_addr(wr_addr),
 .wr_len(wr_len),
 .wr_tag(wr_tag),
 .w_valid(w_valid),
 .w_ready(w_ready),
 .w_data(w_data),
 .w_keep(w_keep),
 .w_last(w_last),
 .b_valid(b_valid),
 .b_ready(b_ready),
 .b_tag(b_tag),
 .b_error(b_error),
 .axi_araddr(alg_araddr),
 .axi_aruser_id(alg_aruser_id),
 .axi_arlen(alg_arlen),
 .axi_aruser_ap(alg_aruser_ap),
 .axi_arvalid(alg_arvalid),
 .axi_awaddr(alg_awaddr),
 .axi_awuser_id(alg_awuser_id),
 .axi_awlen(alg_awlen),
 .axi_awuser_ap(alg_awuser_ap),
 .axi_awvalid(alg_awvalid),
 .axi_wdata(alg_wdata),
 .axi_wstrb(alg_wstrb),
 .axi_awready(axi_awready&&algorithm_owner),.axi_arready(axi_arready&&algorithm_owner),
 .axi_wready(axi_wready&&algorithm_owner),.axi_wusero_last(axi_wusero_last&&algorithm_owner),.axi_wusero_id(axi_wusero_id),
 .axi_rdata(axi_rdata),.axi_rvalid(axi_rvalid&&algorithm_owner),.axi_rlast(axi_rlast),.axi_rid(axi_rid));
wire [27:0] disp_araddr;wire [3:0] disp_arlen,disp_arid;wire disp_arvalid;
wire pv,pre,first,last,display_error;wire [15:0] pixel;
board_display_hmic #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.RAW_BASE(RAW_BASE),.DST_BASE(DST_BASE)) display(
 .clk(core_clk),.rst_n(core_rst_n),.enable(display_enable),.pixel_valid(pv),.pixel_ready(pre),.pixel(pixel),.pixel_first(first),.pixel_last(last),.error(display_error),
 .axi_araddr(disp_araddr),.axi_arlen(disp_arlen),.axi_aruser_id(disp_arid),.axi_arvalid(disp_arvalid),.axi_arready(axi_arready&&display_enable),
 .axi_rdata(axi_rdata),.axi_rvalid(axi_rvalid&&display_enable),.axi_rlast(axi_rlast),.axi_rid(axi_rid));
// Ownership switches only after the old client has completed every transaction.
assign axi_araddr=display_enable?disp_araddr:alg_araddr;
assign axi_arlen=display_enable?disp_arlen:alg_arlen;
assign axi_aruser_id=display_enable?disp_arid:alg_aruser_id;
assign axi_aruser_ap=0;
assign axi_arvalid=display_enable?disp_arvalid:algorithm_owner&&alg_arvalid;
assign axi_awaddr=capture_owner?cap_awaddr:alg_awaddr;
assign axi_awlen=capture_owner?cap_awlen:alg_awlen;
assign axi_awuser_id=capture_owner?cap_awid:alg_awuser_id;
assign axi_awuser_ap=0;
assign axi_awvalid=capture_owner?cap_awvalid:algorithm_owner&&alg_awvalid;
assign axi_wdata=capture_owner?cap_wdata:alg_wdata;
assign axi_wstrb=capture_owner?cap_wstrb:alg_wstrb;
wire vs_o,hs_o,de_o;wire [7:0] video_r,video_g,video_b;wire underflow;
reg [1:0] display_sync /* synthesis PAP_ASYNC_REG=1 */;
sync_vg timing(.clk(pix_clk),.rstn(pixel_rst_n),.vs_out(vs_o),.hs_out(hs_o),.de_out(de_o),.de_re(),.x_act(),.y_act());
hdmi_pixel_bridge #(.FIFO_BITS(11)) bridge(.clk(core_clk),.rst_n(core_rst_n),.in_valid(pv),.in_ready(pre),.in_pixel(pixel),.in_first(first),.in_last(last),
 .pclk(pix_clk),.prst_n(pixel_rst_n),.de(de_o),.vsync(vs_o),.clear_error(!display_sync[1]),.r(video_r),.g(video_g),.b(video_b),.underflow(underflow));
always @(posedge pix_clk or negedge pixel_rst_n)
 if(!pixel_rst_n)display_sync<=0;else display_sync<={display_sync[0],display_enable};
always @(posedge pix_clk)begin
 vs_out<=vs_o;hs_out<=hs_o;de_out<=de_o;
 r_out<=video_r;g_out<=video_g;b_out<=video_b;
end
// Configuration LED remains the original HDMI-configured indicator.
assign hdmi_int_led=init_over_tx;


GTP_INBUFGDS #(
    .IOSTANDARD("DEFAULT"),
    .TERM_DIFF("ON")
) u_gtp (
    .O(clk_125Mhz), // OUTPUT  
    .I(clk_p), // INPUT  
    .IB(clk_n) // INPUT  
);
 ddr3_test u_ddr3_test_h(
             .ref_clk                   (clk_125Mhz            ),
             .resetn                    (rstn_out           ),// input
             .ddr_init_done             (ddr_init_done      ),// output

             .pll_lock                  (pll_lock           ),// output

             .core_clk                  (core_clk),                                  // output

             .phy_pll_lock              (phy_pll_lock),                          // output
             .gpll_lock                 (gpll_lock),                                // output
             .rst_gpll_lock             (rst_gpll_lock),                        // output
             .ddrphy_cpd_lock           (ddrphy_cpd_lock),                    // output
             //.ddr_init_done             (ddr_init_done),                        // output


             .axi_awaddr                (axi_awaddr         ),// input [27:0]
             .axi_awuser_ap             (1'b0               ),// input
             .axi_awuser_id             (axi_awuser_id      ),// input [3:0]
             .axi_awlen                 (axi_awlen          ),// input [3:0]
             .axi_awready               (axi_awready        ),// output
             .axi_awvalid               (axi_awvalid        ),// input
             .axi_wdata                 (axi_wdata          ),
             .axi_wstrb                 (axi_wstrb          ),// input [31:0]
             .axi_wready                (axi_wready         ),// output
             .axi_wusero_id             (axi_wusero_id      ),// output [3:0]
             .axi_wusero_last           (axi_wusero_last    ),// output
             .axi_araddr                (axi_araddr         ),// input [27:0]
             .axi_aruser_ap             (1'b0               ),// input
             .axi_aruser_id             (axi_aruser_id      ),// input [3:0]
             .axi_arlen                 (axi_arlen          ),// input [3:0]
             .axi_arready               (axi_arready        ),// output
             .axi_arvalid               (axi_arvalid        ),// input
             .axi_rdata                 (axi_rdata          ),// output [255:0]
             .axi_rid                   (axi_rid            ),// output [3:0]
             .axi_rlast                 (axi_rlast          ),// output
             .axi_rvalid                (axi_rvalid         ),// output

             .apb_clk                   (1'b0               ),// input
             .apb_rst_n                 (1'b1               ),// input
             .apb_sel                   (1'b0               ),// input
             .apb_enable                (1'b0               ),// input
             .apb_addr                  (8'b0               ),// input [7:0]
             .apb_write                 (1'b0               ),// input
             .apb_ready                 (                   ), // output
             .apb_wdata                 (16'b0              ),// input [15:0]
             .apb_rdata                 (                   ),// output [15:0]
//             .apb_int                   (                   ),// output

             .mem_rst_n                 (mem_rst_n          ),// output
             .mem_ck                    (mem_ck             ),// output
             .mem_ck_n                  (mem_ck_n           ),// output
             .mem_cke                   (mem_cke            ),// output
             .mem_cs_n                  (mem_cs_n           ),// output
             .mem_ras_n                 (mem_ras_n          ),// output
             .mem_cas_n                 (mem_cas_n          ),// output
             .mem_we_n                  (mem_we_n           ),// output
             .mem_odt                   (mem_odt            ),// output
             .mem_a                     (mem_a              ),// output [14:0]
             .mem_ba                    (mem_ba             ),// output [2:0]
             .mem_dqs                   (mem_dqs            ),// inout [3:0]
             .mem_dqs_n                 (mem_dqs_n          ),// inout [3:0]
             .mem_dq                    (mem_dq             ),// inout [31:0]
             .mem_dm                    (mem_dm             ),// output [3:0]
             //debug

  .dbg_gate_start(1'b0),                      // input
  .dbg_cpd_start(1'b0),                        // input
  .dbg_ddrphy_rst_n(1'b1),                  // input
  .dbg_gpll_scan_rst(1'b0),                // input
  .samp_position_dyn_adj(1'b0),        // input
  .init_samp_position_even(32'd0),    // input [31:0]
  .init_samp_position_odd(32'd0),      // input [31:0]
  .wrcal_position_dyn_adj(1'b0),      // input
  .init_wrcal_position(32'd0),            // input [31:0]
  .force_read_clk_ctrl(1'b0),            // input
  .init_slip_step(16'd0),                      // input [15:0]
  .init_read_clk_ctrl(12'd0),              // input [11:0]
  .debug_calib_ctrl(),                  // output [33:0]
  .dbg_slice_status(),                  // output [67:0]
  .dbg_slice_state(),                    // output [87:0]
  .debug_data(),                              // output [275:0]
  .dbg_dll_upd_state(),                // output [1:0]
  .debug_gpll_dps_phase(),          // output [8:0]
  .dbg_rst_dps_state(),                // output [2:0]
  .dbg_tran_err_rst_cnt(),          // output [5:0]
  .dbg_ddrphy_init_fail(),          // output
  .debug_cpd_offset_adj(1'b0),          // input
  .debug_cpd_offset_dir(1'b0),           // input
  .debug_cpd_offset(10'd0),                  // input [9:0]
  .debug_dps_cnt_dir0(),              // output [9:0]
  .debug_dps_cnt_dir1(),              // output [9:0]
  .ck_dly_en(1'b0),                                // input
  .init_ck_dly_step(8'h0),                  // input [7:0]
  .ck_dly_set_bin(),                      // output [7:0]
  .align_error(),                            // output
  .debug_rst_state(),                    // output [3:0]
  .debug_cpd_state()                     // output [3:0]
       );
//心跳信号
     always@(posedge core_clk) begin
        if (!ddr_init_done)
            cnt <= 27'd0;
        else if ( cnt >= TH_1S )
            cnt <= 27'd0;
        else
            cnt <= cnt + 27'd1;
     end

     always @(posedge core_clk)
        begin
        if (!ddr_init_done)
            heart_beat_led <= 1'd1;
        else if ( cnt >= TH_1S )
            heart_beat_led <= ~heart_beat_led;
    end
                 
/////////////////////////////////////////////////////////////////////////////////////

endmodule
