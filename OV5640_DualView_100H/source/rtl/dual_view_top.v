`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Project: OV5640_DualView_100H (FPGA创新设计大赛-紫光同创赛道-畸变矫正赛题)
// Base   : Meyesemi HDMI_DDR3_OV5640_100H demo
// Board  : 盘古100Pro+ (MES2L676-100HP / PG2L100H-FBG676)
// Function:
//   单目OV5640输入(1280x720@30fps RGB565)经DDR3帧缓存后, HDMI(720P30)双画面输出:
//     左半屏(0~639列)   : 摄像头原始彩色画面(原图左半视场)
//     第640列           : 白色分界线
//     右半屏(641~1279列): 同一视场画面的灰度化结果(暂代"畸变矫正后"画面)
//   后续将灰度化模块替换为畸变矫正模块即可, 显示框架不变。
// 
// Revision:
//   1.0 - 基于demo建立工程, 增加输出级双画面显示(line buffer + 灰度化)
//////////////////////////////////////////////////////////////////////////////////
`define UD #1
//cmos1、cmos2二选一，作为视频源输入（摄像头接在哪个座子就选哪个）
//`define CMOS_1      //cmos1作为视频输入；
`define CMOS_2      //cmos2作为视频输入；

module dual_view_top#(
	parameter MEM_ROW_ADDR_WIDTH   = 15         ,
	parameter MEM_COL_ADDR_WIDTH   = 10         ,
	parameter MEM_BADDR_WIDTH      = 3          ,
	parameter MEM_DQ_WIDTH         =  32        ,
	parameter MEM_DQS_WIDTH        = 32/8
)(
	input                                sys_clk              ,//50Mhz
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
/////////////////////////////////////////////////////////////////////////////////////
// ENABLE_DDR
    parameter CTRL_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH;//28
    parameter TH_1S = 27'd33000000;
    parameter H_ACTIVE = 12'd1280;      //行有效像素
    parameter H_SPLIT  = 12'd640;       //左右分屏分界列
/////////////////////////////////////////////////////////////////////////////////////
    reg  [15:0]                 rstn_1ms            ;
    wire                        cmos_scl            ;//cmos i2c clock
    wire                        cmos_sda            ;//cmos i2c data
    wire                        cmos_vsync          ;//cmos vsync
    wire                        cmos_href           ;//cmos hsync refrence,data valid
    wire                        cmos_pclk           ;//cmos pxiel clock
    wire   [7:0]                cmos_data           ;//cmos data
    wire                        cmos_reset          ;
    wire                        initial_en          ;
    wire[15:0]                  cmos1_d_16bit       ;
    wire                        cmos1_href_16bit    ;
    reg [7:0]                   cmos1_d_d0          ;
    reg                         cmos1_href_d0       ;
    reg                         cmos1_vsync_d0      ;
    wire                        cmos1_pclk_16bit    ;
    wire[15:0]                  cmos2_d_16bit       /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos2_href_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
    reg [7:0]                   cmos2_d_d0          /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos2_href_d0       /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos2_vsync_d0      /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos2_pclk_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
    wire[15:0]                  o_rgb565            ;
    wire                        pclk_in_test        ;    
    wire                        vs_in_test          ;
    wire                        de_in_test          ;
    wire[15:0]                  i_rgb565            /*synthesis PAP_MARK_DEBUG="1"*/         ;
    wire                        de_re               ;
//axi bus   
    wire [CTRL_ADDR_WIDTH-1:0]  axi_awaddr                 ;
    wire                        axi_awuser_ap              ;
    wire [3:0]                  axi_awuser_id              ;
    wire [3:0]                  axi_awlen                  ;
    wire                        axi_awready                ;
    wire                        axi_awvalid                ;
    wire [MEM_DQ_WIDTH*8-1:0]   axi_wdata                  ;
    wire [MEM_DQ_WIDTH*8/8-1:0] axi_wstrb                  ;
    wire                        axi_wready                 ;
    wire [3:0]                  axi_wusero_id              ;
    wire                        axi_wusero_last            ;
    wire [CTRL_ADDR_WIDTH-1:0]  axi_araddr                 ;
    wire                        axi_aruser_ap              ;
    wire [3:0]                  axi_aruser_id              ;
    wire [3:0]                  axi_arlen                  ;
    wire                        axi_arready                ;
    wire                        axi_arvalid                ;
    wire [MEM_DQ_WIDTH*8-1:0]   axi_rdata                  ;
    wire                        axi_rvalid                 ;
    wire [3:0]                  axi_rid                    ;
    wire                        axi_rlast                  ;
    reg  [26:0]                 cnt                        ;
    reg  [15:0]                 cnt_1                      ;
    wire                        init_done                  ;
    wire                        vs_o                       ;
    wire                        hs_o                       ;
    wire                        de_o                       ;
/////////////////////////////////////////////////////////////////////////////////////
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
    ms72xx_ctl ms72xx_ctl(
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
    power_on_delay	power_on_delay_inst(
    	.clk_50M                 (sys_clk        ),//input
    	.reset_n                 (1'b1           ),//input	
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
//CMOS 8bit转16bit///////////////////////////////////////////////////////////////////////////////////
//CMOS1
    always@(posedge cmos1_pclk)
        begin
            cmos1_d_d0        <= cmos1_data    ;
            cmos1_href_d0     <= cmos1_href    ;
            cmos1_vsync_d0    <= cmos1_vsync   ;
        end

    cmos_8_16bit cmos1_8_16bit(
    	.pclk           (cmos1_pclk       ),//input
    	.rst_n          (cmos_init_done[0]),//input
    	.pdata_i        (cmos1_d_d0       ),//input[7:0]
    	.de_i           (cmos1_href_d0    ),//input
    	.vs_i           (cmos1_vsync_d0    ),//input
    	
    	.pixel_clk      (cmos1_pclk_16bit ),//output
    	.pdata_o        (cmos1_d_16bit    ),//output[15:0]
    	.de_o           (cmos1_href_16bit ) //output
    );
//CMOS2
    always@(posedge cmos2_pclk)
        begin
            cmos2_d_d0        <= cmos2_data    ;
            cmos2_href_d0     <= cmos2_href    ;
            cmos2_vsync_d0    <= cmos2_vsync   ;
        end

    cmos_8_16bit cmos2_8_16bit(
    	.pclk           (cmos2_pclk       ),//input
    	.rst_n          (cmos_init_done[1]),//input
    	.pdata_i        (cmos2_d_d0       ),//input[7:0]
    	.de_i           (cmos2_href_d0    ),//input
    	.vs_i           (cmos2_vsync_d0    ),//input
    	
    	.pixel_clk      (cmos2_pclk_16bit ),//output
    	.pdata_o        (cmos2_d_16bit    ),//output[15:0]
    	.de_o           (cmos2_href_16bit ) //output
    );
//输入视频源选择//////////////////////////////////////////////////////////////////////////////////////////
`ifdef CMOS_1
assign     pclk_in_test    =    cmos1_pclk_16bit    ;
assign     vs_in_test      =    cmos1_vsync_d0      ;
assign     de_in_test      =    cmos1_href_16bit    ;
assign     i_rgb565        =    {cmos1_d_16bit[4:0],cmos1_d_16bit[10:5],cmos1_d_16bit[15:11]};//{r,g,b}
`elsif CMOS_2
assign     pclk_in_test    =    cmos2_pclk_16bit    ;
assign     vs_in_test      =    cmos2_vsync_d0      ;
assign     de_in_test      =    cmos2_href_16bit    ;
assign     i_rgb565        =    {cmos2_d_16bit[4:0],cmos2_d_16bit[10:5],cmos2_d_16bit[15:11]};//{r,g,b}
`endif
 assign    hdmi_int_led    = |i_rgb565;
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//修改ddr读写模块v1
    fram_buf fram_buf(
        .ddr_clk        (  core_clk             ),//input                         ddr_clk,
        .ddr_rstn       (  ddr_init_done        ),//input                         ddr_rstn,
        //data_in                                  
        .vin_clk        (  pclk_in_test         ),//input                         vin_clk,
        .wr_fsync       (  vs_in_test           ),//input                         wr_fsync,
        .wr_en          (  de_in_test           ),//input                         wr_en,
        .wr_data        (  i_rgb565             ),//input  [15 : 0]  wr_data,
        //data_out
        .vout_clk       (  pix_clk              ),//input                         vout_clk,
        .rd_fsync       (  vs_o               ),//input                         rd_fsync,
        .rd_en          (  de_re                ),//input                         rd_en,
        .vout_de        (  de_o               ),//output                        vout_de,
        .vout_data      (  o_rgb565             ),//output [PIX_WIDTH- 1'b1 : 0]  vout_data,
        .init_done      (  init_done            ),//output reg                    init_done,
        //axi bus
        .axi_awaddr     (  axi_awaddr           ),// output[27:0]
        .axi_awid       (  axi_awuser_id        ),// output[3:0]
        .axi_awlen      (  axi_awlen            ),// output[3:0]
        .axi_awsize     (                       ),// output[2:0]
        .axi_awburst    (                       ),// output[1:0]
        .axi_awready    (  axi_awready          ),// input
        .axi_awvalid    (  axi_awvalid          ),// output               
        .axi_wdata      (  axi_wdata            ),// output[255:0]
        .axi_wstrb      (  axi_wstrb            ),// output[31:0]
        .axi_wlast      (  axi_wusero_last      ),// input
        .axi_wvalid     (                       ),// output
        .axi_wready     (  axi_wready           ),// input
        .axi_bid        (  4'd0                 ),// input[3:0]
        .axi_araddr     (  axi_araddr           ),// output[27:0]
        .axi_arid       (  axi_aruser_id        ),// output[3:0]
        .axi_arlen      (  axi_arlen            ),// output[3:0]
        .axi_arsize     (                       ),// output[2:0]
        .axi_arburst    (                       ),// output[1:0]
        .axi_arvalid    (  axi_arvalid          ),// output
        .axi_arready    (  axi_arready          ),// input
        .axi_rready     (                       ),// output
        .axi_rdata      (  axi_rdata            ),// input[255:0]
        .axi_rvalid     (  axi_rvalid           ),// input
        .axi_rlast      (  axi_rlast            ),// input
        .axi_rid        (  axi_rid              ) // input[3:0]         
    );

/////////////////////////////////////////////////////////////////////////////////////
//双画面显示级（畸变矫正赛题第一阶段）
//屏幕布局(1280x720):
//  列 0~639        : 摄像头原始彩色画面(原图左半视场, 直通显示)
//  列 640          : 白色分界线
//  列 641~1279     : 同一视场的灰度化画面(暂代"畸变矫正后")
//原理: DDR读出的像素流o_rgb565按行扫描, 左半屏像素直通显示并写入行缓存;
//      右半屏期间从行缓存回读本行左半像素, 实时灰度化后显示。
//      行缓存1024x16bit(实际用640), 推断为1块BRAM。
    reg  [10:0]          pixel_x      ;        //当前输出像素列号 0~1279
    reg  [15:0]          lb_rdata     ;        //行缓存回读像素

    always @(posedge pix_clk) begin
        if (!de_o)
            pixel_x <= 11'd0;
        else
            pixel_x <= pixel_x + 11'd1;
    end

    //行缓存写: 左半屏有效像素, 写地址=列号
    wire        lb_we    = de_o && (pixel_x < H_SPLIT);
    wire [9:0]  lb_waddr = pixel_x[9:0];
    //行缓存读: BRAM同步读有1拍延迟, 读地址提前一拍送出
    //右半屏第p列(641~1279...按像素对齐640~1279)需要第(p-640)个缓存像素,
    //在pixel_x=p-1拍送地址(p-1)-639
    wire [9:0]  lb_raddr = ((pixel_x >= 11'd639) && (pixel_x <= 11'd1278))
                            ? (pixel_x - 11'd639) : 10'd0;

    reg [15:0] line_buffer [0:1023];
    always @(posedge pix_clk) begin
        if (lb_we)
            line_buffer[lb_waddr] <= o_rgb565;
    end

    always @(posedge pix_clk) begin
        lb_rdata <= line_buffer[lb_raddr];
    end

    //灰度化: Y = (77R + 150G + 29B)/256, RGB565各分量高位复制扩展到8bit
    //此处灰度化即后续畸变矫正模块的占位, 替换时保持lb_rdata输入/pix接口即可
    wire [7:0]  gray_r8  = {lb_rdata[15:11], lb_rdata[15:13]};  //R5->R8
    wire [7:0]  gray_g8  = {lb_rdata[10:5],  lb_rdata[10:9] };  //G6->G8
    wire [7:0]  gray_b8  = {lb_rdata[4:0],   lb_rdata[4:2]  };  //B5->B8
    wire [15:0] gray_sum = gray_r8 * 8'd77 + gray_g8 * 8'd150 + gray_b8 * 8'd29;
    wire [7:0]  gray_y8  = gray_sum[15:8];
    wire [15:0] gray565  = {gray_y8[7:3], gray_y8[7:2], gray_y8[7:3]};

    //输出像素选择: 左半屏原色 / 分界线白色 / 右半屏灰度
    wire [15:0] pix_out = (pixel_x == H_SPLIT)     ? 16'hFFFF :
                          (pixel_x <  H_SPLIT)     ? o_rgb565  : gray565;

     always@(posedge pix_clk) begin
        r_out<={pix_out[15:11],3'b0   };
        g_out<={pix_out[10:5],2'b0    };
        b_out<={pix_out[4:0],3'b0     }; 
        vs_out<=vs_o;
        hs_out<=hs_o;
        de_out<=de_o;
     end
/////////////////////////////////////////////////////////////////////////////////////
//产生visa时序 
     sync_vg sync_vg(                            
        .clk            (  pix_clk              ),//input                   clk,                                 
        .rstn           (  ddr_init_done            ),//input                   rstn,                            
        .vs_out         (  vs_o                 ),//output reg              vs_out,                                                                                                                                      
        .hs_out         (  hs_o                 ),//output reg              hs_out,            
        .de_out         (  de_re                     ),//output reg              de_out, 
        .de_re          (                  )    
    );  
////////////////////////////////////////////////////////////////////////////////////////////
//ddr    
/*        ddr3_100h ddr3_100h (
             .ref_clk                   (sys_clk            ),
             .resetn                    (rstn_out           ),// input
             .ddr_init_done             (ddr_init_done      ),// output
             .ddrphy_clkin              (core_clk           ),// output
             .pll_lock                  (pll_lock           ),// output

             .axi_awaddr                (axi_awaddr         ),// input [27:0]
             .axi_awuser_ap             (1'b0               ),// input
             .axi_awuser_id             (axi_awuser_id      ),// input [3:0]
             .axi_awlen                 (axi_awlen          ),// input [3:0]
             .axi_awready               (axi_awready        ),// output
             .axi_awvalid               (axi_awvalid        ),// input
             .axi_wdata                 (axi_wdata          ),
             .axi_wstrb                 (axi_wstrb          ),// input [31:0]
             .axi_wready                (axi_wready         ),// output
             .axi_wusero_id             (                   ),// output [3:0]
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
             .apb_int                   (                   ),// output

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
             .debug_data                (                   ),// output [135:0]
             .debug_slice_state         (                   ),// output [51:0]
             .debug_calib_ctrl          (                   ),// output [21:0]
             .ck_dly_set_bin            (                   ),// output [7:0]
             .force_ck_dly_en           (1'b0               ),// input
             .force_ck_dly_set_bin      (8'h05              ),// input [7:0]
             .dll_step                  (                   ),// output [7:0]
             .dll_lock                  (                   ),// output
             .init_read_clk_ctrl        (2'b0               ),// input [1:0]
             .init_slip_step            (4'b0               ),// input [3:0]
             .force_read_clk_ctrl       (1'b0               ),// input
             .ddrphy_gate_update_en     (1'b0               ),// input
             .update_com_val_err_flag   (                   ),// output [3:0]
             .rd_fake_stop              (1'b0               ) // input
       );*/
wire clk_125Mhz ;

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
             .axi_wusero_id             (                   ),// output [3:0]
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
