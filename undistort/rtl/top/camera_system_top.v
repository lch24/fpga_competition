`timescale 1ns/1ps
// Command-driven first complete system. Keeps original Demo camera I2C, PLL,
// sync_vg, HDMI transmitter, pins and HMIC IP outside this core-clock subsystem.
// opcode: 1 BUILD_MAP; 2 SNAPSHOT; 3 CORRECT_NEXT_RAW; 4 DISPLAY_NEXT_OUTPUT.
// No new job before the previous response is accepted. Buffer ownership protects
// camera, detector, correction and display. External client is the detector's DDR
// port; raw_pin/raw_release explicitly reserve/release its source frame.
module camera_system_top #(
 parameter RAW_BASE=32'h00000000,OUTPUT_BASE=32'h01000000,
 parameter SLOT_BYTES=32'h00400000,RAW_SLOTS=3,OUTPUT_SLOTS=2,
 parameter MAP_X_BASE=32'h02000000,MAP_Y_BASE=32'h02800000,MAP_CAPACITY=32'h00800000
)(
 input wire clk,rst_n,ddr_ready,
 input wire camera_pclk,camera_rst_n,camera_vsync,camera_href,input wire [7:0] camera_data,
 input wire hdmi_pclk,hdmi_rst_n,hdmi_de,hdmi_vsync,input wire clear_hdmi_error,
 output wire [7:0] hdmi_r,hdmi_g,hdmi_b,output wire hdmi_underflow,
 input wire cmd_valid,output wire cmd_ready,input wire [2:0] cmd_opcode,
 input wire [31:0] cmd_job_id,cmd_calib_id,input wire cmd_camera_valid,
 input wire [287:0] cmd_params,input wire [15:0] cmd_width,cmd_height,
 input wire cmd_border_replicate,
 output wire rsp_valid,input wire rsp_ready,output reg [7:0] rsp_status,
 output reg [31:0] rsp_job_id,output wire busy,map_valid,
 output wire [31:0] map_calib_id,output wire ownership_error,
 input wire raw_pin_ready,output wire raw_pin_valid,output wire [3:0] raw_pin_slot,
 output wire [31:0] raw_pin_base,output wire [15:0] raw_pin_width,raw_pin_height,
 input wire raw_release_valid,output wire raw_release_ready,input wire [3:0] raw_release_slot,
 input wire ext_rd_valid,output wire ext_rd_ready,input wire [31:0] ext_rd_addr,ext_rd_len,
 input wire [15:0] ext_rd_tag,output wire ext_r_valid,input wire ext_r_ready,
 output wire [31:0] ext_r_data,output wire [3:0] ext_r_keep,output wire [15:0] ext_r_tag,
 output wire ext_r_last,ext_r_error,
 input wire ext_wr_valid,output wire ext_wr_ready,input wire [31:0] ext_wr_addr,ext_wr_len,
 input wire [15:0] ext_wr_tag,input wire ext_w_valid,output wire ext_w_ready,
 input wire [31:0] ext_w_data,input wire [3:0] ext_w_keep,input wire ext_w_last,
 output wire ext_b_valid,input wire ext_b_ready,output wire [15:0] ext_b_tag,output wire ext_b_error,
 output wire [27:0] axi_araddr,axi_awaddr,
 output wire [3:0] axi_aruser_id,axi_arlen,axi_awuser_id,axi_awlen,
 output wire axi_aruser_ap,axi_arvalid,axi_awuser_ap,axi_awvalid,
 input wire axi_arready,axi_awready,
 input wire [255:0] axi_rdata,input wire axi_rvalid,axi_rlast,input wire [3:0] axi_rid,
 output wire [255:0] axi_wdata,output wire [31:0] axi_wstrb,
 input wire axi_wready,axi_wusero_last,input wire [3:0] axi_wusero_id
);
 localparam IDLE=0,BUILD_START=1,BUILD_WAIT=2,CAPTURE_START=3,CAPTURE_WAIT=4,
 CORRECT_START=5,CORRECT_WAIT=6,DISPLAY_START=7,DISPLAY_WAIT=8,RESPONSE=9;
 reg [3:0] state,raw_slot,out_slot;
 reg [31:0] calib;reg [287:0] params;reg [15:0] width,height;reg camera_valid,border;
 reg [15:0] raw_width[0:RAW_SLOTS-1],raw_height[0:RAW_SLOTS-1];
 reg [15:0] out_width[0:OUTPUT_SLOTS-1],out_height[0:OUTPUT_SLOTS-1];
 integer reset_slot;
 wire [31:0] stride={15'd0,width,1'b0},map_stride={14'd0,width,2'b00};
 assign cmd_ready=state==IDLE;assign rsp_valid=state==RESPONSE;assign busy=state!=IDLE;
 assign raw_release_ready=state==IDLE&&!cmd_valid;
 localparam [63:0] RAW_END={32'd0,RAW_BASE}+RAW_SLOTS*64'd1*SLOT_BYTES;
 localparam [63:0] OUT_END={32'd0,OUTPUT_BASE}+OUTPUT_SLOTS*64'd1*SLOT_BYTES;
 localparam [63:0] MX_END={32'd0,MAP_X_BASE}+MAP_CAPACITY;
 localparam [63:0] MY_END={32'd0,MAP_Y_BASE}+MAP_CAPACITY;
 wire layout_ok=RAW_SLOTS>0&&RAW_SLOTS<=16&&OUTPUT_SLOTS>0&&OUTPUT_SLOTS<=16&&
  SLOT_BYTES>0&&MAP_CAPACITY>0&&RAW_END<=64'h40000000&&OUT_END<=64'h40000000&&
  MX_END<=64'h40000000&&MY_END<=64'h40000000&&
  (RAW_END<=OUTPUT_BASE||OUT_END<=RAW_BASE)&&
  (RAW_END<=MAP_X_BASE||MX_END<=RAW_BASE)&&(RAW_END<=MAP_Y_BASE||MY_END<=RAW_BASE)&&
  (OUT_END<=MAP_X_BASE||MX_END<=OUTPUT_BASE)&&(OUT_END<=MAP_Y_BASE||MY_END<=OUTPUT_BASE)&&
  (MX_END<=MAP_Y_BASE||MY_END<=MAP_X_BASE);
 wire image_fits=({48'd0,cmd_width}*cmd_height*2)<=SLOT_BYTES;
 wire map_fits=({48'd0,cmd_width}*cmd_height*4)<=MAP_CAPACITY;
 wire raw_pv,raw_cv,out_pv,out_cv,raw_error,out_error;
 wire [3:0] raw_ps,raw_cs,out_ps,out_cs;
 wire [31:0] raw_pb,raw_cb,out_pb,out_cb;
 wire cap_cmd_ready,cap_rsp_valid;wire [7:0] cap_status;
 wire core_cmd_ready,core_rsp_valid;wire [7:0] core_status;
 wire disp_cmd_ready,disp_rsp_valid;wire [7:0] disp_status;
 wire cap_start=state==CAPTURE_START&&raw_pv;
 wire correct_start=state==CORRECT_START&&raw_cv&&out_pv;
 wire disp_start=state==DISPLAY_START&&out_cv;
 wire raw_consume=(correct_start&&core_cmd_ready)||(state==IDLE&&!cmd_valid&&raw_pin_ready);
 assign raw_pin_valid=state==IDLE&&!cmd_valid&&raw_cv;
 assign raw_pin_slot=raw_cs;assign raw_pin_base=raw_cb;
 assign raw_pin_width=raw_width[raw_cs];assign raw_pin_height=raw_height[raw_cs];
 assign ownership_error=raw_error||out_error;
 frame_manager #(.SLOTS(RAW_SLOTS),.BASE(RAW_BASE),.SLOT_BYTES(SLOT_BYTES)) raw_pool(
  .clk(clk),.rst_n(rst_n),.producer_valid(raw_pv),.producer_ready(cap_start&&cap_cmd_ready),
  .producer_slot(raw_ps),.producer_base(raw_pb),
  .publish_valid(state==CAPTURE_WAIT&&cap_rsp_valid),.publish_slot(raw_slot),.publish_success(cap_status==0),
  .consumer_valid(raw_cv),.consumer_ready(raw_consume),.consumer_slot(raw_cs),.consumer_base(raw_cb),
  .release_valid((state==CORRECT_WAIT&&core_rsp_valid)||(raw_release_ready&&raw_release_valid)),
  .release_slot(state==IDLE?raw_release_slot:raw_slot),.protocol_error(raw_error),.debug_owners());
 frame_manager #(.SLOTS(OUTPUT_SLOTS),.BASE(OUTPUT_BASE),.SLOT_BYTES(SLOT_BYTES)) out_pool(
  .clk(clk),.rst_n(rst_n),.producer_valid(out_pv),.producer_ready(correct_start&&core_cmd_ready),
  .producer_slot(out_ps),.producer_base(out_pb),
  .publish_valid(state==CORRECT_WAIT&&core_rsp_valid),.publish_slot(out_slot),.publish_success(core_status==0),
  .consumer_valid(out_cv),.consumer_ready(disp_start&&disp_cmd_ready),.consumer_slot(out_cs),.consumer_base(out_cb),
  .release_valid(state==DISPLAY_WAIT&&disp_rsp_valid),.release_slot(out_slot),
  .protocol_error(out_error),.debug_owners());
 wire cam_pixel_valid;wire [15:0] cam_pixel;
 camera_byte_unpack unpack(.pclk(camera_pclk),.rst_n(camera_rst_n),.vsync(camera_vsync),.href(camera_href),
  .data(camera_data),.pixel_valid(cam_pixel_valid),.pixel(cam_pixel));

 wire cap_rd_valid;
 wire cap_rd_ready;
 wire [31:0] cap_rd_addr;
 wire [31:0] cap_rd_len;
 wire [15:0] cap_rd_tag;
 wire cap_r_valid;
 wire cap_r_ready;
 wire [31:0] cap_r_data;
 wire [3:0] cap_r_keep;
 wire [15:0] cap_r_tag;
 wire cap_r_last;
 wire cap_r_error;
 wire cap_wr_valid;
 wire cap_wr_ready;
 wire [31:0] cap_wr_addr;
 wire [31:0] cap_wr_len;
 wire [15:0] cap_wr_tag;
 wire cap_w_valid;
 wire cap_w_ready;
 wire [31:0] cap_w_data;
 wire [3:0] cap_w_keep;
 wire cap_w_last;
 wire cap_b_valid;
 wire cap_b_ready;
 wire [15:0] cap_b_tag;
 wire cap_b_error;
 wire core_rd_valid;
 wire core_rd_ready;
 wire [31:0] core_rd_addr;
 wire [31:0] core_rd_len;
 wire [15:0] core_rd_tag;
 wire core_r_valid;
 wire core_r_ready;
 wire [31:0] core_r_data;
 wire [3:0] core_r_keep;
 wire [15:0] core_r_tag;
 wire core_r_last;
 wire core_r_error;
 wire core_wr_valid;
 wire core_wr_ready;
 wire [31:0] core_wr_addr;
 wire [31:0] core_wr_len;
 wire [15:0] core_wr_tag;
 wire core_w_valid;
 wire core_w_ready;
 wire [31:0] core_w_data;
 wire [3:0] core_w_keep;
 wire core_w_last;
 wire core_b_valid;
 wire core_b_ready;
 wire [15:0] core_b_tag;
 wire core_b_error;
 wire disp_rd_valid;
 wire disp_rd_ready;
 wire [31:0] disp_rd_addr;
 wire [31:0] disp_rd_len;
 wire [15:0] disp_rd_tag;
 wire disp_r_valid;
 wire disp_r_ready;
 wire [31:0] disp_r_data;
 wire [3:0] disp_r_keep;
 wire [15:0] disp_r_tag;
 wire disp_r_last;
 wire disp_r_error;
 wire disp_wr_valid;
 wire disp_wr_ready;
 wire [31:0] disp_wr_addr;
 wire [31:0] disp_wr_len;
 wire [15:0] disp_wr_tag;
 wire disp_w_valid;
 wire disp_w_ready;
 wire [31:0] disp_w_data;
 wire [3:0] disp_w_keep;
 wire disp_w_last;
 wire disp_b_valid;
 wire disp_b_ready;
 wire [15:0] disp_b_tag;
 wire disp_b_error;
 wire mem_rd_valid;
 wire mem_rd_ready;
 wire [31:0] mem_rd_addr;
 wire [31:0] mem_rd_len;
 wire [15:0] mem_rd_tag;
 wire mem_r_valid;
 wire mem_r_ready;
 wire [31:0] mem_r_data;
 wire [3:0] mem_r_keep;
 wire [15:0] mem_r_tag;
 wire mem_r_last;
 wire mem_r_error;
 wire mem_wr_valid;
 wire mem_wr_ready;
 wire [31:0] mem_wr_addr;
 wire [31:0] mem_wr_len;
 wire [15:0] mem_wr_tag;
 wire mem_w_valid;
 wire mem_w_ready;
 wire [31:0] mem_w_data;
 wire [3:0] mem_w_keep;
 wire mem_w_last;
 wire mem_b_valid;
 wire mem_b_ready;
 wire [15:0] mem_b_tag;
 wire mem_b_error;
capture_dma capture(
  .clk(clk),
  .rst_n(rst_n),
  .pclk(camera_pclk),
  .prst_n(camera_rst_n),
  .camera_vsync(camera_vsync),
  .camera_valid(cam_pixel_valid),
  .camera_pixel(cam_pixel),
  .cmd_valid(cap_start),
  .cmd_ready(cap_cmd_ready),
  .cmd_base(raw_pb),
  .cmd_stride(stride),
  .cmd_width(width),
  .cmd_height(height),
  .rsp_valid(cap_rsp_valid),
  .rsp_ready(state==CAPTURE_WAIT),
  .rsp_status(cap_status),
  .wr_valid(cap_wr_valid),
  .wr_ready(cap_wr_ready),
  .wr_addr(cap_wr_addr),
  .wr_len(cap_wr_len),
  .wr_tag(cap_wr_tag),
  .w_valid(cap_w_valid),
  .w_ready(cap_w_ready),
  .w_data(cap_w_data),
  .w_keep(cap_w_keep),
  .w_last(cap_w_last),
  .b_valid(cap_b_valid),
  .b_ready(cap_b_ready),
  .b_tag(cap_b_tag),
  .b_error(cap_b_error)
 );
 assign cap_rd_valid=0;assign cap_rd_addr=0;assign cap_rd_len=0;assign cap_rd_tag=0;assign cap_r_ready=0;
undistort_top correction(
  .clk(clk),
  .rst_n(rst_n),
  .cmd_valid(state==BUILD_START||correct_start),
  .cmd_ready(core_cmd_ready),
  .cmd_opcode(state==BUILD_START?2'd1:2'd2),
  .cmd_job_id(rsp_job_id),
  .cmd_calib_id(calib),
  .cmd_camera_valid(camera_valid),
  .cmd_params(params),
  .cmd_width(width),
  .cmd_height(height),
  .cmd_src_base(raw_cb),
  .cmd_dst_base(out_pb),
  .cmd_map_x_base(MAP_X_BASE),
  .cmd_map_y_base(MAP_Y_BASE),
  .cmd_src_stride(stride),
  .cmd_dst_stride(stride),
  .cmd_map_stride(map_stride),
  .cmd_src_capacity(SLOT_BYTES),
  .cmd_dst_capacity(SLOT_BYTES),
  .cmd_map_capacity(MAP_CAPACITY),
  .cmd_border_replicate(border),
  .rsp_valid(core_rsp_valid),
  .rsp_ready(state==BUILD_WAIT||state==CORRECT_WAIT),
  .rsp_job_id(),
  .rsp_status(core_status),
  .busy(),
  .map_valid(map_valid),
  .map_calib_id(map_calib_id),
  .rd_valid(core_rd_valid),
  .rd_ready(core_rd_ready),
  .rd_addr(core_rd_addr),
  .rd_len(core_rd_len),
  .rd_tag(core_rd_tag),
  .r_valid(core_r_valid),
  .r_ready(core_r_ready),
  .r_data(core_r_data),
  .r_keep(core_r_keep),
  .r_tag(core_r_tag),
  .r_last(core_r_last),
  .r_error(core_r_error),
  .wr_valid(core_wr_valid),
  .wr_ready(core_wr_ready),
  .wr_addr(core_wr_addr),
  .wr_len(core_wr_len),
  .wr_tag(core_wr_tag),
  .w_valid(core_w_valid),
  .w_ready(core_w_ready),
  .w_data(core_w_data),
  .w_keep(core_w_keep),
  .w_last(core_w_last),
  .b_valid(core_b_valid),
  .b_ready(core_b_ready),
  .b_tag(core_b_tag),
  .b_error(core_b_error)
 );
 wire disp_pixel_valid,disp_pixel_ready,disp_pixel_first,disp_pixel_last;wire [15:0] disp_pixel;
display_dma display(
  .clk(clk),
  .rst_n(rst_n),
  .cmd_valid(disp_start),
  .cmd_ready(disp_cmd_ready),
  .cmd_base(out_cb),
  .cmd_stride(stride),
  .cmd_width(width),
  .cmd_height(height),
  .pixel_valid(disp_pixel_valid),
  .pixel_ready(disp_pixel_ready),
  .pixel(disp_pixel),
  .pixel_last(disp_pixel_last),
  .pixel_first(disp_pixel_first),
  .rsp_valid(disp_rsp_valid),
  .rsp_ready(state==DISPLAY_WAIT),
  .rsp_status(disp_status),
  .rd_valid(disp_rd_valid),
  .rd_ready(disp_rd_ready),
  .rd_addr(disp_rd_addr),
  .rd_len(disp_rd_len),
  .rd_tag(disp_rd_tag),
  .r_valid(disp_r_valid),
  .r_ready(disp_r_ready),
  .r_data(disp_r_data),
  .r_keep(disp_r_keep),
  .r_tag(disp_r_tag),
  .r_last(disp_r_last),
  .r_error(disp_r_error)
 );
 assign disp_wr_valid=0;assign disp_wr_addr=0;assign disp_wr_len=0;assign disp_wr_tag=0;
 assign disp_w_valid=0;assign disp_w_data=0;assign disp_w_keep=0;assign disp_w_last=0;assign disp_b_ready=0;
 hdmi_pixel_bridge bridge(.clk(clk),.rst_n(rst_n),.in_valid(disp_pixel_valid),.in_ready(disp_pixel_ready),
  .in_pixel(disp_pixel),.in_first(disp_pixel_first),.in_last(disp_pixel_last),
  .pclk(hdmi_pclk),.prst_n(hdmi_rst_n),.de(hdmi_de),.vsync(hdmi_vsync),
  .clear_error(clear_hdmi_error),.r(hdmi_r),.g(hdmi_g),.b(hdmi_b),.underflow(hdmi_underflow));
 wire [3:0] c_rd_valid;
 assign c_rd_valid={ext_rd_valid,disp_rd_valid,core_rd_valid,cap_rd_valid};
 wire [3:0] c_rd_ready;
 assign {ext_rd_ready,disp_rd_ready,core_rd_ready,cap_rd_ready}=c_rd_ready;
 wire [127:0] c_rd_addr;
 assign c_rd_addr={ext_rd_addr,disp_rd_addr,core_rd_addr,cap_rd_addr};
 wire [127:0] c_rd_len;
 assign c_rd_len={ext_rd_len,disp_rd_len,core_rd_len,cap_rd_len};
 wire [63:0] c_rd_tag;
 assign c_rd_tag={ext_rd_tag,disp_rd_tag,core_rd_tag,cap_rd_tag};
 wire [3:0] c_r_valid;
 assign {ext_r_valid,disp_r_valid,core_r_valid,cap_r_valid}=c_r_valid;
 wire [3:0] c_r_ready;
 assign c_r_ready={ext_r_ready,disp_r_ready,core_r_ready,cap_r_ready};
 wire [31:0] shared_r_data;
 assign ext_r_data=shared_r_data;
 assign disp_r_data=shared_r_data;
 assign core_r_data=shared_r_data;
 assign cap_r_data=shared_r_data;
 wire [3:0] shared_r_keep;
 assign ext_r_keep=shared_r_keep;
 assign disp_r_keep=shared_r_keep;
 assign core_r_keep=shared_r_keep;
 assign cap_r_keep=shared_r_keep;
 wire [15:0] shared_r_tag;
 assign ext_r_tag=shared_r_tag;
 assign disp_r_tag=shared_r_tag;
 assign core_r_tag=shared_r_tag;
 assign cap_r_tag=shared_r_tag;
 wire shared_r_last;
 assign ext_r_last=shared_r_last;
 assign disp_r_last=shared_r_last;
 assign core_r_last=shared_r_last;
 assign cap_r_last=shared_r_last;
 wire shared_r_error;
 assign ext_r_error=shared_r_error;
 assign disp_r_error=shared_r_error;
 assign core_r_error=shared_r_error;
 assign cap_r_error=shared_r_error;
 wire [3:0] c_wr_valid;
 assign c_wr_valid={ext_wr_valid,disp_wr_valid,core_wr_valid,cap_wr_valid};
 wire [3:0] c_wr_ready;
 assign {ext_wr_ready,disp_wr_ready,core_wr_ready,cap_wr_ready}=c_wr_ready;
 wire [127:0] c_wr_addr;
 assign c_wr_addr={ext_wr_addr,disp_wr_addr,core_wr_addr,cap_wr_addr};
 wire [127:0] c_wr_len;
 assign c_wr_len={ext_wr_len,disp_wr_len,core_wr_len,cap_wr_len};
 wire [63:0] c_wr_tag;
 assign c_wr_tag={ext_wr_tag,disp_wr_tag,core_wr_tag,cap_wr_tag};
 wire [3:0] c_w_valid;
 assign c_w_valid={ext_w_valid,disp_w_valid,core_w_valid,cap_w_valid};
 wire [3:0] c_w_ready;
 assign {ext_w_ready,disp_w_ready,core_w_ready,cap_w_ready}=c_w_ready;
 wire [127:0] c_w_data;
 assign c_w_data={ext_w_data,disp_w_data,core_w_data,cap_w_data};
 wire [15:0] c_w_keep;
 assign c_w_keep={ext_w_keep,disp_w_keep,core_w_keep,cap_w_keep};
 wire [3:0] c_w_last;
 assign c_w_last={ext_w_last,disp_w_last,core_w_last,cap_w_last};
 wire [3:0] c_b_valid;
 assign {ext_b_valid,disp_b_valid,core_b_valid,cap_b_valid}=c_b_valid;
 wire [3:0] c_b_ready;
 assign c_b_ready={ext_b_ready,disp_b_ready,core_b_ready,cap_b_ready};
 wire [15:0] shared_b_tag;
 assign ext_b_tag=shared_b_tag;
 assign disp_b_tag=shared_b_tag;
 assign core_b_tag=shared_b_tag;
 assign cap_b_tag=shared_b_tag;
 wire shared_b_error;
 assign ext_b_error=shared_b_error;
 assign disp_b_error=shared_b_error;
 assign core_b_error=shared_b_error;
 assign cap_b_error=shared_b_error;
ddr_service arbiter(
  .clk(clk),
  .rst_n(rst_n),
  .c_rd_valid(c_rd_valid),
  .c_rd_ready(c_rd_ready),
  .c_rd_addr(c_rd_addr),
  .c_rd_len(c_rd_len),
  .c_rd_tag(c_rd_tag),
  .c_r_valid(c_r_valid),
  .c_r_ready(c_r_ready),
  .c_r_data(shared_r_data),
  .c_r_keep(shared_r_keep),
  .c_r_tag(shared_r_tag),
  .c_r_last(shared_r_last),
  .c_r_error(shared_r_error),
  .c_wr_valid(c_wr_valid),
  .c_wr_ready(c_wr_ready),
  .c_wr_addr(c_wr_addr),
  .c_wr_len(c_wr_len),
  .c_wr_tag(c_wr_tag),
  .c_w_valid(c_w_valid),
  .c_w_ready(c_w_ready),
  .c_w_data(c_w_data),
  .c_w_keep(c_w_keep),
  .c_w_last(c_w_last),
  .c_b_valid(c_b_valid),
  .c_b_ready(c_b_ready),
  .c_b_tag(shared_b_tag),
  .c_b_error(shared_b_error),
  .rd_valid(mem_rd_valid),
  .rd_ready(mem_rd_ready),
  .rd_addr(mem_rd_addr),
  .rd_len(mem_rd_len),
  .rd_tag(mem_rd_tag),
  .r_valid(mem_r_valid),
  .r_ready(mem_r_ready),
  .r_data(mem_r_data),
  .r_keep(mem_r_keep),
  .r_tag(mem_r_tag),
  .r_last(mem_r_last),
  .r_error(mem_r_error),
  .wr_valid(mem_wr_valid),
  .wr_ready(mem_wr_ready),
  .wr_addr(mem_wr_addr),
  .wr_len(mem_wr_len),
  .wr_tag(mem_wr_tag),
  .w_valid(mem_w_valid),
  .w_ready(mem_w_ready),
  .w_data(mem_w_data),
  .w_keep(mem_w_keep),
  .w_last(mem_w_last),
  .b_valid(mem_b_valid),
  .b_ready(mem_b_ready),
  .b_tag(mem_b_tag),
  .b_error(mem_b_error)
 );
ddr_port_adapter adapter(
  .clk(clk),
  .rst_n(rst_n),
  .ddr_ready(ddr_ready),
  .rd_valid(mem_rd_valid),
  .rd_ready(mem_rd_ready),
  .rd_addr(mem_rd_addr),
  .rd_len(mem_rd_len),
  .rd_tag(mem_rd_tag),
  .r_valid(mem_r_valid),
  .r_ready(mem_r_ready),
  .r_data(mem_r_data),
  .r_keep(mem_r_keep),
  .r_tag(mem_r_tag),
  .r_last(mem_r_last),
  .r_error(mem_r_error),
  .wr_valid(mem_wr_valid),
  .wr_ready(mem_wr_ready),
  .wr_addr(mem_wr_addr),
  .wr_len(mem_wr_len),
  .wr_tag(mem_wr_tag),
  .w_valid(mem_w_valid),
  .w_ready(mem_w_ready),
  .w_data(mem_w_data),
  .w_keep(mem_w_keep),
  .w_last(mem_w_last),
  .b_valid(mem_b_valid),
  .b_ready(mem_b_ready),
  .b_tag(mem_b_tag),
  .b_error(mem_b_error),
  .axi_araddr(axi_araddr),
  .axi_aruser_id(axi_aruser_id),
  .axi_arlen(axi_arlen),
  .axi_aruser_ap(axi_aruser_ap),
  .axi_arvalid(axi_arvalid),
  .axi_arready(axi_arready),
  .axi_rdata(axi_rdata),
  .axi_rvalid(axi_rvalid),
  .axi_rlast(axi_rlast),
  .axi_rid(axi_rid),
  .axi_awaddr(axi_awaddr),
  .axi_awuser_id(axi_awuser_id),
  .axi_awlen(axi_awlen),
  .axi_awuser_ap(axi_awuser_ap),
  .axi_awvalid(axi_awvalid),
  .axi_awready(axi_awready),
  .axi_wdata(axi_wdata),
  .axi_wstrb(axi_wstrb),
  .axi_wready(axi_wready),
  .axi_wusero_last(axi_wusero_last),
  .axi_wusero_id(axi_wusero_id)
 );

 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;raw_slot<=0;out_slot<=0;calib<=0;params<=0;width<=0;height<=0;camera_valid<=0;border<=0;rsp_status<=0;rsp_job_id<=0;
   for(reset_slot=0;reset_slot<RAW_SLOTS;reset_slot=reset_slot+1)begin raw_width[reset_slot]<=0;raw_height[reset_slot]<=0;end
   for(reset_slot=0;reset_slot<OUTPUT_SLOTS;reset_slot=reset_slot+1)begin out_width[reset_slot]<=0;out_height[reset_slot]<=0;end
  end
  else case(state)
   IDLE:if(cmd_valid)begin
    rsp_job_id<=cmd_job_id;calib<=cmd_calib_id;params<=cmd_params;width<=cmd_width;height<=cmd_height;
    camera_valid<=cmd_camera_valid;border<=cmd_border_replicate;rsp_status<=0;
    if(!layout_ok||!image_fits||!map_fits||cmd_width==0||cmd_height==0||cmd_width>1920||cmd_height>1080)begin rsp_status<=1;state<=RESPONSE;end
    else case(cmd_opcode)
     1:state<=BUILD_START;
     2:if(raw_pv)state<=CAPTURE_START;else begin rsp_status<=3;state<=RESPONSE;end
     3:if(raw_cv&&out_pv)begin
      if(cmd_width!=raw_width[raw_cs]||cmd_height!=raw_height[raw_cs])begin rsp_status<=1;state<=RESPONSE;end else state<=CORRECT_START;
     end else begin rsp_status<=3;state<=RESPONSE;end
     4:if(out_cv)begin
      if(cmd_width!=out_width[out_cs]||cmd_height!=out_height[out_cs])begin rsp_status<=1;state<=RESPONSE;end else state<=DISPLAY_START;
     end else begin rsp_status<=3;state<=RESPONSE;end
     default:begin rsp_status<=1;state<=RESPONSE;end
    endcase
   end
   BUILD_START:if(core_cmd_ready)state<=BUILD_WAIT;
   BUILD_WAIT:if(core_rsp_valid)begin rsp_status<=core_status;state<=RESPONSE;end
   CAPTURE_START:if(cap_start&&cap_cmd_ready)begin raw_slot<=raw_ps;state<=CAPTURE_WAIT;end
   CAPTURE_WAIT:if(cap_rsp_valid)begin raw_width[raw_slot]<=width;raw_height[raw_slot]<=height;rsp_status<=cap_status;state<=RESPONSE;end
   CORRECT_START:if(correct_start&&core_cmd_ready)begin raw_slot<=raw_cs;out_slot<=out_ps;state<=CORRECT_WAIT;end
   CORRECT_WAIT:if(core_rsp_valid)begin out_width[out_slot]<=width;out_height[out_slot]<=height;rsp_status<=core_status;state<=RESPONSE;end
   DISPLAY_START:if(disp_start&&disp_cmd_ready)begin out_slot<=out_cs;state<=DISPLAY_WAIT;end
   DISPLAY_WAIT:if(disp_rsp_valid)begin rsp_status<=disp_status;state<=RESPONSE;end
   RESPONSE:if(rsp_ready)state<=IDLE;
   default:state<=IDLE;
  endcase
 end
endmodule
