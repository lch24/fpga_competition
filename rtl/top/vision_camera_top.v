`timescale 1ns/1ps
// Generated named wiring; control below owns exactly one captured frame lease.
// start -> repeat capture request/detection V times -> calibrate -> corrected DDR.
// Camera I2C, clock generation, pins and DDR IP remain the board project's role.
module vision_camera_top #(
 parameter WIDTH=1280,HEIGHT=720,DEPTH=2,
 parameter RAW_BASE=32'h00000000,SLOT_BYTES=32'h00400000,
 parameter GRAY_BASE=32'h03000000,DST_BASE=32'h01000000,
 parameter FIXED_BILINEAR=1, parameter FIXED_ACCUM=1,
 parameter CANDIDATE_BASE=32'h03200000,
 parameter MAP_X_BASE=32'h02000000,MAP_Y_BASE=32'h02800000
)(
 input wire clk,rst_n,ddr_ready,camera_pclk,camera_rst_n,camera_vsync,camera_href,input wire [7:0] camera_data,
 input wire start_valid,output wire start_ready,input wire [63:0] square_size_fp64,
 input wire capture_valid,output wire capture_ready,
 output wire rsp_valid,input wire rsp_ready,output wire [7:0] rsp_status,
 output wire [31:0] result_base,result_stride,output wire [15:0] result_width,result_height,
 output wire [287:0] result_params,output wire [63:0] result_rms,
 output wire busy,output wire [31:0] debug_job,output wire [7:0] debug_view,output wire [5:0] debug_phase,
 output wire ownership_error,
 output wire [27:0] axi_araddr,
 output wire [27:0] axi_awaddr,
 output wire [3:0] axi_aruser_id,
 output wire [3:0] axi_arlen,
 output wire [3:0] axi_awuser_id,
 output wire [3:0] axi_awlen,
 output wire axi_aruser_ap,
 output wire axi_arvalid,
 output wire axi_awuser_ap,
 output wire axi_awvalid,
 input wire axi_arready,
 input wire axi_awready,
 input wire [255:0] axi_rdata,
 input wire axi_rvalid,
 input wire axi_rlast,
 input wire [3:0] axi_rid,
 output wire [255:0] axi_wdata,
 output wire [31:0] axi_wstrb,
 input wire axi_wready,
 input wire axi_wusero_last,
 input wire [3:0] axi_wusero_id
);
 localparam TRIGGER=0,CAP_CMD=1,CAP_WAIT=2,PIN=3,LEASE=4,FAILED_FRAME=5;
 reg [2:0] state;reg [7:0] capture_status;reg [3:0] slot;
 wire sys_ready,sys_done;wire [7:0] sys_status;
 wire pin_valid,pin_ready,release_ready;wire [3:0] pin_slot;wire [31:0] pin_base;
 wire frame_ready,frame_release;
 assign capture_ready=state==TRIGGER&&frame_ready&&ddr_ready;
 assign pin_ready=state==PIN&&frame_ready;
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
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=TRIGGER;capture_status<=0;slot<=0;end
  else case(state)
   TRIGGER:if(capture_valid&&capture_ready)state<=CAP_CMD;
   CAP_CMD:if(sys_ready)state<=CAP_WAIT;
   CAP_WAIT:if(sys_done)begin capture_status<=sys_status;state<=sys_status==0?PIN:FAILED_FRAME;end
   PIN:if(pin_valid&&pin_ready)begin slot<=pin_slot;state<=LEASE;end
   LEASE:if(frame_release&&release_ready)state<=TRIGGER;
   FAILED_FRAME:if(frame_ready)state<=TRIGGER;
   default:state<=TRIGGER;
  endcase
 end
 vision_ddr_top #(.FIXED_ACCUM(FIXED_ACCUM),.FIXED_BILINEAR(FIXED_BILINEAR),.CANDIDATE_BASE(CANDIDATE_BASE),.WIDTH(WIDTH),.HEIGHT(HEIGHT),.DEPTH(DEPTH),.GRAY_BASE(GRAY_BASE),.DST_BASE(DST_BASE),.MAP_X_BASE(MAP_X_BASE),.MAP_Y_BASE(MAP_Y_BASE)) pipeline(
  .clk(clk),.rst_n(rst_n),.start_valid(start_valid),.start_ready(start_ready),.square_size_fp64(square_size_fp64),
  .frame_valid((state==PIN&&pin_valid)||state==FAILED_FRAME),.frame_ready(frame_ready),
  .frame_base(pin_base),.frame_stride(32'(2*WIDTH)),.frame_capacity(SLOT_BYTES),.frame_status(capture_status),
  .frame_release_valid(frame_release),.frame_release_ready(state==LEASE&&release_ready),
  .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),
  .result_base(result_base),.result_stride(result_stride),.result_width(result_width),.result_height(result_height),
  .result_params(result_params),.result_rms(result_rms),.busy(busy),.debug_job(debug_job),.debug_view(debug_view),.debug_phase(debug_phase),
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
  .b_error(b_error));
 // Snapshot-only use of the existing camera/DDR subsystem. Its remap/display
 // command paths are never requested; correction is owned by pipeline above.
 camera_system_top #(.RAW_BASE(RAW_BASE),.RAW_SLOTS(1),.SLOT_BYTES(SLOT_BYTES),
  .OUTPUT_BASE(DST_BASE),.OUTPUT_SLOTS(1),.MAP_X_BASE(MAP_X_BASE),.MAP_Y_BASE(MAP_Y_BASE),.MAP_CAPACITY(4*WIDTH*HEIGHT)) storage(
  .clk(clk),.rst_n(rst_n),.ddr_ready(ddr_ready),.camera_pclk(camera_pclk),.camera_rst_n(camera_rst_n),
  .camera_vsync(camera_vsync),.camera_href(camera_href),.camera_data(camera_data),
  .hdmi_pclk(clk),.hdmi_rst_n(rst_n),.hdmi_de(1'b0),.hdmi_vsync(1'b0),.clear_hdmi_error(1'b0),
  .cmd_valid(state==CAP_CMD),.cmd_ready(sys_ready),.cmd_opcode(3'd2),.cmd_job_id(debug_job),.cmd_calib_id(32'd0),
  .cmd_camera_valid(1'b0),.cmd_params(288'd0),.cmd_width(16'(WIDTH)),.cmd_height(16'(HEIGHT)),.cmd_border_replicate(1'b0),
  .rsp_valid(sys_done),.rsp_ready(state==CAP_WAIT),.rsp_status(sys_status),.ownership_error(ownership_error),
  .raw_pin_valid(pin_valid),.raw_pin_ready(pin_ready),.raw_pin_slot(pin_slot),.raw_pin_base(pin_base),
  .raw_release_valid(state==LEASE&&frame_release),.raw_release_ready(release_ready),.raw_release_slot(slot),
  .ext_rd_valid(rd_valid),
  .ext_rd_ready(rd_ready),
  .ext_rd_addr(rd_addr),
  .ext_rd_len(rd_len),
  .ext_rd_tag(rd_tag),
  .ext_r_valid(r_valid),
  .ext_r_ready(r_ready),
  .ext_r_data(r_data),
  .ext_r_keep(r_keep),
  .ext_r_tag(r_tag),
  .ext_r_last(r_last),
  .ext_r_error(r_error),
  .ext_wr_valid(wr_valid),
  .ext_wr_ready(wr_ready),
  .ext_wr_addr(wr_addr),
  .ext_wr_len(wr_len),
  .ext_wr_tag(wr_tag),
  .ext_w_valid(w_valid),
  .ext_w_ready(w_ready),
  .ext_w_data(w_data),
  .ext_w_keep(w_keep),
  .ext_w_last(w_last),
  .ext_b_valid(b_valid),
  .ext_b_ready(b_ready),
  .ext_b_tag(b_tag),
  .ext_b_error(b_error),
  .axi_araddr(axi_araddr),
  .axi_awaddr(axi_awaddr),
  .axi_aruser_id(axi_aruser_id),
  .axi_arlen(axi_arlen),
  .axi_awuser_id(axi_awuser_id),
  .axi_awlen(axi_awlen),
  .axi_aruser_ap(axi_aruser_ap),
  .axi_arvalid(axi_arvalid),
  .axi_awuser_ap(axi_awuser_ap),
  .axi_awvalid(axi_awvalid),
  .axi_arready(axi_arready),
  .axi_awready(axi_awready),
  .axi_rdata(axi_rdata),
  .axi_rvalid(axi_rvalid),
  .axi_rlast(axi_rlast),
  .axi_rid(axi_rid),
  .axi_wdata(axi_wdata),
  .axi_wstrb(axi_wstrb),
  .axi_wready(axi_wready),
  .axi_wusero_last(axi_wusero_last),
  .axi_wusero_id(axi_wusero_id));
endmodule
