const fs=require('fs'),path=require('path');
const {bus,width}=require('./generate_wiring');
const physical=[['axi_araddr',28,1],['axi_awaddr',28,1],['axi_aruser_id',4,1],['axi_arlen',4,1],['axi_awuser_id',4,1],['axi_awlen',4,1],
 ['axi_aruser_ap',1,1],['axi_arvalid',1,1],['axi_awuser_ap',1,1],['axi_awvalid',1,1],['axi_arready',1,0],['axi_awready',1,0],
 ['axi_rdata',256,0],['axi_rvalid',1,0],['axi_rlast',1,0],['axi_rid',4,0],['axi_wdata',256,1],['axi_wstrb',32,1],
 ['axi_wready',1,0],['axi_wusero_last',1,0],['axi_wusero_id',4,0]];
const code=`${String.fromCharCode(96)}timescale 1ns/1ps
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
 ${physical.map(([n,w,o])=>`${o?'output':'input'} wire ${width(w)}${n}`).join(',\n ')}
);
 localparam TRIGGER=0,CAP_CMD=1,CAP_WAIT=2,PIN=3,LEASE=4,FAILED_FRAME=5;
 reg [2:0] state;reg [7:0] capture_status;reg [3:0] slot;
 wire sys_ready,sys_done;wire [7:0] sys_status;
 wire pin_valid,pin_ready,release_ready;wire [3:0] pin_slot;wire [31:0] pin_base;
 wire frame_ready,frame_release;
 assign capture_ready=state==TRIGGER&&frame_ready&&ddr_ready;
 assign pin_ready=state==PIN&&frame_ready;
 ${bus.map(([n,w])=>`wire ${width(w)}${n};`).join('\n ')}
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
  ${bus.map(([n])=>`.${n}(${n})`).join(',\n  ')});
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
  ${bus.map(([n])=>`.ext_${n}(${n})`).join(',\n  ')},
  ${physical.map(([n])=>`.${n}(${n})`).join(',\n  ')});
endmodule
`;
fs.writeFileSync(path.join(__dirname,'../../rtl/top/vision_camera_top.v'),code);
