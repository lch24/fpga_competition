`include "calib_defs.vh"
`include "calibration_program_defs.vh"
// One instruction/data engine for mutually exclusive calibration phases.
// The owner must remain selected until its complete command response; HOST
// pauses are part of that ownership, not opportunities for another requester.
// READ FIRST: docs/INSTRUCTION_CONTROL_GUIDE.md explains the instruction
// machines. This one executes calibration steps, not math/feature microcode.
// Program source: scripts/calibration/engine/build_rom.py; entry PCs are
// generated in calibration_program_defs.vh. Do not edit ROM words by hand.
module calib_execution_service(
 input wire clk,rst_n,input wire [127:0] request,output wire [95:0] response,
 output wire shared_req_valid,input wire shared_req_ready,output wire [4:0] shared_req_op,
 output wire [63:0] shared_req_a,shared_req_b,output wire shared_rsp_ready,
 input wire shared_rsp_valid,input wire [63:0] shared_rsp_result,input wire [4:0] shared_rsp_flags
);
 `include "calib_workspace_layout.vh"
 wire host_en,host_write,start_valid,rsp_ready,svc_ready;
 wire [31:0] host_addr;wire [63:0] host_data;wire [15:0] start_pc;
 assign {svc_ready,rsp_ready,start_pc,start_valid,host_data,host_addr,host_write,host_en}=request[116:0];
 wire start_ready,host_ready,host_valid,host_error,busy,rsp_valid,svc_valid;
 wire [63:0] host_result;wire [7:0] rsp_status,svc_id;
 assign response={9'd0,svc_id,svc_valid,rsp_status,rsp_valid,busy,host_result,host_error,host_valid,host_ready,start_ready};
 wire imem_en;wire [15:0] imem_addr;reg [31:0] imem_data;
 reg [31:0] program_memory[0:4095];
 initial begin
  `include "calibration_program_init.vh"
 end
 always @(posedge clk)if(imem_en)imem_data<=program_memory[imem_addr[11:0]];
 calib_datapath #(.RAM_WORDS(WORDS),.CONST_BASE(WORDS),.FP_SHARED(1),
   .ENABLE_KERNELS(0),.HOST_CALLS(1),.TRAP_FP(1)) engine(
   .clk(clk),.rst_n(rst_n),.start_valid(start_valid),.start_ready(start_ready),
   .start_pc(start_pc),.program_words(16'd`CALIBRATION_PROGRAM_WORDS),
   .imem_en(imem_en),.imem_addr(imem_addr),.imem_data(imem_data),
   .host_en(host_en),.host_write(host_write),.host_addr(host_addr),.host_data(host_data),
   .host_ready(host_ready),.host_valid(host_valid),.host_error(host_error),.host_result(host_result),
   .busy(busy),.rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),
   .svc_valid(svc_valid),.svc_ready(svc_ready),.svc_id(svc_id),
   .debug_pc(),.cycle_count(),.instruction_count(),
   .shared_req_valid(shared_req_valid),.shared_req_ready(shared_req_ready),.shared_req_op(shared_req_op),
   .shared_req_a(shared_req_a),.shared_req_b(shared_req_b),.shared_rsp_ready(shared_rsp_ready),
   .shared_rsp_valid(shared_rsp_valid),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags));
endmodule
