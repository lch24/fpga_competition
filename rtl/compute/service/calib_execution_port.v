// Packed connection to the one phase-owned calibration execution service.
// The standalone branch keeps module-level numerical regressions usable;
// board hierarchy selects EXTERNAL=1 and contains no private engine/RAM.
// PROGRAM_BASE translates a phase-local start_pc to the shared ROM address.
// This module routes transactions; it does not decode instructions in EXTERNAL
// mode. "host" is the hardware phase adapter. See docs/INSTRUCTION_CONTROL_GUIDE.md.
// Request low bits: host_en, host_write, addr[32], data[64], start_valid,
// start_pc[16], response_ready, service_ready. Upper 11 bits are reserved.
// Response low bits: start_ready, host_ready, host_valid, host_error,
// host_data[64], busy, response_valid, status[8], service_valid, service_id[8].
module calib_execution_port #(parameter EXTERNAL=0,PROGRAM_BASE=0,
 RAM_WORDS=4096,CONST_BASE=3584,TRAP_FP=0,HOST_CALLS=0)(
 input wire clk,rst_n,start_valid,output wire start_ready,
 input wire [15:0] start_pc,program_words,
 output wire imem_en,output wire [15:0] imem_addr,input wire [31:0] imem_data,
 input wire host_en,host_write,input wire [31:0] host_addr,input wire [63:0] host_data,
 output wire host_ready,host_valid,host_error,output wire [63:0] host_result,
 output wire busy,rsp_valid,input wire rsp_ready,output wire [7:0] rsp_status,
 output wire [15:0] debug_pc,output wire [31:0] cycle_count,instruction_count,
 output wire shared_req_valid,input wire shared_req_ready,output wire [4:0] shared_req_op,
 output wire [63:0] shared_req_a,shared_req_b,output wire shared_rsp_ready,
 input wire shared_rsp_valid,input wire [63:0] shared_rsp_result,input wire [4:0] shared_rsp_flags,
 output wire svc_valid,input wire svc_ready,output wire [7:0] svc_id,
 output wire [127:0] execution_req,input wire [95:0] execution_rsp
);
 generate if(EXTERNAL)begin : external_engine
  wire [15:0] absolute_pc=start_pc+PROGRAM_BASE;
  assign execution_req={11'd0,svc_ready,rsp_ready,absolute_pc,start_valid,host_data,host_addr,host_write,host_en};
  assign {svc_id,svc_valid,rsp_status,rsp_valid,busy,host_result,host_error,host_valid,host_ready,start_ready}=execution_rsp[86:0];
  assign imem_en=0;assign imem_addr=0;assign debug_pc=0;assign cycle_count=0;assign instruction_count=0;
  assign shared_req_valid=0;assign shared_req_op=0;assign shared_req_a=0;assign shared_req_b=0;assign shared_rsp_ready=0;
 end else begin : private_engine
  assign execution_req=0;
  calib_datapath #(.RAM_WORDS(RAM_WORDS),.CONST_BASE(CONST_BASE),.FP_SHARED(1),
    .ENABLE_KERNELS(0),.TRAP_FP(TRAP_FP),.HOST_CALLS(HOST_CALLS)) core(
    .clk(clk),.rst_n(rst_n),.start_valid(start_valid),.start_ready(start_ready),.start_pc(start_pc),.program_words(program_words),
    .imem_en(imem_en),.imem_addr(imem_addr),.imem_data(imem_data),
    .host_en(host_en),.host_write(host_write),.host_addr(host_addr),.host_data(host_data),
    .host_ready(host_ready),.host_valid(host_valid),.host_error(host_error),.host_result(host_result),
    .busy(busy),.rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),
    .debug_pc(debug_pc),.cycle_count(cycle_count),.instruction_count(instruction_count),
    .svc_valid(svc_valid),.svc_ready(svc_ready),.svc_id(svc_id),
    .shared_req_valid(shared_req_valid),.shared_req_ready(shared_req_ready),.shared_req_op(shared_req_op),
    .shared_req_a(shared_req_a),.shared_req_b(shared_req_b),.shared_rsp_ready(shared_rsp_ready),
    .shared_rsp_valid(shared_rsp_valid),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags));
 end endgenerate
endmodule
