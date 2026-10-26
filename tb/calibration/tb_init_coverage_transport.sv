`timescale 1ns/1ps
`include "calib_defs.vh"
`include "lm_program_defs.vh"
// Actual initialization adapter with a deliberate host-service protocol model.
// Numeric DLT/coverage is tested separately; this tests the extra metadata read
// cannot shift the seed payload or retain the previous job's coverage bit.
module tb_init_coverage_transport;
 reg clk=0,rst_n=0,cmd_valid=0,seed_ready=0,rsp_ready=0;
 always #5 clk=~clk;
 wire cmd_ready,seed_valid,rsp_valid,seed_coverage_ok,point_rd_en;
 wire [7:0] rsp_status;wire [2:0] seed_id,rsp_seed_count;
 wire [`PAR_STATE_W-1:0] seed_state;
 wire [127:0] request;wire [95:0] response;
 reg point_valid=0,host_valid=0,engine_valid=0;
 reg [63:0] host_result=0;
 reg fit_value=0;reg [7:0] status_value=0;
 integer countdown=0,seed_reads=0,fit_reads=0,i,watchdog;
 localparam SEED_BASE=1024+`PAR_TOTAL_POINTS+4*`PAR_POINTS;
 function [63:0] word;input integer index;begin word=64'h3ff0000000000000+index;end endfunction
 assign response={9'd0,8'd0,1'b0,8'd0,engine_valid,1'b0,host_result,1'b0,host_valid,1'b1,1'b1};
 always @(posedge clk)begin
  if(!rst_n)begin point_valid<=0;host_valid<=0;engine_valid<=0;countdown<=0;seed_reads<=0;fit_reads<=0;end
  else begin
   point_valid<=point_rd_en;
   host_valid<=request[0]&&!request[1];
   if(request[0]&&!request[1])begin
    if(request[33:2]==`LM_STATUS)host_result<=status_value;
    else if(request[33:2]==`LM_COVERAGE_OK)begin host_result<=fit_value;fit_reads<=fit_reads+1;end
    else if(request[33:2]>=SEED_BASE && request[33:2]<SEED_BASE+`PAR_STATE_N)begin
     host_result<=word(request[33:2]-SEED_BASE);seed_reads<=seed_reads+1;
    end else $fatal(1,"unexpected host read %0d",request[33:2]);
   end
   if(request[98])countdown<=3;
   else if(countdown>0)begin countdown<=countdown-1;if(countdown==1)engine_valid<=1;end
   if(engine_valid&&request[115])engine_valid<=0;
  end
 end
 init_controller #(.FP_SHARED(1),.ENGINE_SHARED(1)) dut(
  .clk(clk),.rst_n(rst_n),.execution_req(request),.execution_rsp(response),
  .cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_width(16'd640),.cmd_height(16'd480),
  .point_rd_en(point_rd_en),.point_rd_view_id(),.point_rd_index(),
  .point_rd_valid(point_valid),.point_rd_x_fp32(32'h43000000),.point_rd_y_fp32(32'h43000000),
  .seed_valid(seed_valid),.seed_ready(seed_ready),.seed_id(seed_id),.seed_state(seed_state),.seed_coverage_ok(seed_coverage_ok),
  .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.rsp_seed_count(rsp_seed_count),
  .shared_req_ready(7'd0),.shared_rsp_valid(7'd0),.shared_rsp_result(64'd0),.shared_rsp_flags(5'd0));
 task start;input fit;input [7:0] status;begin
  fit_value=fit;status_value=status;
  @(negedge clk);if(!cmd_ready)$fatal(1,"not idle");cmd_valid=1;
  @(negedge clk);cmd_valid=0;
 end endtask
 task finish;input [7:0] status;begin
  watchdog=0;while(!rsp_valid&&watchdog<5000)begin @(negedge clk);watchdog=watchdog+1;end
  if(!rsp_valid||rsp_status!=status||rsp_seed_count!=(status==0))$fatal(1,"response mismatch");
  repeat(5)begin @(negedge clk);if(!rsp_valid||cmd_ready)$fatal(1,"response stalled");end
  rsp_ready=1;@(negedge clk);rsp_ready=0;
 end endtask
 task good;input fit;begin
  start(fit,0);watchdog=0;
  while(!seed_valid&&watchdog<5000)begin @(negedge clk);watchdog=watchdog+1;end
  if(!seed_valid||seed_id!=2||seed_coverage_ok!==fit)$fatal(1,"seed/coverage mismatch");
  repeat(8)begin
   @(negedge clk);
   if(!seed_valid||seed_coverage_ok!==fit||rsp_valid)$fatal(1,"seed metadata stall");
   for(i=0;i<`PAR_STATE_N;i=i+1)if(seed_state[64*i+:64]!==word(i))$fatal(1,"seed shifted at %0d",i);
  end
  seed_ready=1;@(negedge clk);seed_ready=0;finish(0);
 end endtask
 initial begin
  repeat(3)@(negedge clk);rst_n=1;
  good(1);good(0);good(1);
  if(seed_reads!=3*`PAR_STATE_N||fit_reads!=3)$fatal(1,"wrong host read count");
  start(0,4);finish(4);
  if(seed_coverage_ok!==0||seed_reads!=3*`PAR_STATE_N||fit_reads!=3)$fatal(1,"failed job retained metadata");
  $display("PASS tb_init_coverage_transport: metadata, full seed, stalls, back-to-back, error reset");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
