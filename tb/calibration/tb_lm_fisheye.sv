`timescale 1ns/1ps
`include "calib_defs.vh"
// Fisheye LM case. Corners and the expected state come from independent Python
// code (scripts/calibration/engine/generate_lm_fisheye.py): the data is a plain
// Brown model, the expectation an independent damped Gauss-Newton solve. The
// initial state has zero distortion, so k3 has to be found from nothing.
module tb_lm_fisheye;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,cmd_valid=0,rsp_ready=0;wire cmd_ready,rsp_valid;
 reg [15:0] width=640,height=480;reg [1:0] stage;
 reg [`PAR_STATE_W-1:0] input_state,initial_state,expected;
 wire [`PAR_STATE_W-1:0] result;
 reg [63:0] expected_cost,saved_cost;wire [63:0] cost;
 wire [7:0] status,accepted,outer;wire converged;
 reg [64*`PAR_TOTAL_POINTS-1:0] points;
 wire rd_en;wire [`PAR_VIEW_BITS-1:0] rd_view;wire [`PAR_POINT_BITS-1:0] rd_index;
 reg rd_valid=0;reg [31:0] rd_x,rd_y;
 wire [8:0] fv,fr,fa,sv,sr;wire [44:0] fo;wire [575:0] fpa,fpb;
 wire [63:0] pool_value;wire [4:0] pool_flags;
 fp_calibration_pool #(.CLIENTS(9)) pool(.clk(clk),.rst_n(rst_n),
 .c_req_valid(fv),.c_req_ready(fr),.c_req_op(fo),.c_req_a(fpa),.c_req_b(fpb),
 .c_active(fa),.c_rsp_valid(sv),.c_rsp_ready(sr),.result(pool_value),.flags(pool_flags));
 lm_controller #(.FP_SHARED(1)) dut(.clk(clk),.rst_n(rst_n),
 .shared_req_valid(fv),.shared_req_ready(fr),.shared_req_op(fo),.shared_req_a(fpa),.shared_req_b(fpb),
 .shared_active(fa),.shared_rsp_valid(sv),.shared_rsp_ready(sr),.shared_rsp_result(pool_value),.shared_rsp_flags(pool_flags),
 .cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_width(width),.cmd_height(height),.cmd_state(input_state),.cmd_stage(stage),
 .point_rd_en(rd_en),.point_rd_view_id(rd_view),.point_rd_index(rd_index),.point_rd_valid(rd_valid),.point_rd_x_fp32(rd_x),.point_rd_y_fp32(rd_y),
 .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(status),.rsp_state(result),.rsp_cost_fp64(cost),
 .rsp_converged(converged),.rsp_accepted_steps(accepted),.rsp_outer_iterations(outer));
 integer fd,n,i,ignored,expected_converged,cycles,cases=0,errors=0,reads=0,drop=-1;
 real deviation,k3_value;
 reg done=0;
 task check;input condition;input [511:0] message;begin
 if(condition!==1'b1)begin errors=errors+1;$display("FAIL case=%0d %0s",cases,message);end end endtask
 task near;input [63:0] xbits,ybits;input real tolerance;real x,y,d;begin
 x=$bitstoreal(xbits);y=$bitstoreal(ybits);d=x-y;if(d<0)d=-d;
 check((^xbits)!==1'bx && xbits[62:52]!=2047 && d<=tolerance*(1+(y<0?-y:y)),"independent fisheye reference");
 if(d>tolerance*(1+(y<0?-y:y)))$display("actual=%.17g expected=%.17g diff=%g",x,y,d);
 if(d/(1+(y<0?-y:y))>deviation)deviation=d/(1+(y<0?-y:y));
 end endtask
 always @(posedge clk)begin
  rd_valid<=0;
  if(rst_n && rd_en)begin
   check(rd_view<`PAR_VIEWS && rd_index<`PAR_POINTS,"corner address");
   rd_valid<=reads!=drop;rd_x<=points[(rd_view*`PAR_POINTS+rd_index)*64+:32];
   rd_y<=points[(rd_view*`PAR_POINTS+rd_index)*64+32+:32];reads=reads+1;
  end
 end
 task reset_dut;begin @(negedge clk);rst_n=0;cmd_valid=0;rsp_ready=0;
 repeat(3)@(negedge clk);check(!rsp_valid && !rd_en,"reset cancels interface");rst_n=1;@(negedge clk);end endtask
 task launch;begin @(negedge clk);check(cmd_ready,"command ready");reads=0;input_state=initial_state;cmd_valid=1;
 @(negedge clk);cmd_valid=0;input_state=~initial_state;end endtask
 task await_response;begin cycles=0;
 while(!rsp_valid && cycles<1000000000)begin @(negedge clk);cycles=cycles+1;end
 if(!rsp_valid)$fatal(1,"timeout adapter=%d instruction=%d",dut.state,dut.engine.private_engine.core.sequencer.pc);
 $display("LM_FISHEYE case=%0d cycles=%0d accepted=%0d outer=%0d cost=%.17g status=%0d",cases,cycles,accepted,outer,$bitstoreal(cost),status);
 end endtask
 task consume;reg [7:0] ss,aa,oo;reg cc;begin
 ss=status;aa=accepted;oo=outer;cc=converged;
 repeat(7)begin @(negedge clk);check(rsp_valid && !cmd_ready && status===ss && accepted===aa && outer===oo && converged===cc,"stable output under backpressure");end
 rsp_ready=1;@(negedge clk);rsp_ready=0;cases=cases+1;end endtask
 initial begin
  fd=$fopen("lm_fisheye_vectors.txt","r");if(!fd)$fatal(1,"fixture open");reset_dut();deviation=0;
  while(!$feof(fd))begin
   n=$fscanf(fd,"%d %d %d %h %h %h %h\n",stage,expected_converged,ignored,expected_cost,initial_state,points,expected);
   if(n==7)begin
    launch();await_response();
    check(status==0 && converged==expected_converged,"Fisheye fit converges");
    check(accepted>=1 && outer<=`PAR_LM_MAX_ITERS,"iteration accounting");
    for(i=0;i<`PAR_STATE_N;i=i+1)near(result[64*i+:64],expected[64*i+:64],2e-7);
    if($bitstoreal(cost)>1e-6)$display("cost above the float32 noise floor: %.17g",$bitstoreal(cost));
    check($bitstoreal(cost)<=1e-6,"cost reaches the float32 noise floor");
    k3_value=$bitstoreal(result[512+:64]);
    $display("LM_FISHEYE k3=%.12g expected=%.12g",k3_value,$bitstoreal(expected[512+:64]));
    consume();
   end else if(n!=-1)$fatal(1,"fixture format");
  end
  $fclose(fd);
  $display("LM_FISHEYE_DONE cases=%0d errors=%0d max_deviation=%g",cases,errors,deviation);done=1;
 end
endmodule
