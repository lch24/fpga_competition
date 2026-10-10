`timescale 1ns/1ps
`include "calib_defs.vh"
// Small rank-one arithmetic/control smoke through the BOARD shared engine.
// Residual is independently defined as r[i]=state[0], with exact cost sum.
module tb_lm_service;
 reg clk=0;always #5 clk=~clk;reg rst_n=0,cmd_valid=0,rsp_ready=0;
 wire cmd_ready,rsp_valid;wire [7:0] status,accepted,outer;wire converged;
 reg [`PAR_STATE_W-1:0] input_state=0;wire [`PAR_STATE_W-1:0] result;
 wire [63:0] cost;
 wire [127:0] execution_req;wire [95:0] execution_rsp;
 wire fv,fr,sv,sr;wire [4:0] fo,flags;wire [63:0] fa,fb,fd;
 calib_execution_service engine(.clk(clk),.rst_n(rst_n),.request(execution_req),.response(execution_rsp),
 .shared_req_valid(fv),.shared_req_ready(fr),.shared_req_op(fo),.shared_req_a(fa),.shared_req_b(fb),
 .shared_rsp_valid(sv),.shared_rsp_ready(sr),.shared_rsp_result(fd),.shared_rsp_flags(flags));
 calib_alu alu(.ce(1'b1),.clk(clk),.rst_n(rst_n),.req_valid(fv),.req_ready(fr),.req_op(fo),.req_a(fa),.req_b(fb),
 .rsp_valid(sv),.rsp_ready(sr),.rsp_result(fd),.rsp_flags(flags),.rsp_less(),.rsp_equal(),.rsp_unordered());
 wire ev,er,dr,rr;wire [`PAR_STATE_W-1:0] es;
 reg busy=0;reg [31:0] count=0;reg [63:0] residual=0,rescost=0;
 reg [7:0] mock_status=0;integer mode=0,calls=0,errors=0,cases=0,cycles=0;
 reg done=0;real x,total;
 wire dv=busy && count<`PAR_RESIDUALS;
 wire rv=busy && count==`PAR_RESIDUALS;
 assign er=!busy;
 always @(posedge clk)begin
  if(!rst_n)begin busy<=0;count<=0;end
  else begin
   if(ev && er)begin
    busy<=1;count<=0;residual<=es[63:0];x=$bitstoreal(es[63:0]);total=0;
    for(integer k=0;k<`PAR_RESIDUALS;k=k+1)total=total+x*x;
    if(mode==2 && calls>0 && es[63:0]==0)total=1e10;
    rescost<=$realtobits(total);mock_status<=mode==1?8'd5:8'd0;calls=calls+1;
   end
   if(dv && dr)count<=count+1;
   if(rv && rr)busy<=0;
  end
 end
 lm_controller #(.FP_SHARED(1),.SHARE_RESIDUAL(1),.ENGINE_SHARED(1)) dut(
 .clk(clk),.rst_n(rst_n),.execution_req(execution_req),.execution_rsp(execution_rsp),
 .cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_width(16'd640),.cmd_height(16'd480),.cmd_state(input_state),.cmd_stage(2'd2),
 .ext_cmd_valid(ev),.ext_cmd_ready(er),.ext_cmd_state(es),.ext_point_rd_en(1'b0),.ext_point_rd_view_id(0),.ext_point_rd_index(0),
 .ext_data_valid(dv),.ext_data_ready(dr),.ext_data_index(count[`PAR_RES_BITS-1:0]),.ext_data_fp64(residual),.ext_data_last(count==`PAR_RESIDUALS-1),
 .ext_rsp_valid(rv),.ext_rsp_ready(rr),.ext_rsp_status(mock_status),.ext_rsp_cost_fp64(rescost),
 .point_rd_valid(1'b0),.point_rd_x_fp32(0),.point_rd_y_fp32(0),
 .shared_req_ready(9'd0),.shared_rsp_valid(9'd0),.shared_rsp_result(64'd0),.shared_rsp_flags(5'd0),
 .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(status),.rsp_state(result),.rsp_cost_fp64(cost),
 .rsp_converged(converged),.rsp_accepted_steps(accepted),.rsp_outer_iterations(outer));
 task check;input condition;begin if(condition!==1'b1)begin errors=errors+1;$display("FAIL service case=%0d status=%d cost=%g",cases,status,$bitstoreal(cost));end end endtask
 task run;begin
 @(negedge clk);input_state=0;input_state[63:0]=$realtobits(0.25);calls=0;cmd_valid=1;
 @(negedge clk);cmd_valid=0;cycles=0;
 while(!rsp_valid && cycles<5000000)begin @(negedge clk);cycles=cycles+1;end
 if(!rsp_valid)$fatal(1,"service timeout adapter=%d pc=%d",dut.state,engine.engine.sequencer.pc);
 if(mode==0)begin check(status==0 && converged && accepted>0 && $bitstoreal(cost)<1e-12);check(result[`PAR_STATE_W-1:64]==0);end
 else check(status==5 && !converged && accepted==0 && result===input_state);
 repeat(5)begin @(negedge clk);check(rsp_valid && !cmd_ready);end
 $display("LM_SERVICE case=%0d cycles=%0d evaluations=%0d accepted=%0d cost=%g",cases,cycles,calls,accepted,$bitstoreal(cost));
 rsp_ready=1;@(negedge clk);rsp_ready=0;cases=cases+1;
 end endtask
 initial begin repeat(3)@(negedge clk);rst_n=1;run();mode=1;run();mode=0;run();done=1;
 $display("LM_SERVICE_DONE cases=%d errors=%d",cases,errors);end
endmodule
