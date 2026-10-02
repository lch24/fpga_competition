`timescale 1ns/1ps
// 数值场景使用全部真实子模块；后半部分明确使用故障注入覆盖控制分支。
module tb_lm_controller;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,cmd_valid=0,rsp_ready=0;wire cmd_ready,rsp_valid;wire [7:0] rsp_status;
 reg [15:0] width=640,height=480;reg [1:0] stage;
 reg [1727:0] cmd_state,initial_state,expected,result_snapshot;
 reg [7679:0] points;wire [1727:0] result;wire [63:0] cost;wire converged;wire [7:0] accepted,outer;
 wire rd_en;wire [1:0] rd_view;wire [5:0] rd_index;reg rd_valid=0;reg [31:0] rd_x,rd_y;
 integer drop=-1,reads=0,mode=0,solves=0,rejections=0,evals=0;
 integer fd,report,rc,i,expected_accepted,expected_converged,cycles,cases=0,errors=0;
 reg done=0;reg [63:0] expected_cost,saved_cost;reg [7:0] saved_accepted,saved_outer,saved_status;reg saved_converged;
 reg [1727:0] before_trial;integer last_pc=-1;reg [7:0] last_outer=0;reg force_active=0;
 lm_controller dut(.clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_width(width),.cmd_height(height),.cmd_state(cmd_state),.cmd_stage(stage),
 .point_rd_en(rd_en),.point_rd_view_id(rd_view),.point_rd_index(rd_index),.point_rd_valid(rd_valid),.point_rd_x_fp32(rd_x),.point_rd_y_fp32(rd_y),
 .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.rsp_state(result),.rsp_cost_fp64(cost),
 .rsp_converged(converged),.rsp_accepted_steps(accepted),.rsp_outer_iterations(outer));
 task check;input condition;input [511:0] label;begin if(condition!==1'b1)begin errors=errors+1;if(errors<50)$fdisplay(report,"FAIL case=%0d %0s pc=%0d",cases,label,dut.pc);end end endtask
 task near;input [63:0] bits,expected_bits;input real tolerance;real x,y,d;begin
 x=$bitstoreal(bits);y=$bitstoreal(expected_bits);d=x-y;if(d<0)d=-d;
 check((^bits)!==1'bx && bits[62:52]!=2047 && d<=tolerance*(1+(y<0?-y:y)),"C++ numerical match");
 if(d>tolerance*(1+(y<0?-y:y)))$fdisplay(report,"actual=%.17g expected=%.17g diff=%.9g",x,y,d);end endtask
 always @(posedge clk)begin
  rd_valid<=0;
  if(rst_n && rd_en)begin
   check(rd_view<3 && rd_index<40,"read address");
   check(rd_view==((reads%120)/40) && rd_index==reads%40,"read order and one read per point");
   rd_valid<=reads!=drop;rd_x<=points[(rd_view*40+rd_index)*64+:32];rd_y<=points[(rd_view*40+rd_index)*64+32+:32];reads=reads+1;
  end
  if(rst_n)begin
   if(dut.pc==13 && dut.ds_ready)solves=solves+1;
   if(dut.re_command && dut.re_ready)evals=evals+1;
   if(dut.pc==22 && dut.re_ready)before_trial=dut.current;
   if(dut.pc==30 && last_pc!=30)begin rejections=rejections+1;check(dut.current===before_trial || mode==4 || mode==8,"rejected trial cannot overwrite current");end
   if(outer!=last_outer)begin $display("LM case=%0d outer=%0d accepted=%0d",cases,outer,accepted);last_outer=outer;end
   last_pc=dut.pc;
  end
 end
 // 分支注入保留真实子模块运算；仅改变指定响应或阈值。mode=0完全无注入。
 always @(negedge clk)begin
  // 仅在注入后的下一拍release，避免每个时钟都重新驱动真实算术核的输出网。
  if(force_active)begin
   release dut.jac_status;release dut.re_cost;release dut.ds_status;release dut.ne_status;release dut.ds_load_status;release dut.j_row;force_active=0;
  end
  if(mode==1 && dut.pc==6 && dut.jac_valid)begin force dut.jac_status=8'd4;force_active=1;end
  if(mode==2 && dut.pc==23 && dut.re_valid)begin force dut.re_cost=64'h7fefffffffffffff;force_active=1;end
  if(mode==3 && dut.pc==1 && dut.re_valid)begin force dut.re_cost=64'b0;force_active=1;end
  if((mode==4 || mode==8) && dut.pc==14 && dut.ds_valid)begin force dut.ds_status=8'd4;force_active=1;end
  if(mode==5 && dut.pc==8 && dut.ne_valid)begin force dut.ne_status=8'd5;force_active=1;end
  if(mode==6 && dut.pc==8 && dut.ds_load_valid)begin force dut.ds_load_status=8'd1;force_active=1;end
  if(mode==7 && dut.pc==12)dut.v[2]=0; // 梯度严格收敛。
  if(mode==8 && dut.pc==12)dut.v[2]=$realtobits(1e-7*$bitstoreal(dut.v[3])); // 介于严格/宽松梯度阈值。
  if(mode==9 && dut.pc==2 && dut.outer==0)dut.outer=149; // 从最后一轮边界开始，真实执行该轮。
  if(mode==10 && dut.pc==29)dut.v[4]=0; // 接受后相对步长收敛分支。
  if(mode==11 && dut.pc==23 && dut.re_valid && rejections==0)begin force dut.re_cost=64'h7fefffffffffffff;force_active=1;end
  if(mode==12 && dut.pc==6 && dut.j_valid && dut.j_ready)begin force dut.j_row=8'd255;force_active=1;end
 end
 task launch;begin @(negedge clk);check(cmd_ready,"ready for new LM");reads=0;solves=0;rejections=0;evals=0;last_outer=0;last_pc=-1;before_trial=initial_state;cmd_state=initial_state;cmd_valid=1;
 @(negedge clk);cmd_valid=0;cmd_state=~initial_state;end endtask
 task await_response;begin cycles=0;while(!rsp_valid && cycles<250000000)begin @(negedge clk);cycles=cycles+1;end
 check(rsp_valid,"timeout");if(!rsp_valid)$fatal(1,"LM timeout pc=%0d",dut.pc);
 $fdisplay(report,"case=%0d cycles=%0d outer=%0d accepted=%0d solves=%0d rejected=%0d evals=%0d cost=%.17g status=%0d converged=%0d",cases,cycles,outer,accepted,solves,rejections,evals,$bitstoreal(cost),rsp_status,converged);$fflush(report);end endtask
 task consume;begin result_snapshot=result;saved_cost=cost;saved_accepted=accepted;saved_outer=outer;saved_status=rsp_status;saved_converged=converged;
 repeat(9)begin @(negedge clk);check(rsp_valid && !cmd_ready && result===result_snapshot && cost===saved_cost && accepted===saved_accepted && outer===saved_outer && rsp_status===saved_status && converged===saved_converged,"response backpressure");end
 rsp_ready=1;@(negedge clk);rsp_ready=0;cases=cases+1;end endtask
 task reset_dut;begin @(negedge clk);rst_n=0;cmd_valid=0;rsp_ready=0;repeat(3)@(negedge clk);check(!rd_en && !rsp_valid,"reset valid low");rst_n=1;@(negedge clk);end endtask
 initial begin
 report=$fopen("lm_controller_results.txt","w");fd=$fopen("../lm_controller_vectors.txt","r");if(!fd || !report)$fatal(1,"file open");reset_dut();
 while(!$feof(fd))begin rc=$fscanf(fd,"%d %d %d %h %h %h %h\n",stage,expected_converged,expected_accepted,expected_cost,initial_state,points,expected);
 if(rc==7)begin
  launch();await_response();check(rsp_status==0 && converged==expected_converged && accepted==expected_accepted,"C++ convergence and accepted count");
  check(outer>=accepted && outer<=150 && solves>=accepted,"iteration accounting");
  for(i=0;i<27;i=i+1)near(result[i*64+:64],expected[i*64+:64],2e-7);
  near(cost,expected_cost,2e-12);check(result[8*64+:64]===initial_state[8*64+:64],"k3 invariant");consume();
 end else if(rc!=-1)$fatal(1,"vector format");end
 $fclose(fd);fd=$fopen("../lm_controller_vectors.txt","r");rc=$fscanf(fd,"%d %d %d %h %h %h %h\n",stage,expected_converged,expected_accepted,expected_cost,initial_state,points,expected);$fclose(fd);
 width=1;launch();await_response();check(rsp_status==1 && accepted==0 && outer==0 && result===initial_state,"bad dimensions");consume();width=640;
 stage=3;launch();await_response();check(rsp_status==1,"bad stage");consume();stage=0;
 initial_state[0+:64]=64'h7ff8000000000000;launch();await_response();check(rsp_status==0 && !converged && cost==64'h7ff0000000000000 && outer==0,"invalid baseline is unconverged");consume();initial_state[0+:64]=expected[0+:64];
 drop=0;launch();await_response();check(rsp_status==5 && accepted==0,"missing baseline read");consume();drop=120;
 launch();await_response();check(rsp_status==5 && !converged && accepted==0 && result===initial_state,"missing perturbation read propagates and cleans up");consume();drop=-1;
 mode=1;launch();await_response();check(rsp_status==0 && !converged && accepted==0 && result===initial_state,"invalid Jacobian aborts normal and preserves current");consume();mode=0;
 mode=2;launch();await_response();check(rsp_status==0 && !converged && accepted==0 && solves==16 && rejections==16 && result===initial_state,"16 rejected trials");consume();mode=0;
 mode=4;launch();await_response();check(rsp_status==0 && !converged && accepted==0 && solves==16,"16 failed linear solves");consume();mode=0;
 mode=5;launch();await_response();check(rsp_status==5 && result===initial_state,"normal failure cleanup");consume();mode=0;
 mode=6;launch();await_response();check(rsp_status==1 && result===initial_state,"damped load failure cleanup");consume();mode=0;
 mode=7;launch();await_response();check(rsp_status==0 && converged && accepted==0 && solves==0,"strict gradient stop");consume();mode=0;
 mode=8;launch();await_response();check(rsp_status==0 && converged && accepted==0 && solves==16,"relaxed gradient after 16 failures");consume();mode=0;
 mode=9;launch();await_response();check(rsp_status==0 && !converged && accepted==1 && outer==150,"outer limit is not convergence");consume();mode=0;
 mode=10;launch();await_response();check(rsp_status==0 && converged && accepted==1,"small relative step stop");consume();mode=0;
 mode=11;launch();await_response();check(rsp_status==0 && converged && accepted>0 && rejections==1 && solves==accepted+1,"rejected trial then accepted and continued");consume();mode=0;
 mode=12;launch();await_response();check(rsp_status==1 && accepted==0 && result===initial_state,"early normal input error cancels blocked Jacobian");consume();mode=0;
 launch();repeat(100)@(negedge clk);reset_dut();check(cmd_ready && !rsp_valid,"mid evaluation reset");cases=cases+1;
 mode=3;launch();await_response();check(rsp_status==0 && converged && accepted==0 && outer==0,"cost threshold and recovery");consume();mode=0;
 $fdisplay(report,"RESULT cases=%0d errors=%0d",cases,errors);$fclose(report);done=1;
 end
endmodule
