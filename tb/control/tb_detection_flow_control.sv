`timescale 1ns/1ps
// Exercise layer fallback/mapping and the stream adapter independently of
// expensive pixel arithmetic. In particular, no output may replay while the
// instruction core fetches its next instruction after OUTPUT has completed.
module tb_detection_flow_control;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,ce=1,start=0,cfg_resp_dump_en=0,out_ready=0;
 wire busy,done,out_valid,out_grid_ok,nat_entry,ref_entry,seq_valid,seq_enter;
 wire [1:0] status,o_st;wire [3:0] stage;wire [7:0] dl,seq_id;wire [2:0] child_valid;
 wire [15:0] oc,rc,out_oc,out_total,seq_arg;wire [31:0] out_x,out_y;
 reg [31:0] corner_x=0,corner_y=0;
 reg [2:0] native_good,refine_good;
 wire order_valid=stage==2 && oc<6;
 reg [3:0] ref_count=0;
 wire refine_valid=seq_valid && seq_id==52 && !seq_enter && ref_count<6;
 wire refine_done=seq_valid && seq_id==52 && !seq_enter && ref_count==6;
 wire refine_ok=refine_good[dl];wire map_done=stage==4,resp_dump_done=stage==8;
 wire filter_done=1,order_done=1;
 wire grid_done=seq_id!=52 || ref_count==6;
 wire [31:0] filter_result=seq_id==19?32'h00010000:0;
 wire [31:0] order_result=(seq_id==32 || seq_id==34)?!native_good[dl]:0;
 wire [31:0] grid_result=0;
 detection_flow_control #(.USE_CE(1),.DEPTH(3),.CORNER_N(6)) dut(.*);
 integer ticks=0,t,cycles,count=0,checked=0,natives,maps,dumps;reg finished=0;
 reg previous_stall=0;reg [63:0] held;
 always @(negedge clk)begin ticks=ticks+1;ce=ticks%5!=0 && ticks%11!=0;out_ready=ticks%7<3;end
 always @(posedge clk)begin
  if(rst_n && previous_stall && {out_x,out_y}!==held)$fatal(1,"unstable output under backpressure");
  previous_stall<=rst_n && out_valid && !(ce && out_ready);held<={out_x,out_y};
  if(rst_n && ce)begin
   corner_x<=32'h42000000+out_oc;corner_y<=32'h43000000+out_oc;
   if(seq_enter && seq_id==52)ref_count<=0;
   else if(refine_valid)ref_count<=ref_count+1;
   if(seq_enter && seq_id==1)natives=natives+1;
   if(seq_enter && seq_id==4)maps=maps+1;
   if(seq_enter && seq_id==8)dumps=dumps+1;
   if(out_valid && out_ready)begin
    if(count>=6 || out_x!==32'h42000000+count || out_y!==32'h43000000+count)$fatal(1,"duplicate/reordered output %0d",count);
    count=count+1;
   end
  end
 end
 initial begin
  repeat(4)@(negedge clk);#1;rst_n=1;
  for(t=0;t<4;t=t+1)begin
   case(t)
    0:begin native_good=7;refine_good=7;end // native then two mapped layers
    1:begin native_good=3;refine_good=7;end // failed deepest, native middle, map
    2:begin native_good=7;refine_good=5;end // failed mapped middle, native finest
    3:begin native_good=0;refine_good=0;end // no valid layer
   endcase
   count=0;natives=0;maps=0;dumps=0;cfg_resp_dump_en=t==1;
   @(negedge clk);#1;start=1;@(posedge clk);while(!ce)@(posedge clk);
   @(negedge clk);#1;start=0;cycles=0;
   while(!done)begin @(negedge clk);#1;cycles=cycles+1;if(cycles>50000)$fatal(1,"flow watchdog");end
   if(status!==(t==3?2:1) || count!=(t==3?0:6))$fatal(1,"bad completion case %0d",t);
   if(natives!=(t==0?1:t==3?3:2) || maps!=(t==0?2:t==3?0:1) || dumps!=(t==1))$fatal(1,"wrong layer branch");
   repeat(15)@(negedge clk);checked=checked+1;
  end
  finished=1;$finish;
 end
endmodule
