`timescale 1ns/1ps
// Independent refinement vectors, CE stalls, concurrent arithmetic client,
// and local client cancellation while the shared arithmetic core stays alive.
module tb_feature_program #(parameter COUNT=616);
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,client_rst_n=0,ce=1,start=0;
 reg [63:0] in_a,in_b,in_c,in_bx,in_by;
 reg [31:0] in_x,in_y;
 wire busy,done,out_ok;wire [63:0] out_dx,out_dy,out_convergence;wire [31:0] out_x,out_y;
 wire mv,mr,ma,rv,rr;wire [4:0] mo;wire [63:0] aa,ab,result;wire [4:0] flags;
 wire [5:0] req_ready,rsp_valid;
 reg other_request=0,other_ready=0;reg [1:0] other_state=0;
 integer other_checked=0,other_delay=0,ticks=0,checked=0,errors=0,i,cycles,resets=0,max_cycles=0;
 reg finished=0;
 feature_program #(.USE_CE(1),.FP_SHARED(1)) dut(
  .clk(clk),.rst_n(client_rst_n),.ce(ce),.start(start),.refine(1'b1),
  .in_a(in_a),.in_b(in_b),.in_c(in_c),.in_bx(in_bx),.in_by(in_by),.in_x(in_x),.in_y(in_y),
  .busy(busy),.done(done),.out_ok(out_ok),.out_dx(out_dx),.out_dy(out_dy),
  .out_x(out_x),.out_y(out_y),.out_convergence(out_convergence),
  .math_req_valid(mv),.math_req_ready(mr),.math_req_op(mo),.math_req_a(aa),.math_req_b(ab),.math_active(ma),
  .math_rsp_valid(rv),.math_rsp_ready(rr),.math_result(result),.math_flags(flags));
 // Match the board's slot 5, behind the higher-priority calibration clients.
 assign mr=req_ready[5];assign rv=rsp_valid[5];
 fp_calibration_pool #(.CLIENTS(6)) pool(.clk(clk),.rst_n(rst_n),
  .c_req_valid({mv,4'd0,other_request}),.c_req_ready(req_ready),.c_req_op({mo,20'd0,5'd0}),
  .c_req_a({aa,256'd0,64'h3ff0000000000000}),.c_req_b({ab,256'd0,64'h4000000000000000}),
  .c_active({ma,4'd0,rst_n}),.c_rsp_valid(rsp_valid),.c_rsp_ready({rr,4'd0,other_ready}),.result(result),.flags(flags));
 always @(negedge clk)begin ticks=ticks+1;ce=(ticks%7!=0 && ticks%11!=0);end
 always @(posedge clk)if(rst_n)begin
  case(other_state)
   0:begin other_request<=1;other_ready<=0;other_state<=1;end
   1:if(req_ready[0])begin other_request<=0;other_delay<=0;other_state<=2;end
   2:if(rsp_valid[0])begin
    if(result!==64'h4008000000000000 || flags!==0)begin errors=errors+1;$fatal(1,"wrong response owner");end
    if(other_delay==3)begin other_ready<=1;other_state<=3;end else other_delay<=other_delay+1;
   end
   3:if(rsp_valid[0] && other_ready)begin other_ready<=0;other_checked<=other_checked+1;other_state<=0;end
  endcase
 end
 reg [643:0] vectors[0:COUNT-1];
 reg [3:0] expected_ok;reg [63:0] ex_dx,ex_dy,ex_conv;reg [31:0] ex_x,ex_y;
 task load_case;
  input integer index;
  begin {expected_ok,in_a,in_b,in_c,in_bx,in_by,in_x,in_y,ex_dx,ex_dy,ex_x,ex_y,ex_conv}=vectors[index];end
 endtask
 task launch;
  begin
   @(negedge clk);#1;start=1;
   @(posedge clk);while(!ce)@(posedge clk);
   @(negedge clk);#1;start=0;
   in_a=0;in_b=0;in_c=0;in_bx=0;in_by=0;in_x=0;in_y=0;
  end
 endtask
 initial begin
  $readmemh("vectors.hex",vectors);
  repeat(4)@(negedge clk);#1;rst_n=1;client_rst_n=1;
  for(i=0;i<5;i=i+1)begin
   load_case(3);launch();repeat(10+80*i)@(negedge clk);#1;client_rst_n=0;
   repeat(3)@(negedge clk);#1;client_rst_n=1;
   if(done || busy)$fatal(1,"client reset failed");
   repeat(150)begin @(negedge clk);#1;if(done)$fatal(1,"cancelled result leaked");end
   resets=resets+1;
  end
  for(i=0;i<COUNT;i=i+1)begin
   load_case(i);launch();cycles=0;
   while(!done)begin @(negedge clk);#1;cycles=cycles+1;if(cycles>10000)$fatal(1,"feature timeout");end
   if(out_ok!==expected_ok[0] || busy)$fatal(1,"validity case %0d",i);
   if(out_ok && (out_dx!==ex_dx || out_dy!==ex_dy || out_x!==ex_x || out_y!==ex_y || out_convergence!==ex_conv))
    $fatal(1,"feature numeric case %0d",i);
   checked=checked+1;if(cycles>max_cycles)max_cycles=cycles;
   repeat(3)@(negedge clk);
  end
  finished=1;$finish;
 end
 initial begin #100000000;$fatal(1,"watchdog");end
endmodule
