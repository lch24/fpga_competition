`timescale 1ns/1ps
`include "calib_defs.vh"
// Differential check of the public FP32 interface against the former wide
// combinational reference. Independent stored numerical vectors run separately.
module tb_fp32_basic_pipeline;
 `include "fp_bits.vh"
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,req_valid=0,rsp_ready=0;
 reg [4:0] op=0;
 reg [31:0] a=0,b=0;
 wire req_ready,rsp_valid,less,equal,unordered;
 wire [31:0] result;wire [4:0] flags;
 fp_operator #(.FP_W(32),.ENABLE_EXP(0),.ENABLE_LOG(0),
   .ENABLE_SINCOS(0),.ENABLE_ATAN_ACOS(0)) dut(
   .clk(clk),.rst_n(rst_n),.req_valid(req_valid),.req_ready(req_ready),
   .req_op(op),.req_a(a),.req_b(b),.rsp_valid(rsp_valid),.rsp_ready(rsp_ready),
   .rsp_result(result),.rsp_flags(flags),.rsp_less(less),.rsp_equal(equal),.rsp_unordered(unordered));
 integer checks=0,errors=0,max_cycles=0,total_cycles=0;
 reg done=0;
 reg [31:0] edges[0:19];reg [31:0] rng=32'h41c6ce57;
 integer i,j,k,n;
 task random_word(output reg [31:0] value);
  begin rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);value=rng;end
 endtask
 reg [31:0] x,y;
 task check_case(input [4:0] operation,input [31:0] x,y);
  reg [68:0] expected;reg [36:0] held;
  reg nan_x,nan_y,zero_x,zero_y,exp_less,exp_equal;
  integer cycles,h;
  begin
   expected=fp_eval(operation,{32'd0,x},{32'd0,y},32);
   @(negedge clk);while(!req_ready)@(negedge clk);
   op=operation;a=x;b=y;req_valid=1;
   @(negedge clk);req_valid=0;a=32'hdeadbeef;b=32'hbadc0ffe;op=31;
   cycles=0;
   while(!rsp_valid && cycles<128)begin @(negedge clk);cycles=cycles+1;end
   if(!rsp_valid || {flags,result}!=={expected[68:64],expected[31:0]})begin
    errors=errors+1;
    $display("FAIL op=%d a=%h b=%h result=%h flags=%h expected=%h cycles=%d",operation,x,y,result,flags,expected,cycles);
   end
   if(operation==`PAR_FP_COMPARE)begin
    nan_x=(&x[30:23]) && |x[22:0];nan_y=(&y[30:23]) && |y[22:0];
    zero_x=!(|x[30:0]);zero_y=!(|y[30:0]);
    exp_equal=!(nan_x||nan_y) && (x==y || (zero_x&&zero_y));
    exp_less=!(nan_x||nan_y) && !(zero_x&&zero_y) &&
             ((x[31]!=y[31])?x[31]:(x[31]?(x>y):(x<y)));
    if({unordered,equal,less}!=={nan_x||nan_y,exp_equal,exp_less})errors=errors+1;
   end
   held={flags,result};
   for(h=0;h<(checks%4);h=h+1)begin
    @(negedge clk);
    if(!rsp_valid || req_ready || {flags,result}!==held)errors=errors+1;
   end
   rsp_ready=1;@(negedge clk);rsp_ready=0;
   if(cycles>max_cycles)max_cycles=cycles;
   total_cycles=total_cycles+cycles;checks=checks+1;
   if(errors!=0)$fatal(1,"FP32 differential mismatch");
  end
 endtask
 initial begin
  edges[0]=0;edges[1]=32'h80000000;edges[2]=1;edges[3]=32'h80000001;
  edges[4]=32'h007fffff;edges[5]=32'h00800000;edges[6]=32'h00800001;
  edges[7]=32'h3f800000;edges[8]=32'hbf800000;edges[9]=32'h3f800001;
  edges[10]=32'h3f7fffff;edges[11]=32'h7f7fffff;edges[12]=32'hff7fffff;
  edges[13]=32'h7f800000;edges[14]=32'hff800000;edges[15]=32'h7fc00001;
  edges[16]=32'h7f800001;edges[17]=32'h33800000;edges[18]=32'h33000000;
  edges[19]=32'h80800000;
  repeat(3)@(negedge clk);rst_n=1;
  for(k=0;k<4;k=k+1)for(i=0;i<20;i=i+1)for(j=0;j<20;j=j+1)
   check_case(k==3?`PAR_FP_COMPARE:k,edges[i],edges[j]);
  for(n=0;n<4096;n=n+1)begin
   random_word(x);random_word(y);
   for(k=0;k<3;k=k+1)check_case(k,x,y);
   // Near cancellation and half-ULP ties are sparse in uniform bit patterns.
   check_case(`PAR_FP_SUB,{1'b0,x[30:0]},{1'b0,x[30:0]}^32'd1);
  end
  check_case(`PAR_FP_F32_TO_F64,32'h3f800000,0);
  check_case(`PAR_FP_F64_TO_F32,32'h3f800000,0);
  // Reset abort at every stage, including held response and subnormal loops.
  for(n=0;n<64;n=n+1)begin
   @(negedge clk);while(!req_ready)@(negedge clk);
   op=`PAR_FP_MUL;a=1;b=32'h3f800000;req_valid=1;
   @(negedge clk);req_valid=0;
   repeat(n)@(negedge clk);
   rst_n=0;@(negedge clk);rst_n=1;
   repeat(70)begin @(negedge clk);if(rsp_valid || !req_ready)errors=errors+1;end
   check_case(`PAR_FP_ADD,32'h3f800000,32'h40000000);
  end
  done=1;$finish;
 end
 initial begin #100000000;$fatal(1,"watchdog");end
endmodule
