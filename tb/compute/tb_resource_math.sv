`timescale 1ns/1ps
module tb_resource_math;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,ce=1,in_valid=0,out_ready=0;
 reg [63:0] in_a=0,in_b=0;
 wire in_ready,out_valid;wire [63:0] out_r;
 fp64_div #(.USE_CE(1)) divider(.*);
 reg mul_valid=0,mul_ready_out=0;wire mul_ready,mul_valid_out;wire [63:0] mul_result;
 fp64_mul #(.USE_CE(1)) multiplier(.clk(clk),.rst_n(rst_n),.ce(ce),
  .in_valid(mul_valid),.in_ready(mul_ready),.in_a(in_a),.in_b(in_b),
  .out_valid(mul_valid_out),.out_ready(mul_ready_out),.out_r(mul_result));
 reg req_valid=0,rsp_ready=0;reg [2:0] req_op=0;
 reg [31:0] req_a=0,req_b=0;wire req_ready,rsp_valid,rsp_error;wire [31:0] rsp_result;
 fp32_service fp32(.*);
 integer checked=0,div_checked=0,mul_checked=0,reset_checked=0,fd,scan,i,ticks;
 reg finished=0;
 reg [63:0] a,b,expected;
 reg [99:0] vec[0:2499];
 reg [31:0] expected32;
 task reset_core;
  begin
   @(negedge clk);rst_n=0;in_valid=0;out_ready=0;ce=0;
   repeat(2)@(negedge clk);rst_n=1;ce=1;
   if(out_valid)$fatal(1,"reset leaked divider response");
  end
 endtask
 initial begin
  reset_core();
  for(i=0;i<3;i=i+1)begin
   in_a=64'h4000000000000000;in_b=64'h4008000000000000;in_valid=1;
   @(negedge clk);in_valid=0;repeat(1+25*i)@(negedge clk);
   reset_core();reset_checked=reset_checked+1;
   repeat(60)begin @(negedge clk);if(out_valid)$fatal(1,"cancelled response");end
  end
  fd=$fopen("division_vectors.txt","r");if(!fd)$fatal(1,"missing division vectors");
  while(!$feof(fd))begin
   scan=$fscanf(fd,"%h %h %h\n",a,b,expected);
   if(scan==3)begin
    @(negedge clk);ce=1;in_a=a;in_b=b;in_valid=1;
    do @(posedge clk);while(!in_ready);
    @(negedge clk);in_valid=0;ticks=0;
    while(!out_valid)begin
     ce=(ticks%5!=0);@(negedge clk);ticks=ticks+1;
     if(ticks>100)$fatal(1,"divider latency");
    end
    if(out_r!==expected)$fatal(1,"DIV %h / %h got %h expected %h",a,b,out_r,expected);
    repeat(3)begin @(negedge clk);if(!out_valid||out_r!==expected)$fatal(1,"divider hold");end
    ce=1;out_ready=1;@(negedge clk);out_ready=0;div_checked=div_checked+1;
   end
  end
  $fclose(fd);
  fd=$fopen("multiply_vectors.txt","r");if(!fd)$fatal(1,"missing multiply vectors");
  while(!$feof(fd))begin
   scan=$fscanf(fd,"%h %h %h\n",a,b,expected);
   if(scan==3)begin
    @(negedge clk);ce=1;in_a=a;in_b=b;mul_valid=1;
    do @(posedge clk);while(!mul_ready);
    @(negedge clk);mul_valid=0;ticks=0;
    while(!mul_valid_out)begin
     ce=(ticks%4!=0);@(negedge clk);ticks=ticks+1;
     if(ticks>60)$fatal(1,"multiply latency");
    end
    if(mul_result!==expected)$fatal(1,"MUL %h * %h got %h expected %h",a,b,mul_result,expected);
    repeat(3)begin @(negedge clk);if(!mul_valid_out||mul_result!==expected)$fatal(1,"multiply hold");end
    ce=1;mul_ready_out=1;@(negedge clk);mul_ready_out=0;mul_checked=mul_checked+1;
   end
  end
  $fclose(fd);
  $readmemh("fp_ops.mem",vec);
  for(i=0;i<2500;i=i+1)begin
   @(negedge clk);req_op=vec[i][98:96];req_a=vec[i][95:64];req_b=vec[i][63:32];expected32=vec[i][31:0];req_valid=1;
   do @(posedge clk);while(!req_ready);
   @(negedge clk);req_valid=0;
   wait(rsp_valid);repeat(i%3)@(negedge clk);
   if(rsp_result!==expected32 || rsp_error!==(expected32[30:23]==255))$fatal(1,"FP32 vector %0d got %h expected %h",i,rsp_result,expected32);
   rsp_ready=1;@(negedge clk);rsp_ready=0;checked=checked+1;
  end
  finished=1;$finish;
 end
 initial begin #20000000;$fatal(1,"math timeout");end
endmodule
