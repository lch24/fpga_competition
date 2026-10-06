`timescale 1ns/1ps
module tb_add_pipeline #(parameter BITS=64, COUNT=12000, OP=0);
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,ce=1,in_valid=0,out_ready=0;
 wire in_ready,out_valid;reg [BITS-1:0] in_a,in_b;wire [BITS-1:0] out_r;
 reg [3*BITS-1:0] vectors[0:COUNT-1];
 integer sent=0,checked=0,ticks=0;reg finished=0;
 generate if(OP==0)begin
 fp_add_pipeline #(.BITS(BITS),.USE_CE(1)) dut(.*);
 end else if(OP==1)begin
 fp32_div #(.USE_CE(1)) dut(.*);
 end else if(OP==2)begin
 fp32_log #(.USE_CE(1)) dut(.in_x(in_a),.*);
 end else begin
 fp64_sub #(.USE_CE(1)) dut(.*);
 end endgenerate
 always @(posedge clk)if(rst_n&&ce)begin
   if(in_valid&&in_ready)sent=sent+1;
   if(out_valid&&out_ready)begin
     if(out_r!==vectors[checked][BITS-1:0])
       $fatal(1,"case %0d got %h expected %h operands %h",checked,out_r,vectors[checked][BITS-1:0],vectors[checked][3*BITS-1:BITS]);
     checked=checked+1;
   end
 end
 initial begin
   $readmemh("add_vectors.hex",vectors);
   repeat(3)@(negedge clk);rst_n=1;
   // Fill pipeline and FIFO before reset, cancel all outstanding work.
   in_valid=1;in_a=0;in_b=0;
   repeat(45)@(negedge clk);
   rst_n=0;in_valid=0;repeat(2)@(negedge clk);
   sent=0;checked=0;rst_n=1;
   while(checked<COUNT)begin
     ticks=ticks+1;ce=(ticks%7!=0);
     out_ready=(ticks%113>50); // long enough to fill output queue
     in_valid=sent<COUNT;
     if(in_valid){in_a,in_b}=vectors[sent][3*BITS-1:BITS];
     @(negedge clk);
     if(ticks>COUNT*((OP==0)?6:1000))$fatal(1,"timeout");
   end
   finished=1;$display("PASS add bits=%0d checked=%0d cycles=%0d",BITS,checked,ticks);$finish;
 end
endmodule
