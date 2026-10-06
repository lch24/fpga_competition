`timescale 1ns/1ps
// Independent IEEE-754 boundary vectors: cancellation, subnormal and overflow.
module tb_fp64_add_bounds;
 reg clk=0; always #5 clk=~clk;
 reg rst_n=0,iv=0,ordy=0;
 reg [63:0] a=0,b=0;
 wire ir,ov;wire [63:0] result;
 integer checks=0,errors=0;
 reg done=0;
 fp64_add dut(.clk(clk),.rst_n(rst_n),.in_valid(iv),.in_ready(ir),
 .in_a(a),.in_b(b),.out_valid(ov),.out_ready(ordy),.out_r(result));
 task check;
 input [63:0] x,y,expected;
 begin
   @(negedge clk);a=x;b=y;iv=1;
   do @(posedge clk);while(!ir);
   @(negedge clk);iv=0;
   while(!ov)@(negedge clk);
   repeat(3)begin
     if(result!==expected || !ov)begin
       $display("FAIL ADD %h + %h: got %h expected %h",x,y,result,expected);
       errors=errors+1;
     end
     @(negedge clk);
   end
   ordy=1;@(negedge clk);ordy=0;checks=checks+1;
 end endtask
 initial begin
   repeat(4)@(negedge clk);rst_n=1;
   check(64'h0010000000000001,64'h8010000000000000,64'h0000000000000001);
   check(64'h0010000000000000,64'h800fffffffffffff,64'h0000000000000001);
   check(64'h0000000000000001,64'h0000000000000001,64'h0000000000000002);
   check(64'h000fffffffffffff,64'h0000000000000001,64'h0010000000000000);
   check(64'h8000000000000001,64'h8000000000000001,64'h8000000000000002);
   check(64'h0010000000000000,64'h8010000000000000,64'h0000000000000000);
   check(64'h3ff0000000000000,64'h0000000000000001,64'h3ff0000000000000);
   check(64'h3ff0000000000000,64'h3ca0000000000000,64'h3ff0000000000000);
   check(64'h3ff0000000000001,64'h3ca0000000000000,64'h3ff0000000000002);
   check(64'h7fefffffffffffff,64'h7fefffffffffffff,64'h7ff0000000000000);
   check(64'h8000000000000000,64'h8000000000000000,64'h8000000000000000);
   check(64'h0000000000000000,64'h8000000000000000,64'h0000000000000000);
   done=1;
   if(errors==0)$display("PASS tb_fp64_add_bounds checks=%0d",checks);
   else $display("FAIL tb_fp64_add_bounds errors=%0d",errors);
   $finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
