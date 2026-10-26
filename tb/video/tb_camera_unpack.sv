`timescale 1ns/1ps
module tb_camera_unpack;
 reg clk=0,rst=0,vs=0,href=0;reg [7:0] data=0;
 always #5 clk=~clk;
 wire valid;wire [15:0] pixel;wire [5:0] bad;
 camera_byte_unpack #(.WIDTH(8)) dut(.pclk(clk),.rst_n(rst),.vsync(vs),.href(href),.data(data),.pixel_valid(valid),.pixel(pixel),.line_bad(bad),.capture_vsync());
 integer n=0;reg check=0;
 function [15:0] swap(input [15:0] v);swap={v[4:0],v[10:5],v[15:11]};endfunction
 always @(negedge clk)if(check&&valid)begin
  if(pixel!==swap(16'h1200+n))$fatal(1,"next-line pixel %0d got %h",n,pixel);
  n=n+1;
 end
 task byte_in(input [7:0] v);begin @(negedge clk);#1;href=1;data=v;end endtask
 integer i;
 initial begin
  repeat(3)@(negedge clk);#1;rst=1;
  // A severely short line with only one blank clock: padding MUST yield to
  // the next line's first byte. The broken implementation loses that byte.
  byte_in(8'hab);byte_in(8'hcd);
  @(negedge clk);#1;href=0;
  byte_in(8'h12);
  @(negedge clk);#1;data=0;
  // First new-line pixel emerges after the input and byte-pair registers.
  @(negedge clk);#1;data=8'h12;check=1;
  for(i=1;i<8;i=i+1)begin
   byte_in(i);
   if(i<7)byte_in(8'h12);
  end
  @(negedge clk);#1;href=0;
  repeat(12)@(negedge clk);
  #1;vs=1;repeat(4)@(negedge clk);
  if(n!=8||bad!=1)$fatal(1,"count=%0d bad=%0d",n,bad);
  $display("PASS camera unpack: short-line recovery preserves every next-line byte");$finish;
 end
 initial begin #10000;$fatal(1,"timeout");end
endmodule
