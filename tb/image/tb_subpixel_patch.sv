`timescale 1ns/1ps
// Exercise the changed patch loader at several radii, independently of the
// numerical convergence loop. Full detector regression covers interpolation.
module tb_subpixel_patch;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,ce=1,start=0;reg [7:0] half_win=2;
 wire gray_rd_en;wire [13:0] gray_rd_addr;reg [7:0] gray_rd_data=0;
 integer ticks=0,requests=0,checked=0,cases=0,r,side,x,y,k;
 reg finished=0;
 subpixel_ctrl #(.IMG_W(128),.IMG_H(128),.GRAY_ADDR_W(14),.USE_CE(1)) dut(
  .clk(clk),.rst_n(rst_n),.ce(ce),.cfg_scale(3'd0),.start(start),.n_in(16'd1),.half_win(half_win),
  .pt_rd_x(32'h42800000),.pt_rd_y(32'h42800000),
  .gray_rd_en(gray_rd_en),.gray_rd_addr(gray_rd_addr),.gray_rd_data(gray_rd_data),.out_ready(1'b1));
 function [7:0] pixel(input integer px,py);pixel=(px*37)^(py*19);endfunction
 always @(negedge clk) begin ticks=ticks+1;ce=ticks%5!=0;end
 always @(posedge clk)if(rst_n&&ce&&gray_rd_en)begin
  if(gray_rd_addr!==((64-r-1+requests/side)*128+64-r-1+requests%side))
   $fatal(1,"patch raster r=%d request=%d addr=%d",r,requests,gray_rd_addr);
  gray_rd_data<=pixel(gray_rd_addr%128,gray_rd_addr/128);
  requests=requests+1;
 end
 task run_radius(input integer radius);
  begin
   @(negedge clk);rst_n=0;start=0;r=radius;side=2*r+4;requests=0;half_win=r;
   repeat(3)@(negedge clk);rst_n=1;start=1;
   do @(posedge clk);while(!ce);
   @(negedge clk);start=0;
   wait(dut.state==2 && dut.ist==2);@(negedge clk);
   if(requests!=side*side)$fatal(1,"patch request count");
   for(k=0;k<side*side;k=k+1)begin
    x=64-r-1+k%side;y=64-r-1+k/side;
    if(dut.patch[k]!==pixel(x,y))$fatal(1,"patch memory r=%d k=%d",r,k);
    checked=checked+1;
   end
   cases=cases+1;
  end
 endtask
 initial begin
  run_radius(2);run_radius(3);run_radius(7);run_radius(15);
  finished=1;$finish;
 end
 initial begin #2000000;$fatal(1,"patch loader timeout");end
endmodule
