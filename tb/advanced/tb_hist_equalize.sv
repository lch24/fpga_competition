`timescale 1ns/1ps
// Focused mode-6 checks: LUT build from the histogram, dark-range stretch,
// flat-scene guard, monotonic mapping and disabled-frame transport.
module tb_hist_equalize;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,iv=0,first=0,last=0,bounds=1,enable=1;reg [2:0] mode=6;reg [15:0] ip;
 wire ir,ov;wire [15:0] op;wire [2:0] auto_curve;
 video_adaptive_gamma dut(.clk(clk),.rst_n(rst_n),.in_valid(iv),.in_ready(ir),.in_pixel(ip),.in_first(first),.in_last(last),.in_bounds(bounds),
  .enable(enable),.mode(mode),.out_valid(ov),.out_ready(1'b1),.out_pixel(op),.auto_curve(auto_curve));
 integer put=0,get=0;reg collect=0;reg [15:0] outbuf[0:255];reg [15:0] inbuf[0:255];
 always @(posedge clk)if(ov&&collect&&get<256)begin outbuf[get]<=op;get<=get+1;end
 always @(posedge clk)if(iv&&ir&&collect&&put<256)begin inbuf[put]<=ip;put<=put+1;end
 task send(input [15:0] p,input integer f,l,b);
  begin @(negedge clk);ip=p;first=f;last=l;bounds=b;iv=1;do @(posedge clk);while(!ir);@(negedge clk);iv=0;end
 endtask
 task ramp_frame(input integer lo);
  integer k;reg [4:0] r;reg [5:0] g;reg [15:0] p;
  begin
   for(k=0;k<64;k=k+1)begin r=(lo+k)>>3;g=(lo+k)>>2;p={r,g,r};send(p,k==0,k==63,1);end
  end
 endtask
 task flat_frame(input [15:0] p);
  integer k;begin for(k=0;k<64;k=k+1)send(p,k==0,k==63,1);end
 endtask
 task wait_build;
  begin
   do @(posedge clk);while(!dut.pending_build&&!dut.build_busy);
   do @(posedge clk);while(dut.build_busy);
   repeat(4)@(posedge clk);while(ov)@(posedge clk);
  end
 endtask
 integer i,k;reg [15:0] want;reg [4:0] r5,b5;reg [5:0] g6;
 initial begin
  repeat(5)@(negedge clk);rst_n=1;
  while(dut.clearing)@(negedge clk);
  // Dark ramp: first frame only observes, build makes the map.
  ramp_frame(20);
  wait_build();
  if(!dut.eq_apply)$fatal(1,"dark ramp should build a map");
  // Second frame must use the equalized bank and match eq_rb/eq_g.
  put=0;get=0;collect=1;ramp_frame(20);repeat(4)@(posedge clk);collect=0;
  if(get!=64)$fatal(1,"collected %0d pixels",get);
  for(i=0;i<64;i=i+1)begin
   r5=inbuf[i][15:11];g6=inbuf[i][10:5];b5=inbuf[i][4:0];
   want={dut.eq_rb[r5],dut.eq_g[g6],dut.eq_rb[b5]};
   if(outbuf[i]!==want)$fatal(1,"eq pixel %0d got %h expected %h",i,outbuf[i],want);
   if(i>0&&outbuf[i][15:11]<outbuf[i-1][15:11])$fatal(1,"non-monotonic red at %0d",i);
  end
  if(outbuf[63][15:11]<=inbuf[63][15:11])$fatal(1,"dark red not lifted");
  if(outbuf[63][15:11]<=outbuf[0][15:11]+3)$fatal(1,"dark range not stretched");
  // Flat gray frame: spread below the guard and the map is bypassed.
  flat_frame(16'h0841);wait_build();
  if(dut.eq_apply)$fatal(1,"flat frame should not equalize");
  put=0;get=0;collect=1;flat_frame(16'h0841);repeat(4)@(posedge clk);collect=0;
  for(i=0;i<64;i=i+1)if(outbuf[i]!==16'h0841)$fatal(1,"flat pixel %0d got %h",i,outbuf[i]);
  // Disabled frame is pure transport and must not rebuild the map.
  enable=0;put=0;get=0;collect=1;
  for(k=0;k<64;k=k+1)begin send(16'(16'h4000+16'(k*257)),k==0,k==63,1);end
  repeat(4)@(posedge clk);collect=0;
  for(i=0;i<64;i=i+1)if(outbuf[i]!==16'(16'h4000+16'(i*257)))$fatal(1,"disabled pixel %0d got %h",i,outbuf[i]);
  $display("PASS histogram equalize map build/stretch/monotonic/flat guard/disabled transport pixels=%0d",get);
  $finish;
 end
 initial begin #4000000;$fatal(1,"equalize timeout");end
endmodule
