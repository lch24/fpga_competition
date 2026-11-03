`timescale 1ns/1ps
module tb_adaptive_gamma;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,iv=0,first=0,last=0,bounds=1,enable=1,ordy=0;reg [2:0] mode=1;reg [15:0] ip;
 wire ir,ov;wire [15:0] op;wire [2:0] auto_curve;
 video_adaptive_gamma dut(.clk(clk),.rst_n(rst_n),.in_valid(iv),.in_ready(ir),.in_pixel(ip),.in_first(first),.in_last(last),.in_bounds(bounds),
  .enable(enable),.mode(mode),.out_valid(ov),.out_ready(ordy),.out_pixel(op),.auto_curve(auto_curve));
 reg [15:0] expected[0:100000];integer put=0,get=0,ticks=0,curve=0;reg stalled=0;reg [15:0] held;
 function [7:0] channel(input integer q,input integer bits,input integer c);
  real gamma,r;integer expanded,n;
  begin
   expanded=bits==5?(q<<3)+(q>>2):(q<<2)+(q>>4);
   case(c)1:gamma=0.75;2:gamma=0.5;3:gamma=1.3;4:gamma=2.0;default:gamma=1.0;endcase
   r=255.0*((expanded/255.0)**gamma);n=$rtoi($floor(r+0.5));channel=n;
  end
 endfunction
 function [15:0] reference(input [15:0] p,input integer c);
  reg [7:0] r,g,b;
  begin r=channel(p[15:11],5,c);g=channel(p[10:5],6,c);b=channel(p[4:0],5,c);reference={r[7:3],g[7:2],b[7:3]};end
 endfunction
 always @(posedge clk)if(rst_n)begin
  ticks<=ticks+1;
  if(stalled&&(!ov||op!==held))$fatal(1,"gamma changed under stall");
  stalled<=ov&&!ordy;held<=op;
  if(ov&&ordy)begin if(get>=put||op!==expected[get])$fatal(1,"gamma output %0d got %h expected %h",get,op,expected[get]);get=get+1;end
  if(iv&&ir)begin
   if(first)curve=mode==0?auto_curve:mode-1;
   expected[put]=enable?reference(ip,curve):ip;put=put+1;
  end
 end
 always @(negedge clk)ordy=ticks%5!=0;
 task send(input [15:0] p,input integer f,l,b);
  begin @(negedge clk);ip=p;first=f;last=l;bounds=b;iv=1;do @(posedge clk);while(!ir);@(negedge clk);iv=0;end
 endtask
 task frame(input [15:0] p,input integer valid_area);
  integer k;begin for(k=0;k<16;k=k+1)send(p,k==0,k==15,valid_area);end
 endtask
 integer m,k;
 initial begin
  repeat(3)@(negedge clk);rst_n=1;
  for(m=1;m<=5;m=m+1)begin mode=m;for(k=0;k<256;k=k+1)send(16'(k*257),k==0,k==255,1);end
  enable=0;mode=5;for(k=0;k<256;k=k+1)send(16'(k*197),k==0,k==255,1);
  enable=1;mode=0;
  frame(0,0);frame(0,0);frame(0,0);if(auto_curve!=0)$fatal(1,"padding biased brightness");
  frame(16'h0841,1);frame(16'h0841,1);if(auto_curve!=0)$fatal(1,"premature adaptation");
  frame(16'h0841,1);if(auto_curve!=2)$fatal(1,"dark adaptation");
  enable=0;frame(16'hffff,1);frame(16'hffff,1);frame(16'hffff,1);if(auto_curve!=2)$fatal(1,"disabled pass adapted");
  enable=1;frame(16'hffff,1);frame(16'hffff,1);frame(16'hffff,1);if(auto_curve!=4)$fatal(1,"bright adaptation");
  wait(get==put);repeat(3)@(negedge clk);
  $display("PASS gamma LUTs/identity/elastic stalls/frame consistency/auto hysteresis/padding exclusion pixels=%0d",get);$finish;
 end
 initial begin #2000000;$fatal(1,"gamma timeout");end
endmodule
