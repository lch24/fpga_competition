`timescale 1ns/1ps
module tb_video_scale;
 reg clk=0,rst=0,iv=0,ready=0,first=0,last=0;reg [15:0] pixel=0;
 always #4 clk=~clk;
 wire ir,ov,of,ol,error;wire [15:0] op;wire [23:0] frames;wire [1:0] phase;
 localparam W=12,H=6,OW=16,OH=8;
 video_scale_4_3 #(.WIDTH(W),.HEIGHT(H)) dut(.clk(clk),.rst_n(rst),.in_valid(iv),.in_ready(ir),.in_pixel(pixel),.in_first(first),.in_last(last),
 .out_valid(ov),.out_ready(ready),.out_pixel(op),.out_first(of),.out_last(ol),.error(error),.frames(frames),.phase(phase));
 function [15:0] src(input integer x,y,f);src=(x*4129+y*783+f*17291)^16'hcaf1;endfunction
 function integer channel(input [15:0] p,input integer c);
  case(c)0:channel={p[4:0],p[4:2]};1:channel={p[10:5],p[10:9]};2:channel={p[15:11],p[15:13]};endcase
 endfunction
 function [15:0] expected(input integer x,y,f);
  integer a,b,c,d,u,v,k,s;reg [23:0] rgb;
  begin a=x*3/4;b=y*3/4;c=a<W-1?a+1:a;d=b<H-1?b+1:b;u=x*3%4;v=y*3%4;
   for(k=0;k<3;k=k+1)begin
    s=((4-u)*(4-v)*channel(src(a,b,f),k)+u*(4-v)*channel(src(c,b,f),k)+
       (4-u)*v*channel(src(a,d,f),k)+u*v*channel(src(c,d,f),k)+8)/16;
    rgb[k*8+:8]=s;
   end
   expected={rgb[23:19],rgb[15:10],rgb[7:3]};
  end
 endfunction
 integer input_count=0,output_count=0,cycles=0,trial;
 reg held=0;reg [17:0] held_pixel;
 always @(posedge clk)if(rst)begin
  cycles<=cycles+1;
  if(held&&(!ov||{of,ol,op}!==held_pixel))$fatal(1,"scaler changed stalled output");
  held<=ov&&!ready;held_pixel<={of,ol,op};
  if(iv&&ir)input_count<=input_count+1;
  if(ov&&ready)begin
   if(op!==expected(output_count%OW,(output_count/OW)%OH,output_count/(OW*OH)) ||
      of!==(output_count%(OW*OH)==0)||ol!==(output_count%(OW*OH)==OW*OH-1))$fatal(1,"scale pixel %0d got %h",output_count,op);
   output_count<=output_count+1;
  end
 end else held<=0;
 always @(negedge clk)if(rst)begin
  ready=cycles%11<7;
  iv=input_count<W*H*3 && cycles%5!=0;
  pixel=src(input_count%W,(input_count/W)%H,input_count/(W*H));
  first=input_count%(W*H)==0;last=input_count%(W*H)==W*H-1;
 end else begin iv=0;ready=0;end
 initial begin
  repeat(4)@(negedge clk);rst=1;
  wait(output_count==OW*OH*3);@(negedge clk);
  if(error||frames!=3||input_count!=W*H*3)$fatal(1,"scale counts/error");
  // Reset while either filling or drawing must discard all partial frame state.
  for(trial=0;trial<2;trial=trial+1)begin
   rst=0;repeat(3)@(negedge clk);input_count=0;output_count=0;rst=1;
   if(trial==0)wait(input_count==W+2);else wait(output_count==7);
  end
  rst=0;repeat(3)@(negedge clk);input_count=0;output_count=0;rst=1;
  wait(output_count==OW*OH*3);@(negedge clk);if(error||frames!=3)$fatal(1,"scale reset recovery");
  $display("PASS scaler quarter-phase bilinear pixels, border, tags, random stalls, consecutive frames, reset recovery");$finish;
 end
 initial begin #200000;$fatal(1,"scaler timeout");end
endmodule
