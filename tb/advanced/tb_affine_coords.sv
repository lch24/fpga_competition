`timescale 1ns/1ps
module tb_affine_coords;
 localparam W=8,H=6;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,start=0,ready=0;reg [191:0] matrix;
 wire valid;wire [63:0] descriptor;
 video_affine_coords #(.WIDTH(W),.HEIGHT(H)) dut(.clk(clk),.rst_n(rst_n),.start(start),.matrix(matrix),.out_valid(valid),.out_ready(ready),.descriptor(descriptor));
 integer test,i,t,x,y,ix,iy,j;reg signed [63:0] u,v;reg [3:0] mask;reg [63:0] expected,held;
 reg signed [31:0] a[0:5];reg signed [63:0] aa[0:5];
 initial begin
  repeat(3)@(negedge clk);rst_n=1;
  for(test=0;test<5;test=test+1)begin
   a[0]=65536;a[1]=0;a[2]=0;a[3]=0;a[4]=65536;a[5]=0;
   case(test)
    1:begin a[2]=-32768;a[5]=16384;end
    2:begin a[0]=0;a[1]=65536;a[3]=-65536;a[4]=0;a[5]=(H-1)*65536;end
    3:begin a[1]=12345;a[3]=-8711;a[2]=-16384;end
    4:begin a[0]=32'h7fffffff;a[1]=32'h80000000;a[2]=32'h7fffffff;end
   endcase
   for(j=0;j<6;j=j+1)begin matrix[j*32+:32]=a[j];aa[j]=$signed(a[j]);end
   @(negedge clk);start=1;@(negedge clk);start=0;
   i=0;t=0;
   while(i<W*H)begin
    ready=t%4!=0;t=t+1;
    x=i%W;y=i/W;
    u=aa[0]*x+aa[1]*y+aa[2];v=aa[3]*x+aa[4]*y+aa[5];
    ix=u>>>16;iy=v>>>16;
    mask[0]=ix>=0&&ix<W&&iy>=0&&iy<H;mask[1]=ix+1>=0&&ix+1<W&&iy>=0&&iy<H;
    mask[2]=ix>=0&&ix<W&&iy+1>=0&&iy+1<H;mask[3]=ix+1>=0&&ix+1<W&&iy+1>=0&&iy+1<H;
    expected=ix>=-1&&ix<W&&iy>=-1&&iy<H?{mask,14'(iy),14'(ix),v[15:0],u[15:0]}:64'b0;
    #1;if(!valid||descriptor!==expected)$fatal(1,"affine test=%0d pixel=%0d got=%h expected=%h",test,i,descriptor,expected);
    held=descriptor;@(posedge clk);#1;
    if(!ready&&descriptor!==held)$fatal(1,"affine changed under stall");
    if(ready)i=i+1;
    @(negedge clk);
   end
   if(valid)$fatal(1,"too many coordinates");
  end
  $display("PASS affine identity/negative-edge/90deg/shear/extreme/stalls");$finish;
 end
 initial begin #100000;$fatal(1,"affine timeout");end
endmodule
