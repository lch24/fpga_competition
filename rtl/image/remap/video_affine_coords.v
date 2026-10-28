`timescale 1ns/1ps
// Output -> source inverse affine, signed Q16.16, row-major descriptors.
// Six coefficients [a00,a01,a02,a10,a11,a12], a00 at low bits.
// Raster recurrence uses adders only; no per-pixel multipliers/trigonometry.
module video_affine_coords #(parameter WIDTH=1280,HEIGHT=720)(
 input wire clk,rst_n,start,input wire [191:0] matrix,
 output reg out_valid,input wire out_ready,output wire [63:0] descriptor
);
 reg [$clog2(WIDTH)-1:0] x;
 reg [$clog2(HEIGHT)-1:0] y;
 reg signed [47:0] u,v,row_u,row_v;
 wire signed [31:0] a00=matrix[0+:32],a01=matrix[32+:32],a02=matrix[64+:32];
 wire signed [31:0] a10=matrix[96+:32],a11=matrix[128+:32],a12=matrix[160+:32];
 wire signed [47:0] ix=u>>>16,iy=v>>>16;
 wire near_image=ix>=-1&&ix<WIDTH&&iy>=-1&&iy<HEIGHT;
 wire x0=ix>=0&&ix<WIDTH,x1=ix+1>=0&&ix+1<WIDTH;
 wire y0=iy>=0&&iy<HEIGHT,y1=iy+1>=0&&iy+1<HEIGHT;
 wire [3:0] mask={x1&&y1,x0&&y1,x1&&y0,x0&&y0};
 assign descriptor=near_image?{mask,iy[13:0],ix[13:0],v[15:0],u[15:0]}:64'b0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin out_valid<=0;x<=0;y<=0;u<=0;v<=0;row_u<=0;row_v<=0;end
  else if(start)begin
   out_valid<=1;x<=0;y<=0;u<=a02;v<=a12;row_u<=a02;row_v<=a12;
  end else if(out_valid&&out_ready)begin
   if(x==WIDTH-1)begin
    x<=0;row_u<=row_u+a01;row_v<=row_v+a11;u<=row_u+a01;v<=row_v+a11;
    if(y==HEIGHT-1)out_valid<=0;else y<=y+1'b1;
   end else begin x<=x+1'b1;u<=u+a00;v<=v+a10;end
  end
 end
endmodule
