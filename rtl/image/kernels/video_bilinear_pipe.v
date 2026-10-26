`timescale 1ns/1ps
// Exact ONE-rounding RGB565 bilinear, algebraic difference form avoids both
// complementary-weight products. Six 9x17 horizontal, three 25x17 vertical
// multiplications (wide products may use multiple APMs). Six elastic stages.
// Only valid resets: datapath CE registers allow DSP register packing. II=1.
module video_bilinear_pipe(
 input wire clk,rst_n,in_valid,output wire in_ready,
 input wire [63:0] pixels,input wire [15:0] dx,dy,
 output wire out_valid,input wire out_ready,output wire [15:0] pixel
);
 reg [5:0] valid;wire advance=!valid[5]||out_ready;
 assign in_ready=advance;assign out_valid=valid[5];
 // Cut cache BRAM -> pixel mux -> subtraction -> DSP input into two clocks.
 // This input stage shares the pipeline stall, keeping pixels and weights
 // aligned through arbitrary downstream backpressure without changing II=1.
 reg [63:0] pixels_q;reg [15:0] dx_q,dy_q;
 always @(posedge clk)if(advance)begin pixels_q<=pixels;dx_q<=dx;dy_q<=dy;end
 function [7:0] channel;
  input [15:0] p;input integer c;
  begin case(c)0:channel={p[4:0],p[4:2]};1:channel={p[10:5],p[10:9]};default:channel={p[15:11],p[15:13]};endcase end
 endfunction
 reg [16:0] fx;reg [15:0] fy0,fy1,fy2;
 wire [7:0] result[0:2];
 genvar c;
 generate for(c=0;c<3;c=c+1)begin : g_channel
  reg [7:0] a0,a2;reg signed [8:0] difference0,difference2;
  reg [23:0] top,bottom,top_delayed;reg signed [24:0] difference_y;
  reg signed [40:0] sum;reg [7:0] packed_channel;
  wire [7:0] p0=channel(pixels_q[15:0],c),p1=channel(pixels_q[31:16],c);
  wire [7:0] p2=channel(pixels_q[47:32],c),p3=channel(pixels_q[63:48],c);
  always @(posedge clk)if(advance)begin
   // Invalid input cycles may overwrite stage 1: valid[1] suppresses their use.
   // Do not put the cache tag/address hit cone on the DSP input clock enable.
   begin
    a0<=p0;a2<=p2;difference0<=$signed({1'b0,p1})-$signed({1'b0,p0});difference2<=$signed({1'b0,p3})-$signed({1'b0,p2});
   end
   if(valid[1])begin
    top<=$signed({1'b0,a0,16'b0})+difference0*$signed(fx);
    bottom<=$signed({1'b0,a2,16'b0})+difference2*$signed(fx);
   end
   if(valid[2])begin top_delayed<=top;difference_y<=$signed({1'b0,bottom})-$signed({1'b0,top});end
   if(valid[3])sum<=$signed({1'b0,top_delayed,16'b0})+difference_y*$signed({1'b0,fy2})+41'sh080000000;
   if(valid[4])packed_channel<=sum[39:32];
  end
  assign result[c]=packed_channel;
 end endgenerate
 assign pixel={result[2][7:3],result[1][7:2],result[0][7:3]};
 always @(posedge clk)if(advance)begin
  begin fx<={1'b0,dx_q};fy0<=dy_q;end
  if(valid[1])fy1<=fy0;if(valid[2])fy2<=fy1;
 end
 always @(posedge clk or negedge rst_n)
  if(!rst_n)valid<=0;else if(advance)valid<={valid[4:0],in_valid};
endmodule
