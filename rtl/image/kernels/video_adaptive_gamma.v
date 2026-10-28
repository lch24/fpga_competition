`timescale 1ns/1ps
// One elastic synchronous-ROM stage. Component LUTs preserve RGB565 identity.
// Statistics use ONLY in-bounds contributions, BEFORE enhancement; black
// padding cannot masquerade as night. Curve changes only between whole frames.
module video_adaptive_gamma(
 input wire clk,rst_n,in_valid,output wire in_ready,input wire [15:0] in_pixel,
 input wire in_first,in_last,in_bounds,enable,input wire [2:0] mode,
 output reg out_valid,input wire out_ready,output wire [15:0] out_pixel,
 output reg [2:0] auto_curve
);
 (* syn_romstyle="block_rom" *) reg [4:0] gamma_rb[0:255];
 (* syn_romstyle="block_rom" *) reg [5:0] gamma_g[0:511];
 initial begin
 `include "video_gamma_init.vh"
 end
 reg [4:0] rq,bq;reg [5:0] gq;reg [15:0] raw_q;reg enabled_q;
 wire advance=!out_valid||out_ready;
 assign in_ready=advance;
 assign out_pixel=enabled_q?{rq,gq,bq}:raw_q;
 reg [31:0] sum,count;reg [2:0] frame_curve,candidate;reg [1:0] stable;
 wire [2:0] selected=mode==0?auto_curve:mode-1'b1;
 wire [2:0] curve=in_first?selected:frame_curve;
 wire [7:0] r={in_pixel[15:11],in_pixel[15:13]},g={in_pixel[10:5],in_pixel[10:9]},b={in_pixel[4:0],in_pixel[4:2]};
 wire [9:0] luma={2'b0,r}+{1'b0,g,1'b0}+{2'b0,b};
 wire [31:0] next_sum=(in_first?32'b0:sum)+(in_bounds?{22'b0,luma}:32'b0);
 wire [31:0] next_count=(in_first?32'b0:count)+(in_bounds?32'd1:32'd0);
 // Threshold products are compile-time shifts/adders, not general multipliers.
 wire [31:0] t32=next_count<<7,t64=next_count<<8,t160=(next_count<<9)+(next_count<<7),t208=(next_count<<9)+(next_count<<8)+(next_count<<6);
 wire [2:0] desired=next_sum<t32?3'd2:next_sum<t64?3'd1:next_sum>t208?3'd4:next_sum>t160?3'd3:3'd0;
 always @(posedge clk)if(advance&&in_valid)begin
  rq<=gamma_rb[{curve,in_pixel[15:11]}];bq<=gamma_rb[{curve,in_pixel[4:0]}];gq<=gamma_g[{curve,in_pixel[10:5]}];
  raw_q<=in_pixel;enabled_q<=enable;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin out_valid<=0;sum<=0;count<=0;auto_curve<=0;frame_curve<=0;candidate<=0;stable<=0;end
  else begin
   if(advance)out_valid<=in_valid;
   if(advance&&in_valid)begin
    // A disabled pass is transport only (not another exposure observation).
    // This matters when one logical frame traverses the shared engine twice.
    if(enable)begin sum<=next_sum;count<=next_count;end
    if(in_first)frame_curve<=selected;
    if(enable&&mode==0&&in_last&&next_count!=0)begin
     if(desired==auto_curve)begin stable<=0;candidate<=desired;end
     else if(desired!=candidate)begin candidate<=desired;stable<=1;end
     else if(stable==2)begin auto_curve<=desired;stable<=0;end
     else stable<=stable+1'b1;
    end
   end
  end
 end
endmodule
