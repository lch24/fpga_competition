`timescale 1ns/1ps
// Component LUTs preserve RGB565 identity. Modes 1..5 are fixed gamma curves;
// mode 6 builds a contrast-limited histogram-equalization LUT once per frame.
// Statistics use ONLY in-bounds contributions, BEFORE enhancement; black
// padding cannot masquerade as night. Curve/map changes only between frames.
module video_adaptive_gamma(
 input wire clk,rst_n,in_valid,output wire in_ready,input wire [15:0] in_pixel,
 input wire in_first,in_last,in_bounds,enable,input wire [2:0] mode,
 output reg out_valid,input wire out_ready,output wire [15:0] out_pixel,
 output reg [2:0] auto_curve
);
 localparam EQ_MODE=3'd6;
 (* syn_romstyle="block_rom" *) reg [4:0] gamma_rb[0:255];
 (* syn_romstyle="block_rom" *) reg [5:0] gamma_g[0:511];
 initial begin
 `include "video_gamma_init.vh"
 end
 // Mode 6 mapping banks. Small on purpose: 64 bins, 5/6-bit component tables.
 (* syn_ramstyle="block_ram" *) reg [19:0] hist[0:63];
 (* syn_ramstyle="block_ram" *) reg [7:0] eq_bin[0:63];
 (* syn_ramstyle="block_ram" *) reg [4:0] eq_rb[0:31];
 (* syn_ramstyle="block_ram" *) reg [5:0] eq_g[0:63];
 reg [6:0] clr_cnt;reg clearing;
 // One physical histogram write port, shared by clear, sample and build.
 reg [5:0] sraddr,swaddr;reg [19:0] swdata;reg swe;
 reg [5:0] braddr,bwaddr;reg [19:0] bwdata;reg bwe;
 reg [19:0] hq;
 reg eq_valid,eq_apply,build_busy,pending_build;
 wire [5:0] hist_raddr=build_busy?braddr:sraddr;
 wire [5:0] hist_waddr=clearing?clr_cnt[5:0]:(bwe?bwaddr:swaddr);
 wire [19:0] hist_wdata=clearing?20'd0:(bwe?bwdata:swdata);
 wire hist_we=clearing||bwe||swe;
 always @(posedge clk)begin
  if(hist_we)hist[hist_waddr]<=hist_wdata;
  hq<=hist[hist_raddr];
 end
 reg [5:0] eq_baddr;reg [7:0] eq_bq;
 always @(posedge clk)eq_bq<=eq_bin[eq_baddr];
 reg [4:0] rq,bq;reg [5:0] gq;reg [15:0] raw_q;reg enabled_q;
 wire advance=!out_valid||out_ready;
 assign in_ready=advance&&!build_busy&&!clearing;
 wire in_fire=in_valid&&in_ready;
 assign out_pixel=enabled_q?{rq,gq,bq}:raw_q;
 reg [2:0] frame_curve,candidate;reg [1:0] stable;
 wire [2:0] selected=mode==0?auto_curve:(mode==EQ_MODE?3'd0:mode-1'b1);
 wire [2:0] curve=in_first?selected:frame_curve;
 wire eq_use=eq_valid&&eq_apply&&mode==EQ_MODE;
 wire [4:0] rb_lut=eq_use?eq_rb[in_pixel[15:11]]:gamma_rb[{curve,in_pixel[15:11]}];
 wire [4:0] bb_lut=eq_use?eq_rb[in_pixel[4:0]]:gamma_rb[{curve,in_pixel[4:0]}];
 wire [5:0] g_lut=eq_use?eq_g[in_pixel[10:5]]:gamma_g[{curve,in_pixel[10:5]}];
 wire [7:0] r8={in_pixel[15:11],in_pixel[15:13]};
 wire [7:0] g8={in_pixel[10:5],in_pixel[10:9]};
 wire [7:0] b8={in_pixel[4:0],in_pixel[4:2]};
 wire [9:0] luma={2'b0,r8}+{1'b0,g8,1'b0}+{2'b0,b8};
 wire [5:0] bin=luma[9:4];
 reg [31:0] sum,count;
 wire [31:0] next_sum=(in_first?32'b0:sum)+(in_bounds?{22'b0,luma}:32'b0);
 wire [31:0] next_count=(in_first?32'b0:count)+(in_bounds?32'd1:32'd0);
 wire [31:0] t32=next_count<<7,t64=next_count<<8,t160=(next_count<<9)+(next_count<<7),t208=(next_count<<9)+(next_count<<8)+(next_count<<6);
 wire [2:0] desired=next_sum<t32?3'd2:next_sum<t64?3'd1:next_sum>t208?3'd4:next_sum>t160?3'd3:3'd0;
 reg [1:0] sample_cnt;reg [19:0] sample_total;reg [1:0] hstate;reg [5:0] hbin;
 // Histogram: sample every fourth in-bounds pixel, then a 3-cycle update.
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin sample_cnt<=0;sample_total<=0;hstate<=0;swe<=0;end
  else begin
   swe<=0;
   if(in_fire&&in_bounds&&enable)begin
    sample_cnt<=sample_cnt+1'b1;
    if(sample_cnt==2'd0&&hstate==0)begin
     sraddr<=bin;hbin<=bin;hstate<=1;sample_total<=sample_total+1'b1;
    end
   end
   if(in_fire&&in_first)begin sample_cnt<=0;sample_total<=0;hstate<=0;swe<=0;end
   if(hstate==1)begin hstate<=2;end
   else if(hstate==2)begin swdata<=hq+1'b1;swaddr<=hbin;swe<=1;hstate<=3;end
   else if(hstate==3)begin hstate<=0;end
  end
 end
 // Build FSM: 64 bins, one restoring divide per bin, then the 5-bit bank.
 localparam B_IDLE=0,B_RD=1,B_RD1=2,B_ADD=3,B_DIV=4,B_WR=5,B_RB=6,B_RB1=7,B_RB2=8,B_DONE=9;
 reg [3:0] bstate;reg [5:0] bbin,low_bin,high_bin;reg seen;
 reg [31:0] cdf;reg [31:0] div_num,div_den,div_rem,div_quo;reg [5:0] div_cnt;
 reg [5:0] rb_idx;
 wire [31:0] div_shift={div_rem[30:0],div_num[31]};
 wire div_ge=div_shift>=div_den;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   clearing<=1;clr_cnt<=0;
   bstate<=B_IDLE;bbin<=0;low_bin<=0;high_bin<=0;seen<=0;cdf<=0;
   div_num<=0;div_den<=0;div_rem<=0;div_quo<=0;div_cnt<=0;rb_idx<=0;
   eq_valid<=0;eq_apply<=0;build_busy<=0;pending_build<=0;
   braddr<=0;bwaddr<=0;bwdata<=0;bwe<=0;
  end else begin
   bwe<=0;
   if(clearing)begin
    if(clr_cnt==7'd63)begin clearing<=0;clr_cnt<=0;end
    else clr_cnt<=clr_cnt+1'b1;
   end
   if(in_fire&&in_last&&enable)pending_build<=1;
   case(bstate)
    B_IDLE:if(pending_build&&!clearing&&hstate==0)begin
     pending_build<=0;build_busy<=1;cdf<=0;bbin<=0;seen<=0;low_bin<=0;high_bin<=0;
     braddr<=0;bstate<=B_RD;
    end
    B_RD:begin braddr<=bbin;bstate<=B_RD1;end
    B_RD1:begin bwe<=1;bwaddr<=bbin;bwdata<=20'd0;bstate<=B_ADD;end
    B_ADD:begin
     cdf<=cdf+hq;
     if(hq!=0)begin
      if(!seen)low_bin<=bbin;
      high_bin<=bbin;seen<=1;
     end
     div_num<=((cdf+hq)<<8)-(cdf+hq);div_den<=sample_total==0?32'd1:{12'b0,sample_total};div_rem<=0;div_quo<=0;div_cnt<=6'd32;
     bstate<=B_DIV;
    end
    B_DIV:if(div_cnt==0)bstate<=B_WR;
     else begin
      div_num<={div_num[30:0],1'b0};
      div_quo<={div_quo[30:0],div_ge};
      div_rem<=div_ge?div_shift-div_den:div_shift;
      div_cnt<=div_cnt-1'b1;
     end
    B_WR:begin
     // Contrast-limited blend: three quarters equalized, one quarter identity.
     // Widen before shifting: 8-bit shifts would otherwise drop the upper bits.
     eq_bin[bbin]<=({2'b0,div_quo[7:0]}+({2'b0,div_quo[7:0]}<<1)+({4'b0,bbin}<<2))>>2;
     eq_g[bbin]<=({2'b0,div_quo[7:0]}+({2'b0,div_quo[7:0]}<<1)+({4'b0,bbin}<<2))>>4;
     if(bbin==6'd63)begin rb_idx<=0;eq_baddr<=0;bstate<=B_RB;end
     else begin bbin<=bbin+1'b1;bstate<=B_RD;end
    end
    B_RB:begin eq_baddr<=rb_idx[4]?{rb_idx,1'b1}:{rb_idx,1'b0};bstate<=B_RB1;end
    B_RB1:bstate<=B_RB2;
    B_RB2:begin
     eq_rb[rb_idx]<=eq_bq[7:3];
     if(rb_idx==5'd31)bstate<=B_DONE;
     else begin rb_idx<=rb_idx+1'b1;bstate<=B_RB;end
    end
    B_DONE:begin
     eq_valid<=1;eq_apply<=seen&&((high_bin-low_bin)>=6'd8);build_busy<=0;bstate<=B_IDLE;
    end
   endcase
  end
 end
 always @(posedge clk)if(in_fire)begin
  rq<=rb_lut;bq<=bb_lut;gq<=g_lut;raw_q<=in_pixel;enabled_q<=enable;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin out_valid<=0;sum<=0;count<=0;auto_curve<=0;frame_curve<=0;candidate<=0;stable<=0;end
  else begin
   if(advance)out_valid<=in_fire;
   if(in_fire)begin
    // A disabled pass is transport only (not another exposure observation).
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
