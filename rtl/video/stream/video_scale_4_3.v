`timescale 1ns/1ps
// Display-only 4/3 enlargement. Coordinates are floor(3*out/4) with quarter
// fractions 0,3,2,1; the final neighbor is clamped. Input stays native RGB565.
// Two source rows, split into four synchronous BRAM banks by row/column parity.
// FILL and DRAW alternate: no RAM is overwritten while referenced by output.
// RAM read -> pixel selection -> horizontal blend -> vertical blend -> RGB565.
// One output per clock during DRAW, global elastic stall preserves all stages.
module video_scale_4_3 #(parameter WIDTH=960,HEIGHT=540)(
 input wire clk,rst_n,
 input wire in_valid,output wire in_ready,input wire [15:0] in_pixel,
 input wire in_first,in_last,
 output wire out_valid,input wire out_ready,output wire [15:0] out_pixel,
 output wire out_first,out_last,output reg error,
 output reg [23:0] frames,output wire [1:0] phase
);
 localparam OW=WIDTH*4/3,OH=HEIGHT*4/3;
 localparam XB=$clog2(WIDTH),YB=$clog2(HEIGHT),OXB=$clog2(OW),OYB=$clog2(OH);
 localparam FILL=0,DRAW=1,DRAIN=2;
 reg [1:0] state;
 reg [XB-1:0] wx,sx;reg [YB-1:0] wy,sy;
 reg [OXB-1:0] ox;reg [OYB-1:0] oy;
 reg [1:0] fx,fy;
 (* syn_ramstyle="block_ram" *) reg [15:0] e0[0:WIDTH/2-1],o0[0:WIDTH/2-1];
 (* syn_ramstyle="block_ram" *) reg [15:0] e1[0:WIDTH/2-1],o1[0:WIDTH/2-1];
 reg [15:0] qe0,qo0,qe1,qo1;
 reg [4:0] valid,first_pipe,last_pipe;
 wire advance=!valid[4]||out_ready;
 wire issue=state==DRAW&&advance;
 assign in_ready=state==FILL;
 assign out_valid=valid[4];assign out_first=first_pipe[4];assign out_last=last_pipe[4];
 assign phase=state;
 wire [XB-2:0] even_address=sx[XB-1:1]+sx[0];
 wire [XB-2:0] odd_address=sx[XB-1:1];
 reg x_odd,y_odd,x_edge,y_edge;reg [1:0] fx0,fy0,fx1,fy1,fy2;
 reg [15:0] p00,p10,p01,p11;
 wire [15:0] top_e=y_odd?qe1:qe0,top_o=y_odd?qo1:qo0;
 wire [15:0] bot_e=y_odd?qe0:qe1,bot_o=y_odd?qo0:qo1;
 wire [15:0] left_top=x_odd?top_o:top_e,right_top=x_edge?left_top:(x_odd?top_e:top_o);
 wire [15:0] left_bot=y_edge?left_top:(x_odd?bot_o:bot_e);
 wire [15:0] right_bot=y_edge?right_top:x_edge?left_bot:(x_odd?bot_e:bot_o);
 // RAM contents are never reset. The first two rows are filled before DRAW.
 always @(posedge clk)begin
  if(in_valid&&in_ready)begin
   if(!wy[0])begin if(wx[0])o0[wx[XB-1:1]]<=in_pixel;else e0[wx[XB-1:1]]<=in_pixel;end
   else begin if(wx[0])o1[wx[XB-1:1]]<=in_pixel;else e1[wx[XB-1:1]]<=in_pixel;end
  end
  if(issue)begin
   qe0<=e0[sx==WIDTH-1?odd_address:even_address];qo0<=o0[odd_address];
   qe1<=e1[sx==WIDTH-1?odd_address:even_address];qo1<=o1[odd_address];
   x_odd<=sx[0];y_odd<=sy[0];x_edge<=sx==WIDTH-1;y_edge<=sy==HEIGHT-1;
   fx0<=fx;fy0<=fy;
  end
  if(advance)begin
   p00<=left_top;p10<=right_top;p01<=left_bot;p11<=right_bot;
   fx1<=fx0;fy1<=fy0;fy2<=fy1;
  end
 end
 function [7:0] channel;
  input [15:0] p;input integer c;
  begin case(c)0:channel={p[4:0],p[4:2]};1:channel={p[10:5],p[10:9]};default:channel={p[15:11],p[15:13]};endcase end
 endfunction
 // Explicit shift/add constant weights, no general multipliers or DSPs.
 function [11:0] blend;
  input [9:0] a,b;input [1:0] f;
  reg [11:0] aa,bb;
  begin aa={2'b0,a};bb={2'b0,b};case(f)
   0:blend=aa<<2;
   1:blend=(aa<<1)+aa+bb;
   2:blend=(aa+bb)<<1;
   3:blend=aa+(bb<<1)+bb;
  endcase end
 endfunction
 wire [7:0] result[0:2];genvar c;
 generate for(c=0;c<3;c=c+1)begin:g_channel
  reg [9:0] h0,h1;reg [11:0] sum;reg [7:0] packed_channel;
  always @(posedge clk)if(advance)begin
   h0<=blend({2'b0,channel(p00,c)},{2'b0,channel(p10,c)},fx1);
   h1<=blend({2'b0,channel(p01,c)},{2'b0,channel(p11,c)},fx1);
   sum<=blend(h0,h1,fy2)+12'd8;
   packed_channel<=sum[11:4];
  end
  assign result[c]=packed_channel;
 end endgenerate
 assign out_pixel={result[2][7:3],result[1][7:2],result[0][7:3]};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin valid<=0;first_pipe<=0;last_pipe<=0;end
  else if(advance)begin
   valid<={valid[3:0],issue};
   first_pipe<={first_pipe[3:0],issue&&ox==0&&oy==0};
   last_pipe<={last_pipe[3:0],issue&&ox==OW-1&&oy==OH-1};
  end
 end
 // Address generation is small incremental counters. No division, row*stride
 // arithmetic, or interpolation feeds the input/output ready path.
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=FILL;wx<=0;wy<=0;sx<=0;sy<=0;ox<=0;oy<=0;fx<=0;fy<=0;error<=0;frames<=0;end
  else begin
   if(out_valid&&out_ready&&out_last)frames<=frames+1'b1;
   case(state)
    FILL:if(in_valid)begin
     if(in_first!=(wx==0&&wy==0)||in_last!=(wx==WIDTH-1&&wy==HEIGHT-1))error<=1;
     if(wx==WIDTH-1)begin
      wx<=0;
      if(wy<HEIGHT-1)wy<=wy+1'b1;
      if(wy!=0)state<=DRAW;
     end else wx<=wx+1'b1;
    end
    DRAW:if(issue)begin
     fx<=fx+2'd3;
     if(fx!=0)sx<=sx+1'b1;
     if(ox==OW-1)begin
      ox<=0;sx<=0;fx<=0;fy<=fy+2'd3;
      if(oy==OH-1)state<=DRAIN;
      else begin
       oy<=oy+1'b1;
       if(fy!=0)begin sy<=sy+1'b1;if(sy<HEIGHT-2)state<=FILL;end
      end
     end else ox<=ox+1'b1;
    end
    DRAIN:if(out_valid&&out_ready&&out_last)begin
     state<=FILL;wx<=0;wy<=0;sx<=0;sy<=0;ox<=0;oy<=0;fx<=0;fy<=0;
    end
    default:state<=FILL;
   endcase
  end
 end
endmodule
