`timescale 1ns/1ps
// 4x4 control-point map reader for the realtime remap path.
//
// DDR layout (little endian, 8 bytes per control point):
//   [31:0]  source x, signed Q16.16
//   [63:32] source y, signed Q16.16
// Control points sit every STEP destination pixels.  Each row is padded to a
// 64-point multiple so every HMIC burst is 512 B aligned.  NROW is
// HEIGHT/STEP + 1: the extra bottom row supplies the fourth control point for
// the last STEP destination rows instead of clamping them.
//
// Three complete row buffers rotate.  A band of STEP destination rows uses
// rows k and k+1 while row k+2 is fetched into the third buffer.  A 512-bit
// window holds two 256-bit groups (eight control points); one group fetch is
// needed every 4*STEP output pixels, and the cell advance is a pure 64-bit
// register shift.  Coordinates are interpolated with integer adders and
// shifts only; no floating point runs per video frame.
module video_grid_prefetch #(parameter WIDTH=1280,HEIGHT=720,STEP=4,
 parameter SRC_WIDTH=WIDTH,parameter SRC_HEIGHT=HEIGHT)(
 input wire clk,rst_n,start,input wire [31:0] map_base,
 output reg out_valid,input wire out_ready,output reg [63:0] descriptor,output reg error,
 output wire arvalid,input wire arready,output wire [27:0] araddr,output wire [3:0] arlen,
 input wire rvalid,input wire [255:0] rdata,input wire rlast,input wire [3:0] rid
);
 localparam GW=(WIDTH+STEP-1)/STEP;
 localparam GH=(HEIGHT+STEP-1)/STEP;
 localparam NCOL=GW+1;
 localparam NROW=GH+1;
 localparam STRIDE=((NCOL+63)/64)*64;
 localparam WORDS=STRIDE/4;
 localparam WB=$clog2(STEP);
 localparam GB=WB+2;
 localparam GROUP=1<<GB;
 localparam BLOCKS=STRIDE/64;
 localparam ABITS=$clog2(WORDS);

 // ---------------- raster registers (declared before combinational use) ----
 localparam P_IDLE=0,P_WAIT=1,P_G0=2,P_G1=3,P_G2=4,P_RUN=5,P_BAND=6,P_DONE=7,P_SHIFT=8;
 reg [3:0] pstate;
 reg [10:0] px,py;
 reg [7:0] band;
 reg [1:0] buf_low,buf_high,buf_third;
 reg [511:0] win_low,win_high;
 reg [255:0] q0_low,q0_high;
 // Registered enables keep the 512-bit window CE off the high-fanout
 // a_ready/stage_a_fire cone.
 reg win_load,win_shift;
 reg [ABITS-1:0] ra;
 reg read_en;
 // No DDR request before the frame command or after the last row is emitted.
 reg run;

 // One 256-bit word per four control points.  Simple dual-port block RAM:
 // loader writes one word per HMIC beat, reader fetches one word per cycle.
 (* syn_ramstyle="block_ram" *) reg [255:0] row0[0:WORDS-1];
 (* syn_ramstyle="block_ram" *) reg [255:0] row1[0:WORDS-1];
 (* syn_ramstyle="block_ram" *) reg [255:0] row2[0:WORDS-1];
 reg [255:0] qr0,qr1,qr2;
 always @(posedge clk)
  if(read_en)begin qr0<=row0[ra];qr1<=row1[ra];qr2<=row2[ra];end

 // ---------------- row loader ----------------
 localparam L_REQ=1'b0,L_FILL=1'b1;
 reg lstate;
 reg [7:0] load_row;
 reg [2:0] load_block;
 reg [3:0] load_beat;
 reg [1:0] load_buf;
 reg [2:0] buf_ready;
 reg [2:0] buf_in_use;
 reg [7:0] buf_row[0:2];
 reg ld_error;
 wire [ABITS-1:0] wa=(load_block<<4)+load_beat;
 wire in_use_sel=(load_buf==0)?buf_in_use[0]:(load_buf==1)?buf_in_use[1]:buf_in_use[2];
 wire can_load=(load_row<NROW)&&!in_use_sel;
 assign arvalid=(lstate==L_REQ)&&can_load&&run;
 assign arlen=4'd15;
 wire [31:0] req_word=(map_base>>2)+load_row*(STRIDE*2)+load_block*128;
 assign araddr=req_word[27:0];
 always @(posedge clk)begin
  if(lstate==L_FILL&&rvalid)begin
   case(load_buf)
    2'd0:row0[wa]<=rdata;
    2'd1:row1[wa]<=rdata;
    default:row2[wa]<=rdata;
   endcase
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin lstate<=L_REQ;load_row<=0;load_block<=0;load_beat<=0;load_buf<=0;
   buf_ready<=0;ld_error<=0;buf_row[0]<=0;buf_row[1]<=0;buf_row[2]<=0;end
  else if(start)begin lstate<=L_REQ;load_row<=0;load_block<=0;load_beat<=0;load_buf<=0;
   buf_ready<=0;ld_error<=0;buf_row[0]<=0;buf_row[1]<=0;buf_row[2]<=0;end
  else if(lstate==L_REQ)begin
   if(arvalid&&arready)begin lstate<=L_FILL;load_beat<=0;buf_ready[load_buf]<=0;end
  end else if(rvalid)begin
   if(rid!=0||rlast!=(load_beat==4'd15))ld_error<=1;
   load_beat<=load_beat+1'b1;
   if(rlast)begin
    load_beat<=0;
    if(load_block==BLOCKS-1)begin
     buf_row[load_buf]<=load_row;buf_ready[load_buf]<=1;
     load_row<=load_row+1'b1;load_buf<=load_buf==2?0:load_buf+1'b1;load_block<=0;
    end else load_block<=load_block+1'b1;
    lstate<=L_REQ;
   end
  end
 end

 // ---------------- coordinate interpolation ----------------
 // Weighted sum by a value 0..STEP-1.  The unrolled loop is only shifts and
 // adds, so the two interpolation stages stay shallow.
 function signed [39:0] wmul;
  input signed [39:0] d;
  input [WB-1:0] w;
  integer i;
  begin
   wmul=0;
   for(i=0;i<WB;i=i+1)if(w[i])wmul=wmul+(d<<i);
  end
 endfunction

 wire signed [31:0] a0x=win_low[31:0],a0y=win_low[63:32];
 wire signed [31:0] a1x=win_low[95:64],a1y=win_low[127:96];
 wire signed [31:0] b0x=win_high[31:0],b0y=win_high[63:32];
 wire signed [31:0] b1x=win_high[95:64],b1y=win_high[127:96];
 wire signed [39:0] a0x40={{8{a0x[31]}},a0x},a1x40={{8{a1x[31]}},a1x};
 wire signed [39:0] b0x40={{8{b0x[31]}},b0x},b1x40={{8{b1x[31]}},b1x};
 wire signed [39:0] a0y40={{8{a0y[31]}},a0y},a1y40={{8{a1y[31]}},a1y};
 wire signed [39:0] b0y40={{8{b0y[31]}},b0y},b1y40={{8{b1y[31]}},b1y};
 wire [WB-1:0] fx=px[WB-1:0],fy=py[WB-1:0];
 wire signed [39:0] h0x=(a0x40<<WB)+wmul(a1x40-a0x40,fx);
 wire signed [39:0] h1x=(b0x40<<WB)+wmul(b1x40-b0x40,fx);
 wire signed [39:0] h0y=(a0y40<<WB)+wmul(a1y40-a0y40,fx);
 wire signed [39:0] h1y=(b0y40<<WB)+wmul(b1y40-b0y40,fx);

 reg signed [39:0] ph0x,ph1x,ph0y,ph1y;
 reg [WB-1:0] pfy;
 reg p_valid;
 wire pipe_move=p_valid&&(!out_valid||out_ready);
 wire a_ready=!p_valid||!out_valid||out_ready;
 wire stage_a_fire=(pstate==P_RUN)&&a_ready;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin ph0x<=0;ph1x<=0;ph0y<=0;ph1y<=0;pfy<=0;p_valid<=0;end
  else begin
   if(stage_a_fire)begin ph0x<=h0x;ph1x<=h1x;ph0y<=h0y;ph1y<=h1y;pfy<=fy;p_valid<=1;end
   else if(pipe_move)p_valid<=0;
  end
 end

 wire signed [39:0] vx=(ph0x<<WB)+wmul(ph1x-ph0x,pfy);
 wire signed [39:0] vy=(ph0y<<WB)+wmul(ph1y-ph0y,pfy);
 wire signed [31:0] sx=vx>>>(WB+WB);
 wire signed [31:0] sy=vy>>>(WB+WB);
 wire signed [31:0] ix=sx>>>16,iy=sy>>>16;
 wire x0_ok=ix>=0&&ix<SRC_WIDTH,x1_ok=ix+1>=0&&ix+1<SRC_WIDTH;
 wire y0_ok=iy>=0&&iy<SRC_HEIGHT,y1_ok=iy+1>=0&&iy+1<SRC_HEIGHT;
 wire [3:0] mask={x1_ok&&y1_ok,x0_ok&&y1_ok,x1_ok&&y0_ok,x0_ok&&y0_ok};
 wire [63:0] calc={mask,sy[29:16],sx[29:16],sy[15:0],sx[15:0]};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin out_valid<=0;descriptor<=0;end
  else begin
   if(pipe_move)begin out_valid<=1;descriptor<=error?{4'b0,calc[59:0]}:calc;end
   else if(out_valid&&out_ready)out_valid<=0;
  end
 end

 // ---------------- raster / buffer sequencing ----------------
 wire [255:0] qsel_low=(buf_low==0)?qr0:(buf_low==1)?qr1:qr2;
 wire [255:0] qsel_high=(buf_high==0)?qr0:(buf_high==1)?qr1:qr2;
 // G1/G2 overwrite both words before P_RUN can emit a coordinate. Reset and
 // frame start only invalidate control; clearing these data registers would
 // unnecessarily put start on 1536 data enables. No extra pipeline cycle.
 always @(posedge clk)begin
  if(pstate==P_G1)begin q0_low<=qsel_low;q0_high<=qsel_high;end
  if(win_load)begin win_low<={qsel_low,q0_low};win_high<={qsel_high,q0_high};end
  else if(win_shift)begin win_low<=win_low>>64;win_high<=win_high>>64;end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin pstate<=P_IDLE;px<=0;py<=0;band<=0;buf_low<=0;buf_high<=1;buf_third<=2;
   buf_in_use<=0;win_load<=0;win_shift<=0;read_en<=0;ra<=0;error<=0;run<=0;end
  else if(start)begin pstate<=P_WAIT;px<=0;py<=0;band<=0;buf_low<=0;buf_high<=1;buf_third<=2;
   buf_in_use<=0;win_load<=0;win_shift<=0;read_en<=0;ra<=0;error<=0;run<=1;end
  else begin
   if(ld_error)error<=1;
   win_load<=0;win_shift<=0;
   case(pstate)
    P_WAIT:if(buf_ready[0]&&buf_row[0]==0&&buf_ready[1]&&buf_row[1]==1)begin
     buf_in_use<=3'b011;px<=0;py<=0;band<=0;read_en<=1;ra<=0;pstate<=P_G0;
    end
    // The first word read is issued on the transition into P_G0.  G0 waits
    // for that synchronous read and advances the address for the second word;
    // G1 captures word0 and lets word1 arrive; G2 loads the 512-bit window.
    P_G0:begin ra<=ra+1'b1;pstate<=P_G1;end
    P_G1:begin win_load<=1;pstate<=P_G2;end
    P_G2:begin read_en<=0;pstate<=P_RUN;end
    P_BAND:if(buf_ready[buf_high]&&buf_row[buf_high]==band+1&&buf_ready[buf_low]&&buf_row[buf_low]==band)begin
     read_en<=1;ra<=0;pstate<=P_G0;
    end
    P_RUN:if(stage_a_fire)begin
     if(px==WIDTH-1)begin
      px<=0;
      if(py==HEIGHT-1)begin pstate<=P_DONE;run<=0;end
      else begin
       py<=py+1'b1;
       if(py[WB-1:0]==STEP-1)begin
        band<=band+1'b1;
        buf_in_use[buf_third]<=1;buf_in_use[buf_low]<=0;
        buf_low<=buf_high;buf_high<=buf_third;buf_third<=buf_low;
        pstate<=P_BAND;
       end else begin read_en<=1;ra<=0;pstate<=P_G0;end
      end
     end else if(px[GB-1:0]==GROUP-1)begin
      read_en<=1;ra<=(px>>GB)+1'b1;px<=px+1'b1;pstate<=P_G0;
     end else if(px[WB-1:0]==STEP-1)begin
      win_shift<=1;px<=px+1'b1;pstate<=P_SHIFT;
     end else px<=px+1'b1;
    end
    P_SHIFT:pstate<=P_RUN;
    default:pstate<=P_DONE;
   endcase
  end
 end
endmodule
