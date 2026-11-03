`timescale 1ns/1ps
// Burst RGB565 (little endian) -> Gray8 DDR conversion.
//
// One 64-pixel chunk is read with a single byte transaction (4-byte beats),
// converted into 32-bit gray words, and written back with one transaction.
// The previous version issued a 2-byte read and a 1-byte write PER PIXEL and
// waited for each write completion, so every pixel paid two full DDR round
// trips; 720p needed ~1.8M serialized transactions. The gray value, RGB
// expansion (zero-filled low bits), byte order, dst layout (no row padding)
// and error semantics are unchanged.
// gray=(299*r+587*g+114*b+500)/1000, matching the detector's BGR reference.
module rgb565_gray_dma (
 input wire clk,rst_n,cmd_valid,output wire cmd_ready,
 input wire [15:0] width,height,
 input wire [31:0] src_base,src_stride,dst_base,dst_stride,
 output wire rsp_valid,input wire rsp_ready,output reg [7:0] rsp_status,
 output wire rd_valid,input wire rd_ready,output wire [31:0] rd_addr,rd_len,
 output wire [15:0] rd_tag,input wire r_valid,output wire r_ready,
 input wire [31:0] r_data,input wire [3:0] r_keep,input wire [15:0] r_tag,
 input wire r_last,r_error,
 output wire wr_valid,input wire wr_ready,output wire [31:0] wr_addr,wr_len,
 output wire [15:0] wr_tag,output wire w_valid,input wire w_ready,
 output wire [31:0] w_data,output wire [3:0] w_keep,output wire w_last,
 input wire b_valid,output wire b_ready,input wire [15:0] b_tag,input wire b_error
);
 localparam IDLE=0,RREQ=1,RREAD=2,WDREQ=3,WDATA=4,WACK=5,RESP=6;
 localparam CHUNK=64;
 reg [2:0] state;
 reg [15:0] iw,ih,x,y;
 reg [31:0] src,dst,ss,ds;
 reg [6:0] px;                 // pixels in the current chunk
 reg [31:0] rd_seen;           // source bytes received in this chunk
 reg [4:0] wptr;               // gray words pending/written (0..16)
 reg [15:0] gpair;             // 1..2 unflushed gray bytes
 reg [1:0] gcount;             // valid bytes in gpair (0..2)
 reg [31:0] line32[0:15];      // 64 gray bytes as 16 words
 reg [31:0] beat; reg [3:0] beat_keep; reg beat_last,beat_err,beat_valid;
 reg [3:0] wk;                 // write beat index
 reg [4:0] nbeats;             // ceil(px/4)
 reg [31:0] line_q; reg line_re; reg [3:0] line_ra; reg [1:0] fetch_phase; reg wq_valid;
 reg [31:0] wq;
 reg err_seen;
 // RGB expansion and gray formula are bit-identical to the per-pixel version.
 function [7:0] gray_of(input [15:0] p);
  reg [7:0] red,green,blue; reg [18:0] sum;
  begin
   red={p[15:11],3'b0};green={p[10:5],2'b0};blue={p[4:0],3'b0};
   sum=19'd299*red+19'd587*green+19'd114*blue+19'd500;
   gray_of=sum/1000;
  end
 endfunction
 wire [15:0] pix0={beat[15:8],beat[7:0]};
 wire [15:0] pix1={beat[31:24],beat[23:16]};
 wire [7:0] gray0=gray_of(pix0);
 wire [7:0] gray1=gray_of(pix1);
 wire [6:0] chunk_px=((iw-x)>CHUNK)?CHUNK:(iw-x);
 wire [4:0] beat_index=wk+1'b1;
 wire last_beat=beat_index==nbeats;
 wire [31:0] rd_addr_w=src+y*ss+2*x;
 wire [31:0] wr_addr_w=dst+y*ds+x;
 wire [31:0] rd_len_w={25'd0,chunk_px}*2;
 wire [31:0] wr_len_w={25'd0,px};
 assign cmd_ready=state==IDLE;
 assign rsp_valid=state==RESP;
 assign rd_valid=state==RREQ;
 assign rd_addr=rd_addr_w;assign rd_len=rd_len_w;assign rd_tag=16'h4752;
 assign r_ready=state==RREAD&&!beat_valid;
 assign wr_valid=state==WDREQ;
 assign wr_addr=wr_addr_w;assign wr_len=wr_len_w;assign wr_tag=16'h4757;
 assign w_valid=state==WDATA&&wq_valid;
 assign w_data=wq;
 assign w_keep=last_beat?((px[1:0]==0)?4'hf:((4'b1<<px[1:0])-1'b1)):4'hf;
 assign w_last=last_beat;
 assign b_ready=state==WACK;
 wire [63:0] src_end={32'd0,src_base}+({48'd0,height}-1)*src_stride+2*{48'd0,width};
 wire [63:0] dst_end={32'd0,dst_base}+({48'd0,height}-1)*dst_stride+{48'd0,width};
 wire config_ok=width>0&&height>0&&src_stride>=2*{16'd0,width}&&dst_stride>=width&&
  src_end<=64'h40000000&&dst_end<=64'h40000000&&(src_end<=dst_base||dst_end<=src_base);
 // One beat contributes 4 source bytes (2 pixels) or, at the transaction end,
 // 2 bytes (1 pixel). Any other keep is a malformed reply.
 wire [31:0] beat_bytes=(beat_keep==4'hf)?32'd4:(beat_keep==4'h3)?32'd2:32'd0;
 wire beat_bad=beat_err||(beat_keep!=4'hf&&beat_keep!=4'h3);
 // Pair/word assembly for one captured beat (combinational; uses old gpair).
 reg [31:0] word_next;reg word_we;reg [15:0] pair_next;reg [1:0] count_next;reg pair_bad;
 always @(*)begin
  word_next=0;word_we=0;pair_next=gpair;count_next=gcount;pair_bad=0;
  case(beat_keep)
   4'hf:begin
    if(gcount==2)begin word_next={gray1,gray0,gpair};word_we=1;count_next=0;end
    else if(gcount==0)begin pair_next={gray1,gray0};count_next=2;end
    else pair_bad=1;
   end
   4'h3:begin
    if(gcount==2)begin word_next={8'h0,gray0,gpair};word_we=1;count_next=0;end
    else if(gcount==0)begin pair_next={8'h0,gray0};count_next=1;end
    else pair_bad=1;
   end
   default:pair_bad=1;
  endcase
 end
 always @(posedge clk)if(line_re)line_q<=line32[line_ra];
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;iw<=0;ih<=0;x<=0;y<=0;src<=0;dst<=0;ss<=0;ds<=0;px<=0;
   rd_seen<=0;wptr<=0;gpair<=0;gcount<=0;beat<=0;beat_keep<=0;beat_last<=0;
   beat_err<=0;beat_valid<=0;wk<=0;nbeats<=0;line_re<=0;line_ra<=0;
   fetch_phase<=0;wq_valid<=0;wq<=0;err_seen<=0;rsp_status<=0;
  end else begin
   line_re<=0;
   case(state)
    IDLE:if(cmd_valid)begin
     iw<=width;ih<=height;src<=src_base;dst<=dst_base;ss<=src_stride;ds<=dst_stride;
     x<=0;y<=0;
     if(!config_ok)begin rsp_status<=8'd1;state<=RESP;end
     else state<=RREQ;
    end
    RREQ:if(rd_ready)begin
     px<=chunk_px;
     rd_seen<=0;wptr<=0;gpair<=0;gcount<=0;beat_valid<=0;
     err_seen<=0;fetch_phase<=0;wq_valid<=0;
     state<=RREAD;
    end
    RREAD:begin
     if(beat_valid)begin
      beat_valid<=0;
      if(beat_bad)err_seen<=1;
      rd_seen<=rd_seen+beat_bytes;
      if(beat_last)begin
       // Flush the final partial word with the value produced THIS beat.
       if(beat_bad||err_seen||r_error)begin rsp_status<=8'd5;state<=RESP;end
       else if(rd_seen+beat_bytes!=rd_len_w)begin rsp_status<=8'd5;state<=RESP;end
       else begin
        if(pair_bad)begin rsp_status<=8'd5;state<=RESP;end
        else begin
         if(word_we)line32[wptr]<=word_next;
         else if(count_next!=0)line32[wptr]<={16'd0,pair_next};
         wptr<=word_we?wptr+1'b1:(count_next!=0?wptr+1'b1:wptr);
         nbeats<=((px+3)>>2);
         state<=WDREQ;
        end
       end
      end else begin
       if(word_we)begin line32[wptr]<=word_next;wptr<=wptr+1'b1;end
       gpair<=pair_next;gcount<=count_next;
       if(pair_bad)err_seen<=1;
      end
     end else if(r_valid)begin
      beat<=r_data;beat_keep<=r_keep;beat_last<=r_last;
      beat_err<=r_error||(r_tag!=16'h4752);beat_valid<=1;
     end
    end
    WDREQ:if(wr_ready)begin
     wk<=0;line_re<=1;line_ra<=0;fetch_phase<=1;wq_valid<=0;state<=WDATA;
    end
    WDATA:begin
     if(fetch_phase==1)fetch_phase<=2;
     else if(fetch_phase==2)begin wq<=line_q;wq_valid<=1;fetch_phase<=0;end
     else if(wq_valid&&w_ready)begin
      if(last_beat)state<=WACK;
      else begin wk<=wk+1'b1;line_re<=1;line_ra<=wk+1'b1;fetch_phase<=1;wq_valid<=0;end
     end
    end
    WACK:if(b_valid)begin
     if(b_error||b_tag!=16'h4757)begin rsp_status<=8'd5;state<=RESP;end
     else if(x+px>=iw)begin x<=0;if(y+1'b1>=ih)begin rsp_status<=8'd0;state<=RESP;end else begin y<=y+1'b1;state<=RREQ;end end
     else begin x<=x+px;state<=RREQ;end
    end
    RESP:if(rsp_ready)state<=IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
