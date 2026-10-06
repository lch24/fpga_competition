`timescale 1ns/1ps
// Serial RGB565 (little endian) -> Gray8 DDR conversion. One pixel per read/
// write transaction. No row padding is touched. Completion waits for DDR b.
// RGB expansion uses zero-filled low bits, matching rgb565_to_rgb888.
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
 localparam IDLE=0,RQ=1,READ=2,WQ=3,WRITE=4,ACK=5,RESP=6;
 reg [2:0] state;
 reg [15:0] iw,ih,x,y;reg [31:0] src,dst,ss,ds;reg [7:0] gray;
 reg read_bad;
 wire [7:0] red={r_data[15:11],3'b0},green={r_data[10:5],2'b0},blue={r_data[4:0],3'b0};
 wire [18:0] sum=19'd299*red+19'd587*green+19'd114*blue+19'd500;
 assign cmd_ready=state==IDLE;assign rsp_valid=state==RESP;
 assign rd_valid=state==RQ;assign rd_addr=src+y*ss+2*x;assign rd_len=2;assign rd_tag=16'h4752;
 assign r_ready=state==READ;
 assign wr_valid=state==WQ;assign wr_addr=dst+y*ds+x;assign wr_len=1;assign wr_tag=16'h4757;
 assign w_valid=state==WRITE;assign w_data={24'd0,gray};assign w_keep=1;assign w_last=1;
 assign b_ready=state==ACK;
 wire [63:0] src_end={32'd0,src_base}+({48'd0,height}-1)*src_stride+2*{48'd0,width};
 wire [63:0] dst_end={32'd0,dst_base}+({48'd0,height}-1)*dst_stride+{48'd0,width};
 wire config_ok=width>0&&height>0&&src_stride>=2*{16'd0,width}&&dst_stride>=width&&
  src_end<=64'h40000000&&dst_end<=64'h40000000&&(src_end<=dst_base||dst_end<=src_base);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;iw<=0;ih<=0;x<=0;y<=0;src<=0;dst<=0;ss<=0;ds<=0;gray<=0;rsp_status<=0;read_bad<=0;end
  else case(state)
   IDLE:if(cmd_valid)begin
    iw<=width;ih<=height;src<=src_base;dst<=dst_base;ss<=src_stride;ds<=dst_stride;x<=0;y<=0;
    rsp_status<=config_ok?0:1;read_bad<=0;state<=config_ok?RQ:RESP;
   end
   RQ:if(rd_ready)begin read_bad<=0;state<=READ;end
   READ:if(r_valid)begin
    // Drain a malformed multi-beat reply through last before reporting failure.
    if(r_error||r_tag!=16'h4752||r_keep!=3||!r_last)read_bad<=1;
    if(r_last)begin
     if(read_bad||r_error||r_tag!=16'h4752||r_keep!=3)begin rsp_status<=5;state<=RESP;end
     else begin gray<=sum/1000;state<=WQ;end
    end
   end
   WQ:if(wr_ready)state<=WRITE;
   WRITE:if(w_ready)state<=ACK;
   ACK:if(b_valid)begin
    if(b_error||b_tag!=16'h4757)begin rsp_status<=5;state<=RESP;end
    else if(x==iw-1&&y==ih-1)state<=RESP;
    else begin if(x==iw-1)begin x<=0;y<=y+1'b1;end else x<=x+1'b1;state<=RQ;end
   end
   RESP:if(rsp_ready)state<=IDLE;
   default:state<=IDLE;
  endcase
 end
endmodule
