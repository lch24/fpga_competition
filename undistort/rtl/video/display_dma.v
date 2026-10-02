// DDR frame reader feeding the original Demo rd_buf-style pixel stream.
// in_ready is the destination FIFO capacity; no dependency on HDMI timing.
// The existing Demo timing generator and HDMI PHY remain board-specific.
module display_dma (
 input wire clk,rst_n,input wire cmd_valid,output wire cmd_ready,
 input wire [31:0] cmd_base,cmd_stride,input wire [15:0] cmd_width,cmd_height,
 output wire pixel_valid,input wire pixel_ready,output reg [15:0] pixel,
 output wire pixel_first,pixel_last,
 output wire rsp_valid,input wire rsp_ready,output reg [7:0] rsp_status,
 output wire rd_valid,input wire rd_ready,output wire [31:0] rd_addr,rd_len,
 output wire [15:0] rd_tag,
 input wire r_valid,output wire r_ready,input wire [31:0] r_data,
 input wire [3:0] r_keep,input wire [15:0] r_tag,input wire r_last,r_error
);
 reg [2:0] state;reg [31:0] base,stride;reg [15:0] width,height,x,y;
 assign cmd_ready=state==0;assign rsp_valid=state==4;
 assign rd_valid=state==1;assign rd_addr=base+y*stride+({16'd0,x}<<1);assign rd_len=2;assign rd_tag=0;
 assign r_ready=state==2;assign pixel_valid=state==3;assign pixel_last=x==width-1&&y==height-1;
 assign pixel_first=x==0&&y==0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;base<=0;stride<=0;width<=0;height<=0;x<=0;y<=0;pixel<=0;rsp_status<=0;end
  else case(state)
   0:if(cmd_valid)begin
    base<=cmd_base;stride<=cmd_stride;width<=cmd_width;height<=cmd_height;x<=0;y<=0;rsp_status<=0;
    if(cmd_width==0||cmd_height==0||cmd_stride<({16'd0,cmd_width}<<1))begin rsp_status<=1;state<=4;end else state<=1;
   end
   1:if(rd_ready)state<=2;
   2:if(r_valid)begin
    pixel<=r_data[15:0];if(r_error||!r_last||r_tag!=0||r_keep!=3)begin rsp_status<=5;state<=4;end else state<=3;
   end
   3:if(pixel_ready)begin
    if(pixel_last)state<=4;else begin if(x==width-1)begin x<=0;y<=y+1;end else x<=x+1;state<=1;end
   end
   4:if(rsp_ready)state<=0;
   default:state<=0;
  endcase
 end
endmodule
