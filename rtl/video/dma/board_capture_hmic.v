`timescale 1ns/1ps
// One frozen RGB565 frame. Pack 16 pixels per HMIC write instead of issuing
// one DDR transaction per pixel. WIDTH must be a multiple of 16; BASE aligned.
// A command waits for the next VSYNC, ends at the following VSYNC, drains all
// accepted FIFO words, then reports success/overflow/DDR error. Never publish
// a partial frame. Descriptor/constants remain stable across both clocks.
module board_capture_hmic #(
 parameter WIDTH=1280,HEIGHT=720,BASE=32'h0,FIFO_BITS=8
)(
 input wire clk,rst_n,pclk,prst_n,vsync,pixel_valid,input wire [15:0] pixel,
 input wire cmd_valid,output wire cmd_ready,
 output wire rsp_valid,input wire rsp_ready,output reg [7:0] rsp_status,
 output wire [27:0] axi_awaddr,output wire [3:0] axi_awlen,axi_awuser_id,
 output wire axi_awvalid,input wire axi_awready,
 output wire [255:0] axi_wdata,output wire [31:0] axi_wstrb,
 input wire axi_wready,axi_wusero_last,input wire [3:0] axi_wusero_id
);
 localparam TOTAL=WIDTH*HEIGHT;
 reg request,finished,seen,armed,active,vs_d,bad;
 reg [1:0] req_sync,done_sync /* synthesis PAP_ASYNC_REG=1 */;
 reg [31:0] count,accepted,final_words;reg final_bad;
 reg [255:0] packed_pixels;reg [3:0] lane;
 wire fv,fr,fw;wire [255:0] fd;
 // Explicit lane placement: first RGB565 pixel occupies bits 15:0.
 wire [255:0] complete_word={pixel,packed_pixels[239:0]};
 assign fw=active&&pixel_valid&&count<TOTAL&&lane==15;
 reg [2:0] state;reg [31:0] written;reg [255:0] payload;
 async_fifo #(.WIDTH(256),.ADDR_BITS(FIFO_BITS)) fifo(
  .wclk(pclk),.wrst_n(prst_n),.wvalid(fw),.wready(fr),.wdata(complete_word),
  .rclk(clk),.rrst_n(rst_n),.rvalid(fv),.rready(state==1),.rdata(fd));
 assign cmd_ready=state==0;assign rsp_valid=state==6;
 assign axi_awaddr=(BASE>>2)+(written<<3);
 assign axi_awlen=0;assign axi_awuser_id=0;assign axi_awvalid=state==2;
 assign axi_wdata=payload;assign axi_wstrb=32'hffffffff;
 always @(posedge pclk or negedge prst_n)begin
  if(!prst_n)begin req_sync<=0;finished<=0;seen<=0;armed<=0;active<=0;vs_d<=0;bad<=0;
   count<=0;accepted<=0;final_words<=0;final_bad<=0;packed_pixels<=0;lane<=0;end
  else begin
   req_sync<={req_sync[0],request};vs_d<=vsync;
   if(req_sync[1]!=seen)begin seen<=req_sync[1];armed<=1;bad<=0;count<=0;accepted<=0;lane<=0;end
   if(vsync&&!vs_d)begin
    if(active)begin active<=0;final_words<=accepted;final_bad<=bad||count!=TOTAL||lane!=0;finished<=seen;end
    else if(armed)begin armed<=0;active<=1;end
   end
   if(active&&pixel_valid)begin
    count<=count+1;
    if(count>=TOTAL)bad<=1;
    else begin
     packed_pixels[lane*16+:16]<=pixel;lane<=lane+1;
     if(lane==15)begin if(fr)accepted<=accepted+1;else bad<=1;end
    end
   end
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;request<=0;done_sync<=0;written<=0;payload<=0;rsp_status<=0;end
  else begin
   done_sync<={done_sync[0],finished};
   case(state)
    0:if(cmd_valid)begin
     written<=0;rsp_status<=0;
     if(WIDTH%16!=0||WIDTH==0||HEIGHT==0||BASE%32!=0)begin rsp_status<=1;state<=6;end
     else begin request<=~request;state<=1;end
    end
    1:if(fv)begin payload<=fd;state<=2;end
      else if(done_sync[1]==request&&written==final_words)begin
       if(rsp_status==0&&(final_bad||written!=TOTAL/16))rsp_status<=3;state<=6;
      end
    2:if(axi_awready)state<=3;
    3:if(axi_wready)begin
     if(axi_wusero_last)begin if(axi_wusero_id!=0)rsp_status<=5;state<=5;end
     else state<=4;
    end
    4:if(axi_wusero_last)begin if(axi_wusero_id!=0)rsp_status<=5;state<=5;end
    5:begin written<=written+1;state<=1;end
    6:if(rsp_ready)state<=0;
    default:state<=0;
   endcase
  end
 end
endmodule
