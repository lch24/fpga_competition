`timescale 1ns/1ps
// Queued frame descriptors are consumed on VSYNC. Unlike the photograph DMA,
// the next descriptor can be queued BEFORE the previous frame ends: no forced
// every-other-frame gap. Ordered START/DATA/END tokens ensure publication is
// after ALL physical write completions. A malformed/overflowed frame is failed,
// never published. Sensor cannot be backpressured; missing descriptors skip a
// WHOLE frame. Pixel clock and DDR clock are independently reset together.
module video_capture_hmic #(parameter WIDTH=1280,HEIGHT=720,FIFO_BITS=8)(
 input wire clk,rst_n,pclk,prst_n,vsync,pixel_valid,input wire [15:0] pixel,
 input wire cmd_valid,output wire cmd_ready,input wire [31:0] cmd_base,
 output wire rsp_valid,input wire rsp_ready,output reg [7:0] rsp_status,output reg [31:0] rsp_base,
 output wire [31:0] skipped_frames,
 output wire [27:0] axi_awaddr,output wire [3:0] axi_awlen,axi_awuser_id,
 output wire axi_awvalid,input wire axi_awready,
 output wire [255:0] axi_wdata,output wire [31:0] axi_wstrb,
 input wire axi_wready,axi_wusero_last,input wire [3:0] axi_wusero_id
);
 localparam TOTAL=WIDTH*HEIGHT;
 wire dv,dr;wire [31:0] db;
 async_fifo #(.WIDTH(32),.ADDR_BITS(2)) descriptors(
  .wclk(clk),.wrst_n(rst_n),.wvalid(cmd_valid),.wready(cmd_ready),.wdata(cmd_base),
  .rclk(pclk),.rrst_n(prst_n),.rvalid(dv),.rready(dr),.rdata(db));
 reg vs_d,active,send_start,send_end,bad;
 reg [31:0] start_base,count,end_status;
 reg [3:0] lane;reg [255:0] pack_word;
 wire edge_vs=vsync&&!vs_d;
 wire [FIFO_BITS:0] level;wire [FIFO_BITS:0] free_words=(1<<FIFO_BITS)-level;
 assign dr=edge_vs&&dv&&!send_start&&!send_end&&free_words>=3;
 wire qv,qr,qready;wire [287:0] qdata;
 wire data_word=active&&pixel_valid&&count<TOTAL&&lane==15;
 wire fv=send_end||send_start||(data_word&&free_words>2);
 wire [287:0] fd=send_end?{2'b10,22'b0,end_status[7:0],256'b0}:
                 send_start?{2'b00,start_base[29:0],256'b0}:{32'h40000000,pixel,pack_word[255:16]};
 async_bram_fifo #(.WIDTH(288),.ADDR_BITS(FIFO_BITS)) words(
  .wclk(pclk),.wrst_n(prst_n),.wvalid(fv),.wready(qready),.wdata(fd),
  .rclk(clk),.rrst_n(rst_n),.rvalid(qv),.rready(qr),.rdata(qdata),.wlevel(level));
 reg [31:0] skipped,skip_gray;reg [31:0] sg1,sg2 /* synthesis PAP_ASYNC_REG=1 */;
 integer bit_i;reg [31:0] skip_binary;
 always @*begin skip_binary[31]=sg2[31];for(bit_i=30;bit_i>=0;bit_i=bit_i-1)skip_binary[bit_i]=skip_binary[bit_i+1]^sg2[bit_i];end
 assign skipped_frames=skip_binary;
 always @(posedge clk or negedge rst_n)if(!rst_n)begin sg1<=0;sg2<=0;end else begin sg1<=skip_gray;sg2<=sg1;end
 always @(posedge pclk or negedge prst_n)begin
  if(!prst_n)begin vs_d<=0;active<=0;send_start<=0;send_end<=0;bad<=0;count<=0;lane<=0;pack_word<=0;start_base<=0;end_status<=0;skipped<=0;skip_gray<=0;end
  else begin
   vs_d<=vsync;
   if(send_end&&qready)send_end<=0;
   else if(send_start&&qready)send_start<=0;
   if(edge_vs)begin
    if(active)begin send_end<=1;end_status<=bad||count!=TOTAL||lane!=0?3:0;end
    active<=dr;count<=0;lane<=0;bad<=0;
    if(dr)begin start_base<=db;send_start<=1;if(db[4:0]!=0||db[31:30]!=0)bad<=1;end
    else begin skipped<=skipped+1;skip_gray<=((skipped+1)>>1)^(skipped+1);end
   end else if(active&&pixel_valid)begin
    count<=count+1;
    if(count>=TOTAL)bad<=1;
    else begin
     pack_word<={pixel,pack_word[255:16]};lane<=lane+1;
     if(lane==15&&(!qready||free_words<=2||send_start||send_end))bad<=1;
    end
   end
  end
 end
 localparam WAIT_TOKEN=0,ADDRESS=1,DATA=2,COMPLETE=3,RESPONSE=4;
 reg [2:0] state;reg [31:0] base,written;reg frame_open;
 reg [255:0] payload;
 assign qr=state==WAIT_TOKEN;
 assign rsp_valid=state==RESPONSE;
 assign axi_awaddr=(base>>2)+(written<<3);assign axi_awlen=0;assign axi_awuser_id=0;
 assign axi_awvalid=state==ADDRESS;assign axi_wdata=payload;assign axi_wstrb=32'hffffffff;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=WAIT_TOKEN;base<=0;written<=0;frame_open<=0;payload<=0;rsp_status<=0;rsp_base<=0;end
  else case(state)
   WAIT_TOKEN:if(qv)begin
    case(qdata[287:286])
     0:begin base<={2'b0,qdata[285:256]};rsp_base<={2'b0,qdata[285:256]};written<=0;frame_open<=1;rsp_status<=0;end
     1:if(frame_open&&written<TOTAL/16)begin payload<=qdata[255:0];state<=ADDRESS;end else rsp_status<=3;
     2:begin
      if(rsp_status==0)rsp_status<=qdata[263:256]!=0?qdata[263:256]:(written!=TOTAL/16?3:0);
      frame_open<=0;state<=RESPONSE;
     end
     default:rsp_status<=3;
    endcase
   end
   ADDRESS:if(axi_awready)state<=DATA;
   DATA:if(axi_wready)begin
    if(axi_wusero_last)begin if(axi_wusero_id!=0)rsp_status<=5;written<=written+1;state<=WAIT_TOKEN;end
    else state<=COMPLETE;
   end
   COMPLETE:if(axi_wusero_last)begin if(axi_wusero_id!=0)rsp_status<=5;written<=written+1;state<=WAIT_TOKEN;end
   RESPONSE:if(rsp_ready)state<=WAIT_TOKEN;
   default:state<=WAIT_TOKEN;
  endcase
 end
endmodule
