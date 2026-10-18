`timescale 1ns/1ps
// Board mode reads a full immutable frame at a runtime-selected DDR base.
// Default fixed-base mode retains the standalone half-frame comparison API.
// Deassert enable, wait idle, then change base/owner; accepted reads drain.
// Read up to 16 HMIC beats into a local buffer; stream to hdmi_pixel_bridge.
// HMIC addresses count 32-bit words; one beat contains sixteen RGB565 pixels.
// WIDTH multiple of 32, both bases 32-byte aligned, tightly packed frames.
module board_display_hmic #(
 parameter DYNAMIC_BASE=0,WIDTH=1280,HEIGHT=720,RAW_BASE=32'h0,DST_BASE=32'h01000000
)(
 input wire clk,rst_n,enable,input wire [31:0] frame_base,output wire idle,
 output wire frame_start,
 output wire pixel_valid,input wire pixel_ready,output wire [15:0] pixel,
 output wire pixel_first,pixel_last,output reg error,
 output wire [27:0] axi_araddr,output wire [3:0] axi_arlen,axi_aruser_id,
 output wire axi_arvalid,input wire axi_arready,
 input wire [255:0] axi_rdata,input wire axi_rvalid,axi_rlast,input wire [3:0] axi_rid
);
 reg [2:0] state;reg half;reg [15:0] x,y;
 reg [8:0] size,pos;reg [4:0] beats,received;
 reg [255:0] buffer[0:15];
 // Dynamic mode presents a whole selected frame; legacy mode compares halves.
 reg [31:0] base;
 reg out_valid,out_first,out_last;reg [15:0] out_pixel;

 localparam LINE_PIXELS=DYNAMIC_BASE?WIDTH:WIDTH/2;
 wire [31:0] byte_addr=(DYNAMIC_BASE?base:(half?DST_BASE:RAW_BASE))+y*(2*WIDTH)+2*x;
 assign idle=(state==0||state==4)&&!out_valid;
 assign frame_start=state==0&&enable&&x==0&&y==0&&!half;
 assign axi_araddr=byte_addr>>2;
 assign axi_arlen=beats-1;assign axi_aruser_id=0;assign axi_arvalid=state==1;
 // Elastic output register separates the 256:1 pixel selection from the
 // HDMI FIFO input. One pixel/clock after filling; payload holds under stalls.
 wire output_ready=!out_valid||pixel_ready;
 assign pixel_valid=out_valid&&enable;
 assign pixel=out_pixel;assign pixel_first=out_first;assign pixel_last=out_last;
 wire [15:0] next_pixel=(!DYNAMIC_BASE&&half&&x==0&&pos==0)?16'hffff:buffer[pos[7:4]][pos[3:0]*16+:16];
 wire next_first=!half&&y==0&&x==0&&pos==0;
 wire next_last=(DYNAMIC_BASE||half)&&y==HEIGHT-1&&x+pos==LINE_PIXELS-1;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin out_valid<=0;out_first<=0;out_last<=0;out_pixel<=0;end
  else if(!enable)out_valid<=0;
  else if(output_ready)begin
   out_valid<=state==3;
   if(state==3)begin out_pixel<=next_pixel;out_first<=next_first;out_last<=next_last;end
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;half<=0;x<=0;y<=0;size<=0;pos<=0;beats<=0;received<=0;error<=0;base<=0;end
  else case(state)
   0:if(enable)begin
    if(WIDTH%32!=0||WIDTH==0||HEIGHT==0||(DYNAMIC_BASE?frame_base%32!=0:(RAW_BASE%32!=0||DST_BASE%32!=0)))begin error<=1;state<=4;end
    else begin
     if(x==0&&y==0&&!half)base<=frame_base;
     size<=LINE_PIXELS-x>=256?256:LINE_PIXELS-x;
     beats<=LINE_PIXELS-x>=256?16:(LINE_PIXELS-x)/16;
     received<=0;pos<=0;state<=1;
    end
   end else begin x<=0;y<=0;half<=0;error<=0;end
   1:if(axi_arready)state<=2;
   2:if(axi_rvalid)begin
    buffer[received[3:0]]<=axi_rdata;received<=received+1;
    if(axi_rid!=0||axi_rlast!=(received==beats-1))error<=1;
    if(axi_rlast)begin
     if(error||axi_rid!=0||received!=beats-1)state<=4;else if(!enable)begin state<=0;x<=0;y<=0;half<=0;end else state<=3;
    end
   end
   3:if(!enable)begin state<=0;x<=0;y<=0;half<=0;end
    else if(output_ready)begin
    if(pos==size-1)begin
     pos<=0;state<=0;
     if(x+size==LINE_PIXELS)begin x<=0;half<=DYNAMIC_BASE?0:~half;if(DYNAMIC_BASE||half)y<=y==HEIGHT-1?0:y+1;end
     else x<=x+size;
    end else pos<=pos+1;
   end
   4:if(!enable)begin state<=0;x<=0;y<=0;half<=0;error<=0;end // Stop after draining erroneous burst. Reset jointly with DDR.
   default:state<=0;
  endcase
 end
endmodule
