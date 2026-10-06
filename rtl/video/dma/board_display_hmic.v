`timescale 1ns/1ps
// Immutable frame comparison: left half reads raw x=0..W/2-1, right half
// reads corrected x=0..W/2-1, with a white separator. No spatial scaling.
// Read up to 16 HMIC beats into a local buffer; stream to hdmi_pixel_bridge.
// HMIC addresses count 32-bit words; one beat contains sixteen RGB565 pixels.
// WIDTH multiple of 32, both bases 32-byte aligned, tightly packed frames.
module board_display_hmic #(
 parameter WIDTH=1280,HEIGHT=720,RAW_BASE=32'h0,DST_BASE=32'h01000000
)(
 input wire clk,rst_n,enable,
 output wire pixel_valid,input wire pixel_ready,output wire [15:0] pixel,
 output wire pixel_first,pixel_last,output reg error,
 output wire [27:0] axi_araddr,output wire [3:0] axi_arlen,axi_aruser_id,
 output wire axi_arvalid,input wire axi_arready,
 input wire [255:0] axi_rdata,input wire axi_rvalid,axi_rlast,input wire [3:0] axi_rid
);
 reg [2:0] state;reg half;reg [15:0] x,y;
 reg [8:0] size,pos;reg [4:0] beats,received;
 reg [255:0] buffer[0:15];
 wire [31:0] byte_addr=(half?DST_BASE:RAW_BASE)+y*(2*WIDTH)+2*x;
 assign axi_araddr=byte_addr>>2;
 assign axi_arlen=beats-1;assign axi_aruser_id=0;assign axi_arvalid=state==1;
 assign pixel_valid=state==3;
 assign pixel=(half&&x==0&&pos==0)?16'hffff:buffer[pos[7:4]][pos[3:0]*16+:16];
 assign pixel_first=!half&&y==0&&x==0&&pos==0;
 assign pixel_last=half&&y==HEIGHT-1&&x+pos==WIDTH/2-1;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;half<=0;x<=0;y<=0;size<=0;pos<=0;beats<=0;received<=0;error<=0;end
  else case(state)
   0:if(enable)begin
    if(WIDTH%32!=0||WIDTH==0||HEIGHT==0||RAW_BASE%32!=0||DST_BASE%32!=0)begin error<=1;state<=4;end
    else begin
     size<=WIDTH/2-x>=256?256:WIDTH/2-x;
     beats<=WIDTH/2-x>=256?16:(WIDTH/2-x)/16;
     received<=0;pos<=0;state<=1;
    end
   end
   1:if(axi_arready)state<=2;
   2:if(axi_rvalid)begin
    buffer[received[3:0]]<=axi_rdata;received<=received+1;
    if(axi_rid!=0||axi_rlast!=(received==beats-1))error<=1;
    if(axi_rlast)begin
     if(error||axi_rid!=0||received!=beats-1)state<=4;else state<=3;
    end
   end
   3:if(pixel_ready)begin
    if(pos==size-1)begin
     pos<=0;state<=0;
     if(x+size==WIDTH/2)begin x<=0;half<=~half;if(half)y<=y==HEIGHT-1?0:y+1;end
     else x<=x+size;
    end else pos<=pos+1;
   end
   4:state<=4; // Stop after draining erroneous burst. Reset jointly with DDR.
   default:state<=0;
  endcase
 end
endmodule
