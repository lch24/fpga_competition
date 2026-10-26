`timescale 1ns/1ps
// Paint small red squares into a disposable COPY of a photo, never the source.
// Sequential coordinate scan / one masked native write per marked pixel; no
// per-video-pixel comparator bank, extra frame RAM, or floating-point operator.
module corner_review_dma #(parameter WIDTH=1280,HEIGHT=720,POINTS=40,VIEWS=3)(
 input wire clk,rst_n,point_valid,input wire [71:0] point,
 output wire points_complete,
 input wire cmd_valid,output wire cmd_ready,input wire [1:0] view,input wire [31:0] dst_base,
 output wire rsp_valid,input wire rsp_ready,output reg [7:0] status,output wire busy,
 output wire awvalid,input wire awready,output wire [27:0] awaddr,output wire [3:0] awlen,
 output wire [255:0] wdata,output wire [31:0] wstrb,input wire wready,wdone,input wire [3:0] wid
);
 function [12:0] round_pixel(input [31:0] value);
  reg [11:0] n;reg valid;integer k;
  begin
   n=0;valid=!value[31]&&value[30:23]<138;
   if(value[30:23]==126)n=1;
   for(k=0;k<=10;k=k+1)
    if(value[30:23]==127+k)n=({1'b1,value[22:0]}>>(23-k))+((value>>(22-k))&1);
   round_pixel={valid,n};
  end
 endfunction
 localparam TOTAL=POINTS*VIEWS;
 localparam [3:0] IDLE=0,READ=1,PREP=2,ADDRESS=3,DATA=4,COMPLETE=5,NEXT=6,RESPONSE=7;
 reg [3:0] state;
 (* syn_ramstyle="block_ram" *) reg [24:0] coordinates[0:TOTAL-1];
 reg [24:0] coordinate;reg [7:0] count,index,first_index;
 wire [12:0] px=round_pixel(point[31:0]),py=round_pixel(point[63:32]);
 wire [1:0] pv=point[71:70];wire [5:0] pi=point[69:64];
 wire [7:0] address=pv*POINTS+pi;
 wire point_ok=px[12]&&py[12]&&px[11:0]<WIDTH&&py[11:0]<HEIGHT;
 assign points_complete=count==TOTAL;
 always @(posedge clk)begin
  if(point_valid&&pv<VIEWS&&pi<POINTS)coordinates[address]<={point_ok,py[11:0],px[11:0]};
  if(state==READ)coordinate<=coordinates[index];
 end
 always @(posedge clk or negedge rst_n)
  if(!rst_n)count<=0;
  else if(point_valid)begin
   // New job starts with view 0 / point 0. Missing/out-of-order packets must
   // never publish stale coordinates from the previous job.
   if(address==0&&pv==0&&pi==0)count<=1;
   else if(pv<VIEWS&&pi<POINTS&&address==count&&count<TOTAL)count<=count+1'b1;
   else count<=0;
  end
 reg [2:0] ox,oy;reg [31:0] base,byte_address;
 wire signed [12:0] x=$signed({1'b0,coordinate[11:0]})+$signed({10'b0,ox})-13'sd2;
 wire signed [12:0] y=$signed({1'b0,coordinate[23:12]})+$signed({10'b0,oy})-13'sd2;
 wire within_image=coordinate[24]&&x>=0&&x<WIDTH&&y>=0&&y<HEIGHT;
 assign cmd_ready=state==IDLE;assign busy=state!=IDLE;assign rsp_valid=state==RESPONSE;
 assign awvalid=state==ADDRESS;assign awlen=0;
 assign awaddr=(byte_address>>2)&28'hffffff8;
 // HMIC WSTRB has one bit per byte, native beat is 32 bytes / 16 pixels.
 assign wstrb=32'h3<<byte_address[4:0];
 assign wdata=256'hf800<<(byte_address[4:0]*8);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;index<=0;first_index<=0;ox<=0;oy<=0;base<=0;byte_address<=0;status<=0;end
  else case(state)
   IDLE:if(cmd_valid)begin
    status<=0;base<=dst_base;index<=view*POINTS;first_index<=view*POINTS;ox<=0;oy<=0;
    if(!points_complete||view>=VIEWS||dst_base[4:0]!=0)begin status<=1;state<=RESPONSE;end else state<=READ;
   end
   READ:state<=PREP;
   PREP:if(within_image)begin byte_address<=base+y*(2*WIDTH)+x*2;state<=ADDRESS;end else state<=NEXT;
   ADDRESS:if(awready)state<=DATA;
   DATA:if(wready)begin
    if(wdone)begin if(wid!=0)status<=5;state<=NEXT;end else state<=COMPLETE;
   end
   COMPLETE:if(wdone)begin if(wid!=0)status<=5;state<=NEXT;end
   NEXT:begin
    if(ox!=4)begin ox<=ox+1'b1;state<=PREP;end
    else begin
     ox<=0;
     if(oy!=4)begin oy<=oy+1'b1;state<=PREP;end
     else begin oy<=0;if(index-first_index==POINTS-1)state<=RESPONSE;else begin index<=index+1'b1;state<=READ;end end
    end
   end
   RESPONSE:if(rsp_ready)state<=IDLE;
   default:state<=IDLE;
  endcase
 end
endmodule
