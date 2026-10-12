`timescale 1ns/1ps
// Two 256-pixel BRAM banks. A native burst is issued only after all its data
// is staged: HMIC has no WVALID. Frame completion means final WUSERO_LAST.
module video_frame_writer #(parameter PIXELS=1280*720)(
 input wire clk,rst_n,start,input wire [31:0] dst_base,
 input wire in_valid,output wire in_ready,input wire [15:0] pixel,
 output reg done,error,
 output wire awvalid,input wire awready,output wire [27:0] awaddr,
 output wire [3:0] awlen,output wire [255:0] wdata,
 input wire wready,wdone,input wire [3:0] wid
);
 localparam IDLE=0,LOAD=1,ADDRESS=2,DATA=3,COMPLETE=4;
 reg [2:0] state;reg active,wb,rb;reg [1:0] full;
 reg [7:0] wp;reg [3:0] rp;reg [31:0] accepted,written,base;
 reg [255:0] pack,q;
 (* syn_ramstyle="block_ram" *) reg [255:0] memory[0:31];
 assign in_ready=active&&accepted<PIXELS&&!full[wb];
 wire take=in_valid&&in_ready;
 wire [3:0] next_rp=state==DATA&&wready?rp+1'b1:rp;
 always @(posedge clk)begin
  if(take&&wp[3:0]==15)memory[{wb,wp[7:4]}]<={pixel,pack[255:16]};
  q<=memory[{rb,next_rp}];
 end
 assign awvalid=state==ADDRESS;assign awaddr=(base>>2)+(written<<7);
 assign awlen=15;assign wdata=q;
 wire completed=(state==DATA&&wready&&rp==15&&wdone)||(state==COMPLETE&&wdone);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;active<=0;wb<=0;rb<=0;full<=0;wp<=0;rp<=0;accepted<=0;written<=0;base<=0;pack<=0;done<=0;error<=0;end
  else if(start)begin state<=IDLE;active<=1;wb<=0;rb<=0;full<=0;wp<=0;rp<=0;accepted<=0;written<=0;base<=dst_base;done<=0;error<=0;end
  else begin
   done<=0;
   if(take)begin
    pack<={pixel,pack[255:16]};accepted<=accepted+1;wp<=wp+1'b1;
    if(wp==255)begin full[wb]<=1;wb<=~wb;end
   end
   case(state)
    IDLE:if(full[rb])begin rp<=0;state<=LOAD;end
    LOAD:state<=ADDRESS;
    ADDRESS:if(awready)state<=DATA;
    DATA:if(wready)begin
     if(wdone&&(rp!=15||wid!=0))error<=1;
     if(rp==15)state<=wdone?IDLE:COMPLETE;else rp<=rp+1'b1;
    end
    COMPLETE:if(wdone)state<=IDLE;
    default:state<=IDLE;
   endcase
   if(completed)begin
    if(wid!=0)error<=1;
    full[rb]<=0;rb<=~rb;written<=written+1;
    if(written==PIXELS/256-1)begin active<=0;done<=1;end
   end
  end
 end
endmodule
