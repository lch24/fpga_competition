// Explicit buffer ownership. FREE -> FILLING -> READY -> READING -> FREE.
// Writers may only acquire FREE buffers. A frame with a failed/overflowed
// write is never published. Readers retain ownership until release, allowing
// calibration, remap, or HDMI to pin a frame for any duration.
module frame_manager #(parameter SLOTS=4,BASE=0,SLOT_BYTES=4194304)(
 input wire clk,rst_n,
 output reg producer_valid,input wire producer_ready,
 output reg [3:0] producer_slot,output wire [31:0] producer_base,
 input wire publish_valid,input wire [3:0] publish_slot,input wire publish_success,
 output reg consumer_valid,input wire consumer_ready,
 output reg [3:0] consumer_slot,output wire [31:0] consumer_base,
 input wire release_valid,input wire [3:0] release_slot,
 output reg protocol_error,output wire [SLOTS*2-1:0] debug_owners
);
 localparam FREE=0,FILLING=1,READY=2,READING=3;
 reg [1:0] owner[0:SLOTS-1];integer i,pp,cp;
 reg plock,clocked;reg [3:0] pslot,cslot;
 assign producer_base=BASE+producer_slot*SLOT_BYTES;
 assign consumer_base=BASE+consumer_slot*SLOT_BYTES;
 genvar k;generate for(k=0;k<SLOTS;k=k+1)begin:debug assign debug_owners[k*2+:2]=owner[k];end endgenerate
 always @*begin
  producer_valid=0;consumer_valid=0;producer_slot=0;consumer_slot=0;
  for(i=SLOTS-1;i>=0;i=i-1)begin
   if(owner[(pp+i)%SLOTS]==FREE)begin producer_valid=1;producer_slot=(pp+i)%SLOTS;end
   if(owner[(cp+i)%SLOTS]==READY)begin consumer_valid=1;consumer_slot=(cp+i)%SLOTS;end
  end
  if(plock)begin producer_valid=1;producer_slot=pslot;end
  if(clocked)begin consumer_valid=1;consumer_slot=cslot;end
 end
 integer j;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin for(j=0;j<SLOTS;j=j+1)owner[j]<=FREE;pp<=0;cp<=0;protocol_error<=0;plock<=0;clocked<=0;pslot<=0;cslot<=0;end
  else begin
   if(producer_valid&&!producer_ready)begin plock<=1;pslot<=producer_slot;end
   if(consumer_valid&&!consumer_ready)begin clocked<=1;cslot<=consumer_slot;end
   if(producer_valid&&producer_ready)begin owner[producer_slot]<=FILLING;pp<=(producer_slot+1)%SLOTS;plock<=0;end
   if(consumer_valid&&consumer_ready)begin owner[consumer_slot]<=READING;cp<=(consumer_slot+1)%SLOTS;clocked<=0;end
   if(publish_valid)begin
    if(publish_slot>=SLOTS||owner[publish_slot]!=FILLING)protocol_error<=1;
    else owner[publish_slot]<=publish_success?READY:FREE;
   end
   if(release_valid)begin
    if(release_slot>=SLOTS||owner[release_slot]!=READING)protocol_error<=1;
    else owner[release_slot]<=FREE;
   end
  end
 end
endmodule
