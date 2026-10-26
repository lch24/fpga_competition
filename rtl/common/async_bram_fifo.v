`timescale 1ns/1ps
// Dual-clock, synchronous-read BRAM FIFO. FWFT holding register is stable when
// stalled. A consumed word is replaced by a synchronous read of the next word
// on the same edge, so a nonempty stream supports one transfer per read clock.
// Reset both domains jointly, Gray CDC/skew constraints still required.
module async_bram_fifo #(parameter WIDTH=288,ADDR_BITS=8)(
 input wire wclk,wrst_n,wvalid,output wire wready,input wire [WIDTH-1:0] wdata,
 input wire rclk,rrst_n,output wire rvalid,input wire rready,output wire [WIDTH-1:0] rdata,
 output wire [ADDR_BITS:0] wlevel
);
 (* syn_ramstyle="block_ram" *) reg [WIDTH-1:0] memory[0:(1<<ADDR_BITS)-1];
 reg [ADDR_BITS:0] wb,wg,rb,rg;
 reg [ADDR_BITS:0] rg1,rg2,wg1,wg2 /* synthesis PAP_ASYNC_REG=1 */;
 reg loaded,full;reg [WIDTH-1:0] q;
 wire pop=rvalid&&rready;wire push=wvalid&&wready;
 wire [ADDR_BITS:0] wn=wb+push,rn=rb+pop;
 wire [ADDR_BITS:0] wgn=(wn>>1)^wn,rgn=(rn>>1)^rn;
 wire empty_after_pop=rgn==wg2;
 // Compare the NEXT write pointer, including this edge's accepted transfer.
 // Registering full removes pointer comparison from the RAM write-enable path.
 wire full_next=wgn=={~rg2[ADDR_BITS:ADDR_BITS-1],rg2[ADDR_BITS-2:0]};
 wire fetch=(!loaded||pop)&&!empty_after_pop;
 assign wready=!full;
 assign rvalid=loaded;assign rdata=q;
 reg [ADDR_BITS:0] read_binary;integer bi;
 always @*begin read_binary[ADDR_BITS]=rg2[ADDR_BITS];for(bi=ADDR_BITS-1;bi>=0;bi=bi-1)read_binary[bi]=read_binary[bi+1]^rg2[bi];end
 assign wlevel=wb-read_binary;
 always @(posedge wclk)if(push)memory[wb[ADDR_BITS-1:0]]<=wdata;
 // rn is rb when stalled/empty, or rb+1 when the current output is consumed.
 // Keep the current slot occupied until pop; the writer cannot overwrite it
 // while output is stalled. Reset-free q is visible only when loaded is set.
 always @(posedge rclk)if(fetch)q<=memory[rn[ADDR_BITS-1:0]];
 always @(posedge wclk or negedge wrst_n)begin
  if(!wrst_n)begin wb<=0;wg<=0;rg1<=0;rg2<=0;full<=0;end
  else begin rg1<=rg;rg2<=rg1;full<=full_next;if(push)begin wb<=wn;wg<=wgn;end end
 end
 always @(posedge rclk or negedge rrst_n)begin
  if(!rrst_n)begin rb<=0;rg<=0;wg1<=0;wg2<=0;loaded<=0;end
  else begin
   wg1<=wg;wg2<=wg1;
   if(!loaded||pop)loaded<=fetch;
   if(pop)begin rb<=rn;rg<=rgn;end
  end
 end
endmodule
