// Power-of-two asynchronous FIFO. Pointer CDC uses Gray code plus two flops.
// Distributed/asynchronous-read RAM, not a synchronous-read BRAM wrapper.
// Reset both sides together; PAP_ASYNC_REG marks the PDS synchronizer stages.
// Gray-bus skew/delay and final synchronizer placement still need timing review.
module async_fifo #(parameter WIDTH=16,ADDR_BITS=8,REGISTER_READ_VALID=0)(
 input wire wclk,wrst_n,wvalid,output wire wready,input wire [WIDTH-1:0] wdata,
 input wire rclk,rrst_n,output wire rvalid,input wire rready,output wire [WIDTH-1:0] rdata,
 output wire [ADDR_BITS:0] wlevel
);
 localparam DEPTH=1<<ADDR_BITS;
 reg [WIDTH-1:0] mem[0:DEPTH-1];
 reg [ADDR_BITS:0] wb,wg,rb,rg;
 reg [ADDR_BITS:0] rg1,rg2,wg1,wg2 /* synthesis PAP_ASYNC_REG=1 */;
 wire [ADDR_BITS:0] wn=wb+{{ADDR_BITS{1'b0}},wvalid&&wready};
 wire [ADDR_BITS:0] rn=rb+{{ADDR_BITS{1'b0}},rvalid&&rready};
 wire [ADDR_BITS:0] wgn=(wn>>1)^wn,rgn=(rn>>1)^rn;
 wire full=wg=={~rg2[ADDR_BITS:ADDR_BITS-1],rg2[ADDR_BITS-2:0]};
 reg [ADDR_BITS:0] read_binary;integer bi;
 always @*begin read_binary[ADDR_BITS]=rg2[ADDR_BITS];for(bi=ADDR_BITS-1;bi>=0;bi=bi-1)read_binary[bi]=read_binary[bi+1]^rg2[bi];end
 assign wlevel=wb-read_binary;
 assign wready=!full;assign rdata=mem[rb[ADDR_BITS-1:0]];
 // Optional registered empty test for command/control consumers. Compare the
 // NEXT read pointer, so the last dequeue clears valid immediately. Newly
 // synchronized writes become visible one read clock later; sustained II=1
 // and data stability under backpressure are unchanged. No payload copy.
 generate if(REGISTER_READ_VALID)begin : g_read_valid
  reg available;
  always @(posedge rclk or negedge rrst_n)
   if(!rrst_n)available<=0;else available<=rgn!=wg2;
  assign rvalid=available;
 end else begin : g_comb_read_valid
  assign rvalid=rg!=wg2;
 end endgenerate
 always @(posedge wclk or negedge wrst_n)begin
  if(!wrst_n)begin wb<=0;wg<=0;rg1<=0;rg2<=0;end
  else begin
   rg1<=rg;rg2<=rg1;
   if(wvalid&&wready)begin mem[wb[ADDR_BITS-1:0]]<=wdata;wb<=wn;wg<=wgn;end
  end
 end
 always @(posedge rclk or negedge rrst_n)begin
  if(!rrst_n)begin rb<=0;rg<=0;wg1<=0;wg2<=0;end
  else begin wg1<=wg;wg2<=wg1;if(rvalid&&rready)begin rb<=rn;rg<=rgn;end end
 end
endmodule
