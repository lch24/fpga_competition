// Power-of-two asynchronous FIFO. Pointer CDC uses Gray code plus two flops.
// Distributed/asynchronous-read RAM, not a synchronous-read BRAM wrapper.
// Reset both sides together; PAP_ASYNC_REG marks the PDS synchronizer stages.
// Gray-bus skew/delay and final synchronizer placement still need timing review.
module async_fifo #(parameter WIDTH=16,ADDR_BITS=8)(
 input wire wclk,wrst_n,wvalid,output wire wready,input wire [WIDTH-1:0] wdata,
 input wire rclk,rrst_n,output wire rvalid,input wire rready,output wire [WIDTH-1:0] rdata
);
 localparam DEPTH=1<<ADDR_BITS;
 reg [WIDTH-1:0] mem[0:DEPTH-1];
 reg [ADDR_BITS:0] wb,wg,rb,rg;
 reg [ADDR_BITS:0] rg1,rg2,wg1,wg2 /* synthesis PAP_ASYNC_REG=1 */;
 wire [ADDR_BITS:0] wn=wb+{{ADDR_BITS{1'b0}},wvalid&&wready};
 wire [ADDR_BITS:0] rn=rb+{{ADDR_BITS{1'b0}},rvalid&&rready};
 wire [ADDR_BITS:0] wgn=(wn>>1)^wn,rgn=(rn>>1)^rn;
 wire full=wg=={~rg2[ADDR_BITS:ADDR_BITS-1],rg2[ADDR_BITS-2:0]};
 assign wready=!full;assign rvalid=rg!=wg2;assign rdata=mem[rb[ADDR_BITS-1:0]];
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
