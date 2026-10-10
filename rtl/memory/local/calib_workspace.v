// Single-owner synchronous workspace. No array reset: preserves block-RAM
// inference. Task adapter initializes live regions before starting a program.
// Two independent ports, one access each per cycle, reads have one-cycle delay.
// Ownership logic must forbid simultaneous accesses when either is a write
// to the same address; no dependence on device-specific collision semantics.
module calib_workspace #(parameter WORDS=4096,ADDR_W=$clog2(WORDS))(
 input wire clk,
 input wire a_en,a_write,input wire [ADDR_W-1:0] a_addr,input wire [63:0] a_data,
 output reg [63:0] a_result,
 input wire b_en,b_write,input wire [ADDR_W-1:0] b_addr,input wire [63:0] b_data,
 output reg [63:0] b_result
);
 reg [63:0] memory[0:WORDS-1];
 always @(posedge clk)if(a_en)begin
  if(a_write)memory[a_addr]<=a_data;
  else a_result<=memory[a_addr];
 end
 always @(posedge clk)if(b_en)begin
  if(b_write)memory[b_addr]<=b_data;
  else b_result<=memory[b_addr];
 end
endmodule
