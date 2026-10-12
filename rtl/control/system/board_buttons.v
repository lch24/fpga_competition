`timescale 1ns/1ps
// Active-low mechanical keys: two-flop CDC, stable press/release debounce,
// one pulse per press. A held key never repeats. Default: 20 ms at 125 MHz.
module board_buttons #(parameter DEBOUNCE_CYCLES=2500000)(
 input wire clk,rst_n,input wire [4:0] key_n,output reg [4:0] pressed
);
 localparam CW=DEBOUNCE_CYCLES<2?1:$clog2(DEBOUNCE_CYCLES);
 reg [4:0] meta,sync /* synthesis PAP_ASYNC_REG=1 */;
 reg [4:0] stable;
 reg [CW-1:0] count[0:4];
 integer i;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin meta<=5'b11111;sync<=5'b11111;end
  else begin meta<=key_n;sync<=meta;end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   stable<=5'b11111;pressed<=0;
   for(i=0;i<5;i=i+1)count[i]<=0;
  end else begin
   pressed<=0;
   for(i=0;i<5;i=i+1)begin
    if(sync[i]==stable[i])count[i]<=0;
    else if(DEBOUNCE_CYCLES<=1||count[i]==DEBOUNCE_CYCLES-1)begin
     stable[i]<=sync[i];count[i]<=0;pressed[i]<=!sync[i];
    end else count[i]<=count[i]+1'b1;
   end
  end
 end
endmodule
