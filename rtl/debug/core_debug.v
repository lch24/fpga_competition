`timescale 1ns/1ps
// Read-only observation, never influences DDR ownership or ready signals.
// Counters wrap modulo 2^32. Errors and busy-key presses remain visible.
module core_debug(
 input wire clk,rst_n,functional_reset_n,input wire [31:0] state_word,
 input wire [4:0] pressed,input wire busy,display_error,underflow,
 input wire read_accept,read_return,write_accept,write_return,
 input wire [31:0] read_addr,write_addr,output wire [255:0] snapshot
);
 reg [31:0] previous,events,reads,returns,writes,last_addr,dwell;
 reg [15:0] ignored;
 reg [7:0] first_error;reg sticky_display,sticky_underflow;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin previous<=0;dwell<=0;events<=0;reads<=0;returns<=0;writes<=0;last_addr<=0;
   ignored<=0;first_error<=0;sticky_display<=0;sticky_underflow<=0;end
  else begin
   previous<=state_word;
   if(previous!=state_word)dwell<=0;else if(dwell!=32'hffffffff)dwell<=dwell+1'b1;
   if(previous!=state_word)events<=events+1'b1;
   if(functional_reset_n)begin
    if(busy&&|pressed&&ignored!=16'hffff)ignored<=ignored+1'b1;
    if(first_error==0&&state_word[23:16]!=0)first_error<=state_word[23:16];
    if(display_error)sticky_display<=1;if(underflow)sticky_underflow<=1;
    if(read_accept)begin reads<=reads+1'b1;last_addr<=read_addr;end
    if(write_accept)begin writes<=writes+1'b1;last_addr<=write_addr;end
    if(read_return||write_return)returns<=returns+1'b1;
   end
  end
 end
 assign snapshot={ignored,first_error,5'b0,sticky_underflow,sticky_display,functional_reset_n,
  last_addr,writes,returns,reads,dwell,events,state_word};
endmodule
