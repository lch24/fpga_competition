`timescale 1ns/1ps
// Bundled-data mailbox. Both halves reset ONLY with debug POR, independently
// of the functional domain. A stopped source leaves last data and age intact.
// Request argument and source holding register remain stable through ACK.
module debug_snapshot #(parameter WIDTH=256)(
 input wire clk,rst_n,poll,input wire [15:0] argument,
 output reg [WIDTH-1:0] data,output reg [15:0] tag,output reg update,
 input wire src_clk,input wire [WIDTH-1:0] src_data,
 output wire [15:0] src_argument
);
 reg request,ack;
 reg [15:0] arg_hold;
 reg [WIDTH-1:0] hold_data;
 reg [1:0] req_sync,ack_sync /* synthesis PAP_ASYNC_REG=1 */;
 reg [2:0] settle;
 wire source_rst_n;
 reset_sync source_reset(.clk(src_clk),.arst_n(rst_n),.rst_n(source_rst_n));
 assign src_argument=arg_hold;
 always @(posedge src_clk or negedge source_rst_n)begin
  if(!source_rst_n)begin req_sync<=0;ack<=0;settle<=0;hold_data<=0;end
  else begin
   req_sync<={req_sync[0],request};
   if(req_sync[1]!=ack)begin
    // Also allows a synchronous debug RAM read to finish.
    if(settle==3)begin hold_data<=src_data;ack<=req_sync[1];settle<=0;end
    else settle<=settle+1'b1;
   end else settle<=0;
  end
 end
 reg pending;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin request<=0;arg_hold<=0;ack_sync<=0;pending<=0;data<=0;tag<=0;update<=0;end
  else begin
   ack_sync<={ack_sync[0],ack};update<=0;
   if(pending&&ack_sync[1]==request)begin data<=hold_data;tag<=arg_hold;update<=1;pending<=0;end
   else if(!pending&&poll)begin arg_hold<=argument;request<=~request;pending<=1;end
  end
 end
endmodule
