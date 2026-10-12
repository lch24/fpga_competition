`timescale 1ns/1ps
// 8N1 byte transport. TX data is captured on valid/ready; RX uses only
// synchronized samples, validates start/stop bits, and pulses valid once.
module debug_uart #(parameter CLOCK_HZ=27000000, BAUD=115200)(
 input wire clk,rst_n,rx,output reg tx,
 input wire [7:0] tx_data,input wire tx_valid,output wire tx_ready,
 output reg [7:0] rx_data,output reg rx_valid,rx_error
);
 localparam DIV=(CLOCK_HZ+BAUD/2)/BAUD;
 localparam CW=$clog2(DIV+1);
 reg [CW-1:0] tc,rc;
 reg [3:0] tn,rn;
 reg [9:0] shift;
 reg [7:0] data;
 reg [2:0] state;
 reg [1:0] sync_rx /* synthesis PAP_ASYNC_REG=1 */;
 assign tx_ready=tn==0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin tx<=1;tc<=0;tn<=0;shift<=10'h3ff;end
  else if(tn==0)begin
   if(tx_valid)begin shift<={1'b1,tx_data,1'b0};tx<=0;tn<=10;tc<=DIV-1;end
  end else if(tc!=0)tc<=tc-1'b1;
  else begin tc<=DIV-1;shift<={1'b1,shift[9:1]};tx<=shift[1];tn<=tn-1'b1;end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin sync_rx<=3;rc<=0;rn<=0;state<=0;data<=0;rx_data<=0;rx_valid<=0;rx_error<=0;end
  else begin
   sync_rx<={sync_rx[0],rx};rx_valid<=0;rx_error<=0;
   case(state)
    0:if(!sync_rx[1])begin rc<=DIV/2-1;state<=1;end
    1:if(rc!=0)rc<=rc-1'b1;else if(sync_rx[1])state<=0;
      else begin rc<=DIV-1;rn<=0;state<=2;end
    2:if(rc!=0)rc<=rc-1'b1;else begin
      data[rn[2:0]]<=sync_rx[1];rc<=DIV-1;
      if(rn==7)state<=3;else rn<=rn+1'b1;
     end
    3:if(rc!=0)rc<=rc-1'b1;else begin
      if(sync_rx[1])begin rx_data<=data;rx_valid<=1;end else rx_error<=1;
      // A break remains here until the line returns high.
      if(sync_rx[1])state<=0;else state<=4;
     end
    4:if(sync_rx[1])state<=0;
   endcase
  end
 end
endmodule
