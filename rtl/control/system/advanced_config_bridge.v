`timescale 1ns/1ps
// Sys-clock shadow registers -> CRC-checked held mailbox -> core frame boundary.
// Joint asynchronous reset, synchronously released in BOTH domains by caller.
// Payload held from CRC validation until round-trip ack; only toggles get 2FFs.
module advanced_config_bridge(
 input wire clk,rst_n,write_valid,input wire [7:0] write_op,input wire [15:0] write_data,
 output wire busy,output reg [31:0] errors,
 input wire core_clk,core_rst_n,output wire cfg_valid,input wire cfg_ready,output wire [207:0] cfg_payload
);
 localparam [207:0] IDENTITY={16'd0,32'd0,32'd65536,32'd0,32'd0,32'd0,32'd65536};
 localparam IDLE=0,CRC=1,ACK=2;
 // Writes are rejected throughout CRC/ACK, so shadow itself is the held
 // mailbox. A second 208-bit payload register would be redundant area.
 reg [1:0] state;reg [207:0] shadow;reg [12:0] written;
 reg [15:0] expected,crc;reg [4:0] at;reg request,ack;
 reg [1:0] ack_sync,req_sync /* synthesis PAP_ASYNC_REG=1 */;
 function [15:0] crc_byte;input [15:0] c;input [7:0] b;reg [15:0] v;integer k;
  begin v=c^{b,8'b0};for(k=0;k<8;k=k+1)v=v[15]?(v<<1)^16'h1021:v<<1;crc_byte=v;end
 endfunction
 wire [15:0] next_crc=crc_byte(crc,shadow[at*8+:8]);
 assign busy=state!=IDLE;assign cfg_payload=shadow;
 assign cfg_valid=core_rst_n&&(req_sync[1]!=ack);
 always @(posedge core_clk or negedge core_rst_n)
  if(!core_rst_n)begin req_sync<=0;ack<=0;end
  else begin req_sync<={req_sync[0],request};if(cfg_valid&&cfg_ready)ack<=req_sync[1];end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;shadow<=IDENTITY;written<=0;expected<=0;crc<=16'hffff;at<=0;request<=0;ack_sync<=0;errors<=0;end
  else begin
   ack_sync<={ack_sync[0],ack};
   if(write_valid)begin
    if(state!=IDLE)errors<=errors+1'b1;
    else if(write_op>=8'h60&&write_op<=8'h6c)begin
     shadow[(write_op-8'h60)*16+:16]<=write_data;written[write_op-8'h60]<=1;
    end else if(write_op==8'h6d)begin
     written<=0;
     if(!(&written)||shadow[207:197]!=0||shadow[196:194]>5)errors<=errors+1'b1;
     else begin expected<=write_data;crc<=16'hffff;at<=0;state<=CRC;end
    end
   end
   case(state)
    CRC:begin
     crc<=next_crc;
     if(at==25)begin
      if(next_crc!=expected)begin errors<=errors+1'b1;state<=IDLE;end
      else begin request<=!request;state<=ACK;end
     end else at<=at+1'b1;
    end
    ACK:if(ack_sync[1]==request)state<=IDLE;
   endcase
  end
 end
endmodule
