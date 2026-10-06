// One scalar (1..4 bytes) transaction. Capture data/address atomically,
// send command, data, then wait for DDR completion. Reused for map and pixels.
module output_writer (
 input wire clk,rst_n,input wire in_valid,output wire in_ready,
 input wire [31:0] in_addr,in_data,input wire [2:0] in_bytes,
 output wire out_valid,input wire out_ready,output reg out_error,
 output wire wr_valid,input wire wr_ready,output wire [31:0] wr_addr,wr_len,
 output wire [15:0] wr_tag,
 output wire w_valid,input wire w_ready,output wire [31:0] w_data,
 output wire [3:0] w_keep,output wire w_last,
 input wire b_valid,output wire b_ready,input wire b_error,input wire [15:0] b_tag
);
 reg [2:0] state,bytes;reg [31:0] addr,data;
 assign in_ready=state==0;assign out_valid=state==4;
 assign wr_valid=state==1;assign wr_addr=addr;assign wr_len={29'd0,bytes};assign wr_tag=0;
 assign w_valid=state==2;assign w_data=data;assign w_keep=(4'b1<<bytes)-1'b1;assign w_last=1;
 assign b_ready=state==3;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;bytes<=0;addr<=0;data<=0;out_error<=0;end
  else case(state)
   0:if(in_valid)begin
    addr<=in_addr;data<=in_data;bytes<=in_bytes;out_error<=in_bytes==0||in_bytes>4;
    state<=(in_bytes==0||in_bytes>4)?4:1;
   end
   1:if(wr_ready)state<=2;
   2:if(w_ready)state<=3;
   3:if(b_valid)begin out_error<=b_error||b_tag!=0;state<=4;end
   4:if(out_ready)state<=0;
   default:state<=0;
  endcase
 end
endmodule
