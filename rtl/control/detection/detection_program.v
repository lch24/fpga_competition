// One blocking instruction controller for a complete multiscale detection job.
// Services are registered datapath adapters, not private task schedulers.
// CALL keeps valid/id/arg stable; enter is exactly one enabled clock. The
// completion cannot be consumed on the entry clock (old done may be sticky).
// CE freezes PC, ROM, loop context and service handshakes together.
module detection_program #(parameter USE_CE=0)(
 input wire clk,rst_n,ce,start,
 output wire busy,output reg done,output reg [1:0] status,
 output wire service_valid,output wire service_enter,
 output reg [7:0] service_id,output reg [15:0] service_arg,
 input wire service_done,input wire [31:0] service_result,
 output wire [7:0] debug_pc
);
 localparam IDLE=0,FETCH=1,EXEC=2,WAIT_SERVICE=3;
 reg [1:0] state;
 reg [7:0] pc;
 reg [31:0] instruction,rom[0:255],result;
 reg enter;
 reg [15:0] index,limit;
 wire enabled=!USE_CE || ce;
 assign busy=state!=IDLE;
 assign service_valid=rst_n && enabled && state==WAIT_SERVICE;
 assign service_enter=service_valid && enter;
 assign debug_pc=pc;
 integer i;
 initial begin
  for(i=0;i<256;i=i+1)rom[i]=32'hf0000003;
  `include "detection_program_init.vh"
 end
 always @(posedge clk)if(rst_n && enabled && state==FETCH)instruction<=rom[pc];
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;pc<=0;result<=0;enter<=0;index<=0;limit<=0;
   done<=0;status<=0;service_id<=0;service_arg<=0;end
  else if(enabled)case(state)
   IDLE:if(start)begin done<=0;status<=0;pc<=0;index<=0;limit<=0;state<=FETCH;end
   FETCH:state<=EXEC;
   EXEC:case(instruction[31:28])
    0:begin service_id<=instruction[23:16];
      service_arg<=instruction[24]?index:instruction[15:0];enter<=1;state<=WAIT_SERVICE;end
    1:begin pc<=instruction[27]?instruction[7:0]:
       (result[instruction[20:16]]==instruction[26]?instruction[7:0]:pc+1'b1);state<=FETCH;end
    2:begin index<=0;limit<=instruction[24]?result[31:16]:instruction[15:0];pc<=pc+1'b1;state<=FETCH;end
    3:begin index<=index+1'b1;pc<=index+1'b1<limit?instruction[7:0]:pc+1'b1;state<=FETCH;end
    15:begin status<=instruction[1:0];done<=1;state<=IDLE;end
    default:begin status<=3;done<=1;state<=IDLE;end
   endcase
   WAIT_SERVICE:begin enter<=0;if(!enter && service_done)begin result<=service_result;pc<=pc+1'b1;state<=FETCH;end end
   default:begin status<=3;done<=1;state<=IDLE;end
  endcase
 end
endmodule
