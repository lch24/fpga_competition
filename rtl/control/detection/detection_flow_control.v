// Instruction core plus thin adapters for layer/corner-stream transactions.
// Algorithm stage order and loops live in build_detection_program.py.
// This module never computes pixels, projections, or floating-point values.
module detection_flow_control #(parameter USE_CE=0,DEPTH=3,CORNER_N=40,REPLAY_RESP=0)(
 input wire clk,rst_n,ce,start,cfg_resp_dump_en,
 output wire busy,done,output wire [1:0] status,
 output reg [3:0] stage,output reg [7:0] dl,
 output reg nat_entry,ref_entry,output reg [DEPTH-1:0] child_valid,
 output reg [15:0] oc,rc,out_oc,output reg [1:0] o_st,
 input wire order_valid,refine_valid,refine_done,refine_ok,map_done,resp_dump_done,
 input wire [31:0] corner_x,corner_y,
 output reg out_valid,input wire out_ready,output reg [31:0] out_x,out_y,
 output reg [15:0] out_total,output reg out_grid_ok,
 output wire seq_valid,seq_enter,output wire [7:0] seq_id,output wire [15:0] seq_arg,
 input wire filter_done,order_done,grid_done,
 input wire [31:0] filter_result,order_result,grid_result
);
 `include "detection_services.vh"
 localparam ST_IDLE=0,ST_NATIVE=1,ST_ORDER=2,ST_REFINE=3,ST_MAP=4,
   ST_NEXT=5,ST_FIN=6,ST_OUT=7,ST_DUMP=8;
 localparam O_IDLE=0,O_RD=1,O_EMIT=2;
 reg local_done;reg [31:0] local_result;
 wire service_done=seq_id[7:4]==1?filter_done:seq_id[7:4]==2?order_done:seq_id[7:4]==3?grid_done:local_done;
 wire [31:0] service_result=seq_id[7:4]==1?filter_result:seq_id[7:4]==2?order_result:seq_id[7:4]==3?grid_result:local_result;
 detection_program #(.USE_CE(USE_CE)) program_control(.clk(clk),.rst_n(rst_n),.ce(ce),.start(start),
  .busy(busy),.done(done),.status(status),.service_valid(seq_valid),.service_enter(seq_enter),
  .service_id(seq_id),.service_arg(seq_arg),.service_done(service_done),.service_result(service_result),.debug_pc());
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin stage<=ST_IDLE;dl<=0;nat_entry<=0;ref_entry<=0;child_valid<=0;
   oc<=0;rc<=0;out_oc<=0;o_st<=O_IDLE;out_valid<=0;out_x<=0;out_y<=0;
   out_total<=0;out_grid_ok<=0;local_done<=0;local_result<=0;end
  else if(!USE_CE || ce)begin
   nat_entry<=0;ref_entry<=0;
   if(!seq_valid)local_done<=0;
   if(seq_enter && seq_id[7:4]==0)begin
    local_done<=0;local_result<=0;
    case(seq_id)
     DET_BOOT:begin stage<=ST_IDLE;dl<=DEPTH-1;child_valid<=0;oc<=0;rc<=0;
       out_valid<=0;out_grid_ok<=0;out_total<=0;o_st<=O_IDLE;out_oc<=0;
       local_result<=REPLAY_RESP && cfg_resp_dump_en;local_done<=1;end
     DET_NATIVE:begin stage<=ST_NATIVE;nat_entry<=1;local_done<=1;end
     DET_ORDER_OUTPUT:begin stage<=ST_ORDER;oc<=0;end
     DET_REFINE:begin stage<=ST_REFINE;rc<=0;ref_entry<=1;local_done<=1;end
     DET_MAP:stage<=ST_MAP;
     DET_NEXT_LAYER:begin
       local_result<={30'd0,child_valid[dl],dl==0};local_done<=1;stage<=ST_NEXT;
       if(dl!=0)dl<=dl-1'b1;
     end
     DET_FINISH:begin local_result<=!child_valid[0];local_done<=1;stage<=ST_FIN;
       if(child_valid[0])begin out_total<=CORNER_N;out_grid_ok<=1;end end
     DET_OUTPUT:begin stage<=ST_OUT;out_oc<=0;o_st<=O_RD;local_result<=cfg_resp_dump_en;end
     DET_DUMP:stage<=ST_DUMP;
     default:begin local_done<=1;local_result<=1;end
    endcase
   end else begin
    if(stage==ST_ORDER && order_valid && oc<CORNER_N)begin
      oc<=oc+1'b1;if(oc+1'b1>=CORNER_N)local_done<=1;
    end
    if(stage==ST_REFINE && !ref_entry)begin
      if(refine_done)child_valid[dl]<=refine_ok;
      else if(refine_valid && rc+1'b1<CORNER_N)rc<=rc+1'b1;
    end
    if(stage==ST_MAP && map_done)local_done<=1;
    if(stage==ST_DUMP && resp_dump_done)local_done<=1;
    if(stage==ST_OUT && seq_valid && seq_id==DET_OUTPUT && !local_done)case(o_st)
     O_IDLE:o_st<=O_RD;
     O_RD:o_st<=O_EMIT;
     O_EMIT:if(!out_valid)begin out_valid<=1;out_x<=corner_x;out_y<=corner_y;end
       else if(out_ready)begin out_valid<=0;
         if(out_oc+1'b1>=CORNER_N)begin local_done<=1;o_st<=O_IDLE;end
         else begin out_oc<=out_oc+1'b1;o_st<=O_RD;end
       end
    endcase
   end
   if(done && !busy)stage<=ST_IDLE;
  end
 end
endmodule
