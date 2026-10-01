// One map pair per request. map_x/map_y are separate FP32 planes.
module map_reader (
 input wire clk,rst_n,input wire [31:0] map_x_base,map_y_base,map_stride,
 input wire in_valid,output wire in_ready,input wire [15:0] in_x,in_y,
 output wire out_valid,input wire out_ready,output reg [31:0] src_x,src_y,
 output reg out_error,
 output wire rd_valid,input wire rd_ready,output wire [31:0] rd_addr,rd_len,
 output wire [15:0] rd_tag,
 input wire r_valid,output wire r_ready,input wire [31:0] r_data,
 input wire [3:0] r_keep,input wire r_last,r_error,input wire [15:0] r_tag
);
 reg [2:0] state;reg [31:0] offset;
 assign in_ready=state==0;assign out_valid=state==5;
 assign rd_valid=state==1||state==3;
 assign rd_addr=(state==1?map_x_base:map_y_base)+offset;
 assign rd_len=4;assign rd_tag=state==1?16'd0:16'd1;
 assign r_ready=state==2||state==4;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;offset<=0;src_x<=0;src_y<=0;out_error<=0;end
  else case(state)
   0:if(in_valid)begin offset<=in_y*map_stride+({16'd0,in_x}<<2);out_error<=0;state<=1;end
   1:if(rd_ready)state<=2;
   2:if(r_valid)begin src_x<=r_data;if(r_error||!r_last||r_keep!=15||r_tag!=0)out_error<=1;state<=3;end
   3:if(rd_ready)state<=4;
   4:if(r_valid)begin src_y<=r_data;if(r_error||!r_last||r_keep!=15||r_tag!=1)out_error<=1;state<=5;end
   5:if(out_ready)state<=0;
   default:state<=0;
  endcase
 end
endmodule
