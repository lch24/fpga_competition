// Blocking two-plane map writer. A result token is committed only after both
// scalar writes complete. Completion holds until acknowledged. All errors are
// sticky through the job; bad metadata invalidates the job, without deadlocking.
module map_table_writer (
 input wire clk,rst_n,input wire cfg_valid,output wire cfg_ready,
 input wire [15:0] cfg_width,cfg_height,
 input wire [31:0] cfg_map_x_base,cfg_map_y_base,cfg_stride,
 input wire map_valid,output wire map_ready,
 input wire [31:0] map_src_x,map_src_y,map_pixel_id,
 input wire [15:0] map_dst_x,map_dst_y,input wire map_last,map_error,
 output wire rsp_valid,input wire rsp_ready,output reg rsp_error,
 output wire wr_valid,input wire wr_ready,output wire [31:0] wr_addr,wr_len,
 output wire [15:0] wr_tag,
 output wire w_valid,input wire w_ready,output wire [31:0] w_data,
 output wire [3:0] w_keep,output wire w_last,
 input wire b_valid,output wire b_ready,input wire [15:0] b_tag,input wire b_error
);
 reg [3:0] state;reg [15:0] width,height,x,y;reg [31:0] id,xb,yb,stride,sx,sy;
 reg last; wire wi_ready,wo_valid,wo_error;
 wire wi_valid=state==2||state==4;
 wire [31:0] wi_addr=(state==2?xb:yb)+y*stride+({16'd0,x}<<2);
 output_writer writer(.clk(clk),.rst_n(rst_n),.in_valid(wi_valid),.in_ready(wi_ready),
  .in_addr(wi_addr),.in_data(state==2?sx:sy),.in_bytes(3'd4),
  .out_valid(wo_valid),.out_ready(state==3||state==5),.out_error(wo_error),
  .wr_valid(wr_valid),.wr_ready(wr_ready),.wr_addr(wr_addr),.wr_len(wr_len),.wr_tag(wr_tag),
  .w_valid(w_valid),.w_ready(w_ready),.w_data(w_data),.w_keep(w_keep),.w_last(w_last),
  .b_valid(b_valid),.b_ready(b_ready),.b_tag(b_tag),.b_error(b_error));
 assign cfg_ready=state==0;assign map_ready=state==1;assign rsp_valid=state==6;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;width<=0;height<=0;x<=0;y<=0;id<=0;xb<=0;yb<=0;stride<=0;sx<=0;sy<=0;last<=0;rsp_error<=0;end
  else case(state)
   0:if(cfg_valid)begin width<=cfg_width;height<=cfg_height;xb<=cfg_map_x_base;yb<=cfg_map_y_base;stride<=cfg_stride;x<=0;y<=0;id<=0;rsp_error<=0;state<=1;end
   1:if(map_valid)begin
    sx<=map_src_x;sy<=map_src_y;last<=x==width-1&&y==height-1;
    if(map_error||map_pixel_id!=id||map_dst_x!=x||map_dst_y!=y||map_last!=(x==width-1&&y==height-1))rsp_error<=1;
    state<=2;
   end
   2:if(wi_ready)state<=3;
   3:if(wo_valid)begin if(wo_error)rsp_error<=1;state<=4;end
   4:if(wi_ready)state<=5;
   5:if(wo_valid)begin
    if(wo_error)rsp_error<=1;
    if(last)state<=6;else begin id<=id+1;if(x==width-1)begin x<=0;y<=y+1;end else x<=x+1;state<=1;end
   end
   6:if(rsp_ready)state<=0;
   default:state<=0;
  endcase
 end
endmodule
