`timescale 1ns/1ps
`include "calib_defs.vh"
// Serial orchestration only; no numerical algorithm is replaced here.
// External callers supply ordered frame descriptors, never job/view/index IDs.
// The last supplied RGB565 frame is retained until remap completes.
module vision_sequence #(parameter WIDTH=1280,HEIGHT=720)(
 input wire clk,rst_n,layout_ok,
 input wire start_valid,output wire start_ready,input wire [63:0] square_size_fp64,
 input wire frame_valid,output wire frame_ready,input wire [31:0] frame_base,frame_stride,frame_capacity,
 input wire [7:0] frame_status,input wire frame_config_ok,
 output wire frame_release_valid,input wire frame_release_ready,
 output wire rsp_valid,input wire rsp_ready,output reg [7:0] rsp_status,
 output wire busy,output reg [31:0] job,output reg [7:0] view_id,
 output wire [5:0] debug_phase,
 output reg [31:0] src_base,src_stride,src_capacity,output reg [63:0] square,
 output wire algorithm_rst_n,
 output wire gray_cmd_valid,input wire gray_cmd_ready,
 input wire gray_rsp_valid,output wire gray_rsp_ready,input wire [7:0] gray_rsp_status,
 output wire det_process,input wire det_done,input wire [1:0] det_status,
 input wire det_valid,output wire det_ready,input wire [31:0] det_x,det_y,
 input wire [15:0] det_total,input wire det_grid_ok,
 output wire collect_valid,input wire collect_ready,
 output wire corner_valid,input wire corner_ready,output wire [7:0] corner_index,
 output wire corner_last,
 output wire view_rsp_valid,input wire view_rsp_ready,output reg [7:0] view_status,
 output wire cal_cmd_valid,input wire cal_cmd_ready,
 output wire mb_begin_valid,input wire mb_begin_ready,
 input wire mb_valid,output wire mb_ready,input wire [7:0] mb_status,
 input wire [31:0] mb_id,input wire [15:0] mb_width,mb_height,input wire [287:0] mb_params,
 output reg [287:0] params,
 output wire remap_cmd_valid,input wire remap_cmd_ready,output wire [1:0] remap_opcode,
 input wire remap_rsp_valid,output wire remap_rsp_ready,input wire [7:0] remap_status
);
 localparam IDLE=0,COLLECT=1,FRAME=2,GRAY_CMD=3,GRAY_WAIT=4,DET_START=5,
 DET_ARM=6,DET_WAIT=7,VIEW_RSP=8,RELEASE=9,CAL_CMD=10,CAL_WAIT=11,
 BUILD_CMD=12,BUILD_WAIT=13,REMAP_CMD=14,REMAP_WAIT=15,RESPONSE=16;
 reg [5:0] state;reg held,release_to_frame;reg [8:0] point_count;
 assign start_ready=state==IDLE;assign busy=state!=IDLE;
 assign frame_ready=state==FRAME;assign frame_release_valid=state==RELEASE;
 assign rsp_valid=state==RESPONSE;assign debug_phase=state;
 assign algorithm_rst_n=rst_n&&state!=IDLE;
 assign gray_cmd_valid=state==GRAY_CMD;assign gray_rsp_ready=state==GRAY_WAIT;
 assign det_process=state==DET_START;
 // DET_ARM deliberately ignores the previous frame's sticky done at start.
 assign det_ready=(state==DET_ARM||state==DET_WAIT)&&(point_count>=`PAR_POINTS||mb_valid||corner_ready);
 assign corner_valid=(state==DET_ARM||state==DET_WAIT)&&det_valid&&point_count<`PAR_POINTS&&!mb_valid;
 assign corner_index=point_count[7:0];assign corner_last=point_count==`PAR_POINTS-1;
 assign collect_valid=state==COLLECT&&mb_begin_ready;
 assign mb_begin_valid=state==COLLECT&&collect_ready;
 assign view_rsp_valid=state==VIEW_RSP&&!mb_valid;
 assign cal_cmd_valid=state==CAL_CMD;
 assign mb_ready=state==CAL_WAIT;
 assign remap_cmd_valid=state==BUILD_CMD||state==REMAP_CMD;
 assign remap_opcode=state==BUILD_CMD?2'd1:2'd2;
 assign remap_rsp_ready=state==BUILD_WAIT||state==REMAP_WAIT;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;job<=0;view_id<=0;src_base<=0;src_stride<=0;src_capacity<=0;
   square<=0;params<=0;rsp_status<=0;view_status<=0;point_count<=0;held<=0;release_to_frame<=0;end
  else begin
   if(det_valid&&det_ready&&point_count<511)point_count<=point_count+1'b1;
   case(state)
    IDLE:if(start_valid)begin
     job<=job+1'b1;view_id<=0;square<=square_size_fp64;held<=0;params<=0;rsp_status<=0;
     if(!layout_ok||square_size_fp64[63]||square_size_fp64[62:0]==0||square_size_fp64[62:52]==2047)
      begin rsp_status<=1;state<=RESPONSE;end else state<=COLLECT;
    end
    COLLECT:if(collect_ready&&mb_begin_ready)state<=FRAME;
    FRAME:if(frame_valid)begin
     src_base<=frame_base;src_stride<=frame_stride;src_capacity<=frame_capacity;
     held<=frame_status==0;
     if(frame_status!=0)begin rsp_status<=frame_status;state<=RESPONSE;end
     else if(!frame_config_ok)begin rsp_status<=1;release_to_frame<=0;state<=RELEASE;end
     else state<=GRAY_CMD;
    end
    GRAY_CMD:if(gray_cmd_ready)state<=GRAY_WAIT;
    GRAY_WAIT:if(gray_rsp_valid)begin
     if(gray_rsp_status!=0)begin view_status<=gray_rsp_status;state<=VIEW_RSP;end
     else begin point_count<=0;state<=DET_START;end
    end
    DET_START:state<=DET_ARM;
    DET_ARM:if(!det_done)state<=DET_WAIT;
    DET_WAIT:if(det_done)begin
     // status 01 includes "no board"; it is NOT the shared OK status (0).
     view_status<=det_status!=2'b01 ? 8'd4:
      (!det_grid_ok||det_total!=`PAR_POINTS||point_count!=`PAR_POINTS)?8'd2:8'd0;
     state<=VIEW_RSP;
    end
    VIEW_RSP:begin
     if(mb_valid)state<=CAL_WAIT;
     else if(view_rsp_ready)begin
      if(view_status!=0)state<=CAL_WAIT;
      else if(view_id==`PAR_VIEWS-1)state<=CAL_CMD;
      else begin release_to_frame<=1;state<=RELEASE;end
     end
    end
    RELEASE:if(frame_release_ready)begin
     held<=0;
     if(release_to_frame)begin view_id<=view_id+1'b1;state<=FRAME;end else state<=RESPONSE;
    end
    CAL_CMD:if(mb_valid)state<=CAL_WAIT;else if(cal_cmd_ready)state<=CAL_WAIT;
    CAL_WAIT:if(mb_valid)begin
     if(mb_status!=0||mb_id!=job||mb_width!=WIDTH||mb_height!=HEIGHT)begin
      rsp_status<=mb_status!=0?mb_status:8'd4;release_to_frame<=0;state<=held?RELEASE:RESPONSE;
     end else begin params<=mb_params;state<=BUILD_CMD;end
    end
    BUILD_CMD:if(remap_cmd_ready)state<=BUILD_WAIT;
    BUILD_WAIT:if(remap_rsp_valid)begin
     if(remap_status!=0)begin rsp_status<=remap_status;release_to_frame<=0;state<=held?RELEASE:RESPONSE;end
     else state<=REMAP_CMD;
    end
    REMAP_CMD:if(remap_cmd_ready)state<=REMAP_WAIT;
    REMAP_WAIT:if(remap_rsp_valid)begin rsp_status<=remap_status;release_to_frame<=0;state<=held?RELEASE:RESPONSE;end
    RESPONSE:if(rsp_ready)state<=IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
