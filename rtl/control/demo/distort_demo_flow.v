`timescale 1ns/1ps
`include "distort_demo.vh"
// Optional wrapper: never instantiated by the normal board configuration.
module distort_demo_flow #(
 parameter RAW_BASE=32'h0,SLOT_BYTES=32'h00200000,DST_BASE=32'h01000000,
 parameter LIVE_BASE=32'h00800000,parameter REVIEW_CYCLES=250000000
)(
 input wire preview_enable,
 input wire clk,rst_n,camera_ready,input wire [4:0] pressed,input wire display_idle,display_frame_start,
 output wire start_valid,input wire start_ready,output wire start_remap,output wire [31:0] start_base,
 input wire frame_ready,output wire frame_valid,output wire [7:0] frame_status,output wire [31:0] frame_base,
 input wire frame_release_valid,output wire frame_release_ready,
 output wire cap_cmd_valid,input wire cap_cmd_ready,output wire [31:0] cap_base,
 input wire cap_rsp_valid,output wire cap_rsp_ready,input wire [7:0] cap_status,input wire [31:0] cap_rsp_base,
 input wire rsp_valid,input wire [7:0] rsp_status,output wire rsp_ready,
 output wire capture_owner,display_enable,display_owner,output wire [31:0] display_base,
 output wire [7:0] status,output wire waiting,output wire [3:0] phase,
 output wire [2:0] captured,output wire calibrated,corrected,output wire [1:0] selected,output wire busy,
 output wire video_active,output wire map_ready,output wire display_corrected,
 output reg remap_bypass,
 output wire remap_valid,input wire remap_ready,output wire [31:0] remap_src,remap_dst,
 input wire remap_rsp_valid,output wire remap_rsp_ready,input wire [7:0] remap_status,
 output wire [31:0] captured_frames,corrected_frames,displayed_frames,failed_frames
,
 input wire toggle_mode,input wire [31:0] remap_cycles,
 output reg distort_mode,output wire mode_pending,output reg demo_map_ready,
 output wire [31:0] remap_map,output wire [255:0] demo_snapshot
);
 wire f_start_valid;
 wire f_start_remap;
 wire [31:0] f_start_base;
 wire f_frame_valid;
 wire [7:0] f_frame_status;
 wire [31:0] f_frame_base;
 wire f_frame_release_ready;
 wire f_cap_cmd_valid;
 wire [31:0] f_cap_base;
 wire f_cap_rsp_ready;
 wire f_rsp_ready;
 wire f_capture_owner;
 wire f_display_enable;
 wire f_display_owner;
 wire [31:0] f_display_base;
 wire [7:0] f_status;
 wire f_waiting;
 wire [3:0] f_phase;
 wire [2:0] f_captured;
 wire f_calibrated;
 wire f_corrected;
 wire [1:0] f_selected;
 wire f_busy;
 wire f_video_active,f_preview_active;
 wire f_map_ready;
 wire f_remap_valid,f_remap_bypass;reg skip_correction;
 wire [31:0] f_remap_src;
 wire [31:0] f_remap_dst;
 wire f_remap_rsp_ready;
 wire [31:0] f_captured_frames;
 wire [31:0] f_corrected_frames;
 wire [31:0] f_displayed_frames;
 wire [31:0] f_failed_frames;

 assign frame_valid=f_frame_valid;
 assign frame_status=f_frame_status;
 assign frame_base=f_frame_base;
 assign frame_release_ready=f_frame_release_ready;
 assign capture_owner=f_capture_owner;
 assign display_owner=f_display_owner;
 assign display_base=f_display_base;
 assign waiting=f_waiting&&!mode_pending;
 assign captured=f_captured;
 assign calibrated=f_calibrated;
 assign corrected=f_corrected;
 assign selected=f_selected;
 assign video_active=f_video_active;
 assign map_ready=f_map_ready;
 assign captured_frames=f_captured_frames;
 assign corrected_frames=f_corrected_frames;
 assign displayed_frames=f_displayed_frames;
 assign failed_frames=f_failed_frames;

 localparam M_RUN=0,M_DRAIN=1,M_CLEAR_SEND=2,M_CLEAR_WAIT=3,M_RESET=4,M_BUILD_SEND=5,M_BUILD_WAIT=6;
 reg flow_clear;reg [2:0] mode_state; reg [7:0] demo_error;
 localparam C_IDLE=0,C_WAIT=1,C_REMAP=2,C_PROCESS=3,C_DONE=4;
 reg [2:0] capture_state;reg [31:0] photo_target;reg [7:0] photo_status;
 localparam R_IDLE=0,R_SEND=1,R_WAIT=2,R_RESPONSE=3;
 reg [1:0] remap_state;reg first_pass,photo_job;
 reg [31:0] source,destination,mapping,final_destination;
 reg [7:0] remap_result;
 reg [16:0] build_subcycles;
 reg [31:0] first_cycles,last_first_cycles,last_second_cycles,last_total_cycles,demo_failures,build_ms;
 wire [4:0] flow_pressed;
 wire flow_live=f_phase==8||f_phase==9||f_preview_active;
 wire internal_command=mode_state>=M_CLEAR_SEND;
 assign mode_pending=mode_state!=M_RUN;
 assign flow_pressed=mode_state==M_RUN&&!toggle_mode ? pressed :
                     mode_state==M_DRAIN&&f_video_active ? 5'b10000 : 5'b0;
 assign display_enable=f_display_enable&&!mode_pending&&!toggle_mode;
 assign busy=f_busy||mode_pending||capture_state!=C_IDLE||(!f_preview_active&&remap_state!=R_IDLE);
 assign phase=mode_pending?4'd10:capture_state>=C_REMAP?4'd11:f_phase;
 assign status=demo_error!=0?demo_error:f_status;
 assign start_valid=internal_command?(mode_state==M_CLEAR_SEND||mode_state==M_BUILD_SEND):f_start_valid;
 assign start_remap=internal_command?1'b1:f_start_remap;
 assign start_base=internal_command?(mode_state==M_CLEAR_SEND?`DEMO_CLEAR_COMMAND:`DEMO_BUILD_COMMAND):f_start_base;
 wire f_start_ready=!internal_command&&start_ready;
 wire f_rsp_valid=!internal_command&&rsp_valid;
 wire [7:0] f_rsp_status=rsp_status;
 assign rsp_ready=internal_command?(mode_state==M_CLEAR_WAIT||mode_state==M_BUILD_WAIT):f_rsp_ready;
 // Photo command goes to scratch. Capture completion is held until remapping ends.
 assign cap_cmd_valid=f_cap_cmd_valid&&capture_state==C_IDLE;
 wire f_cap_cmd_ready=cap_cmd_ready&&capture_state==C_IDLE;
 assign cap_base=distort_mode&&!flow_live?`DEMO_PHOTO_BASE:f_cap_base;
 assign cap_rsp_ready=capture_state==C_WAIT?1'b1:capture_state==C_IDLE?f_cap_rsp_ready:1'b0;
 wire f_cap_rsp_valid=capture_state==C_DONE?1'b1:capture_state==C_IDLE?cap_rsp_valid:1'b0;
 wire [7:0] f_cap_status=capture_state==C_DONE?photo_status:cap_status;
 wire [31:0] f_cap_rsp_base=capture_state==C_DONE?photo_target:cap_rsp_base;
 // Only one physical remapper, with a stable command throughout backpressure.
 assign remap_valid=remap_state==R_SEND;
 assign remap_src=source;assign remap_dst=destination;assign remap_map=mapping;
 assign remap_rsp_ready=remap_state==R_WAIT;
 wire f_remap_ready=remap_state==R_IDLE&&capture_state!=C_REMAP;
 wire f_remap_rsp_valid=remap_state==R_RESPONSE&&!photo_job;
 wire [7:0] f_remap_status=remap_result;
 assign demo_snapshot={build_ms,demo_failures,last_total_cycles,last_second_cycles,last_first_cycles,
                       {12'b0,demo_error,remap_state,capture_state,mode_state,photo_job,first_pass,mode_pending,distort_mode},
                       32'b0,32'b0};
 wire flow_reset_n=rst_n && !flow_clear;

 video_board_flow #(.RAW_BASE(RAW_BASE),.SLOT_BYTES(SLOT_BYTES),.DST_BASE(DST_BASE),.LIVE_BASE(LIVE_BASE),.REVIEW_CYCLES(REVIEW_CYCLES)) normal_flow(
.preview_enable(preview_enable&&mode_state==M_RUN&&!toggle_mode),.preview_active(f_preview_active),
.clk(clk),
.rst_n(flow_reset_n),
.camera_ready(camera_ready),
.pressed(flow_pressed),
.display_idle(display_idle),
.display_frame_start(display_frame_start),
.start_valid(f_start_valid),
.start_ready(f_start_ready),
.start_remap(f_start_remap),
.start_base(f_start_base),
.frame_ready(frame_ready),
.frame_valid(f_frame_valid),
.frame_status(f_frame_status),
.frame_base(f_frame_base),
.frame_release_valid(frame_release_valid),
.frame_release_ready(f_frame_release_ready),
.cap_cmd_valid(f_cap_cmd_valid),
.cap_cmd_ready(f_cap_cmd_ready),
.cap_base(f_cap_base),
.cap_rsp_valid(f_cap_rsp_valid),
.cap_rsp_ready(f_cap_rsp_ready),
.cap_status(f_cap_status),
.cap_rsp_base(f_cap_rsp_base),
.rsp_valid(f_rsp_valid),
.rsp_status(f_rsp_status),
.rsp_ready(f_rsp_ready),
.capture_owner(f_capture_owner),
.display_enable(f_display_enable),
.display_owner(f_display_owner),
.display_base(f_display_base),
.status(f_status),
.waiting(f_waiting),
.phase(f_phase),
.captured(f_captured),
.calibrated(f_calibrated),
.corrected(f_corrected),
.selected(f_selected),
.busy(f_busy),
.video_active(f_video_active),
.map_ready(f_map_ready),.display_corrected(display_corrected),
.remap_bypass(f_remap_bypass),
.remap_valid(f_remap_valid),
.remap_ready(f_remap_ready),
.remap_src(f_remap_src),
.remap_dst(f_remap_dst),
.remap_rsp_valid(f_remap_rsp_valid),
.remap_rsp_ready(f_remap_rsp_ready),
.remap_status(f_remap_status),
.captured_frames(f_captured_frames),
.corrected_frames(f_corrected_frames),
.displayed_frames(f_displayed_frames),
.failed_frames(f_failed_frames));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   flow_clear<=0;mode_state<=M_RUN;distort_mode<=0;demo_map_ready<=0;demo_error<=0;
   capture_state<=C_IDLE;photo_target<=0;photo_status<=0;
   remap_bypass<=0;skip_correction<=0;remap_state<=R_IDLE;first_pass<=0;photo_job<=0;source<=0;destination<=0;mapping<=0;final_destination<=0;remap_result<=0;
   first_cycles<=0;last_first_cycles<=0;last_second_cycles<=0;last_total_cycles<=0;demo_failures<=0;build_ms<=0;build_subcycles<=0;
  end else begin
   case(mode_state)
    M_RUN:if(toggle_mode)begin mode_state<=M_DRAIN;demo_error<=0;end
    M_DRAIN:if(!f_busy&&!f_preview_active&&display_idle&&capture_state==C_IDLE&&remap_state==R_IDLE)mode_state<=M_CLEAR_SEND;
    M_CLEAR_SEND:if(start_ready)mode_state<=M_CLEAR_WAIT;
    M_CLEAR_WAIT:if(rsp_valid)begin
     if(rsp_status!=0)begin demo_error<=rsp_status;mode_state<=M_RUN;end
     else begin flow_clear<=1;mode_state<=M_RESET;end
    end
    M_RESET:begin
     flow_clear<=0;     distort_mode<=!distort_mode;
     if(!distort_mode&&!demo_map_ready)begin mode_state<=M_BUILD_SEND;build_ms<=0;build_subcycles<=0;end
     else mode_state<=M_RUN;
    end
    M_BUILD_SEND:if(start_ready)mode_state<=M_BUILD_WAIT;
    M_BUILD_WAIT:begin
     if(build_subcycles==124999)begin
      build_subcycles<=0;if(build_ms!=32'hffffffff)build_ms<=build_ms+1'b1;
     end else build_subcycles<=build_subcycles+1'b1;
     if(rsp_valid)begin
      demo_map_ready<=rsp_status==0;demo_error<=rsp_status;
      // Never allow photos through with an incomplete artificial map.
      if(rsp_status!=0)distort_mode<=0;
      mode_state<=M_RUN;
     end
    end
   endcase
   case(capture_state)
    C_IDLE:if(cap_cmd_valid&&cap_cmd_ready&&distort_mode&&!flow_live)begin
     photo_target<=f_cap_base;capture_state<=C_WAIT;
    end
    C_WAIT:if(cap_rsp_valid)begin photo_status<=cap_status;capture_state<=cap_status==0?C_REMAP:C_DONE;end
    C_REMAP:if(remap_state==R_IDLE)capture_state<=C_PROCESS;
    C_PROCESS:if(remap_state==R_RESPONSE&&photo_job)begin photo_status<=remap_result;capture_state<=C_DONE;end
    C_DONE:if(f_cap_rsp_ready)capture_state<=C_IDLE;
   endcase
   case(remap_state)
    R_IDLE:begin
     if(capture_state==C_REMAP)begin
      remap_bypass<=0;skip_correction<=0;photo_job<=1;first_pass<=1;source<=`DEMO_PHOTO_BASE;destination<=photo_target;
      mapping<=`DEMO_MAP_BASE;first_cycles<=0;remap_state<=R_SEND;
     end else if(f_remap_valid)begin
      remap_bypass<=!distort_mode&&f_remap_bypass;skip_correction<=f_remap_bypass;
      photo_job<=0;first_pass<=distort_mode;source<=f_remap_src;final_destination<=f_remap_dst;
      destination<=distort_mode&&!f_remap_bypass?`DEMO_MIDDLE_BASE:f_remap_dst;
      mapping<=distort_mode?`DEMO_MAP_BASE:32'h02000000;first_cycles<=0;remap_state<=R_SEND;
     end
    end
    R_SEND:if(remap_ready)remap_state<=R_WAIT;
    R_WAIT:if(remap_rsp_valid)begin
     remap_result<=remap_status;
     if(first_pass)begin first_cycles<=remap_cycles;last_first_cycles<=remap_cycles;end
     else last_second_cycles<=remap_cycles;
     if(remap_status!=0)demo_failures<=demo_failures+1'b1;
     if(first_pass&&!photo_job&&!skip_correction&&remap_status==0)begin
      first_pass<=0;source<=`DEMO_MIDDLE_BASE;destination<=final_destination;
      mapping<=32'h02000000;remap_state<=R_SEND;
     end else begin
      last_total_cycles<=first_pass?remap_cycles:first_cycles+remap_cycles;
      remap_state<=R_RESPONSE;
     end
    end
    R_RESPONSE:if(photo_job||f_remap_rsp_ready)remap_state<=R_IDLE;
   endcase
  end
 end
endmodule
