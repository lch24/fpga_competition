`timescale 1ns/1ps
// Photos are still immutable calibration leases. Live input uses THREE slots,
// corrected output TWO slots. No slot is reused while capture/remap/display
// owns it. Stop drains accepted descriptors and native transactions, no reset
// is used to abort DDR. KEY4 builds map once, then toggles live video.
// The same capture/remap/output leases implement pre-capture preview. Preview
// uses bypass (the demo wrapper substitutes its artificial map). Photo/review
// transitions drain all leases first; photos remain immutable. Review time starts
// at the first display frame, then returns to preview. No extra image buffers.
// During corrected VIDEO, KEY0 toggles before/after for new jobs only.
module video_board_flow #(
 parameter RAW_BASE=32'h0,SLOT_BYTES=32'h00200000,DST_BASE=32'h01000000,
 parameter LIVE_BASE=32'h00800000,
 parameter REVIEW_CYCLES=250000000 // 2 seconds at the board's 125 MHz clock
)(
 input wire preview_enable,output wire preview_active,
 input wire clk,rst_n,camera_ready,input wire [4:0] pressed,input wire display_idle,display_frame_start,
 output wire start_valid,input wire start_ready,output reg start_remap,output wire [31:0] start_base,
 input wire frame_ready,output wire frame_valid,output wire [7:0] frame_status,output wire [31:0] frame_base,
 input wire frame_release_valid,output wire frame_release_ready,
 output wire cap_cmd_valid,input wire cap_cmd_ready,output wire [31:0] cap_base,
 input wire cap_rsp_valid,output wire cap_rsp_ready,input wire [7:0] cap_status,input wire [31:0] cap_rsp_base,
 input wire rsp_valid,input wire [7:0] rsp_status,output wire rsp_ready,
 output wire capture_owner,display_enable,display_owner,output wire [31:0] display_base,
 output reg [7:0] status,output wire waiting,output wire [3:0] phase,
 output reg [2:0] captured,output reg calibrated,corrected,output reg [1:0] selected,output wire busy,
 output wire video_active,output reg map_ready,output wire display_corrected,
 output reg remap_bypass, // Latched with each descriptor: show calibration input unchanged.
 output wire remap_valid,input wire remap_ready,output wire [31:0] remap_src,remap_dst,
 input wire remap_rsp_valid,output wire remap_rsp_ready,input wire [7:0] remap_status,
 output reg [31:0] captured_frames,corrected_frames,displayed_frames,failed_frames
);
 localparam IDLE=0,DRAIN=1,COMMAND=2,CAPTURE=3,START=4,OFFER=5,LEASE=6,RESULT=7,VIDEO=8,STOP=9;
 localparam SELECT=0,PHOTO=1,CALIBRATE=2,MAP=3,PREVIEW=4;
 reg [3:0] state;reg [2:0] action;reg [1:0] target,view_index;
 reg preview;
 localparam REVIEW_BITS=REVIEW_CYCLES<2?1:$clog2(REVIEW_CYCLES+1);
 reg [REVIEW_BITS-1:0] review_left;
 reg review_started;
 reg showing;reg show_before; // KEY0 toggles only in VIDEO; new sessions start corrected.
 reg [2:0] raw_used,raw_ready;
 reg [1:0] ready_queue[0:2];reg [1:0] queue_read,queue_write,queue_count;
 reg capture_pending,remap_pending,remap_running;
 reg [1:0] capture_slot,source_slot;
 reg destination_slot,display_slot,has_display;
 reg [1:0] dst_ready,dst_used,dst_corrected;reg displayed_corrected;
 wire photo_press=|pressed[2:0];wire [1:0] requested=pressed[0]?0:pressed[1]?1:2;
 wire [1:0] free_raw=!raw_used[0]?0:!raw_used[1]?1:2;
 wire any_free_raw=!(&raw_used);
 wire [1:0] next_raw=ready_queue[queue_read];
 wire next_dst=dst_used[0]; // choose 0 unless owned
 wire next_display=dst_ready[0]?0:1;
 wire live=state==VIDEO||state==STOP;
 // A preview exit stops allocation, then drains every accepted DDR job.
 wire preview_exit=preview&&(!preview_enable||photo_press||pressed[3]||pressed[4]);
 wire stop_live=preview?preview_exit:pressed[4];
 wire push_ready=state==VIDEO&&!stop_live&&cap_rsp_valid&&cap_status==0;
 wire pop_ready=state==VIDEO&&!stop_live&&!remap_pending&&!remap_running&&queue_count!=0&&!(&dst_used);
 wire [1:0] failures={1'b0,cap_rsp_valid&&cap_status!=0}+{1'b0,remap_rsp_valid&&remap_rsp_ready&&remap_status!=0};
 wire [1:0] completed_slot=cap_rsp_base==LIVE_BASE?0:cap_rsp_base==LIVE_BASE+SLOT_BYTES?1:2;
 assign preview_active=state==VIDEO&&preview;
 assign phase=preview_active?4'd12:state;
 assign waiting=state==IDLE||preview_active;assign busy=state!=IDLE&&!preview_active;
 assign video_active=state==VIDEO&&!preview;
 assign display_corrected=state==VIDEO&&has_display&&displayed_corrected;
 assign display_enable=state==IDLE?showing:state==VIDEO&&(has_display||(|dst_ready));
 assign display_owner=display_enable||state==DRAIN||state==STOP;
 // Candidate base is presented BEFORE the display's frame-start/latch edge.
 assign display_base=live?DST_BASE+(|dst_ready?next_display:display_slot)*SLOT_BYTES:RAW_BASE+selected*SLOT_BYTES;
 assign start_valid=state==START;assign start_base=RAW_BASE+selected*SLOT_BYTES;
 assign frame_valid=state==OFFER;assign frame_status=0;assign frame_base=RAW_BASE+view_index*SLOT_BYTES;
 assign frame_release_ready=state==LEASE||state==RESULT;
 assign cap_cmd_valid=live?capture_pending:state==COMMAND;
 assign cap_base=live?LIVE_BASE+capture_slot*SLOT_BYTES:RAW_BASE+target*SLOT_BYTES;
 assign cap_rsp_ready=live||state==CAPTURE;
 assign capture_owner=live||state==COMMAND||state==CAPTURE;
 assign rsp_ready=state==OFFER||state==RESULT;
 assign remap_valid=remap_pending;assign remap_src=LIVE_BASE+source_slot*SLOT_BYTES;
 assign remap_dst=DST_BASE+destination_slot*SLOT_BYTES;assign remap_rsp_ready=remap_running;
 // Counters are telemetry, one cycle behind the ownership events. Register
 // event predicates before the 32-bit counter enables; key/mode decode no
 // longer fans out through every bit of a counter on the same cycle.
 reg captured_event,corrected_event,displayed_event;
 reg [1:0] failed_events;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   captured_event<=0;corrected_event<=0;displayed_event<=0;failed_events<=0;
   captured_frames<=0;corrected_frames<=0;displayed_frames<=0;failed_frames<=0;
  end else begin
   captured_event<=push_ready;
   corrected_event<=state==VIDEO&&!stop_live&&remap_rsp_valid&&remap_rsp_ready&&remap_status==0&&!remap_bypass;
   displayed_event<=state==VIDEO&&display_frame_start&&(|dst_ready);
   failed_events<=live?failures:2'b0;
   if(captured_event)captured_frames<=captured_frames+1'b1;
   if(corrected_event)corrected_frames<=corrected_frames+1'b1;
   if(displayed_event)displayed_frames<=displayed_frames+1'b1;
   if(failed_events!=0)failed_frames<=failed_frames+failed_events;
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   preview<=0;review_left<=0;review_started<=0;show_before<=0;remap_bypass<=0;state<=IDLE;action<=SELECT;target<=0;view_index<=0;selected<=0;showing<=0;
   captured<=0;calibrated<=0;corrected<=0;map_ready<=0;start_remap<=0;status<=0;
   raw_used<=0;raw_ready<=0;capture_pending<=0;remap_pending<=0;remap_running<=0;
   queue_read<=0;queue_write<=0;queue_count<=0;
   capture_slot<=0;source_slot<=0;destination_slot<=0;display_slot<=0;has_display<=0;dst_ready<=0;dst_used<=0;dst_corrected<=0;displayed_corrected<=0;

  end else begin
   if(push_ready)begin ready_queue[queue_write]<=completed_slot;queue_write<=queue_write==2?0:queue_write+1'b1;end
   if(pop_ready)queue_read<=queue_read==2?0:queue_read+1'b1;
   case({push_ready,pop_ready})2'b10:queue_count<=queue_count+1'b1;2'b01:queue_count<=queue_count-1'b1;default:queue_count<=queue_count;endcase
   case(state)
    IDLE:begin
     if(showing&&display_frame_start)review_started<=1;
     if(review_started&&review_left!=0)review_left<=review_left-1'b1;
     if(photo_press)begin
      if(captured[requested]||camera_ready)begin target<=requested;action<=captured[requested]?SELECT:PHOTO;state<=DRAIN;status<=0;end
      else status<=8'h80;
     end else if(pressed[3])begin
      if(&captured)begin action<=CALIBRATE;state<=DRAIN;status<=0;end else status<=8'h81;
     end else if(pressed[4])begin
      if(calibrated&&camera_ready)begin action<=MAP;state<=DRAIN;status<=0;end else status<=8'h82;
     end else if(preview_enable&&camera_ready&&(!showing||(review_started&&review_left==0)))begin
      action<=PREVIEW;state<=DRAIN;
     end
    end
    DRAIN:if(display_idle)begin
     if(action==PREVIEW)begin
      showing<=0;review_started<=0;
      if(preview_enable&&camera_ready)begin
       state<=VIDEO;preview<=1;show_before<=1;has_display<=0;dst_used<=0;dst_ready<=0;
      end else state<=IDLE;
     end
     else if(action==SELECT)begin selected<=target;showing<=1;review_left<=REVIEW_CYCLES;review_started<=0;state<=IDLE;end
     else if(action==PHOTO)state<=COMMAND;
     else if(action==MAP&&map_ready)begin state<=VIDEO;preview<=0;show_before<=0;has_display<=0;dst_used<=0;dst_ready<=0;end
     else begin
      start_remap<=action==MAP;view_index<=0;
      if(action==CALIBRATE)begin calibrated<=0;corrected<=0;map_ready<=0;end
      state<=START;
     end
    end
    COMMAND:if(cap_cmd_ready)state<=CAPTURE;
    CAPTURE:if(cap_rsp_valid)begin
     status<=cap_status;state<=IDLE;
     if(cap_status==0)begin captured[target]<=1;selected<=target;showing<=1;review_left<=REVIEW_CYCLES;review_started<=0;end
    end
    START:if(start_ready)state<=start_remap?RESULT:OFFER;
    OFFER:if(frame_ready)state<=LEASE;
    LEASE:if(frame_release_valid)begin if(view_index==2)state<=RESULT;else begin view_index<=view_index+1'b1;state<=OFFER;end end
    RESULT:state<=RESULT;
    VIDEO:begin
     if(!preview&&pressed[0]&&!pressed[4])show_before<=!show_before;
     if(stop_live)begin
      if(preview)begin
       action<=PREVIEW;
       if(preview_enable)begin
        if(photo_press)begin
         if(captured[requested]||camera_ready)begin target<=requested;action<=captured[requested]?SELECT:PHOTO;status<=0;end else status<=8'h80;
        end
        else if(pressed[3])begin if(&captured)begin action<=CALIBRATE;status<=0;end else status<=8'h81;end
        else if(pressed[4])begin if(calibrated&&camera_ready)begin action<=MAP;status<=0;end else status<=8'h82;end
       end
      end
      state<=STOP;raw_used<=raw_used&~raw_ready;raw_ready<=0;queue_read<=0;queue_write<=0;queue_count<=0;end
     // Reserve before presenting VALID: payload cannot change during stalls.
     if(!stop_live&&!capture_pending&&any_free_raw&&camera_ready)begin capture_slot<=free_raw;capture_pending<=1;raw_used[free_raw]<=1;end
     if(pop_ready)begin
      remap_bypass<=preview?1'b1:(show_before^pressed[0]);
      source_slot<=next_raw;destination_slot<=next_dst;remap_pending<=1;raw_ready[next_raw]<=0;dst_used[next_dst]<=1;
     end
     if(display_frame_start&&(|dst_ready))begin
      if(has_display)dst_used[display_slot]<=0;
      display_slot<=next_display;displayed_corrected<=dst_corrected[next_display];has_display<=1;dst_ready[next_display]<=0;
     end
    end
    STOP:begin
     // READY inputs are unleased and can be discarded; queued/capturing inputs
     // remain owned until their END response. Finish any pending remap too.
     raw_ready<=0;
     if(raw_used==0&&!capture_pending&&!remap_pending&&!remap_running&&display_idle)begin
      if(!preview)showing<=0;
      state<=preview?DRAIN:IDLE;dst_ready<=0;dst_used<=0;has_display<=0;
     end
    end
    default:state<=IDLE;
   endcase
   if(live)begin
    if(capture_pending&&cap_cmd_ready)capture_pending<=0;
    if(cap_rsp_valid)begin
     if(cap_status==0&&state==VIDEO&&!stop_live)begin raw_ready[completed_slot]<=1;end
     else begin raw_used[completed_slot]<=0;if(cap_status!=0)status<=cap_status;end
    end
    if(remap_pending&&remap_ready)begin remap_pending<=0;remap_running<=1;end
    if(remap_rsp_valid&&remap_rsp_ready)begin
     remap_running<=0;raw_used[source_slot]<=0;
     if(remap_status==0&&state==VIDEO&&!stop_live)begin
      dst_ready[destination_slot]<=1;dst_corrected[destination_slot]<=!remap_bypass;
      if(!remap_bypass)begin corrected<=1;end
     end
     else begin dst_used[destination_slot]<=0;if(remap_status!=0)status<=remap_status;end
    end
   end
   if(rsp_valid&&rsp_ready)begin
    status<=rsp_status;
    if(start_remap&&rsp_status==0)begin
     map_ready<=1;state<=VIDEO;preview<=0;show_before<=0;has_display<=0;dst_used<=0;dst_ready<=0;
    end else begin state<=IDLE;showing<=0;if(!start_remap)calibrated<=rsp_status==0;end
   end
  end
 end
endmodule
