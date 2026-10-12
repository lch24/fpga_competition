`timescale 1ns/1ps
// DDR-domain UI. Drain display transactions before changing DDR ownership.
// Photos are immutable until reset; calibration leases them in slot order.
module board_flow #(
 parameter RAW_BASE=32'h0,SLOT_BYTES=32'h00200000,DST_BASE=32'h01000000
)(
 input wire clk,rst_n,camera_ready,input wire [4:0] pressed,input wire display_idle,
 output wire start_valid,input wire start_ready,output reg start_remap,
 output wire [31:0] start_base,
 input wire frame_ready,output wire frame_valid,output wire [7:0] frame_status,
 output wire [31:0] frame_base,
 input wire frame_release_valid,output wire frame_release_ready,
 output wire cap_cmd_valid,input wire cap_cmd_ready,output wire [31:0] cap_base,
 input wire cap_rsp_valid,output wire cap_rsp_ready,input wire [7:0] cap_status,
 input wire rsp_valid,input wire [7:0] rsp_status,output wire rsp_ready,
 output wire capture_owner,display_enable,display_owner,output wire [31:0] display_base,
 output reg [7:0] status,output wire waiting,output wire [3:0] phase,
 output reg [2:0] captured,output reg calibrated,corrected,
 output reg [1:0] selected,output wire busy
);
 localparam IDLE=0,DRAIN=1,COMMAND=2,CAPTURE=3,START=4,OFFER=5,LEASE=6,RESULT=7;
 localparam SELECT=0,PHOTO=1,CALIBRATE=2,REMAP=3;
 reg [3:0] state;reg [1:0] action,target,view_index;
 reg showing,show_corrected;
 wire photo_press=|pressed[2:0];
 wire [1:0] requested=pressed[0]?0:pressed[1]?1:2;
 assign phase=state;assign waiting=state==IDLE;assign busy=state!=IDLE;
 assign display_enable=showing&&state==IDLE;
 assign display_owner=showing&&(state==IDLE||state==DRAIN);
 assign display_base=show_corrected?DST_BASE:RAW_BASE+selected*SLOT_BYTES;
 assign start_valid=state==START;assign start_base=RAW_BASE+selected*SLOT_BYTES;
 assign frame_valid=state==OFFER;assign frame_status=0;
 assign frame_base=RAW_BASE+view_index*SLOT_BYTES;
 assign frame_release_ready=state==LEASE||state==RESULT;
 assign cap_cmd_valid=state==COMMAND;assign cap_rsp_ready=state==CAPTURE;
 assign cap_base=RAW_BASE+target*SLOT_BYTES;
 assign capture_owner=state==COMMAND||state==CAPTURE;
 // Release tokens must be consumed before the result, even across CDC FIFOs.
 assign rsp_ready=state==OFFER||state==RESULT;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;action<=SELECT;target<=0;view_index<=0;selected<=0;
   showing<=0;show_corrected<=0;captured<=0;calibrated<=0;corrected<=0;
   start_remap<=0;status<=0;
  end else begin
   case(state)
    IDLE:begin
     if(photo_press)begin
      if(captured[requested]||camera_ready)begin
       target<=requested;action<=captured[requested]?SELECT:PHOTO;state<=DRAIN;status<=0;
      end else status<=8'h80;
     end else if(pressed[3])begin
      if(&captured)begin action<=CALIBRATE;state<=DRAIN;status<=0;end
      else status<=8'h81;
     end else if(pressed[4])begin
      if(calibrated&&captured[selected])begin action<=REMAP;state<=DRAIN;status<=0;end
      else status<=8'h82;
     end
    end
    DRAIN:if(display_idle)begin
     if(action==SELECT)begin selected<=target;show_corrected<=0;showing<=1;state<=IDLE;end
     else if(action==PHOTO)state<=COMMAND;
     else begin
      start_remap<=action==REMAP;view_index<=0;
      if(action==CALIBRATE)begin calibrated<=0;corrected<=0;end
      state<=START;
     end
    end
    COMMAND:if(cap_cmd_ready)state<=CAPTURE;
    CAPTURE:if(cap_rsp_valid)begin
     status<=cap_status;state<=IDLE;
     if(cap_status==0)begin captured[target]<=1;selected<=target;showing<=1;show_corrected<=0;end
    end
    START:if(start_ready)state<=start_remap?RESULT:OFFER;
    OFFER:if(frame_ready)state<=LEASE;
    LEASE:if(frame_release_valid)begin
     if(view_index==2)state<=RESULT;
     else begin view_index<=view_index+1'b1;state<=OFFER;end
    end
    RESULT:state<=RESULT;
    default:state<=IDLE;
   endcase
   if(rsp_valid&&rsp_ready)begin
    status<=rsp_status;state<=IDLE;
    if(start_remap)begin corrected<=rsp_status==0;show_corrected<=rsp_status==0;end
    else begin calibrated<=rsp_status==0;show_corrected<=0;end
   end
  end
 end
endmodule
