`timescale 1ns/1ps
// One calibration job after power-up. Wait CAPTURE_WAIT_CYCLES before EACH
// view so the operator can reposition the chessboard. No new board pins.
// The frame remains frozen until the pipeline explicitly releases it.
// Successful completion transfers DDR permanently to display; failure stops.
module board_flow #(parameter CAPTURE_WAIT_CYCLES=625000000)(
 input wire clk,rst_n,camera_ready,
 output wire start_valid,input wire start_ready,
 input wire frame_ready,output wire frame_valid,output wire [7:0] frame_status,
 input wire frame_release_valid,output wire frame_release_ready,
 output wire cap_cmd_valid,input wire cap_cmd_ready,
 input wire cap_rsp_valid,output wire cap_rsp_ready,input wire [7:0] cap_status,
 input wire rsp_valid,input wire [7:0] rsp_status,output wire rsp_ready,
 output wire capture_owner,display_enable,output reg [7:0] status,
 output wire waiting,output wire [3:0] phase
);
 localparam BOOT=0,WAIT=1,COMMAND=2,CAPTURE=3,OFFER=4,LEASE=5,DONE=6;
 reg [3:0] state;reg [31:0] timer;reg [7:0] saved_status;
 assign phase=state;assign waiting=state==WAIT&&frame_ready;
 assign start_valid=state==BOOT&&camera_ready;
 assign cap_cmd_valid=state==COMMAND;assign cap_rsp_ready=state==CAPTURE;
 assign capture_owner=state==COMMAND||state==CAPTURE;
 assign frame_valid=state==OFFER;assign frame_status=saved_status;
 assign frame_release_ready=state==LEASE;
 assign rsp_ready=state!=BOOT&&state!=COMMAND&&state!=CAPTURE&&state!=DONE;
 assign display_enable=state==DONE&&status==0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=BOOT;timer<=0;saved_status<=0;status<=0;end
  else begin
   case(state)
    BOOT:if(start_valid&&start_ready)begin state<=WAIT;timer<=0;end
    WAIT:if(frame_ready)begin
     if(CAPTURE_WAIT_CYCLES==0||timer>=CAPTURE_WAIT_CYCLES-1)begin timer<=0;state<=COMMAND;end
     else timer<=timer+1;
    end else timer<=0;
    COMMAND:if(cap_cmd_ready)state<=CAPTURE;
    CAPTURE:if(cap_rsp_valid)begin saved_status<=cap_status;state<=OFFER;end
    OFFER:if(frame_ready)state<=saved_status==0?LEASE:WAIT;
    LEASE:if(frame_release_valid)begin state<=WAIT;timer<=0;end
    DONE:state<=DONE;
    default:state<=BOOT;
   endcase
   if(rsp_valid&&rsp_ready)begin status<=rsp_status;state<=DONE;end
  end
 end
endmodule
