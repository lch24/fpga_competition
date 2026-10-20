`timescale 1ns/1ps
`include "calib_defs.vh"
// Passive recorder. Debug reset is independent of per-job worker reset.
// Corner RAM has synchronous read, no bulk reset; count validates its contents.
module algorithm_debug(
 input wire clk,rst_n,functional_reset_n,start_cal,
 input wire [31:0] job,input wire [5:0] phase,input wire [7:0] view,
 input wire detect_done,input wire [15:0] detect_count,input wire [7:0] detect_flags,
 input wire corner_valid,input wire [31:0] corner_x,corner_y,
 input wire candidate_valid,reduced_model,low_coverage,input wire [3:0] check_reason,check_warnings,
 input wire result_valid,input wire [287:0] params,
 input wire [15:0] accepted_steps,
 input wire diag_valid,metrics_valid,converged,input wire [3:0] cal_phase,
 input wire [7:0] diag_status,input wire [63:0] rms,
 input wire rsp_valid,input wire [7:0] rsp_status,
 input wire [71:0] live,input wire rd_accept,rd_return,wr_accept,
 input wire [11:0] bus_state,input wire [31:0] address,
 input wire [15:0] argument,output wire [511:0] snapshot
);
 reg [31:0] reads,returns,writes,last_address;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin reads<=0;returns<=0;writes<=0;last_address<=0;end
  else if(functional_reset_n)begin
   if(rd_accept)begin reads<=reads+1'b1;last_address<=address;end
   if(rd_return)returns<=returns+1'b1;
   if(wr_accept)writes<=writes+1'b1;
  end
 end
 localparam N=`PAR_TOTAL_POINTS;
 reg [63:0] corners[0:N-1];reg [63:0] corner_q;
 reg [15:0] count;reg [31:0] events,saved_job;
 reg [287:0] saved_params;reg [63:0] saved_rms;
 reg saved_candidate,saved_reduced,saved_low_coverage;reg [3:0] saved_reason,saved_warnings;
 reg param_valid,metric_valid,conv;reg [7:0] last_status,last_error;
 reg [15:0] saved_accepted;reg [31:0] error_job;
 reg [3:0] saved_phase;reg [15:0] last_detect_count;reg [7:0] last_detect_flags,last_detect_view;
 always @(posedge clk)begin
  if(argument<N)corner_q<=corners[argument];else corner_q<=0;
  if(rst_n&&functional_reset_n&&corner_valid&&count<N)corners[count]<= {corner_y,corner_x};
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin saved_low_coverage<=0;saved_reduced<=0;saved_candidate<=0;saved_reason<=0;saved_warnings<=0;saved_accepted<=0;error_job<=0;count<=0;events<=0;saved_job<=0;saved_params<=0;saved_rms<=0;
   last_detect_count<=0;last_detect_flags<=0;last_detect_view<=0;param_valid<=0;metric_valid<=0;conv<=0;last_status<=0;last_error<=0;saved_phase<=0;end
  else if(functional_reset_n)begin
   if(detect_done)begin last_detect_count<=detect_count;last_detect_flags<=detect_flags;last_detect_view<=view;end
   if(start_cal)begin saved_low_coverage<=0;saved_reduced<=0;saved_candidate<=0;saved_reason<=0;saved_warnings<=0;last_detect_count<=0;last_detect_flags<=0;last_detect_view<=0;
    saved_accepted<=0;last_status<=0;saved_phase<=0;count<=0;param_valid<=0;metric_valid<=0;conv<=0;saved_job<=job+1'b1;end
   else if(corner_valid&&count<N)count<=count+1'b1;
   if(result_valid)begin saved_params<=params;param_valid<=1;end
   if(diag_valid)begin
    saved_low_coverage<=low_coverage;saved_reduced<=reduced_model;saved_candidate<=candidate_valid;saved_reason<=check_reason;saved_warnings<=check_warnings;
    if(candidate_valid)saved_params<=params;
    saved_accepted<=accepted_steps;saved_rms<=rms;metric_valid<=metrics_valid;conv<=converged;saved_phase<=cal_phase;end
   if(rsp_valid||diag_valid)begin
    events<=events+1'b1;last_status<=diag_valid?diag_status:rsp_status;
    if((diag_valid?diag_status:rsp_status)!=0)begin last_error<=diag_valid?diag_status:rsp_status;error_job<=saved_job;end
   end
  end
 end
 wire corner_ok=argument<count&&argument<N;
 wire [7:0] flags={saved_low_coverage,saved_reduced,saved_candidate,corner_ok,conv,metric_valid,param_valid,functional_reset_n};
 // Bytes 0..15 header; 16..59 parameters+RMS or one corner; 60 count,62 tag.
 wire [351:0] payload=argument==16'hffff ?{saved_rms,saved_params}:
                          argument==16'hfffe ?{64'b0,error_job,saved_accepted,last_detect_view,last_detect_count,last_address,writes,returns,reads,4'b0,bus_state,live}:
                          {288'b0,(corner_ok?corner_q:64'b0)};
 assign snapshot={argument,count,payload,last_error,last_status,{saved_warnings,saved_phase},last_detect_flags,
                  events,{saved_reason,cal_phase},flags,view,{2'b0,phase},saved_job};
endmodule
