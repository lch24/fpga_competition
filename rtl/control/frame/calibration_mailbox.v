// Join calibration packet and successful completion by job ID. Either may arrive
// first. Only the verified joint result can start map generation. Failed jobs
// emit a failure response without publishing new parameters. Caller launches
// exactly one calibration task through begin_valid/begin_ready.
module calibration_mailbox (
 input wire clk,rst_n,input wire begin_valid,output wire begin_ready,input wire [31:0] begin_job_id,
 input wire param_valid,output wire param_ready,input wire [31:0] param_calib_id,
 input wire [15:0] param_width,param_height,input wire param_camera_valid,input wire [287:0] param_values,
 input wire calib_rsp_valid,output wire calib_rsp_ready,input wire [31:0] calib_rsp_id,input wire [7:0] calib_rsp_status,
 output wire result_valid,input wire result_ready,output reg [7:0] result_status,
 output reg [31:0] result_calib_id,output reg [15:0] result_width,result_height,
 output reg [287:0] result_values
);
 reg active,packet_seen,response_seen,output_pending;
 reg camera_valid;reg [31:0] packet_id,response_id;reg [7:0] response_status;
 assign begin_ready=!active;assign param_ready=active&&!packet_seen&&!output_pending;
 assign calib_rsp_ready=active&&!response_seen&&!output_pending;
 assign result_valid=output_pending;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin active<=0;packet_seen<=0;response_seen<=0;output_pending<=0;camera_valid<=0;packet_id<=0;response_id<=0;response_status<=0;result_status<=0;result_calib_id<=0;result_width<=0;result_height<=0;result_values<=0;end
  else begin
   if(begin_valid&&begin_ready)begin active<=1;packet_seen<=0;response_seen<=0;result_calib_id<=begin_job_id;output_pending<=0;end
   if(param_valid&&param_ready)begin packet_seen<=1;packet_id<=param_calib_id;camera_valid<=param_camera_valid;result_width<=param_width;result_height<=param_height;result_values<=param_values;end
   if(calib_rsp_valid&&calib_rsp_ready)begin response_seen<=1;response_id<=calib_rsp_id;response_status<=calib_rsp_status;end
   if(active&&response_seen&&!output_pending)begin
    if(response_id!=result_calib_id)begin result_status<=4;output_pending<=1;end
    else if(response_status!=0)begin result_status<=response_status;output_pending<=1;end
    else if(packet_seen)begin
     result_status<=(packet_id!=result_calib_id||!camera_valid)?8'd4:8'd0;output_pending<=1;
    end
   end
   if(result_valid&&result_ready)begin active<=0;output_pending<=0;end
  end
 end
endmodule
