`timescale 1ns/1ps
`include "calib_defs.vh"
// Three complete tasks without forcing/mocking any child: last-view detector
// failure, repeated-view init rejection, then a genuine full calibration.
module tb_configurable_top;
 reg clk=0,rst_n=0;always #5 clk=~clk;
 reg collect_valid=0,corner_valid=0,corner_last=0,view_rsp_valid=0,cmd_valid=0;
 reg camera_ready=0,diag_ready=0,rsp_ready=0;
 wire collect_ready,corner_ready,view_rsp_ready,cmd_ready,camera_valid,diag_valid,rsp_valid;
 reg [31:0] job=0,x=0,y=0;reg [7:0] view_id=0,point_index=0,view_status=0;
 wire [`PAR_VIEWS-1:0] dbg_done,dbg_error,dbg_usable;
 wire [8*`PAR_VIEWS-1:0] dbg_status;wire [`PAR_POINT_BITS*`PAR_VIEWS-1:0] dbg_count;
 wire [7:0] diag_status,rsp_status;wire [3:0] phase;wire metrics,converged,usable;
 wire [31:0] rsp_job;wire [287:0] camera;wire [63:0] rms;
 wire [`PAR_POSES_W-1:0] poses;wire [`PAR_VIEW_RMS_W-1:0] vrms;
 wire [2:0] best_id;wire [15:0] accepted;
 calib_top dut(.clk(clk),.rst_n(rst_n),.collect_valid(collect_valid),.collect_ready(collect_ready),.collect_job_id(job),
 .corner_valid(corner_valid),.corner_ready(corner_ready),.corner_job_id(job),.corner_view_id(view_id),.corner_point_index(point_index),.corner_x_fp32(x),.corner_y_fp32(y),.corner_last(corner_last),
 .view_rsp_valid(view_rsp_valid),.view_rsp_ready(view_rsp_ready),.view_rsp_job_id(job),.view_rsp_view_id(view_id),.view_rsp_status(view_status),
 .dbg_view_done(dbg_done),.dbg_view_status(dbg_status),.dbg_view_point_count(dbg_count),.dbg_view_format_error(dbg_error),.dbg_view_usable(dbg_usable),
 .cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_job_id(job),.cmd_width(16'd1280),.cmd_height(16'd720),.cmd_square_size_fp64(64'h4004000000000000),
 .camera_valid(camera_valid),.camera_ready(camera_ready),.camera_last(),.camera_calib_id(),.camera_width(),.camera_height(),.camera_usable(usable),.camera_params(camera),
 .diag_valid(diag_valid),.diag_ready(diag_ready),.diag_job_id(),.diag_status(diag_status),.diag_phase(phase),.diag_metrics_valid(metrics),.diag_converged(converged),.diag_weak_geometry(),
 .diag_rms_fp64(rms),.diag_view_rms_fp64(vrms),.diag_max_error_fp64(),.diag_poses_fp64(poses),.diag_seed_id(best_id),.diag_accepted_steps(accepted),
 .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.rsp_job_id(rsp_job));
 reg [63:0] points[0:`PAR_TOTAL_POINTS-1];
 reg [63:0] expected_m[0:1+13*`PAR_VIEWS];
 integer errors=0,cycles=0,cases=0,lm_calls=0,seeds=0,report,i,v,p;
 reg done=0;reg [287:0] held_camera;reg [`PAR_POSES_W-1:0] held_poses;
 always @(posedge clk)if(rst_n&&!done)begin
 cycles=cycles+1;
 if(dut.pc==dut.LM_CMD&&dut.lm_ready)lm_calls=lm_calls+1;
 if(dut.seed_valid&&dut.pc==dut.INIT_WAIT)seeds=seeds+1;
 end
 task check(input bit ok,input string msg);begin if(!ok)begin errors=errors+1;$fdisplay(report,"FAIL %s cycles=%0d",msg,cycles);end end endtask
 task start_job;begin
 job=job+1;lm_calls=0;seeds=0;collect_valid=1;do @(posedge clk);while(!collect_ready);@(negedge clk);collect_valid=0;repeat(2)@(negedge clk);
 check(dbg_done==0&&dbg_count==0,"new job clears every view");end endtask
 task send_view(input integer number,input bit repeated,input bit failed);begin
 view_id=number;
 if(!failed)for(integer q=0;q<`PAR_POINTS;q=q+1)begin
 point_index=q;x=points[(repeated?0:number)*`PAR_POINTS+q][31:0];y=points[(repeated?0:number)*`PAR_POINTS+q][63:32];corner_last=q==`PAR_POINTS-1;corner_valid=1;
 do @(posedge clk);while(!corner_ready);@(negedge clk);corner_valid=0;
 end
 view_status=failed?8'h55:0;view_rsp_valid=1;do @(posedge clk);while(!view_rsp_ready);@(negedge clk);view_rsp_valid=0;
 end endtask
 task launch;begin cmd_valid=1;do @(posedge clk);while(!cmd_ready);@(negedge clk);cmd_valid=0;end endtask
 task finish_job(input [7:0] expected_status);begin
 wait(diag_valid);@(negedge clk);check(diag_status==expected_status,"diagnostic status");
 held_camera=camera;held_poses=poses;
 repeat(7)begin @(negedge clk);check(diag_valid&&camera==held_camera&&poses==held_poses&&!rsp_valid,"output held under backpressure");end
 camera_ready=1;diag_ready=1;@(negedge clk);camera_ready=0;diag_ready=0;
 check(rsp_valid&&rsp_status==expected_status&&rsp_job==job,"completion job/status");
 repeat(3)@(negedge clk);rsp_ready=1;@(negedge clk);rsp_ready=0;cases=cases+1;
 $fdisplay(report,"case=%0d status=%0d phase=%0d seeds=%0d lm_calls=%0d cycles=%0d",cases,expected_status,phase,seeds,lm_calls,cycles);
 end endtask
 real actual,expected,difference;
 initial begin
 report=$fopen("config_top_results.txt","w");if(!report)$fatal(1,"open report");
 $readmemh("points.hex",points);$readmemh("metrics.hex",expected_m);
 repeat(5)@(negedge clk);rst_n=1;@(negedge clk);
 start_job();for(v=0;v<`PAR_VIEWS;v=v+1)send_view(v,0,v==`PAR_VIEWS-1);
 finish_job(8'h55);check(phase==0&&dbg_status[8*(`PAR_VIEWS-1)+:8]==8'h55&&lm_calls==0,"last view failure propagated");
 start_job();for(v=0;v<`PAR_VIEWS;v=v+1)send_view(v,1,0);launch();finish_job(4);
 check(phase==1&&seeds==0&&lm_calls==0,"all repeated views rejected by real init");
 start_job();for(v=0;v<`PAR_VIEWS;v=v+1)send_view(v,0,0);launch();wait(diag_valid);@(negedge clk);
 check(seeds==5&&lm_calls==15&&metrics&&phase>=3,"all real seeds and stages completed");
 check(diag_status==0 || (diag_status==4&&!converged),"iteration cap may reject unconverged result");
 check(best_id==0,"known-camera Zhang candidate is best on this fixture");
 check(rms[62:52]!=2047&&!$isunknown(rms)&&$bitstoreal(rms)<0.001,"final RMS fits analytical pinhole data");
 for(i=0;i<12*`PAR_VIEWS;i=i+1)begin
 actual=$bitstoreal(poses[64*i+:64]);expected=$bitstoreal(expected_m[2+`PAR_VIEWS+i]);difference=actual-expected;if(difference<0)difference=-difference;if(expected<0)expected=-expected;
 check(poses[64*i+52+:11]!=2047&&!$isunknown(poses[64*i+:64])&&difference<0.002*(1+expected),"final physical pose matches known geometry");end
 $fdisplay(report,"full metrics rms=%g seed=%0d converged=%0d accepted=%0d",$bitstoreal(rms),best_id,converged,accepted);
 finish_job(diag_status);
 check(errors==0,"complete suite");done=1;$fdisplay(report,"RESULT cases=%0d errors=%0d cycles=%0d",cases,errors,cycles);$fclose(report);
 end
endmodule
