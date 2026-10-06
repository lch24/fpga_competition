`timescale 1ns/1ps
`include "calib_defs.vh"
// CONTROL_ONLY explicitly mocks child interfaces; default uses the complete real RTL.
module tb_calib_top;
 parameter CONTROL_ONLY=0;
 parameter REAL_INPUT=0;
 reg [4095:0] vector_file;
 integer vector_arg;
 reg [15:0] test_width=640,test_height=480;
 reg [63:0] test_square=64'h4039000000000000;
 reg  clk=0;
 reg  rst_n=0;
 reg  collect_valid=0;
 wire  collect_ready;
 reg [31:0] collect_job_id=0;
 reg  corner_valid=0;
 wire  corner_ready;
 reg [31:0] corner_job_id=0;
 reg [7:0] corner_view_id=0;
 reg [7:0] corner_point_index=0;
 reg [31:0] corner_x_fp32=0;
 reg [31:0] corner_y_fp32=0;
 reg  corner_last=0;
 reg  view_rsp_valid=0;
 wire  view_rsp_ready;
 reg [31:0] view_rsp_job_id=0;
 reg [7:0] view_rsp_view_id=0;
 reg [7:0] view_rsp_status=0;
 wire [`PAR_VIEWS-1:0] dbg_view_done;
 wire [8*`PAR_VIEWS-1:0] dbg_view_status;
 wire [`PAR_POINT_BITS*`PAR_VIEWS-1:0] dbg_view_point_count;
 wire [`PAR_VIEWS-1:0] dbg_view_format_error;
 wire [`PAR_VIEWS-1:0] dbg_view_usable;
 reg  cmd_valid=0;
 wire  cmd_ready;
 reg [31:0] cmd_job_id=0;
 reg [15:0] cmd_width=0;
 reg [15:0] cmd_height=0;
 reg [63:0] cmd_square_size_fp64=0;
 wire  camera_valid;
 reg  camera_ready=0;
 wire  camera_last;
 wire [31:0] camera_calib_id;
 wire [15:0] camera_width;
 wire [15:0] camera_height;
 wire  camera_usable;
 wire [`PAR_CAMERA_W-1:0] camera_params;
 wire  diag_valid;
 reg  diag_ready=0;
 wire [31:0] diag_job_id;
 wire [7:0] diag_status;
 wire [3:0] diag_phase;
 wire  diag_metrics_valid;
 wire  diag_converged;
 wire  diag_weak_geometry;
 wire [63:0] diag_rms_fp64;
 wire [`PAR_VIEW_RMS_W-1:0] diag_view_rms_fp64;
 wire [63:0] diag_max_error_fp64;
 wire [`PAR_POSES_W-1:0] diag_poses_fp64;
 wire [2:0] diag_seed_id;
 wire [15:0] diag_accepted_steps;
 wire  rsp_valid;
 reg  rsp_ready=0;
 wire [7:0] rsp_status;
 wire [31:0] rsp_job_id;
 always #5 clk=~clk;
 calib_top dut(.clk(clk),.rst_n(rst_n),.collect_valid(collect_valid),.collect_ready(collect_ready),.collect_job_id(collect_job_id),.corner_valid(corner_valid),.corner_ready(corner_ready),.corner_job_id(corner_job_id),.corner_view_id(corner_view_id),.corner_point_index(corner_point_index),.corner_x_fp32(corner_x_fp32),.corner_y_fp32(corner_y_fp32),.corner_last(corner_last),.view_rsp_valid(view_rsp_valid),.view_rsp_ready(view_rsp_ready),.view_rsp_job_id(view_rsp_job_id),.view_rsp_view_id(view_rsp_view_id),.view_rsp_status(view_rsp_status),.dbg_view_done(dbg_view_done),.dbg_view_status(dbg_view_status),.dbg_view_point_count(dbg_view_point_count),.dbg_view_format_error(dbg_view_format_error),.dbg_view_usable(dbg_view_usable),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_job_id(cmd_job_id),.cmd_width(cmd_width),.cmd_height(cmd_height),.cmd_square_size_fp64(cmd_square_size_fp64),.camera_valid(camera_valid),.camera_ready(camera_ready),.camera_last(camera_last),.camera_calib_id(camera_calib_id),.camera_width(camera_width),.camera_height(camera_height),.camera_usable(camera_usable),.camera_params(camera_params),.diag_valid(diag_valid),.diag_ready(diag_ready),.diag_job_id(diag_job_id),.diag_status(diag_status),.diag_phase(diag_phase),.diag_metrics_valid(diag_metrics_valid),.diag_converged(diag_converged),.diag_weak_geometry(diag_weak_geometry),.diag_rms_fp64(diag_rms_fp64),.diag_view_rms_fp64(diag_view_rms_fp64),.diag_max_error_fp64(diag_max_error_fp64),.diag_poses_fp64(diag_poses_fp64),.diag_seed_id(diag_seed_id),.diag_accepted_steps(diag_accepted_steps),.rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.rsp_job_id(rsp_job_id));

 integer errors=0,cases=0,report,fd,rc,i,v,cycles=0,expected_id,expected_steps,expected_converged,expected_weak;
 reg [`PAR_STATE_W-1:0] expected_seeds[0:4];
 integer camera_count=0,diag_count=0,lm_commands=0,lm_responses=0,seed_inputs=0,read_count=0;
 reg done=0,monitor=0,held_camera=0,held_diag=0,held_rsp=0;
 reg [64*`PAR_TOTAL_POINTS-1:0] points;
 reg [287:0] expected_camera;
 reg [64*(2+13*`PAR_VIEWS)-1:0] expected_metrics;
 wire [64*(2+13*`PAR_VIEWS)-1:0] actual_metrics={diag_poses_fp64,diag_max_error_fp64,diag_view_rms_fp64,diag_rms_fp64};
 wire [64*(2+13*`PAR_VIEWS)+639:0] output_snapshot={camera_last,camera_calib_id,camera_width,camera_height,camera_usable,camera_params,
   diag_job_id,diag_status,diag_phase,diag_metrics_valid,diag_converged,diag_weak_geometry,
   actual_metrics,diag_seed_id,diag_accepted_steps,rsp_status,rsp_job_id};
 reg [64*(2+13*`PAR_VIEWS)+639:0] saved_snapshot;
 reg [`PAR_STATE_W-1:0] previous_state;
 integer previous_stage,previous_seed;
 reg [`PAR_VIEW_BITS-1:0] last_read_view;reg [`PAR_POINT_BITS-1:0] last_read_index;reg previous_read=0;
 task check;input condition;input [767:0] label;begin
  if(condition!==1'b1)begin errors=errors+1;if(errors<60)begin $display("FAIL case=%0d %0s pc=%0d",cases,label,dut.pc);$fdisplay(report,"FAIL case=%0d %0s pc=%0d",cases,label,dut.pc);end end
 end endtask
 function real fp32;input [31:0] x;real value;begin
  value=(x[30:23]==0)?(x[22:0]/8388608.0)*(2.0**(-126)):(1.0+x[22:0]/8388608.0)*(2.0**(integer'(x[30:23])-127));
  fp32=x[31]?-value:value;
 end endfunction
 task near64;input [63:0] a,b;input real tolerance;real x,y,d;begin
  x=$bitstoreal(a);y=$bitstoreal(b);d=x-y;if(d<0)d=-d;
  check((^a)!==1'bx && a[62:52]!=2047 && d<=tolerance*(1+(y<0?-y:y)),"C++ diagnostic comparison");
  if(d>tolerance*(1+(y<0?-y:y)))$fdisplay(report,"actual=%.17g expected=%.17g difference=%.9g",x,y,d);
 end endtask
 task near32;input [31:0] a,b;real x,y,d;begin
  x=fp32(a);y=fp32(b);d=x-y;if(d<0)d=-d;
  check((^a)!==1'bx && a[30:23]!=255 && d<=2e-5*(1+(y<0?-y:y)),"C++ camera comparison");
  $fdisplay(report,"camera actual=%.12g expected=%.12g difference=%.9g",x,y,d);
 end endtask
 always @(posedge clk) begin
  if(rst_n && monitor)begin
   cycles=cycles+1;
   if(held_camera)check(camera_valid && output_snapshot===saved_snapshot,"camera payload stable during stall");
   if(held_diag)check(diag_valid && output_snapshot===saved_snapshot,"diagnostic payload stable during stall");
   if(held_rsp)check(rsp_valid && output_snapshot===saved_snapshot,"response payload stable during stall");
   held_camera=camera_valid && !camera_ready;held_diag=diag_valid && !diag_ready;held_rsp=rsp_valid && !rsp_ready;saved_snapshot=output_snapshot;
   if(camera_valid && camera_ready)camera_count=camera_count+1;
   if(diag_valid && diag_ready)diag_count=diag_count+1;
   if(rsp_valid)check(diag_count==1 && camera_count==(rsp_status==0?1:0) && !diag_valid && !camera_valid,"all outputs precede response exactly once");
   if(dut.read_en)begin
    check(dut.read_view<`PAR_VIEWS && dut.read_index<`PAR_POINTS && dbg_view_usable[dut.read_view],"read only committed legal corner");
    read_count=read_count+1;
   end
   if(!CONTROL_ONLY)begin
    if(previous_read)check(dut.rd_valid && dut.rd_x===points[(last_read_view*`PAR_POINTS+last_read_index)*64+:32] && dut.rd_y===points[(last_read_view*`PAR_POINTS+last_read_index)*64+32+:32],"fixed-latency RAM return and data");
    previous_read=dut.read_en;last_read_view=dut.read_view;last_read_index=dut.read_index;
   end
   check(!(dut.init_owner && dut.lm_owner) && !(dut.lm_owner && dut.check_owner) && !(dut.init_owner && dut.check_owner),"exclusive RAM owner");
   if(dut.pc==dut.INIT_WAIT && dut.seed_valid && dut.seed_ready)begin expected_seeds[dut.seed_count]=dut.seed_state;seed_inputs=seed_inputs+1;end
   if(dut.pc==dut.LM_CMD && dut.lm_ready)begin
    check(dut.stage==lm_commands%3,"three LM stages in order");
    if(dut.stage==0)check(dut.current===expected_seeds[dut.seed_index],"load each original seed");
    else check(dut.current===previous_state,"chain last accepted state even if unconverged");
    lm_commands=lm_commands+1;
   end
   if(dut.pc==dut.LM_WAIT && dut.lm_valid)begin
    lm_responses=lm_responses+1;previous_state=dut.lm_state;
    $fdisplay(report,"LM seed=%0d stage=%0d cycles=%0d cost=%.17g converged=%0d accepted=%0d status=%0d",dut.seed_ids[dut.seed_index],dut.stage,cycles,$bitstoreal(dut.lm_cost),dut.lm_converged,dut.lm_accepted,dut.lm_status);$fflush(report);
    if(!CONTROL_ONLY)$display("TOP LM seed=%0d stage=%0d cycles=%0d cost=%.12g",dut.seed_ids[dut.seed_index],dut.stage,cycles,$bitstoreal(dut.lm_cost));
   end
  end else begin held_camera=0;held_diag=0;held_rsp=0;previous_read=0;end
 end
 task reset_dut;begin
  @(negedge clk);monitor=0;rst_n=0;collect_valid=0;corner_valid=0;view_rsp_valid=0;cmd_valid=0;camera_ready=0;diag_ready=0;rsp_ready=0;
  repeat(3)@(negedge clk);check(!collect_ready && !corner_ready && !view_rsp_ready && !cmd_ready && !camera_valid && !diag_valid && !rsp_valid,"reset masks all handshakes");
  rst_n=1;@(negedge clk);check(collect_ready,"reset returns idle");
 end endtask
 task begin_job;begin
  @(negedge clk);check(collect_ready,"new collect ready");camera_count=0;diag_count=0;cycles=0;read_count=0;lm_commands=0;lm_responses=0;seed_inputs=0;monitor=1;
  collect_job_id=32'h80000000+cases;collect_valid=1;
  @(negedge clk);collect_valid=0;collect_job_id=0;check(!corner_ready && !view_rsp_ready,"clear bubble blocks input");
  @(negedge clk);check(dbg_view_done==0 && dbg_view_point_count==0 && dbg_view_usable==0,"new collect clears old debug state");
  corner_job_id=32'h80000000+cases;view_rsp_job_id=corner_job_id;
 end endtask
 task send_point;input integer view_number,point_number;begin
  corner_view_id=view_number;corner_point_index=point_number;corner_last=(point_number==(`PAR_POINTS-1));
  corner_x_fp32=points[(view_number*`PAR_POINTS+point_number)*64+:32];corner_y_fp32=points[(view_number*`PAR_POINTS+point_number)*64+32+:32];corner_valid=1;
  @(posedge clk);check(corner_ready,"corner accepted");@(negedge clk);corner_valid=0;
 end endtask
 task send_view;input integer view_number;input [7:0] code;begin
  view_rsp_view_id=view_number;view_rsp_status=code;view_rsp_valid=1;
  @(posedge clk);check(view_rsp_ready,"view response accepted independently");@(negedge clk);view_rsp_valid=0;
 end endtask
 task fill_points;begin
  for(v=0;v<`PAR_VIEWS;v=v+1)begin
   for(i=0;i<`PAR_POINTS;i=i+1)send_point(v,i);
   send_view(v,0);
  end
  repeat(2)@(negedge clk);check(cmd_ready && dbg_view_usable==7,"all views committed before command");
 end endtask
 task launch;begin
  cmd_job_id=32'h80000000+cases;cmd_width=test_width;cmd_height=test_height;cmd_square_size_fp64=test_square;cmd_valid=1;
  @(posedge clk);check(cmd_ready,"command handshake");@(negedge clk);cmd_valid=0;
  cmd_job_id=0;cmd_width=0;cmd_height=0;cmd_square_size_fp64=0;
 end endtask
 task await_output;integer watchdog;begin
  watchdog=0;while(!diag_valid && watchdog<1500000000)begin @(negedge clk);watchdog=watchdog+1;end
  if(!diag_valid)$fatal(1,"TOP timeout pc=%0d LM pc=%0d",dut.pc,dut.optimizer.pc);
 end endtask
 task finish_job;input [7:0] code;input [3:0] where;input integer order;reg [(`PAR_POINT_BITS+11)*`PAR_VIEWS-1:0] debug_saved;begin
  await_output();check(diag_status==code && diag_phase==where && diag_job_id==32'h80000000+cases,"diagnostic status/phase/job");
  check(camera_valid==(code==0),"camera only on success");debug_saved={dbg_view_done,dbg_view_status,dbg_view_point_count,dbg_view_format_error,dbg_view_usable};
  repeat(7)begin @(negedge clk);check(!rsp_valid && !collect_ready && !cmd_ready,"busy while output stalled");end
  if(order==0)begin
   diag_ready=1;@(negedge clk);diag_ready=0;repeat(4)@(negedge clk);
   if(code==0)check(camera_valid && !rsp_valid,"diag-first waits for camera");
   camera_ready=1;@(negedge clk);camera_ready=0;
  end else if(order==1)begin
   camera_ready=1;@(negedge clk);camera_ready=0;repeat(4)@(negedge clk);
   check(diag_valid && !rsp_valid,"camera-first waits for diagnostic");
   diag_ready=1;@(negedge clk);diag_ready=0;
  end else begin camera_ready=1;diag_ready=1;@(negedge clk);camera_ready=0;diag_ready=0;end
  check(rsp_valid && rsp_status==code && rsp_job_id==32'h80000000+cases,"completion after output");
  repeat(5)@(negedge clk);check(!collect_ready && !cmd_ready,"completion stall prevents new task");
  rsp_ready=1;@(negedge clk);rsp_ready=0;check(collect_ready,"next task without reset");
  check(debug_saved==={dbg_view_done,dbg_view_status,dbg_view_point_count,dbg_view_format_error,dbg_view_usable},"debug retained after response");
  $fdisplay(report,"case=%0d cycles=%0d reads=%0d seeds=%0d lm_calls=%0d status=%0d seed=%0d steps=%0d",cases,cycles,read_count,seed_inputs,lm_commands,code,diag_seed_id,diag_accepted_steps);$fflush(report);
  monitor=0;cases=cases+1;
 end endtask
 // Deliberate child-interface model for control branch tests only; no arithmetic claims from these cases.
 reg mock_init_ready=0,mock_init_valid=0,mock_seed_valid=0,mock_lm_ready=0,mock_lm_valid=0,mock_check_ready=0,mock_check_valid=0;
 reg [7:0] mock_init_status=0,mock_lm_status=0,mock_check_status=0,mock_accepted=0;
 reg [2:0] mock_seed_id=0,mock_count=0;
 reg [`PAR_STATE_W-1:0] mock_seed_state=0,mock_lm_state=0;
 reg [63:0] mock_cost=0;reg mock_converged=0,mock_usable=0,mock_metrics=0;
 task mock_children;begin
  force dut.compute_rst_n=0;
  force dut.init_ready=mock_init_ready;force dut.init_valid=mock_init_valid;force dut.init_status=mock_init_status;force dut.init_count=mock_count;
  force dut.seed_valid=mock_seed_valid;force dut.seed_id=mock_seed_id;force dut.seed_state=mock_seed_state;
  force dut.lm_ready=mock_lm_ready;force dut.lm_valid=mock_lm_valid;force dut.lm_status=mock_lm_status;
  force dut.lm_state=mock_lm_state;force dut.lm_cost=mock_cost;force dut.lm_converged=mock_converged;force dut.lm_accepted=mock_accepted;
  force dut.check_ready=mock_check_ready;force dut.check_valid=mock_check_valid;force dut.check_status=mock_check_status;
  force dut.check_usable=mock_usable;force dut.check_metrics=mock_metrics;force dut.check_weak=1;
  force dut.check_camera=288'h12345678;force dut.check_rms=64'h3ff0000000000000;
  force dut.check_view_rms=192'h123;force dut.check_max=64'h4000000000000000;force dut.check_poses=2304'h456;
 end endtask
 task clear_mock;begin
  mock_init_ready=0;mock_init_valid=0;mock_seed_valid=0;mock_lm_ready=0;mock_lm_valid=0;mock_check_ready=0;mock_check_valid=0;
  mock_init_status=0;mock_lm_status=0;mock_check_status=0;mock_count=0;mock_converged=0;mock_usable=0;mock_metrics=0;
 end endtask
 task mock_init_start;begin
  while(dut.pc!=dut.INIT_CMD)@(negedge clk);
  repeat(3)begin @(negedge clk);check(dut.initializer.cmd_valid && dut.width==640 && dut.height==480,"held init request and locked config");end
  mock_init_ready=1;@(negedge clk);mock_init_ready=0;
 end endtask
 function [`PAR_STATE_W-1:0] seed_pattern;input [2:0] id;integer word;
  begin for(word=0;word<`PAR_STATE_N;word=word+1)seed_pattern[64*word+:64]=(word==0)?64'h100+id:64'h3ff0000000000000+word+id*`PAR_STATE_N;end
 endfunction
 task mock_seed;input [2:0] id;begin
  mock_seed_id=id;mock_seed_state=seed_pattern(id);mock_seed_valid=1;
  do @(posedge clk);while(!dut.seed_ready && dut.pc==dut.INIT_WAIT);
  @(negedge clk);mock_seed_valid=0;
 end endtask
 task mock_init_end;input [2:0] count;input [7:0] code;begin
  mock_count=count;mock_init_status=code;mock_init_valid=1;@(negedge clk);mock_init_valid=0;
 end endtask
 task mock_lm;input real cost_value;input converged_value;input [7:0] code;begin
  while(dut.pc!=dut.LM_CMD)@(negedge clk);
  repeat(2)@(negedge clk);check(dut.optimizer.cmd_valid,"LM request remains valid until ready");
  mock_lm_state=dut.current+1728'h1000;mock_lm_ready=1;@(negedge clk);mock_lm_ready=0;
  repeat(2)@(negedge clk);mock_cost=$realtobits(cost_value);mock_converged=converged_value;mock_accepted=dut.stage+1;mock_lm_status=code;
  mock_lm_valid=1;@(negedge clk);mock_lm_valid=0;
 end endtask
 task mock_check;input [7:0] code;input usable,metric;begin
  while(dut.pc!=dut.CHECK_CMD)@(negedge clk);
  repeat(3)@(negedge clk);mock_check_ready=1;@(negedge clk);mock_check_ready=0;
  mock_check_status=code;mock_usable=usable;mock_metrics=metric;mock_check_valid=1;@(negedge clk);mock_check_valid=0;
 end endtask
 task one_seed;begin clear_mock();begin_job();fill_points();launch();mock_init_start();mock_seed(2);mock_init_end(1,0);end endtask
 integer n,s,dump_file;
 initial begin
  if(!REAL_INPUT && (`PAR_VIEWS!=3 || `PAR_BOARD_ROWS!=5 || `PAR_BOARD_COLS!=8 || `PAR_LM_MAX_ITERS!=150 || `PAR_LM_MAX_TRIES!=16))$fatal(1,"legacy synthetic/control fixtures require the default configuration; use run_configurable.ps1 or real input");
  report=$fopen(CONTROL_ONLY?"calib_top_control_results.txt":"calib_top_results.txt","w");
  vector_file="../../../data/calibration/calib_top_vectors.txt";
  vector_arg=$value$plusargs("VECTOR_FILE=%s",vector_file);
  fd=$fopen(REAL_INPUT?"real_vectors.txt":vector_file,"r");if(!fd || !report)$fatal(1,"file open");
  if(REAL_INPUT)begin
   rc=$fscanf(fd,"%d %d %h\n",test_width,test_height,test_square);
   if(rc!=3 || CONTROL_ONLY)$fatal(1,"real input configuration");
  end
  rc=$fscanf(fd,"%d %d %d %d %h %h %h\n",expected_id,expected_steps,expected_converged,expected_weak,expected_camera,expected_metrics,points);$fclose(fd);
  if(rc!=7)$fatal(1,"vector format");reset_dut();
  if(REAL_INPUT)begin
   begin_job();fill_points();launch();await_output();
   check(diag_status==0 && diag_metrics_valid && diag_converged==expected_converged && diag_weak_geometry==expected_weak,"real C++ pipeline success and flags");
   check(seed_inputs>=1 && seed_inputs<=5 && lm_commands==3*seed_inputs && lm_responses==lm_commands,"every real seed executes three stages");
   check(camera_width==test_width && camera_height==test_height && camera_calib_id==32'h80000000+cases && camera_usable,"real image configuration");
   dump_file=$fopen("actual_camera.hex","w");if(!dump_file)$fatal(1,"camera dump open");
   for(i=0;i<9;i=i+1)begin
    $fdisplay(dump_file,"%08h",camera_params[32*i+:32]);
    near32(camera_params[32*i+:32],expected_camera[32*i+:32]);
   end
   $fclose(dump_file);
   dump_file=$fopen("actual_metrics.hex","w");if(!dump_file)$fatal(1,"diagnostic dump open");
   for(i=0;i<2+13*`PAR_VIEWS;i=i+1)begin
    $fdisplay(dump_file,"%016h",actual_metrics[64*i+:64]);
    near64(actual_metrics[64*i+:64],expected_metrics[64*i+:64],i<2+`PAR_VIEWS?2e-7:2e-5);
   end
   $fclose(dump_file);
   check(camera_params[6*32+:32]==0,"real k3 remains fixed zero");
   $fdisplay(report,"C++ accepted=%0d; RTL seed=%0d accepted=%0d",expected_steps,diag_seed_id,diag_accepted_steps);
   finish_job(0,4,0);
  end else if(!CONTROL_ONLY)begin
   begin_job();fill_points();launch();await_output();
   check(diag_status==0 && diag_metrics_valid && diag_converged==expected_converged && diag_weak_geometry==expected_weak,"full C++ pipeline success");
   check(seed_inputs==5 && lm_commands==15 && lm_responses==15,"all five real seeds and fifteen LM stages executed");
   check(camera_width==640 && camera_height==480 && camera_calib_id==32'h80000000+cases && camera_usable,"locked camera configuration");
   for(i=0;i<9;i=i+1)near32(camera_params[32*i+:32],expected_camera[32*i+:32]);
   for(i=0;i<2+13*`PAR_VIEWS;i=i+1)near64(actual_metrics[64*i+:64],expected_metrics[64*i+:64],i<2+`PAR_VIEWS?2e-7:2e-5);
   check(camera_params[6*32+:32]==0,"k3 fixed zero");
   $fdisplay(report,"C++ best_seed=%0d steps=%0d; RTL best_seed=%0d steps=%0d",expected_id,expected_steps,diag_seed_id,diag_accepted_steps);
   finish_job(0,4,0);
   // A second real task without reset: identical views rejected by initialization.
   points[2560+:2560]=points[0+:2560];points[5120+:2560]=points[0+:2560];
   begin_job();fill_points();launch();finish_job(4,1,1);
   check(lm_commands==0 && !diag_metrics_valid && diag_seed_id==7,"repeated views rejected before LM");
  end else begin
   mock_children();
   // Each input failure is followed by a fresh collect without resetting the cache.
   begin_job();send_view(0,2);finish_job(2,0,0);check(dbg_view_done==1 && dbg_view_status[7:0]==2,"detector failure preserved");
   begin_job();corner_job_id=9;send_point(0,0);finish_job(1,0,1);check(dbg_view_point_count==0,"bad corner job never forwarded");
   begin_job();corner_view_id=255;corner_valid=1;@(negedge clk);corner_valid=0;finish_job(1,0,2);
   begin_job();view_rsp_job_id=9;send_view(0,0);finish_job(1,0,0);
   begin_job();send_view(255,2);finish_job(1,0,1);
   begin_job();send_point(0,1);finish_job(1,0,2);check(dbg_view_format_error==1,"point order flagged");
   begin_job();send_view(0,0);finish_job(1,0,0);check(dbg_view_done==1 && dbg_view_format_error==1,"incomplete success retained");
   begin_job();send_point(1,0);finish_job(1,0,1);check(dbg_view_format_error==2,"view order flagged");
   begin_job();for(i=0;i<`PAR_POINTS;i=i+1)send_point(0,i);send_view(0,0);send_view(0,0);finish_job(1,0,2);
   for(n=0;n<5;n=n+1)begin
    begin_job();fill_points();cmd_job_id=32'h80000000+cases;cmd_width=640;cmd_height=480;cmd_square_size_fp64=$realtobits(1.0);
    case(n)0:cmd_job_id=0;1:cmd_width=1;2:cmd_height=1;3:cmd_square_size_fp64=0;4:cmd_square_size_fp64=64'h7ff0000000000000;endcase
    cmd_valid=1;@(negedge clk);cmd_valid=0;finish_job(1,0,n%3);
   end
   clear_mock();begin_job();fill_points();launch();mock_init_start();mock_init_end(0,4);finish_job(4,1,0);
   clear_mock();begin_job();fill_points();launch();mock_init_start();mock_init_end(0,0);finish_job(4,1,1);
   clear_mock();begin_job();fill_points();launch();mock_init_start();mock_seed(1);mock_seed(1);finish_job(1,1,2);
   clear_mock();begin_job();fill_points();launch();mock_init_start();mock_seed(5);finish_job(1,1,0);
   clear_mock();begin_job();fill_points();launch();mock_init_start();mock_seed(1);mock_init_end(2,0);finish_job(1,1,1);
   // For each LM stage a hardware failure must stop the whole task.
   for(n=0;n<3;n=n+1)begin one_seed();for(s=0;s<=n;s=s+1)mock_lm(2.0,0,s==n?5:0);finish_job(5,2,n);check(lm_commands==n+1,"no later stage after hardware failure");end
   // Tied cost keeps the earlier seed; worse candidate cannot overwrite best.
   for(n=0;n<3;n=n+1)begin
    clear_mock();begin_job();fill_points();launch();mock_init_start();mock_seed(0);mock_seed(2);mock_seed(4);mock_init_end(3,0);
    for(s=0;s<9;s=s+1)mock_lm(s<3?5.0:(s<6?2.0:2.0),s%3==2,0);
    mock_check(0,1,1);check(diag_seed_id==2 && diag_accepted_steps==6 && diag_converged && dut.best_state==seed_pattern(2)+1728'h3000,"strict minimum, tied first, accumulated accepted steps");
    finish_job(0,4,n);
   end
   // Lower-cost unconverged candidate wins; validation rejects it, no fallback.
   clear_mock();begin_job();fill_points();launch();mock_init_start();mock_seed(1);mock_seed(3);mock_init_end(2,0);
   for(s=0;s<6;s=s+1)mock_lm(s<3?5.0:1.0,s<3,0);
   mock_check(4,0,1);check(diag_seed_id==3 && !diag_converged && diag_metrics_valid,"minimum selected before convergence validation");finish_job(4,3,0);
   one_seed();for(s=0;s<3;s=s+1)mock_lm(1.0,1,0);mock_check(5,0,0);check(!diag_metrics_valid && actual_metrics==0,"invalid diagnostic fields zeroed");finish_job(5,3,1);
   one_seed();for(s=0;s<3;s=s+1)mock_lm(1.0,1,0);mock_check(0,0,1);finish_job(1,3,2);
   one_seed();for(s=0;s<3;s=s+1)mock_lm(1.0,1,0);
   // Reset during an outstanding validation command.
   while(dut.pc!=dut.CHECK_CMD)@(negedge clk);reset_dut();clear_mock();cases=cases+1;
   // Reset during collection, init, LM, output and pending response.
   begin_job();send_point(0,0);reset_dut();cases=cases+1;
   begin_job();fill_points();launch();mock_init_start();reset_dut();clear_mock();cases=cases+1;
   one_seed();while(dut.pc!=dut.LM_CMD)@(negedge clk);mock_lm_ready=1;@(negedge clk);reset_dut();clear_mock();cases=cases+1;
   one_seed();for(s=0;s<3;s=s+1)mock_lm(1.0,1,0);mock_check(0,1,1);reset_dut();clear_mock();cases=cases+1;
   one_seed();for(s=0;s<3;s=s+1)mock_lm(1.0,1,0);mock_check(0,1,1);camera_ready=1;diag_ready=1;@(negedge clk);check(rsp_valid,"response reached before reset");reset_dut();clear_mock();cases=cases+1;
   // Infinite/NaN/negative final cost cannot become a best solution.
   for(n=0;n<3;n=n+1)begin
    one_seed();mock_lm(1.0,1,0);mock_lm(1.0,1,0);
    mock_lm(n==0?$bitstoreal(64'h7ff0000000000000):(n==1?$bitstoreal(64'h7ff8000000000000):-1.0),0,0);
    finish_job(4,2,n);check(diag_seed_id==7 && !diag_metrics_valid,"no finite nonnegative best candidate");
   end
   // -0 is a valid zero cost; later larger cost cannot replace it.
   clear_mock();begin_job();fill_points();launch();mock_init_start();mock_seed(1);mock_seed(4);mock_init_end(2,0);
   for(s=0;s<6;s=s+1)mock_lm(s<3?$bitstoreal(64'h8000000000000000):1.0,1,0);
   mock_check(0,1,1);check(dut.best_cost==0 && diag_seed_id==1,"normalize zero and preserve earlier better solution");finish_job(0,4,0);
   // All five slots, strict replacement by the last slot, 450 accepted steps fits 16 bits.
   clear_mock();begin_job();fill_points();launch();mock_init_start();for(s=0;s<5;s=s+1)mock_seed(s);mock_init_end(5,0);
   for(s=0;s<15;s=s+1)begin
    while(dut.pc!=dut.LM_CMD)@(negedge clk);
    mock_lm_state=dut.current+1728'h1000;mock_lm_ready=1;@(negedge clk);mock_lm_ready=0;
    mock_cost=$realtobits(5.0-s/3);mock_converged=1;mock_accepted=150;mock_lm_valid=1;@(negedge clk);mock_lm_valid=0;
   end
   mock_check(0,1,1);check(diag_seed_id==4 && diag_accepted_steps==450 && lm_commands==15,"last slot wins and accepted sum does not truncate");finish_job(0,4,1);
   clear_mock();begin_job();fill_points();launch();mock_init_start();mock_seed(3);mock_seed(1);finish_job(1,1,2);
   clear_mock();begin_job();fill_points();launch();mock_init_start();for(s=0;s<5;s=s+1)mock_seed(s);mock_seed(0);finish_job(1,1,0);
   // Both input streams together: last point wins this cycle, response waits one cycle.
   clear_mock();begin_job();for(i=0;i<39;i=i+1)send_point(0,i);
   corner_view_id=0;corner_point_index=39;corner_last=1;corner_x_fp32=points[39*64+:32];corner_y_fp32=points[39*64+32+:32];corner_valid=1;
   view_rsp_view_id=0;view_rsp_status=0;view_rsp_valid=1;
   @(posedge clk);check(corner_ready && !view_rsp_ready,"last point has priority over same-view success");
   @(negedge clk);corner_valid=0;@(posedge clk);check(view_rsp_ready,"view success accepted next cycle");@(negedge clk);view_rsp_valid=0;
   check(dbg_view_done==1 && dbg_view_usable==1,"concurrent streams committed correctly");
   send_view(1,2);finish_job(2,0,1);
   // Invalid stream prevents concurrent good stream from writing anything.
   begin_job();view_rsp_job_id=0;view_rsp_view_id=0;view_rsp_status=0;view_rsp_valid=1;
   corner_view_id=0;corner_point_index=0;corner_last=0;corner_valid=1;
   @(posedge clk);check(view_rsp_ready && !corner_ready,"invalid job has deterministic priority");
   @(negedge clk);view_rsp_valid=0;corner_valid=0;finish_job(1,0,2);check(dbg_view_point_count==0 && dbg_view_done==0,"no partial commit beside invalid stream");
   // Early command remains pending through collection and clear; accept only after all three views.
   begin_job();cmd_valid=1;cmd_job_id=32'h80000000+cases;cmd_width=640;cmd_height=480;cmd_square_size_fp64=$realtobits(25.0);
   for(v=0;v<`PAR_VIEWS;v=v+1)begin for(i=0;i<`PAR_POINTS;i=i+1)begin check(!cmd_ready,"early command held until complete collection");send_point(v,i);end send_view(v,0);end
   while(!cmd_ready)@(negedge clk);@(negedge clk);cmd_valid=0;
   mock_init_start();mock_init_end(0,4);finish_job(4,1,0);
   // Successful recovery after all cancellation paths.
   one_seed();for(s=0;s<3;s=s+1)mock_lm(1.0,s==2,0);mock_check(0,1,1);finish_job(0,4,2);
  end
  $fdisplay(report,"RESULT cases=%0d errors=%0d",cases,errors);$fclose(report);
  done=1;$display("TOP_RESULT control=%0d cases=%0d errors=%0d",CONTROL_ONLY,cases,errors);
 end
endmodule
