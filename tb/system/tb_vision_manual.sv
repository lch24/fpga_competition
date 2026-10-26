`timescale 1ns/1ps
`include "calib_defs.vh"
// Real sequence controller, protocol models for numerical workers.
module tb_vision_manual #(parameter BUILD_ONLY=0,ENABLE_DISTORT_DEMO=0,ENABLE_CORNER_REVIEW=0);
 reg clk=0,rst_n=0,finished=0,start_valid=0,start_remap=0;
 reg [31:0] remap_base=0,frame_base=0;
 reg frame_valid=0,frame_release_ready=0,rsp_ready=0,gray_rsp_valid=0;
 reg det_done=0,det_valid=0,mb_valid=0,remap_rsp_valid=0;
 reg [7:0] mb_status=0,remap_status=0;
 wire start_ready,frame_ready,frame_release_valid,rsp_valid,gray_cmd_valid,gray_rsp_ready;
 wire det_process,det_ready,view_rsp_valid,cal_cmd_valid,mb_ready,remap_cmd_valid,remap_rsp_ready;
 wire [7:0] rsp_status,view_status,view_id;wire [1:0] remap_opcode;
 wire [31:0] job,src_base;wire [287:0] params;wire algorithm_rst_n;
 always #12.5 clk=~clk;
 reg task_active=0;
 reg review_continue=0;wire review_wait;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)task_active<=0;
  else if(start_valid&&start_ready)task_active<=1;
  else if(rsp_valid&&rsp_ready)task_active<=0;
 end
 always @(negedge clk)if(rst_n&&algorithm_rst_n!==task_active)
  $fatal(1,"worker reset must span the complete task, including response stalls");
 vision_sequence #(.WIDTH(64),.HEIGHT(32),.MANUAL_CONTROL(1),.BUILD_ONLY(BUILD_ONLY),.ENABLE_DISTORT_DEMO(ENABLE_DISTORT_DEMO),.ENABLE_CORNER_REVIEW(ENABLE_CORNER_REVIEW)) dut(
 .review_wait(review_wait),.review_continue(review_continue),
 .clk(clk),.rst_n(rst_n),.layout_ok(1'b1),.start_valid(start_valid),.start_ready(start_ready),
 .start_remap(start_remap),.remap_base(remap_base),.square_size_fp64(64'h3ff0000000000000),
 .frame_valid(frame_valid),.frame_ready(frame_ready),.frame_base(frame_base),.frame_stride(32'd128),.frame_capacity(32'd4096),
 .frame_status(8'd0),.frame_config_ok(1'b1),.frame_release_valid(frame_release_valid),.frame_release_ready(frame_release_ready),
 .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.job(job),.view_id(view_id),.src_base(src_base),.algorithm_rst_n(algorithm_rst_n),
 .gray_cmd_valid(gray_cmd_valid),.gray_cmd_ready(1'b1),.gray_rsp_valid(gray_rsp_valid),.gray_rsp_ready(gray_rsp_ready),.gray_rsp_status(8'd0),
 .det_process(det_process),.det_done(det_done),.det_status(2'b01),.det_valid(det_valid),.det_ready(det_ready),.det_x(32'd0),.det_y(32'd0),.det_total(16'(`PAR_POINTS)),.det_grid_ok(1'b1),
 .collect_ready(1'b1),.corner_ready(1'b1),.view_rsp_valid(view_rsp_valid),.view_rsp_ready(1'b1),.view_status(view_status),
 .cal_cmd_valid(cal_cmd_valid),.cal_cmd_ready(1'b1),.mb_begin_ready(1'b1),.mb_valid(mb_valid),.mb_ready(mb_ready),.mb_status(mb_status),
 .mb_id(job),.mb_width(16'd64),.mb_height(16'd32),.mb_params(288'h12345678),.params(params),
 .remap_cmd_valid(remap_cmd_valid),.remap_cmd_ready(1'b1),.remap_opcode(remap_opcode),.remap_rsp_valid(remap_rsp_valid),.remap_rsp_ready(remap_rsp_ready),.remap_status(remap_status));
 task tick;@(negedge clk);endtask
 task start(input bit remap);begin wait(start_ready);tick();start_remap=remap;start_valid=1;tick();start_valid=0;end endtask
 task response(input integer code);
  begin wait(rsp_valid);tick();repeat(3)begin if(rsp_status!=code||!rsp_valid)$fatal(1,"response hold/status");tick();end
   rsp_ready=1;tick();rsp_ready=0;end
 endtask
 task release_frame;
  begin wait(frame_release_valid);tick();repeat(3)tick();frame_release_ready=1;tick();frame_release_ready=0;end
 endtask
 task calibration(input integer code);
  integer v,j;begin
   start(0);
   // Model a brief erroneous IDLE decode between FSM transitions. It must not
   // reset workers; the old combinational reset fails this targeted check.
   wait(frame_ready);tick();
   force dut.state=0;#1;
   if(!algorithm_rst_n)$fatal(1,"FSM decode glitch reset the workers");
   release dut.state;
   // Restore the state without advancing protocol signals (between clock edges).
   dut.state=2;
   for(v=0;v<`PAR_VIEWS;v=v+1)begin
    wait(frame_ready);tick();frame_base=v*32'h200000;frame_valid=1;tick();frame_valid=0;
    wait(gray_rsp_ready);tick();gray_rsp_valid=1;tick();gray_rsp_valid=0;
    wait(det_process);tick();det_done=0;repeat(3)tick();
    for(j=0;j<`PAR_POINTS;j=j+1)begin det_valid=1;tick();end
    det_valid=0;det_done=1;
    wait(view_rsp_valid);if(view_status!=0||view_id!=v)$fatal(1,"view commit");
    if(v<`PAR_VIEWS-1)release_frame();
   end
   if(ENABLE_CORNER_REVIEW)begin
    wait(review_wait);repeat(50)begin tick();if(cal_cmd_valid||det_process||gray_cmd_valid||!algorithm_rst_n)$fatal(1,"review did not pause calculation / preserve workers");end
    review_continue=1;tick();review_continue=0;
   end
   wait(mb_ready);tick();mb_status=code;mb_valid=1;tick();mb_valid=0;
   release_frame();response(code);
  end
 endtask
 task correct(input integer base,input integer code);
  begin remap_base=base;start(1);
   wait(remap_cmd_valid);if(remap_opcode!=1||src_base!=base||params!=288'h12345678)$fatal(1,"map descriptor/params");
   wait(remap_rsp_ready);tick();remap_status=BUILD_ONLY?code:0;remap_rsp_valid=1;tick();remap_rsp_valid=0;
   if(!BUILD_ONLY)begin
    wait(remap_cmd_valid);if(remap_opcode!=2)$fatal(1,"remap opcode");
    wait(remap_rsp_ready);tick();remap_status=code;remap_rsp_valid=1;tick();remap_rsp_valid=0;
   end
   response(code);if(BUILD_ONLY&&remap_cmd_valid)$fatal(1,"map-only issued pixel remap");
  end
 endtask
 initial begin
  repeat(3)tick();rst_n=1;start(1);response(8'h82);
  if(ENABLE_DISTORT_DEMO)begin
   remap_base=32'hffffff00;start(1);
   wait(remap_cmd_valid);if(remap_opcode!=3||params!=0)$fatal(1,"demo must not seed calibration");
   wait(remap_rsp_ready);tick();remap_rsp_valid=1;tick();remap_rsp_valid=0;response(0);
   remap_base=0;start(1);response(8'h82);
  end
  calibration(4);start(1);response(8'h82);
  calibration(0);repeat(10)tick();if(remap_cmd_valid||params!=288'h12345678)$fatal(1,"auto correction/params lost");
  correct(32'h400000,5);correct(32'h200000,0);correct(0,0);
  calibration(0);correct(0,0);
  if(ENABLE_DISTORT_DEMO)begin
   remap_base=32'hffffff04;start(1);response(0);
   if(params!=0)$fatal(1,"clear did not invalidate parameters");
   remap_base=0;start(1);response(8'h82);
  end
  rst_n=0;tick();rst_n=1;start(1);response(8'h82);
  finished=1;$display("PASS tb_vision_manual staged calibration, retained params, selected remap, failures, reset");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
