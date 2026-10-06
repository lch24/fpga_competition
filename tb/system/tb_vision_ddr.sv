`timescale 1ns/1ps
`include "calib_defs.vh"
// Integration control/data-path test: detector coordinates and final calibration
// numerics are explicit test doubles. Gray DMA, corner_store, metadata protocol,
// mailbox, DDR arbitration, map generation and every output pixel are real RTL.
module tb_vision_ddr;
 reg finished=0;
 reg clk=0,rst_n=0;always #5 clk=~clk;
 reg start_valid=0,frame_valid=0,frame_release_ready=0,rsp_ready=0;
 wire start_ready,frame_ready,frame_release_valid,rsp_valid,busy;
 reg [63:0] square_size_fp64=64'h3ff0000000000000;
 reg [31:0] frame_base=32,frame_stride=133,frame_capacity=6384;
 reg [7:0] frame_status=0;wire [7:0] rsp_status,debug_view;
 wire [31:0] debug_job,result_base,result_stride;wire [15:0] result_width,result_height;
 wire [5:0] debug_phase;wire [287:0] result_params;wire [63:0] result_rms;
 `include "logical_bus.vh"
 vision_ddr_top #(.CANDIDATE_BASE(524288),.WIDTH(64),.HEIGHT(48),.DEPTH(1),.GRAY_BASE(8192),.DST_BASE(16384),.MAP_X_BASE(32768),.MAP_Y_BASE(49152)) dut(.*);
 `include "logical_memory.vh"
 reg fake_done=0,fake_valid=0,fake_grid=1;reg [15:0] fake_total=`PAR_POINTS;
 reg [31:0] fake_x=0,fake_y=0;integer mode=0,cal_fail=0,det_count=0,cal_count=0,releases=0;
 reg fake_pv=0,fake_rv=0;reg [7:0] fake_rs=0;reg [31:0] fake_id;
 reg [287:0] identity={160'd0,32'h41c00000,32'h42000000,32'h42800000,32'h42800000};
 integer i,x,y,p,initial_writes,writes=0,expected,checks=0;
 always @(posedge clk)if(wr_valid&&wr_ready)writes<=writes+1;
 initial begin
  force dut.detector.f_start=0;
  force dut.ddone=fake_done;force dut.dstatus=2'b01;force dut.dv=fake_valid;
  force dut.dx=fake_x;force dut.dy=fake_y;force dut.total=fake_total;force dut.grid_ok=fake_grid;
  forever begin
   wait(dut.process_frame);@(negedge clk);fake_done=0;fake_grid=mode!=1;fake_total=`PAR_POINTS;det_count=det_count+1;
   repeat(7)@(negedge clk);
   for(integer n=0;n<(mode==1?0:mode==2?`PAR_POINTS-1:mode==3?`PAR_POINTS+1:`PAR_POINTS);n=n+1)begin
    fake_x=32'h42000000+n*65536;fake_y=32'h41800000+n*32768;fake_valid=1;
    do @(posedge clk);while(!dut.dr);@(negedge clk);fake_valid=0;
    repeat(n%3)@(negedge clk);
   end
   fake_done=1;wait(!dut.process_frame);
  end
 end
 initial forever begin
  wait(dut.ccv&&dut.ccr);@(negedge clk);cal_count=cal_count+1;
  if(dut.calibration.dbg_view_usable!={`PAR_VIEWS{1'b1}})$fatal(1,"actual corner store not committed");
  fake_id=debug_job;fake_rs=cal_fail?8'd4:8'd0;
  force dut.calibration.pc=0; // Explicitly suspend numerical children for this TB.
  force dut.pv=fake_pv;force dut.rv=fake_rv;force dut.rs=fake_rs;
  force dut.pid=fake_id;force dut.rid=fake_id;force dut.pw=16'd64;force dut.ph=16'd48;
  force dut.pp=identity;force dut.usable=1'b1;
  repeat(8)@(negedge clk);
  if(!cal_fail)begin fake_pv=1;do @(posedge clk);while(!dut.pr);@(negedge clk);fake_pv=0;end
  // Response intentionally later than packet; maps cannot begin before both.
  repeat(9)@(negedge clk);fake_rv=1;do @(posedge clk);while(!dut.rr);@(negedge clk);fake_rv=0;
  wait(!busy);
  release dut.calibration.pc;release dut.pv;release dut.rv;release dut.rs;
  release dut.pid;release dut.rid;release dut.pw;release dut.ph;release dut.pp;release dut.usable;
 end
 initial forever begin
  wait(frame_release_valid);repeat(5)@(negedge clk);frame_release_ready=1;
  @(negedge clk);frame_release_ready=0;releases=releases+1;wait(!frame_release_valid);
 end
 task launch;
  begin @(negedge clk);start_valid=1;do @(posedge clk);while(!start_ready);@(negedge clk);start_valid=0;end
 endtask
 task frame;
  begin wait(frame_ready);@(negedge clk);frame_valid=1;@(negedge clk);frame_valid=0;end
 endtask
 task result(input integer status);
  begin wait(rsp_valid);repeat(8)begin @(negedge clk);if(!rsp_valid||rsp_status!==status||start_ready)$fatal(1,"result %d expected %d phase %d",rsp_status,status,debug_phase);end
   if(violations!=0)$fatal(1,"DDR protocol violation");
   rsp_ready=1;@(negedge clk);rsp_ready=0;repeat(3)@(negedge clk);checks=checks+1;
  end
 endtask
 initial begin
  for(i=0;i<70000;i=i+1)memory.mem[i]=8'ha5;
  for(y=0;y<48;y=y+1)for(x=0;x<64;x=x+1)begin
   p=(x*421+y*733)&65535;memory.mem[32+y*133+x*2]=p&255;memory.mem[33+y*133+x*2]=p>>8;
  end
  repeat(5)@(negedge clk);rst_n=1;
  // Complete success: real grayscale + maps + remap, all output pixels compared.
  launch();repeat(`PAR_VIEWS)frame();result(0);
  if(result_base!=16384||result_stride!=128||releases!=`PAR_VIEWS||cal_count!=1)$fatal(1,"result descriptor/ownership");
  for(y=0;y<48;y=y+1)for(x=0;x<64;x=x+1)begin
   p=(x*421+y*733)&65535;
   if({memory.mem[16385+y*128+2*x],memory.mem[16384+y*128+2*x]}!==p[15:0])$fatal(1,"corrected pixel %d %d",x,y);
  end
  mode=1;launch();frame();result(2);
  mode=2;launch();frame();result(2);
  mode=3;launch();frame();result(2);
  mode=0;cal_fail=1;launch();repeat(`PAR_VIEWS)frame();result(4);cal_fail=0;
  memory.inject_error(32,33);launch();frame();result(5);memory.clear_error_injection();
  memory.inject_error(32768,45055);launch();repeat(`PAR_VIEWS)frame();result(5);memory.clear_error_injection();
  frame_base=8192;initial_writes=writes;launch();frame();result(1);if(writes!=initial_writes)$fatal(1,"overlap wrote DDR");frame_base=32;
  frame_base=524288;initial_writes=writes;launch();frame();result(1);if(writes!=initial_writes)$fatal(1,"candidate workspace overlap wrote DDR");frame_base=32;
  frame_status=3;launch();frame();result(3);frame_status=0;
  square_size_fp64=64'h7ff0000000000000;launch();result(1);square_size_fp64=64'h3ff0000000000000;
  // Retry after every failure; old sticky detector done must not finish new view.
  launch();repeat(`PAR_VIEWS)frame();result(0);
  finished=1;$display("PASS vision DDR: %0d jobs, actual pixels, metadata, last-frame lease, errors and retry; detector/calibration numerics mocked",checks);$finish;
 end
 initial begin #200000000;$fatal(1,"vision timeout phase=%d calpc=%d",debug_phase,dut.calibration.pc);end
endmodule
