`timescale 1ns/1ps
module tb_demo_flow;
 reg finished=0;
 reg toggle_mode=0;wire distort_mode,mode_pending,demo_map_ready;wire [31:0] remap_map;wire [255:0] demo_snapshot;
 reg clk=0;always #4 clk=~clk;reg rst=0;reg [4:0] key=0;
 reg display_idle=1,dfs=0,cap_ready=1,cap_rsp=0;reg [7:0] cap_status=0;reg [31:0] cap_return=0;
 reg start_ready=1,fr=1,release_v=0,alg_rsp=0;reg [7:0] alg_status=0;
 reg remap_ready=0,remap_rsp=0;reg [7:0] remap_status=0;
 wire remap_bypass;wire cap_v,cap_rr,sv,sr,fv,frr,rr,display_enable,video,map_ready,remap_v,remap_rr;
 wire [31:0] cap_base,start_base,frame_base,display_base,src,dst,captured_count,corrected_count,displayed_count,failed_count;
 wire [7:0] status;wire [3:0] phase;wire [2:0] captured;wire calibrated,corrected;
 distort_demo_flow dut(.preview_enable(1'b0),.toggle_mode(toggle_mode),.remap_cycles(32'd100),.distort_mode(distort_mode),.mode_pending(mode_pending),.demo_map_ready(demo_map_ready),.remap_map(remap_map),.demo_snapshot(demo_snapshot),.clk(clk),.rst_n(rst),.camera_ready(1'b1),.pressed(key),.display_idle(display_idle),.display_frame_start(dfs),
  .start_valid(sv),.start_ready(start_ready),.start_remap(sr),.start_base(start_base),
  .frame_ready(fr),.frame_valid(fv),.frame_status(),.frame_base(frame_base),.frame_release_valid(release_v),.frame_release_ready(frr),
  .cap_cmd_valid(cap_v),.cap_cmd_ready(cap_ready),.cap_base(cap_base),.cap_rsp_valid(cap_rsp),.cap_rsp_ready(cap_rr),.cap_status(cap_status),.cap_rsp_base(cap_return),
  .rsp_valid(alg_rsp),.rsp_status(alg_status),.rsp_ready(rr),.capture_owner(),.display_enable(display_enable),.display_owner(),.display_base(display_base),
  .status(status),.waiting(),.phase(phase),.captured(captured),.calibrated(calibrated),.corrected(corrected),.selected(),.busy(),
  .video_active(video),.map_ready(map_ready),.remap_bypass(remap_bypass),.remap_valid(remap_v),.remap_ready(remap_ready),.remap_src(src),.remap_dst(dst),
  .remap_rsp_valid(remap_rsp),.remap_rsp_ready(remap_rr),.remap_status(remap_status),
  .captured_frames(captured_count),.corrected_frames(corrected_count),.displayed_frames(displayed_count),.failed_frames(failed_count));
 reg [31:0] queue[0:63];integer head=0,tail=0;
 always @(posedge clk)if(cap_v&&cap_ready)begin queue[tail]=cap_base;tail=tail+1;end
 task tick;begin @(negedge clk);end endtask
 task press(input integer k);begin tick;key=1<<k;tick;key=0;end endtask
 task capture_done(input [7:0] s);
  begin wait(head<tail);tick;cap_return=queue[head];head=head+1;cap_status=s;cap_rsp=1;do @(posedge clk);while(!cap_rr);tick;cap_rsp=0;end
 endtask
 task remap_done(input [7:0] s);
  begin tick;remap_status=s;remap_rsp=1;do @(posedge clk);while(!remap_rr);tick;remap_rsp=0;end
 endtask
 // Telemetry counters commit one clock after the display ownership event.
 task flip;begin tick;dfs=1;tick;dfs=0;tick;end endtask
 task accept_remap;begin tick;remap_ready=1;do @(posedge clk);while(!remap_v);tick;remap_ready=0;end endtask
 reg [31:0] held_src,held_dst;integer k,builds=0;
 task algorithm_done;
  begin wait(rr);tick;alg_rsp=1;tick;alg_rsp=0;end
 endtask
 task toggle;begin tick;toggle_mode=1;tick;toggle_mode=0;end endtask
 task change_to_demo;
  begin
   toggle;wait(sv);if(start_base!=32'hffffff04)$fatal(1,"missing invalidation");@(posedge clk);tick;algorithm_done;
   wait(sv);if(start_base!=32'hffffff00)$fatal(1,"missing artificial build");@(posedge clk);tick;algorithm_done;
   wait(!mode_pending);if(!distort_mode||!demo_map_ready)$fatal(1,"demo not ready");
  end
 endtask
 task photo(input integer slot,input integer code);
  begin
   press(slot);wait(cap_v);if(cap_base!=32'h01400000)$fatal(1,"photo scratch address");
   capture_done(0);wait(remap_v);
   if(src!=32'h01400000||dst!=slot*32'h00200000||remap_map!=32'h03800000||captured[slot])$fatal(1,"photo publication/address");
   held_src=src;held_dst=dst;
   repeat(5)begin tick;if(src!=held_src||dst!=held_dst||!remap_v)$fatal(1,"stalled photo");end
   accept_remap;remap_done(code);wait(phase==0);tick;
   if(captured[slot]!=(code==0))$fatal(1,"photo validity");
  end
 endtask
 initial begin
  repeat(4)tick;rst=1;change_to_demo;
  photo(0,5);photo(0,0);photo(1,0);photo(2,0);
  if(!display_enable||display_base!=32'h00400000)$fatal(1,"photo display");
  press(0);wait(phase==0);tick;if(display_base!=0)$fatal(1,"select existing photo");
  press(3);wait(sv);if(sr)$fatal(1,"calibration replaced by build");@(posedge clk);tick;
  for(k=0;k<3;k=k+1)begin wait(fv);if(frame_base!=k*32'h00200000)$fatal(1,"wrong detection source");@(posedge clk);tick;release_v=1;tick;release_v=0;end
  algorithm_done;wait(phase==0);if(!calibrated)$fatal(1,"calibration");
  press(4);wait(sv);if(start_base>=32'hffffff00)$fatal(1,"correction used demo command");@(posedge clk);tick;algorithm_done;wait(video);
  wait(tail-head==3);capture_done(0);wait(remap_v);
  if(remap_map!=32'h03800000||dst!=32'h01600000)$fatal(1,"first pass");
  accept_remap;remap_done(0);wait(remap_v);
  if(remap_map!=32'h02000000||src!=32'h01600000||corrected_count!=0)$fatal(1,"second pass/early publish");
  accept_remap;remap_done(0);wait(corrected_count==1);flip;
  if(displayed_count!=1||demo_snapshot[191:160]!=200)$fatal(1,"publication/cycle accounting");
  // Before view uses artificial map directly into the leased output slot.
  press(0);capture_done(0);wait(remap_v);
  if(remap_bypass||remap_map!=32'h03800000||dst==32'h01600000||dst==display_base)$fatal(1,"before view must be distorted, output slot owned");
  held_dst=dst;press(0); // Toggle back during a stalled before-view command.
  repeat(4)begin tick;if(dst!=held_dst||remap_map!=32'h03800000)$fatal(1,"toggle retargeted command");end
  accept_remap;remap_done(0);wait(display_base==held_dst);flip;
  if(corrected_count!=1||displayed_count!=2||demo_snapshot[191:160]!=100)$fatal(1,"before view incorrectly corrected/counted");
  if(captured!=7||!calibrated||!map_ready)$fatal(1,"comparison invalidated calibration");
  // Stop while another artificial command is stalled; drain accepted work.
  capture_done(0);wait(remap_v);held_src=src;held_dst=dst;display_idle=0;toggle;
  repeat(6)begin tick;if(src!=held_src||dst!=held_dst||!remap_v)$fatal(1,"switch aborted request");end
  accept_remap;remap_done(0);wait(remap_v);accept_remap;remap_done(0);
  while(head<tail)capture_done(0);
  repeat(5)tick;if(!mode_pending||sv)$fatal(1,"ignored display drain");display_idle=1;
  wait(sv);if(start_base!=32'hffffff04)$fatal(1,"missing clear");@(posedge clk);tick;algorithm_done;
  wait(!mode_pending);tick;if(distort_mode||captured||calibrated||map_ready)$fatal(1,"mode switch leaked old data");
  // Normal input remains direct, with no artificial image processing.
  press(1);wait(cap_v);if(cap_base!=32'h00200000)$fatal(1,"normal capture address");capture_done(0);wait(phase==0);
  if(captured!=2||remap_v)$fatal(1,"normal path");
  // A pending camera command is not cancelled or retargeted during switching.
  cap_ready=0;press(2);wait(cap_v);held_dst=cap_base;toggle;
  repeat(6)begin tick;if(!cap_v||cap_base!=held_dst)$fatal(1,"switch changed stalled capture");end
  cap_ready=1;capture_done(0);wait(sv);@(posedge clk);tick;algorithm_done;
  wait(!mode_pending);tick;if(!distort_mode||captured||!demo_map_ready)$fatal(1,"cached map switch");
  // Failed map generation leaves normal mode active and can be retried.
  rst=0;repeat(3)tick;rst=1;toggle;wait(sv);@(posedge clk);tick;algorithm_done;
  wait(sv);@(posedge clk);tick;alg_status=5;algorithm_done;wait(!mode_pending);tick;
  if(distort_mode||demo_map_ready||status!=5)$fatal(1,"failed demo map became usable");
  alg_status=0;change_to_demo;
  finished=1;$display("PASS demo_flow photo failures/retry, image publication, independent calibration, two-pass video, switch drain, invalidation and normal path");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout phase=%d mode=%d capture=%d remap=%d",phase,dut.mode_state,dut.capture_state,dut.remap_state);end
endmodule
