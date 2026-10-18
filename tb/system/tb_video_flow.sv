`timescale 1ns/1ps
module tb_video_flow;
 reg finished=0;
 reg clk=0;always #4 clk=~clk;reg rst=0;reg [4:0] key=0;
 reg display_idle=1,dfs=0,cap_ready=1,cap_rsp=0;reg [7:0] cap_status=0;reg [31:0] cap_return=0;
 reg start_ready=1,fr=1,release_v=0,alg_rsp=0;reg [7:0] alg_status=0;
 reg remap_ready=0,remap_rsp=0;reg [7:0] remap_status=0;
 wire remap_bypass;wire cap_v,cap_rr,sv,sr,fv,frr,rr,display_enable,video,map_ready,remap_v,remap_rr;
 wire [31:0] cap_base,start_base,frame_base,display_base,src,dst,captured_count,corrected_count,displayed_count,failed_count;
 wire [7:0] status;wire [3:0] phase;wire [2:0] captured;wire calibrated,corrected;
 video_board_flow dut(.preview_enable(1'b0),.clk(clk),.rst_n(rst),.camera_ready(1'b1),.pressed(key),.display_idle(display_idle),.display_frame_start(dfs),
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
 initial begin
  repeat(4)tick;rst=1;
  for(k=0;k<3;k=k+1)begin press(k);capture_done(0);wait(phase==0);end
  if(captured!=7)$fatal(1,"photos missing");
  press(3);wait(sv);@(posedge clk);tick;
  for(k=0;k<3;k=k+1)begin wait(fv);@(posedge clk);tick;release_v=1;tick;release_v=0;end
  wait(rr);tick;alg_rsp=1;tick;alg_rsp=0;wait(phase==0);if(!calibrated)$fatal(1,"calibration result");
  press(4);wait(sv);if(!sr)$fatal(1,"map-only command");builds=builds+1;@(posedge clk);tick;
  wait(rr);alg_rsp=1;tick;alg_rsp=0;wait(video);if(!map_ready)$fatal(1,"map not ready");
  wait(tail-head==3);capture_done(0);wait(remap_v);held_src=src;held_dst=dst;
  repeat(12)begin tick;if(src!=held_src||dst!=held_dst||!remap_v)$fatal(1,"remap payload changed under stall");end
  accept_remap;remap_done(0);
  if(!display_enable||displayed_count!=0)$fatal(1,"publication/early flip");flip;if(displayed_count!=1)$fatal(1,"first display commit");
  capture_done(0);wait(remap_v);if(dst==display_base)$fatal(1,"overwriting display");
  accept_remap;remap_done(0);flip;if(corrected_count!=2||displayed_count!=2)$fatal(1,"video counters");
  // Toggle while a corrected descriptor is stalled: command must remain immutable.
  capture_done(0);wait(remap_v);held_src=src;held_dst=dst;
  press(0);if(remap_bypass||src!=held_src||dst!=held_dst)$fatal(1,"toggle changed pending descriptor");
  accept_remap;remap_done(0);flip;
  capture_done(0);wait(remap_v);if(!remap_bypass||dst==display_base)$fatal(1,"before view descriptor/ownership");
  press(0);repeat(3)tick;if(!remap_bypass)$fatal(1,"pending before view changed");
  accept_remap;remap_done(0);flip;
  if(corrected_count!=3||displayed_count!=4||captured!=7||!calibrated||!map_ready)$fatal(1,"preview changed calibration/counters");
  capture_done(0);wait(remap_v);if(remap_bypass)$fatal(1,"return to correction");held_src=src;held_dst=dst;display_idle=0;
  press(4);if(video||display_enable)$fatal(1,"stop not disabling stream");
  repeat(8)begin tick;if(src!=held_src||dst!=held_dst||!remap_v)$fatal(1,"stop aborted pending request");end
  accept_remap;remap_done(0);while(head<tail)capture_done(0);
  repeat(5)tick;if(phase!=9)$fatal(1,"stop ignored display drain");display_idle=1;wait(phase==0);if(!map_ready)$fatal(1,"stop lost map");
  press(4);wait(video);if(sv||builds!=1)$fatal(1,"unnecessary rebuild");capture_done(3);tick;if(failed_count!=1)$fatal(1,"capture error not recorded");
  finished=1;$display("PASS video_flow photo leases, map-only start, buffer ownership, stalled commands, stop drain and restart");$finish;
 end
 initial begin #1000000;$fatal(1,"flow timeout phase=%d used=%b ready=%b pending=%b running=%b head=%d tail=%d",phase,dut.raw_used,dut.raw_ready,dut.remap_pending,dut.remap_running,head,tail);end
endmodule
