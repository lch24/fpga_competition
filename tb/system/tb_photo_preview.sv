`timescale 1ns/1ps
module tb_photo_preview;
 wire display_corrected;
 reg finished=0;reg camera_ready=0;
 reg toggle_mode=0;wire distort_mode,mode_pending,demo_map_ready;wire [31:0] remap_map;wire [255:0] demo_snapshot;
 reg clk=0;always #4 clk=~clk;reg rst=0;reg [4:0] key=0;
 reg display_idle=1,dfs=0,cap_ready=1,cap_rsp=0;reg [7:0] cap_status=0;reg [31:0] cap_return=0;
 reg start_ready=1,fr=1,release_v=0,alg_rsp=0;reg [7:0] alg_status=0;
 reg remap_ready=0,remap_rsp=0;reg [7:0] remap_status=0;
 wire remap_bypass;wire cap_v,cap_rr,sv,sr,fv,frr,rr,display_enable,video,map_ready,remap_v,remap_rr;
 wire [31:0] cap_base,start_base,frame_base,display_base,src,dst,captured_count,corrected_count,displayed_count,failed_count;
 wire [7:0] status;wire [3:0] phase;wire [2:0] captured;wire calibrated,corrected;
 distort_demo_flow #(.REVIEW_CYCLES(32)) dut(.preview_enable(1'b1),.toggle_mode(toggle_mode),.remap_cycles(32'd100),.distort_mode(distort_mode),.mode_pending(mode_pending),.demo_map_ready(demo_map_ready),.remap_map(remap_map),.demo_snapshot(demo_snapshot),.clk(clk),.rst_n(rst),.camera_ready(camera_ready),.pressed(key),.display_idle(display_idle),.display_frame_start(dfs),
  .start_valid(sv),.start_ready(start_ready),.start_remap(sr),.start_base(start_base),
  .frame_ready(fr),.frame_valid(fv),.frame_status(),.frame_base(frame_base),.frame_release_valid(release_v),.frame_release_ready(frr),
  .cap_cmd_valid(cap_v),.cap_cmd_ready(cap_ready),.cap_base(cap_base),.cap_rsp_valid(cap_rsp),.cap_rsp_ready(cap_rr),.cap_status(cap_status),.cap_rsp_base(cap_return),
  .rsp_valid(alg_rsp),.rsp_status(alg_status),.rsp_ready(rr),.capture_owner(),.display_enable(display_enable),.display_owner(),.display_base(display_base),
  .status(status),.waiting(),.phase(phase),.captured(captured),.calibrated(calibrated),.corrected(corrected),.selected(),.busy(),
  .video_active(video),.map_ready(map_ready),.display_corrected(display_corrected),.remap_bypass(remap_bypass),.remap_valid(remap_v),.remap_ready(remap_ready),.remap_src(src),.remap_dst(dst),
  .remap_rsp_valid(remap_rsp),.remap_rsp_ready(remap_rr),.remap_status(remap_status),
  .captured_frames(captured_count),.corrected_frames(corrected_count),.displayed_frames(displayed_count),.failed_frames(failed_count));
 reg [31:0] queue[0:255];integer head=0,tail=0;
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
 task drain_preview;
  begin
   // Pending requests must finish despite stop; no new requests may be allocated.
   while(dut.normal_flow.state==9)begin
    if(remap_v)begin accept_remap;remap_done(0);end
    else if(head<tail)capture_done(0);
    else tick;
   end
  end
 endtask
 task preview_frame(input integer artificial);
  begin
   wait(phase==12);
   if(video||calibrated||captured!=0)$fatal(1,"preview incorrectly changes calibration state");
   capture_done(0);wait(remap_v);
   if(src<32'h00800000||src>=32'h00e00000||dst<32'h01000000||dst>=32'h01400000)$fatal(1,"preview buffer partition");
   if(artificial)begin
    if(remap_bypass||remap_map!=32'h03800000)$fatal(1,"demo preview must show artificial distortion");
   end else if(!remap_bypass)$fatal(1,"normal preview must not require calibration/map");
   held_src=src;held_dst=dst;
   repeat(5)begin tick;if(!remap_v||src!=held_src||dst!=held_dst)$fatal(1,"preview stall stability");end
   accept_remap;remap_done(0);wait(display_enable);flip;
   if(corrected_count!=0||corrected||map_ready||display_corrected)$fatal(1,"preview marked corrected");
  end
 endtask
 task hold_review(input integer slot);
  begin
   wait(phase==0);tick;
   if(!display_enable||display_base!=slot*32'h00200000)$fatal(1,"wrong review photo");
   // Timer must not expire before the display starts fetching the photo.
   repeat(40)begin tick;if(phase!=0||!display_enable)$fatal(1,"review timer started before visible frame");end
   flip;
   repeat(28)begin tick;if(phase!=0||!display_enable)$fatal(1,"review shorter than configured interval");end
   wait(phase==12);
  end
 endtask
 initial begin
  repeat(4)tick;rst=1;repeat(6)tick;
  if(cap_v||display_enable)$fatal(1,"capture before camera ready");
  camera_ready=1;preview_frame(0);
  // Photograph 1: stop preview and drain capture descriptors before using photo DDR.
  press(1);drain_preview;wait(cap_v);
  if(cap_base!=32'h00200000)$fatal(1,"normal photo address");
  capture_done(0);hold_review(1);
  if(captured!=2)$fatal(1,"photo mask");
  // Existing photo: no photo capture/write, just two-second review.
  press(1);drain_preview;wait(phase==0);
  if(head!=tail||cap_v)$fatal(1,"review recaptured a photo");
  hold_review(1);
  // Toggle mode during an accepted, stalled preview remap.
  capture_done(0);wait(remap_v);held_src=src;held_dst=dst;
  toggle;repeat(6)begin tick;if(!remap_v||src!=held_src||dst!=held_dst)$fatal(1,"mode switch aborted preview command");end
  drain_preview;
  wait(sv);if(start_base!=32'hffffff04)$fatal(1,"missing mode clear");@(posedge clk);tick;algorithm_done;
  wait(sv);if(start_base!=32'hffffff00)$fatal(1,"missing demo map build");@(posedge clk);tick;
  repeat(10)begin tick;if(cap_v)$fatal(1,"preview running before map ready");end
  algorithm_done;wait(!mode_pending);preview_frame(1);
  press(2);drain_preview;wait(cap_v);
  if(cap_base!=32'h01400000)$fatal(1,"demo photo scratch");
  capture_done(0);wait(remap_v);
  if(remap_map!=32'h03800000||dst!=32'h00400000||remap_bypass)$fatal(1,"photo/preview mapping mismatch");
  accept_remap;remap_done(0);hold_review(2);
  if(captured!=4)$fatal(1,"demo photo mask");
  // Not enough photos: reject calibration and resume preview, do not start algorithm.
  press(3);drain_preview;wait(phase==12);
  if(status!=8'h81||sv)$fatal(1,"incomplete calibration handling");
  // Capture failure never marks the photo valid, and preview resumes automatically.
  press(0);drain_preview;wait(cap_v);capture_done(5);wait(phase==12);
  if(captured!=4||status!=5)$fatal(1,"failed capture leaked validity");
  // Fill the remaining slots, then start calibration and map from live preview.
  for(k=0;k<2;k=k+1)begin
   press(k);drain_preview;wait(cap_v);capture_done(0);wait(remap_v);
   accept_remap;remap_done(0);hold_review(k);
  end
  if(captured!=7)$fatal(1,"missing final photos");
  press(3);drain_preview;wait(sv);if(sr)$fatal(1,"preview calibration opcode");@(posedge clk);tick;
  for(k=0;k<3;k=k+1)begin wait(fv);if(frame_base!=k*32'h00200000)$fatal(1,"calibration photo lease");@(posedge clk);tick;release_v=1;tick;release_v=0;end
  algorithm_done;wait(phase==12);if(!calibrated)$fatal(1,"calibration did not return to preview");
  press(4);drain_preview;wait(sv);if(!sr)$fatal(1,"preview map opcode");@(posedge clk);tick;algorithm_done;
  wait(video);capture_done(0);wait(remap_v);
  if(dst!=32'h01600000||remap_map!=32'h03800000)$fatal(1,"video first pass after preview");
  accept_remap;remap_done(0);wait(remap_v);
  if(remap_map!=32'h02000000)$fatal(1,"video fitted map");
  accept_remap;remap_done(0);flip;
  if(!display_corrected||!corrected||corrected_count!=1)$fatal(1,"corrected display metadata");
  finished=1;$display("PASS photo_preview camera-ready, normal/demo preview, stable stalled requests, photo/review timer, mode drain and capture failure");$finish;
 end
 initial begin #1000000;$display("DEBUG phase=%d mode=%d cap=%d remap=%d head=%d tail=%d used=%b",phase,dut.mode_state,dut.capture_state,dut.remap_state,head,tail,dut.normal_flow.raw_used);$fatal(1,"preview timeout phase=%d mode=%d capture=%d remap=%d head=%d tail=%d",phase,dut.mode_state,dut.capture_state,dut.remap_state,head,tail);end
endmodule
