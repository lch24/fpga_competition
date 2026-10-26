`timescale 1ns/1ps
module tb_corner_review_flow;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0;reg [4:0] pressed=0;reg display_idle=1,review_wait=0,release_valid=0;
 reg remap_ready=0,remap_rsp=0,mark_ready=0,mark_rsp=0,rsp=0;reg [7:0] remap_status=0,mark_status=0,rs=0;
 wire start_valid,start_remap,frame_valid,release_ready,rsp_ready;
 wire display_enable,review_ready,review_continue,mv,mq,rv,rr,bypass,calibrated;wire [31:0] src,dst,display_base;wire [1:0] selected;wire [7:0] status;
 video_board_flow #(.ENABLE_CORNER_REVIEW(1)) dut(.clk(clk),.rst_n(rst_n),.preview_enable(1'b0),.preview_active(),.camera_ready(1'b1),.pressed(pressed),
  .display_idle(display_idle),.display_frame_start(1'b0),.start_valid(start_valid),.start_ready(1'b1),.start_remap(start_remap),.start_base(),
  .frame_ready(1'b1),.frame_valid(frame_valid),.frame_status(),.frame_base(),.frame_release_valid(release_valid),.frame_release_ready(release_ready),
  .cap_cmd_ready(1'b1),.cap_rsp_valid(1'b0),.cap_status(8'b0),.cap_rsp_base(32'b0),
  .rsp_valid(rsp),.rsp_status(rs),.rsp_ready(rsp_ready),.display_enable(display_enable),.display_base(display_base),.status(status),.calibrated(calibrated),.selected(selected),
  .remap_bypass(bypass),.remap_valid(rv),.remap_ready(remap_ready),.remap_src(src),.remap_dst(dst),.remap_rsp_valid(remap_rsp),.remap_rsp_ready(rr),.remap_status(remap_status),
  .review_wait(review_wait),.review_ready(review_ready),.review_continue(review_continue),.mark_valid(mv),.mark_ready(mark_ready),.mark_rsp_valid(mark_rsp),.mark_rsp_ready(mq),.mark_status(mark_status));
 task tick;@(negedge clk);endtask
 task press(input integer k);begin tick();pressed=1<<k;tick();pressed=0;end endtask
 task render(input integer v,copy_error,paint_error);
  reg [63:0] held;
  begin
   wait(rv);#1;if(src!=v*32'h200000||dst!=32'h1000000||!bypass||display_enable)$fatal(1,"copy target/identity or early display");
   held={src,dst};repeat(4)begin tick();if(!rv||{src,dst}!==held)$fatal(1,"copy command stall");end
   remap_ready=1;tick();remap_ready=0;wait(rr);tick();remap_status=copy_error;remap_rsp=1;tick();remap_rsp=0;
   if(copy_error==0)begin
    wait(mv);if(selected!=v||display_enable)$fatal(1,"marker view/early display");
    repeat(4)tick();mark_ready=1;tick();mark_ready=0;wait(mq);tick();mark_status=paint_error;mark_rsp=1;tick();mark_rsp=0;
   end
   wait(dut.state==15);#1;
   if(copy_error==0&&paint_error==0)begin
    if(!review_ready||!display_enable||display_base!=32'h1000000||calibrated)$fatal(1,"review display/ready/early params");
   end else if(display_enable||review_ready||status==0)$fatal(1,"published incomplete annotated frame");
  end
 endtask
 initial begin
  repeat(4)tick();rst_n=1;tick();dut.captured=7; // existing immutable photo fixtures
  press(3);
  // First two leases release after detection. The third remains held until
  // AFTER review and calculation, matching vision_sequence on the board.
  repeat(2)begin wait(dut.state==6);tick();release_valid=1;tick();release_valid=0;end
  wait(dut.state==6&&dut.view_index==2);tick();review_wait=1;render(0,0,0);
  repeat(100)tick();if(!review_ready||start_valid||review_continue)$fatal(1,"review timed out or auto-started calculation");
  display_idle=0;press(1);repeat(7)tick();if(rv||display_enable)$fatal(1,"did not drain display before overwrite");
  display_idle=1;render(1,0,0);
  press(2);render(2,5,0);press(3);repeat(5)tick();if(review_continue)$fatal(1,"continued with failed review frame");
  press(2);render(2,0,5);press(0);render(0,0,0);
  display_idle=0;press(3);repeat(5)tick();if(review_continue||display_enable)$fatal(1,"resume did not drain display");
  display_idle=1;wait(review_continue);repeat(8)tick();if(start_valid||rv||mv)$fatal(1,"resume redetected or copied frames");
  review_wait=0;wait(release_ready);tick();release_valid=1;tick();release_valid=0;
  wait(rsp_ready);tick();rs=0;rsp=1;tick();rsp=0;tick();
  if(!calibrated||status!=0)$fatal(1,"calibration response not accepted after resume");
  press(4);wait(start_valid);if(!start_remap)$fatal(1,"KEY4 no longer builds map");
  $display("PASS two-KEY3 UI/photo selection/stable commands/persistent review/drain/error gating/continue/KEY4");$finish;
 end
 initial begin #1000000;$fatal(1,"review flow timeout state=%d",dut.state);end
endmodule
