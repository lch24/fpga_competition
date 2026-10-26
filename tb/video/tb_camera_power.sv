`timescale 1ns/1ps
// Warm reconfiguration must restart every delay, even with the clock stopped
// when reset asserts. Small cycle parameters keep the test independent of MHz.
module tb_camera_power;
 reg clk=0,run_clk=1,rst=1;
 always #5 if(run_clk)clk=~clk;
 wire r1,r2,pwdn,enable;
 board_power_on_delay #(.POWER_WAIT_CYCLES(8),.RESET_WAIT_CYCLES(3),.CONFIG_WAIT_CYCLES(11)) dut(
  .clk_50M(clk),.reset_n(rst),.camera1_rstn(r1),.camera2_rstn(r2),
  .camera_pwnd(pwdn),.initial_en(enable));
 task tick;begin @(posedge clk);#1;end endtask
 task reset_without_clock;
  begin
   @(negedge clk);run_clk=0;#2;rst=0;#1;
   if({r1,r2,pwdn,enable}!==4'b0010)$fatal(1,"asynchronous sensor reset failed");
   #10;rst=1;#10;
   if({r1,r2,pwdn,enable}!==4'b0010)$fatal(1,"startup advanced without clock");
   run_clk=1;
  end
 endtask
 task boot;
  integer i;
  begin
   for(i=0;i<8;i=i+1)begin tick();if(!pwdn||r1||enable)$fatal(1,"power delay shortened");end
   tick();if(pwdn||r1||enable)$fatal(1,"power stage transition");
   for(i=0;i<3;i=i+1)begin tick();if(r1||enable)$fatal(1,"RESETB delay shortened");end
   tick();if(!r1||!r2||enable)$fatal(1,"RESETB release");
   for(i=0;i<11;i=i+1)begin tick();if(enable)$fatal(1,"SCCB settling shortened");end
   tick();if(!enable)$fatal(1,"SCCB enable missing");
   repeat(5)begin tick();if({r1,r2,pwdn,enable}!==4'b1101)$fatal(1,"startup outputs unstable");end
  end
 endtask
 initial begin
  reset_without_clock();boot();
  reset_without_clock();boot();
  reset_without_clock();repeat(4)tick(); // Interrupted before power settling.
  reset_without_clock();boot();
  $display("PASS camera power: cold/warm/interrupted starts, full delays, asynchronous reset");$finish;
 end
 initial begin #10000;$fatal(1,"camera power timeout");end
endmodule
