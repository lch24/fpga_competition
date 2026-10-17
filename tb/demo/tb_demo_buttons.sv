`timescale 1ns/1ps
module tb_demo_buttons;
 reg finished=0;
 reg clk=0,rst_n=0;always #4 clk=~clk;
 reg [4:0] key_n=31;wire [4:0] pressed;wire toggle_mode;
 integer shorts=0,toggles=0;
 board_buttons #(.DEBOUNCE_CYCLES(3),.ENABLE_DISTORT_DEMO(1),.LONG_PRESS_CYCLES(20)) dut(.*);
 always @(posedge clk)begin if(pressed[3])shorts=shorts+1;if(toggle_mode)toggles=toggles+1;end
 task tick(input integer n);repeat(n)@(negedge clk);endtask
 initial begin
  tick(3);rst_n=1;tick(5);
  key_n[3]=0;tick(1);key_n[3]=1;tick(8);if(shorts||toggles)$fatal(1,"bounce");
  key_n[3]=0;tick(12);if(shorts)$fatal(1,"short before release");key_n[3]=1;tick(10);
  if(shorts!=1||toggles!=0)$fatal(1,"short press");
  key_n[3]=0;tick(100);if(toggles!=1||shorts!=1)$fatal(1,"long repeat");key_n[3]=1;tick(10);
  if(shorts!=1)$fatal(1,"long release triggered calibration");
  key_n[3]=0;tick(30);key_n[3]=1;tick(10);if(toggles!=2)$fatal(1,"rearm");
  finished=1;$display("PASS demo_buttons debounce, short-on-release, long-once, no release calibration");$finish;
 end
 initial begin #10000;$fatal(1,"timeout");end
endmodule
