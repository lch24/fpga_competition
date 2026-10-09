`timescale 1ns/1ps
module tb_algorithm_clock;
 reg clkin=0;always #4 clkin=~clkin;
 wire clkout,locked;reg finished=0;integer i;real last_edge,period;
 algorithm_clock dut(.*);
 initial begin
  wait(locked);repeat(5)@(posedge clkout);last_edge=$realtime;
  for(i=0;i<100;i=i+1)begin
   @(posedge clkout);period=$realtime-last_edge;last_edge=$realtime;
   if(period<24.95 || period>25.05)$fatal(1,"PLL period %f ns, expected 25",period);
  end
  finished=1;$display("PASS 40MHz PLL, 100 periods checked with vendor model");$finish;
 end
 initial begin #1000000;$fatal(1,"PLL did not lock");end
endmodule
