`timescale 1ns/1ps
module tb_board_buttons;
 reg clk=0,rst_n=0,finished=0;reg [4:0] key_n=31;wire [4:0] pressed;
 integer counts[0:4];integer i;
 always #4 clk=~clk;
 board_buttons #(.DEBOUNCE_CYCLES(4)) dut(.*);
 always @(posedge clk)for(integer j=0;j<5;j=j+1)if(pressed[j])counts[j]=counts[j]+1;
 task tick(input integer n);repeat(n)@(negedge clk);endtask
 initial begin
  for(i=0;i<5;i=i+1)counts[i]=0;
  tick(3);rst_n=1;tick(3);
  key_n[0]=0;tick(1);key_n[0]=1;tick(2);key_n[0]=0;tick(2);key_n[0]=1;tick(8);
  if(counts[0])$fatal(1,"bounce triggered");
  for(i=0;i<5;i=i+1)begin key_n[i]=0;tick(30);key_n[i]=1;tick(10);end
  for(i=0;i<5;i=i+1)if(counts[i]!=1)$fatal(1,"held key repeats");
  key_n[0]=0;tick(10);key_n[0]=1;tick(1);key_n[0]=0;tick(10);
  if(counts[0]!=2)$fatal(1,"release bounce rearmed");
  rst_n=0;tick(2);key_n=31;rst_n=1;tick(10);
  finished=1;$display("PASS tb_board_buttons bounce, holds, all keys, reset");$finish;
 end
 initial begin #10000;$fatal(1,"timeout");end
endmodule
