`timescale 1ns/1ps
// Independent command-sequence oracle, random service latency/CE and reset.
module tb_detection_program;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,ce=1,start=0;
 wire busy,done,service_valid,service_enter;wire [1:0] status;
 wire [7:0] service_id,debug_pc;wire [15:0] service_arg;
 reg service_done=0;reg [31:0] service_result=0;
 detection_program #(.USE_CE(1)) dut(.*);
 reg [55:0] commands[0:4095];reg [33:0] cases_data[0:31];
 integer cases_count,commands_count,checked=0,ticks=0,pos=0,ending,t,k,delay_left=-1;
 integer fd,watchdog=0,cancelled=0;reg active=0,finished=0;
 reg [7:0] held_id;reg [15:0] held_arg;
 always @(negedge clk)begin ticks=ticks+1;ce=ticks%7!=0 && ticks%11!=0;end
 always @(posedge clk)if(rst_n && ce && active)begin
  if(service_enter)begin
   if(delay_left!=-1)$fatal(1,"duplicate entry");
   if(pos>=ending || {service_id,service_arg}!==commands[pos][55:32])$fatal(1,"command %0d got %h:%h expected %h",pos,service_id,service_arg,commands[pos]);
   held_id=service_id;held_arg=service_arg;delay_left=(pos%9)+1;
   service_done<=0;service_result<=commands[pos][31:0];
  end else if(service_valid)begin
   if({held_id,held_arg}!=={service_id,service_arg})$fatal(1,"unstable request");
   if(service_done)begin pos=pos+1;delay_left=-1;end
   else if(delay_left==0)service_done<=1;
   else delay_left=delay_left-1;
  end
 end
 task launch;
 begin
  @(negedge clk);#1;start=1;
  @(posedge clk);while(!ce)@(posedge clk);
  @(negedge clk);#1;start=0;
 end endtask
 initial begin
  fd=$fopen("counts.txt","r");k=$fscanf(fd,"%d %d",cases_count,commands_count);$fclose(fd);
  $readmemh("commands.hex",commands,0,commands_count-1);$readmemh("cases.hex",cases_data,0,cases_count-1);
  repeat(4)@(negedge clk);#1;rst_n=1;
  // Cancel while waiting, with an old asserted completion present at restart.
  launch();wait(service_valid);repeat(4)@(negedge clk);#1;rst_n=0;
  repeat(3)@(negedge clk);#1;
  if(busy || service_valid || done)$fatal(1,"reset did not cancel");
  cancelled=1;rst_n=1;service_done=1;
  for(t=0;t<cases_count;t=t+1)begin
   ending=cases_data[t][33:2];active=1;launch();watchdog=0;
   while(!done)begin @(negedge clk);#1;watchdog=watchdog+1;if(watchdog>150000)$fatal(1,"timeout");end
   if(pos!=ending || status!==cases_data[t][1:0])$fatal(1,"case %0d result %0d pos %0d expected %h",t,status,pos,cases_data[t]);
   active=0;checked=checked+1;repeat(3)@(negedge clk);
  end
  finished=1;$finish;
 end
endmodule
