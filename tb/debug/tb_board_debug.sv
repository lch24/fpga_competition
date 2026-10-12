`timescale 1ns/1ps
module tb_board_debug;
 reg clk=0,core=0,alg=0,run_core=1,run_alg=1,rx=1;
 always #5 clk=~clk;
 always #4 if(run_core)core=~core;
 always #13 if(run_alg)alg=~alg;
 wire tx,rst;wire [15:0] arg;wire [511:0] ad;
 reg [255:0] cd=0;reg [7:0] health=0;
 reg fn=0,start=0,cv=0,pv=0,dv=0,rv=0,camvs=0;integer before_heartbeat;
 reg [31:0] x=0,y=0,job=0;
 reg [287:0] params=288'h123456;
 reg [7:0] status=0;
 reg finished=0;
 algorithm_debug rec(.clk(alg),.rst_n(rst),.functional_reset_n(fn),.start_cal(start),
 .detect_done(dv),.detect_count(16'd40),.detect_flags(8'd5),.job(job),.phase(6'd11),.view(8'd2),.corner_valid(cv),.corner_x(x),.corner_y(y),
 .result_valid(pv),.params(params),.diag_valid(dv),.metrics_valid(1'b1),.accepted_steps(16'd7),.converged(1'b1),
 .cal_phase(4'd2),.diag_status(status),.rms(64'h3ff0000000000000),
 .rsp_valid(rv),.rsp_status(status),.argument(arg),.snapshot(ad),
 .live(72'h98765432123456789a),.rd_accept(1'b0),.rd_return(1'b0),.wr_accept(1'b0),.bus_state(12'h123),.address(32'b0));
 board_debug #(.CLOCK_HZ(1000000),.BAUD(100000)) dut(.clk(clk),.rx(rx),.tx(tx),.debug_rst_n(rst),
 .camera_clk(alg),.camera_vsync(camvs),.core_clk(core),.algorithm_clk(alg),.core_data(cd),.algorithm_data(ad),.algorithm_argument(arg),.health(health));
 reg [7:0] bytes[0:129];reg [7:0] last_packet[0:129];
 integer trace;initial trace=$fopen("uart_packets.hex","w");
 integer packets=0,i,j;reg [15:0] crc;
 function [15:0] crc_byte(input [15:0] c,input [7:0] b);
 reg [15:0] v;integer k;begin v=c^{b,8'b0};for(k=0;k<8;k=k+1)v=v[15]?(v<<1)^16'h1021:v<<1;crc_byte=v;end endfunction
 task receive_byte(output reg [7:0] value);
 integer k;begin
  @(negedge tx);#150;
  for(k=0;k<8;k=k+1)begin value[k]=tx;#100;end
  if(tx!==1)$fatal(1,"bad TX stop");
 end endtask
 initial forever begin
  for(i=0;i<130;i=i+1)receive_byte(bytes[i]);
  if(bytes[0]!=8'ha5||bytes[1]!=8'h5a||bytes[2]!=1||bytes[6]!=120)$fatal(1,"header");
  crc=16'hffff;for(j=2;j<128;j=j+1)crc=crc_byte(crc,bytes[j]);
  if(crc!={bytes[129],bytes[128]})$fatal(1,"CRC");
  for(j=0;j<130;j=j+1)last_packet[j]=bytes[j];
  for(j=0;j<130;j=j+1)$fwrite(trace,"%02x",bytes[j]);$fwrite(trace,"\n");
  packets=packets+1;
 end
 task send_byte(input [7:0] b);
 integer k;begin
  @(negedge clk);rx=0;repeat(10)@(negedge clk);
  for(k=0;k<8;k=k+1)begin rx=b[k];repeat(10)@(negedge clk);end
  rx=1;repeat(10)@(negedge clk);
 end endtask
 task command(input [7:0] op,input [15:0] index);
 begin send_byte(8'ha5);send_byte(8'h5a);send_byte(op);send_byte(index[7:0]);send_byte(index[15:8]);send_byte(op^index[7:0]^index[15:8]);end endtask
 task query(input [7:0] op,input [15:0] index);
 integer old;begin
  wait(!dut.pending&&!dut.sending);repeat(100)@(posedge clk);
  old=packets;command(op,index);wait(packets>old);
  if(last_packet[3]!=op)$fatal(1,"wrong opcode");
 end endtask
 initial begin
  wait(rst);wait(packets==1);
  if(last_packet[14]!=0)$fatal(1,"health before init");
  @(negedge alg);camvs=1;@(negedge alg);camvs=0;
  fn=1;health=8'hff;
  @(negedge alg);start=1;@(negedge alg);start=0;job=1;
  x=32'h3f800000;y=32'h40000000;cv=1;@(negedge alg);cv=0;
  pv=1;dv=1;@(negedge alg);pv=0;dv=0;
  query(8'h53,16'hffff);
  if(last_packet[15]!=1)$fatal(1,"camera frame count");
  if({last_packet[83],last_packet[82],last_packet[81],last_packet[80]}!=32'h123456)$fatal(1,"params");
  if(!last_packet[70][1]||!last_packet[70][2])$fatal(1,"result flags");
  query(8'h43,0);
  if({last_packet[83],last_packet[82],last_packet[81],last_packet[80]}!=x)$fatal(1,"corner X");
  if({last_packet[87],last_packet[86],last_packet[85],last_packet[84]}!=y)$fatal(1,"corner Y");
  query(8'h43,9);if(last_packet[70][4])$fatal(1,"invalid corner accepted");
  query(8'h44,0);
  if(last_packet[80]!=8'h9a||last_packet[84]!=8'h12)$fatal(1,"progress");
  fn=0;query(8'h53,0);
  if(last_packet[70][0]||!last_packet[70][1])$fatal(1,"reset/result retention");
  run_alg=0;repeat(110000)@(posedge clk);query(8'h53,0);
  if(!last_packet[12][3])$fatal(1,"stopped algorithm not stale");
  run_core=0;repeat(110000)@(posedge clk);query(8'h53,0);
  if(!last_packet[12][2])$fatal(1,"stopped DDR not stale");
  run_core=1;run_alg=1;query(8'h53,0);
  if(last_packet[12][3:2]!=0)$fatal(1,"clock recovery");
  send_byte(8'ha5);send_byte(8'h5a);send_byte(8'h53);send_byte(0);send_byte(0);send_byte(0);
  send_byte(8'ha5);send_byte(8'h5a);repeat(25000)@(posedge clk);
  rx=0;repeat(200)@(posedge clk);rx=1;repeat(100)@(posedge clk);
  query(8'h53,0);if(last_packet[24]<3)$fatal(1,"RX errors not counted");
  // Repeated requests exceed the one pending command slot: count, never block.
  repeat(8)command(8'h53,0);
  query(8'h53,0);if(last_packet[28]==0)$fatal(1,"busy query not counted");
  fn=1;@(negedge alg);start=1;@(negedge alg);start=0;
  query(8'h43,0);if(last_packet[70][4])$fatal(1,"old corner visible");
  before_heartbeat=packets;repeat(1100000)@(posedge clk);
  if(packets<=before_heartbeat)$fatal(1,"periodic heartbeat absent");
  $display("PASS tb_board_debug: reset, corners, results, clock stop/recovery, parser faults, CRC");finished=1;$finish;
 end
 initial begin #50000000;$fatal(1,"timeout");end
endmodule
