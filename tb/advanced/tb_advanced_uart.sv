`timescale 1ns/1ps
module tb_advanced_uart;
 reg clk=0,core=0,alg=0,rx=1;always #5 clk=~clk;always #7 core=~core;always #13 alg=~alg;
 wire tx,rst,wv,busy,cv;wire [7:0] wo;wire [15:0] wd,arg,carg;wire [31:0] errors;wire [207:0] cp;
 wire [255:0] cd={32'ha1020003,32'b0,32'd456,32'd123,32'd600,32'd1,64'b0};
 wire [511:0] ad={arg,496'b0};
 reg cq=0;integer writes=0,packets=0,i,j;reg [7:0] bytes[0:129],last_packet[0:129];reg [15:0] crc;
 board_debug #(.CLOCK_HZ(1000000),.BAUD(100000),.ENABLE_ADVANCED(1)) dut(.clk(clk),.rx(rx),.tx(tx),.debug_rst_n(rst),
  .camera_clk(alg),.camera_vsync(1'b0),.core_clk(core),.algorithm_clk(alg),.core_data(cd),.algorithm_data(ad),
  .algorithm_argument(arg),.core_argument(carg),.health(8'hff),.advanced_write_valid(wv),.advanced_write_op(wo),.advanced_write_data(wd),.advanced_errors(errors));
 advanced_config_bridge mailbox(.clk(clk),.rst_n(rst),.write_valid(wv),.write_op(wo),.write_data(wd),.busy(busy),.errors(errors),
  .core_clk(core),.core_rst_n(rst),.cfg_valid(cv),.cfg_ready(cq),.cfg_payload(cp));
 always @(posedge clk)if(wv)writes<=writes+1;
 function [15:0] crc_byte(input [15:0] c,input [7:0] b);
  reg [15:0] v;integer k;begin v=c^{b,8'b0};for(k=0;k<8;k=k+1)v=v[15]?(v<<1)^16'h1021:v<<1;crc_byte=v;end
 endfunction
 task receive_byte(output reg [7:0] value);
  integer k;begin @(negedge tx);#150;for(k=0;k<8;k=k+1)begin value[k]=tx;#100;end if(tx!==1)$fatal(1,"TX stop");end
 endtask
 initial forever begin
  for(i=0;i<130;i=i+1)receive_byte(bytes[i]);
  if(bytes[0]!=8'ha5||bytes[1]!=8'h5a||bytes[2]!=1||bytes[6]!=120)$fatal(1,"UART header");
  crc=16'hffff;for(j=2;j<128;j=j+1)crc=crc_byte(crc,bytes[j]);
  if(crc!={bytes[129],bytes[128]})$fatal(1,"UART packet CRC");
  for(j=0;j<130;j=j+1)last_packet[j]=bytes[j];packets=packets+1;
 end
 task send_byte(input [7:0] b);
  integer k;begin @(negedge clk);rx=0;repeat(10)@(negedge clk);
   for(k=0;k<8;k=k+1)begin rx=b[k];repeat(10)@(negedge clk);end
   rx=1;repeat(10)@(negedge clk);end
 endtask
 task command(input [7:0] op,input [15:0] data,input integer bad);
  begin send_byte(8'ha5);send_byte(8'h5a);send_byte(op);send_byte(data[7:0]);send_byte(data[15:8]);send_byte(op^data[7:0]^data[15:8]^bad);end
 endtask
 reg [207:0] p={16'd3,32'd0,32'd65536,32'd0,32'd0,32'd0,32'd65536};reg [15:0] c;integer n,old;
 initial begin
  wait(rst); // Initial status is pending: writes must not be rejected by it.
  c=16'hffff;for(n=0;n<26;n=n+1)c=crc_byte(c,p[n*8+:8]);
  for(n=0;n<13;n=n+1)command(8'h60+n,p[n*16+:16],0);
  command(8'h6d,c,0);wait(cv);
  if(writes!=14||cp!==p||errors!=0||dut.rejected!=0)$fatal(1,"UART config transfer while pending");
  command(8'h60,16'habcd,1);if(writes!=14||dut.rx_errors!=1)$fatal(1,"bad outer checksum accepted");
  command(8'h60,16'habcd,0);repeat(4)@(posedge clk);if(errors!=1)$fatal(1,"busy config not rejected");
  cq=1;wait(!busy);cq=0;
  wait(packets>=1);wait(!dut.pending&&!dut.sending);old=packets;command(8'h47,0,0);wait(packets>old&&last_packet[3]==8'h47);
  if(last_packet[3]!=8'h47||carg!=16'hfffb||{last_packet[127],last_packet[126]}!=16'hfffb||!last_packet[13][3])$fatal(1,"advanced page/tag op=%h carg=%h tag=%h flags=%h",last_packet[3],carg,{last_packet[127],last_packet[126]},last_packet[13]);
  if({last_packet[59],last_packet[58],last_packet[57],last_packet[56]}!=1||last_packet[63]!=8'ha1)$fatal(1,"sys-domain error insertion/marker");
  $display("PASS actual UART staged config/CRC/mailbox/busy/snapshot page/TX CRC");$finish;
 end
 initial begin #1000000;$fatal(1,"UART timeout");end
endmodule
