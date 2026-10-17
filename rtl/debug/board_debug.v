`timescale 1ns/1ps
// Independent sys_clk monitor. Commands: A5 5A op lo hi xor(op,lo,hi).
// op=53 status/results (FFFF), op=44 progress (FFFE), op=43 corner (lo/hi).
// Response: A5 5A 01 op seqLE16 lengthLE16(120), payload, CRC16-CCITT LE.
// CRC covers version through payload, initial FFFF. No functional backpressure.
module board_debug #(parameter CLOCK_HZ=27000000,BAUD=115200,ENABLE_DISTORT_DEMO=0)(
 input wire clk,rx,output wire tx,output wire debug_rst_n,
 input wire camera_clk,camera_vsync,
 input wire core_clk,algorithm_clk,input wire [255:0] core_data,
 input wire [511:0] algorithm_data,output wire [15:0] algorithm_argument,core_argument,
 input wire [7:0] health
);
 reg [7:0] por=0;
 always @(posedge clk)if(!(&por))por<=por+1'b1;
 assign debug_rst_n=&por;
 wire rst_n=debug_rst_n;
 localparam MS=CLOCK_HZ/1000;
 reg [$clog2(MS)-1:0] divider;reg [31:0] uptime;
 wire tick=divider==MS-1;
 always @(posedge clk)if(!rst_n)begin divider<=0;uptime<=0;end
 else if(tick)begin divider<=0;uptime<=uptime+1'b1;end else divider<=divider+1'b1;
 wire [255:0] core_snapshot;wire [511:0] alg_snapshot;
 wire cu,au;wire [15:0] atag,ctag;
 reg [15:0] argument;
 wire camera_rst_n;
 reset_sync camera_reset(.clk(camera_clk),.arst_n(rst_n),.rst_n(camera_rst_n));
 reg [7:0] camera_frames;reg camera_vs_d;
 always @(posedge camera_clk or negedge camera_rst_n)begin
  if(!camera_rst_n)begin camera_frames<=0;camera_vs_d<=0;end
  else begin camera_vs_d<=camera_vsync;
   if(camera_vsync&&!camera_vs_d)camera_frames<=camera_frames+1'b1;
  end
 end
 wire [7:0] camera_snapshot;wire camera_update;
 reg [7:0] camera_age;reg camera_seen;
 debug_snapshot #(.WIDTH(8)) cams(.clk(clk),.rst_n(rst_n),.poll(tick),.argument(16'b0),
  .data(camera_snapshot),.tag(),.update(camera_update),.src_clk(camera_clk),.src_data(camera_frames),.src_argument());
 always @(posedge clk)begin
  if(!rst_n)begin camera_age<=255;camera_seen<=0;end
  else if(camera_update)begin camera_age<=0;camera_seen<=1;end
  else if(tick&&camera_age!=255)camera_age<=camera_age+1'b1;
 end
 debug_snapshot #(.WIDTH(256)) cs(.clk(clk),.rst_n(rst_n),.poll(tick),.argument(argument),
  .data(core_snapshot),.tag(ctag),.update(cu),.src_clk(core_clk),.src_data(core_data),.src_argument(core_argument));
 debug_snapshot #(.WIDTH(512)) als(.clk(clk),.rst_n(rst_n),.poll(tick),.argument(argument),
  .data(alg_snapshot),.tag(atag),.update(au),.src_clk(algorithm_clk),.src_data(algorithm_data),.src_argument(algorithm_argument));
 reg [31:0] core_age,alg_age;
 reg core_seen,alg_seen;
 reg [7:0] hs1,hs2 /* synthesis PAP_ASYNC_REG=1 */;
 always @(posedge clk)begin
  hs1<=health;hs2<=hs1;
  if(!rst_n)begin core_age<=32'hffffffff;alg_age<=32'hffffffff;core_seen<=0;alg_seen<=0;end
  else begin
   if(cu)begin core_age<=0;core_seen<=1;end else if(tick&&core_age!=32'hffffffff)core_age<=core_age+1'b1;
   if(au)begin alg_age<=0;alg_seen<=1;end else if(tick&&alg_age!=32'hffffffff)alg_age<=alg_age+1'b1;
  end
 end
 wire [7:0] rdata;wire rv,re,tr;
 reg [7:0] tx_byte;reg sending;reg [7:0] pos;
 debug_uart #(.CLOCK_HZ(CLOCK_HZ),.BAUD(BAUD)) uart(.clk(clk),.rst_n(rst_n),.rx(rx),.tx(tx),
  .tx_data(tx_byte),.tx_valid(sending),.tx_ready(tr),.rx_data(rdata),.rx_valid(rv),.rx_error(re));
 reg [2:0] parser;reg [7:0] op,lo,hi;
 reg [31:0] rx_errors,rejected;
 reg [7:0] parser_age;
 reg pending;reg [7:0] pending_op,packet_op;
 reg [15:0] packet_number,packet_seq,crc;
 reg [959:0] packet;
 reg [9:0] heartbeat;
 reg [7:0] wait_ms;
 reg [31:0] previous_events;
 function [15:0] crc_byte;
  input [15:0] c;input [7:0] b;reg [15:0] v;integer i;
  begin v=c^{b,8'b0};for(i=0;i<8;i=i+1)v=v[15]?(v<<1)^16'h1021:v<<1;crc_byte=v;end
 endfunction
 always @*begin
  case(pos)
   0:tx_byte=8'ha5;1:tx_byte=8'h5a;2:tx_byte=1;3:tx_byte=packet_op;
   4:tx_byte=packet_seq[7:0];5:tx_byte=packet_seq[15:8];6:tx_byte=120;7:tx_byte=0;
   128:tx_byte=crc[7:0];129:tx_byte=crc[15:8];
   default:tx_byte=packet[(pos-8)*8+:8];
  endcase
 end
 always @(posedge clk)begin
  if(!rst_n)begin parser<=0;op<=0;lo<=0;hi<=0;rx_errors<=0;rejected<=0;parser_age<=0;
   argument<=16'hffff;pending<=1;pending_op<=8'h53;packet_op<=0;packet_number<=0;packet_seq<=0;
   sending<=0;pos<=0;crc<=16'hffff;packet<=0;heartbeat<=0;wait_ms<=0;previous_events<=0;end
  else begin
   if(tick)begin
    if(parser_age<255)parser_age<=parser_age+1'b1;
    if(parser!=0&&parser_age==20)begin parser<=0;rx_errors<=rx_errors+1'b1;end
    if(heartbeat==999)begin
     heartbeat<=0;if(!pending)begin pending<=1;pending_op<=8'h53;argument<=16'hffff;wait_ms<=0;end
    end else heartbeat<=heartbeat+1'b1;
    if(pending&&wait_ms<255)wait_ms<=wait_ms+1'b1;
   end
   // Coalesce change reports. Event counter exposes intermediate transitions.
   if(ctag!=16'hfffd&&(!ENABLE_DISTORT_DEMO||ctag!=16'hfffc)&&core_snapshot[63:32]!=previous_events)begin
    previous_events<=core_snapshot[63:32];
    if(!pending)begin pending<=1;pending_op<=8'h53;argument<=16'hffff;wait_ms<=0;end
   end
   if(re)begin rx_errors<=rx_errors+1'b1;parser<=0;end
   if(rv)begin
    parser_age<=0;
    case(parser)
     0:if(rdata==8'ha5)parser<=1;
     1:if(rdata==8'h5a)parser<=2;else parser<=rdata==8'ha5 ?1:0;
     2:begin op<=rdata;parser<=3;end
     3:begin lo<=rdata;parser<=4;end
     4:begin hi<=rdata;parser<=5;end
     5:begin
      parser<=0;
      if(rdata!=(op^lo^hi)||(op!=8'h53&&op!=8'h43&&op!=8'h44&&op!=8'h56&&(!ENABLE_DISTORT_DEMO||op!=8'h45)))rx_errors<=rx_errors+1'b1;
      else if(pending)rejected<=rejected+1'b1;
      else begin pending<=1;pending_op<=op;argument<=op==8'h53 ?16'hffff:op==8'h44 ?16'hfffe:op==8'h56 ?16'hfffd:ENABLE_DISTORT_DEMO&&op==8'h45 ?16'hfffc:{hi,lo};wait_ms<=0;end
     end
     default:parser<=0;
    endcase
   end
   // Bounded wait: a stopped domain must never prevent a UART response.
   if(!sending&&pending&&wait_ms>=4&&((atag==argument&&ctag==argument&&alg_age<100&&core_age<100)||wait_ms>=20))begin
    packet<={alg_snapshot,core_snapshot,rejected,rx_errors,alg_age,core_age,
     camera_snapshot,hs2,4'b0,(atag==argument&&ctag==argument),alg_seen,core_seen,1'b0,
     2'b0,camera_seen,(camera_age>=100),(alg_age>=100),(core_age>=100),2'b0,uptime};
    packet_seq<=packet_number;packet_number<=packet_number+1'b1;packet_op<=pending_op;
    pending<=0;sending<=1;pos<=0;crc<=16'hffff;
   end
   if(sending&&tr)begin
    if(pos>=2&&pos<128)crc<=crc_byte(crc,tx_byte);
    if(pos==129)sending<=0;else pos<=pos+1'b1;
   end
  end
 end
endmodule
