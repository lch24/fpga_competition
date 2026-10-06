`timescale 1ns/1ps
module tb_candidate_cache;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,clear=0,allow_step=1,gr=0,gw=0;
 reg [14:0] ga=0,wa=0;reg [63:0] wd=0;
 wire [63:0] gd;wire step_en,error;
 reg inject_read_error=0,inject_write_error=0;
 `include "logical_bus.vh"
 `include "logical_memory.vh"
 candidate_ddr_cache #(.LINE_BITS(2)) dut(
  .clk(clk),.rst_n(rst_n),.clear(clear),.advance(step_en&&allow_step),.backing_base(32'd4096),
  .rd_en(gr),.rd_addr(ga),.rd_data(gd),.wr_en(gw),.wr_addr(wa),.wr_data(wd),.step_en(step_en),.error(error),
  .rd_valid(rd_valid),.rd_ready(rd_ready),.rd_address(rd_addr),.rd_length(rd_len),.rd_tag(rd_tag),
  .r_valid(r_valid),.r_ready(r_ready),.r_data(r_data),.r_keep(r_keep),.r_tag(r_tag),.r_last(r_last),.r_error(r_error||inject_read_error),
  .wr_valid(wr_valid),.wr_ready(wr_ready),.wr_address(wr_addr),.wr_length(wr_len),.wr_tag(wr_tag),
  .w_valid(w_valid),.w_ready(w_ready),.w_data(w_data),.w_keep(w_keep),.w_last(w_last),
  .b_valid(b_valid),.b_ready(b_ready),.b_tag(b_tag),.b_error(b_error||inject_write_error));
 reg [63:0] reference[0:32767];
 integer i,j,checked=0,reads=0,writes=0,faults=0;
 reg finished=0;reg [63:0] expected,held;
 always @(posedge clk) begin
  if(rd_valid&&rd_ready)reads<=reads+1;
  if(wr_valid&&wr_ready)writes<=writes+1;
 end
 task access;
  input re,we;input [14:0] ra,wa_in;input [63:0] value;input stall;
  integer before_w;
  begin
   @(negedge clk);gr=re;gw=we;ga=ra;wa=wa_in;wd=value;allow_step=!stall;
   expected=reference[ra];held=gd;before_w=writes;#1;
   if(stall)begin
    wait(step_en);
    repeat(9)begin @(negedge clk);if(gd!==held)$fatal(1,"read advanced during other-cache stall");end
    if(writes!=before_w+(we?1:0))$fatal(1,"repeated write during stalled commit");
    allow_step=1;
   end
   do @(posedge clk);while(!step_en);
   if(we)reference[wa_in]=value;
   @(negedge clk);
   if(re && gd!==expected)$fatal(1,"candidate %d got %h expected %h",ra,gd,expected);
   gr=0;gw=0;checked=checked+1;
  end
 endtask
 initial begin
  for(i=0;i<32768;i=i+1)begin
   reference[i]={32'hcaff0000+i,32'h01234567^i};
   for(j=0;j<8;j=j+1)memory.mem[4096+i*8+j]=reference[i]>>(8*j);
  end
  memory.mem[4095]=8'ha5;memory.mem[266240]=8'h5a;
  repeat(4)@(negedge clk);rst_n=1;
  for(i=0;i<64;i=i+1)access(1,0,i,0,0,i%7==0);
  // Cache aliases, A/B bank boundary, read-before-write and subsequent refill.
  for(i=0;i<64;i=i+1)begin
   access(1,1,i*257,16384+i,{32'h81234567+i,32'hfedcba98-i},i%3==0);
   access(1,0,16384+i,0,0,0);
   access(1,1,16384+i,16384+i,{32'h13579bdf+i,32'h2468ace0-i},1);
   access(1,0,16384+i,0,0,0);
  end
  access(0,1,0,32767,64'habcdef0123456789,1);
  access(1,0,32767,0,0,0);
  if(error||violations||memory.mem[4095]!=8'ha5||memory.mem[266240]!=8'h5a)$fatal(1,"protocol or bounds");
  @(negedge clk);clear=1;@(negedge clk);clear=0;
  access(1,0,32767,0,0,1);
  @(negedge clk);inject_read_error=1;gr=1;ga=8192;
  wait(error);@(negedge clk);gr=0;inject_read_error=0;
  if(step_en||dut.state!=0)$fatal(1,"read fault not drained");
  faults=faults+1;clear=1;@(negedge clk);clear=0;
  inject_write_error=1;gw=1;wa=10;wd=0;
  wait(error);@(negedge clk);gw=0;inject_write_error=0;
  if(step_en||dut.state!=0)$fatal(1,"write fault not drained");
  faults=faults+1;clear=1;@(negedge clk);clear=0;
  access(1,0,32767,0,0,0);
  @(negedge clk);gr=1;ga=9000;
  wait(dut.state==2);@(negedge clk);rst_n=0;gr=0;
  repeat(3)@(negedge clk);rst_n=1;
  repeat(10)@(negedge clk);
  if(rd_valid||wr_valid||error||!step_en)$fatal(1,"reset cancellation");
  faults=faults+1;
  finished=1;$finish;
 end
 initial begin #10000000;$fatal(1,"candidate cache timeout");end
endmodule
