`timescale 1ns/1ps
module tb_gray_cache;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,clear=0,gr=0,gw=0,inject_read_error=0,inject_write_error=0;
 reg [15:0] ga=0,wa=0;reg [7:0] wd=0;
 wire [7:0] gd;wire step_en,error;
 `include "logical_bus.vh"
 `include "logical_memory.vh"
 gray_ddr_cache #(.ADDR_W(16),.LINE_BITS(2)) dut(
  .clk(clk),.rst_n(rst_n),.clear(clear),.backing_base(32'd4096),.logical_base(16'd17),
  .rd_en(gr),.rd_addr(ga),.rd_data(gd),.wr_en(gw),.wr_addr(wa),.wr_data(wd),.step_en(step_en),.error(error),
  .rd_valid(rd_valid),.rd_ready(rd_ready),.rd_address(rd_addr),.rd_length(rd_len),.rd_tag(rd_tag),
  .r_valid(r_valid),.r_ready(r_ready),.r_data(r_data),.r_keep(r_keep),.r_tag(r_tag),.r_last(r_last),.r_error(r_error||inject_read_error),
  .wr_valid(wr_valid),.wr_ready(wr_ready),.wr_address(wr_addr),.wr_length(wr_len),.wr_tag(wr_tag),
  .w_valid(w_valid),.w_ready(w_ready),.w_data(w_data),.w_keep(w_keep),.w_last(w_last),
  .b_valid(b_valid),.b_ready(b_ready),.b_tag(b_tag),.b_error(b_error||inject_write_error));
 reg [7:0] reference[0:8191];
 integer i,checked=0,misses=0,cycles=0,faults=0;
 reg finished=0;reg [7:0] expected;
 always @(posedge clk)begin cycles<=cycles+1;if(rd_valid&&rd_ready)misses<=misses+1;end
 task access;
  input read_enable,write_enable;input [15:0] read_index,write_index;input [7:0] value;
  begin
   @(negedge clk);gr=read_enable;gw=write_enable;ga=read_index+17;wa=write_index+17;wd=value;
   do @(posedge clk);while(!step_en);
   expected=reference[read_index];if(write_enable)reference[write_index]=value;
   @(negedge clk);if(read_enable && gd!==expected)$fatal(1,"cache index %0d got %h expected %h",read_index,gd,expected);
   gr=0;gw=0;checked=checked+1;
  end
 endtask
 initial begin
  for(i=0;i<8192;i=i+1)begin reference[i]=(i*37)^(i>>3);memory.mem[4096+i]=reference[i];end
  repeat(4)@(negedge clk);rst_n=1;
  // Sequential scan, conflicting lines, unaligned writes, simultaneous source
  // reads and pyramid writes, then read-after-write across cache eviction.
  for(i=0;i<256;i=i+1)access(1,0,i,0,0);
  for(i=0;i<128;i=i+1)access(1,1,(i*257)%4096,4096+i,i^8'ha7);
  for(i=0;i<128;i=i+1)access(1,0,4096+i,0,0);
  for(i=0;i<256;i=i+1)access(0,1,0,1001+i,i*3);
  for(i=0;i<256;i=i+1)access(1,0,1001+i,0,0);
  wait(dut.state==0 && dut.write_keep_q==0);
  if(error||violations)$fatal(1,"cache DDR protocol");
  @(negedge clk);clear=1;@(negedge clk);clear=0;
  for(i=0;i<128;i=i+1)access(1,0,4096+i,0,0);
  if(error||violations)$fatal(1,"cache clear/reload");
  // Read failure drains through last before exposing terminal error.
  @(negedge clk);inject_read_error=1;gr=1;ga=7017;
  wait(error);@(negedge clk);gr=0;inject_read_error=0;
  if(step_en || dut.state!=0)$fatal(1,"read error did not stop engine");
  faults=faults+1;clear=1;@(negedge clk);clear=0;
  // Write failure is reported after the completion handshake.
  inject_write_error=1;access(0,1,0,100,8'h53);
  wait(error);@(negedge clk);inject_write_error=0;
  if(step_en || dut.state!=0)$fatal(1,"write error did not stop engine");
  faults=faults+1;clear=1;@(negedge clk);clear=0;
  // Reset cancels an outstanding refill together with its DDR endpoint.
  gr=1;ga=6017;wait(dut.state==2);@(negedge clk);rst_n=0;gr=0;
  repeat(3)@(negedge clk);rst_n=1;
  repeat(10)@(negedge clk);
  if(rd_valid||wr_valid||error||!step_en)$fatal(1,"cache reset cancellation");
  faults=faults+1;
  finished=1;$finish;
 end
 initial begin #10000000;$fatal(1,"cache timeout");end
endmodule
