`timescale 1ns/1ps
module tb_video_capture;
 localparam W=64,H=4,N=W*H;
 reg clk=0,pclk=0;always #4 clk=~clk;always #5 pclk=~pclk;
 reg rst=0,vs=0,pv=0,cmd=0;reg [15:0] pixel=0;reg [31:0] base=0;
 wire ready,rsp;wire [7:0] status;wire [31:0] rsp_base,skipped;
 wire [27:0] addr;wire [3:0] len,id;wire av;wire [255:0] data;wire [31:0] strb;
 reg busy=0,complete=0,block_ddr=0;reg [31:0] cycle=0;reg [27:0] current_addr;
 wire aq=!busy&&!complete&&!block_ddr&&cycle%5!=0,wr=busy&&!block_ddr&&cycle%3!=0;
 reg [15:0] memory[0:2047];integer i,responses=0;reg [7:0] results[0:7];reg [31:0] bases[0:7];
 video_capture_hmic #(.WIDTH(W),.HEIGHT(H),.FIFO_BITS(4)) dut(
  .clk(clk),.rst_n(rst),.pclk(pclk),.prst_n(rst),.vsync(vs),.pixel_valid(pv),.pixel(pixel),.line_short(6'd0),
  .source_frame_error(4'd0),
  .cmd_valid(cmd),.cmd_ready(ready),.cmd_base(base),.rsp_valid(rsp),.rsp_ready(1'b1),.rsp_status(status),.rsp_base(rsp_base),.skipped_frames(skipped),
  .axi_awaddr(addr),.axi_awlen(len),.axi_awuser_id(id),.axi_awvalid(av),.axi_awready(aq),
  .axi_wdata(data),.axi_wstrb(strb),.axi_wready(wr),.axi_wusero_last(complete),.axi_wusero_id(4'b0));
 always @(posedge clk)if(rst)begin
  cycle<=cycle+1;
  if(av&&aq)begin busy<=1;current_addr<=addr;if(len!=0||id!=0)$fatal(1,"capture burst");end
  if(wr)begin
   for(i=0;i<16;i=i+1)memory[current_addr*2+i]<=data[i*16+:16];busy<=0;complete<=1;
  end
  if(complete)complete<=0;
  if(rsp)begin results[responses]<=status;bases[responses]<=rsp_base;responses<=responses+1;end
 end
 task descriptor(input [31:0] a);
  begin @(negedge clk);base=a;cmd=1;do @(posedge clk);while(!ready);@(negedge clk);cmd=0;end
 endtask
 task boundary;
  begin @(negedge pclk);vs=1;repeat(4)@(negedge pclk);vs=0;repeat(4)@(negedge pclk);end
 endtask
 task pixels(input integer count,seed);
  integer k;begin for(k=0;k<count;k=k+1)begin pv=1;pixel=16'(seed+k);@(negedge pclk);pv=0;@(negedge pclk);end end
 endtask
 initial begin
  repeat(5)@(negedge clk);rst=1;
  descriptor(0);descriptor(512);descriptor(1024);repeat(10)@(negedge pclk);
  boundary;pixels(N,0);boundary;pixels(N,1000);boundary;pixels(N,2000);boundary;
  wait(responses==3);#1;
  for(i=0;i<3;i=i+1)if(results[i]!=0||bases[i]!=i*512)$fatal(1,"consecutive frame status/base %0d",i);
  for(i=0;i<N;i=i+1)if(memory[i]!==16'(i)||memory[N+i]!==16'(1000+i)||memory[2*N+i]!==16'(2000+i))$fatal(1,"capture packing %0d",i);
  // Full data FIFO still reserves START/END markers; overflow cannot hang stop.
  descriptor(1536);repeat(10)@(negedge pclk);block_ddr=1;
  boundary;pixels(N,3000);boundary;block_ddr=0;wait(responses==4);#1;
  // A frame whose write FIFO overflowed must still be rejected, and the code must
  // be one of the FIFO-loss codes: 0x84 (one dropped packed word) or 0x85 (two or
  // more). Which one depends on how many pushes collide with a nearly full FIFO,
  // so accept either and always print what was actually returned.
  if(results[3]!=8'h84&&results[3]!=8'h85)$fatal(1,"overflow accepted status=%0d",results[3]);
  descriptor(0);repeat(10)@(negedge pclk);boundary;pixels(N-16,4000);boundary;
  wait(responses==5);#1;if(results[4]!=8'h81)$fatal(1,"short frame accepted status=%0d",results[4]);
  descriptor(0);repeat(10)@(negedge pclk);boundary;pixels(N,5000);boundary;
  wait(responses==6);#1;if(results[5]!=0)$fatal(1,"recovery failed");
  for(i=0;i<N;i=i+1)if(memory[i]!==16'(5000+i))$fatal(1,"recovery pixels");
  $display("PASS video_capture consecutive VSYNC frames, RGB565 order, overflow/short-frame drain and recovery");$finish;
 end
 initial begin #1000000;$fatal(1,"capture timeout responses=%d state=%d",responses,dut.state);end
endmodule
