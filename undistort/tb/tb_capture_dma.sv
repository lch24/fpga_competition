`timescale 1ns/1ps
module tb_capture_dma;
 reg clk=0,pclk=0;always #5 clk=~clk;always #3 pclk=~pclk;
 reg rst_n=0,camera_vsync=0,camera_valid=0;reg [15:0] camera_pixel=0;
 reg cmd_valid=0,rsp_ready=0,wr_ready=0,w_ready=0,b_valid=0;wire cmd_ready,rsp_valid,wr_valid,w_valid,b_ready;
 wire [7:0] rsp_status;wire [31:0] wr_addr,wr_len,w_data;wire [15:0] wr_tag;wire [3:0] w_keep;wire w_last;
 reg [7:0] mem[0:63];integer writes=0,i;
 capture_dma #(.FIFO_BITS(2)) dut(.clk(clk),.rst_n(rst_n),.pclk(pclk),.prst_n(rst_n),
 .camera_vsync(camera_vsync),.camera_valid(camera_valid),.camera_pixel(camera_pixel),
 .cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_base(32'd0),.cmd_stride(32'd8),.cmd_width(16'd4),.cmd_height(16'd3),
 .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.wr_valid(wr_valid),.wr_ready(wr_ready),
 .wr_addr(wr_addr),.wr_len(wr_len),.wr_tag(wr_tag),.w_valid(w_valid),.w_ready(w_ready),.w_data(w_data),
 .w_keep(w_keep),.w_last(w_last),.b_valid(b_valid),.b_ready(b_ready),.b_tag(16'd0),.b_error(1'b0));
 integer address;reg pending=0;
 always @(posedge clk)if(rst_n)begin
  if(wr_valid&&wr_ready)begin address=wr_addr;pending<=1;end
  w_ready<=pending&&!b_valid;
  if(w_valid&&w_ready)begin mem[address]<=w_data[7:0];mem[address+1]<=w_data[15:8];writes<=writes+1;pending<=0;b_valid<=1;w_ready<=0;end
  if(b_valid&&b_ready)b_valid<=0;
 end
 task frame(input integer slow);
  integer n;
  begin repeat(8)@(negedge pclk);camera_vsync=1;repeat(3)@(negedge pclk);camera_vsync=0;
   for(n=0;n<12;n=n+1)begin camera_valid=1;camera_pixel=16'h1000+n;@(negedge pclk);camera_valid=0;repeat(slow)@(negedge pclk);end
   repeat(4)@(negedge pclk);camera_vsync=1;repeat(3)@(negedge pclk);camera_vsync=0;end
 endtask
 task launch;
  begin @(negedge clk);cmd_valid=1;do @(posedge clk);while(!cmd_ready);@(negedge clk);cmd_valid=0;end
 endtask
 task finish_job(input integer status);
  begin wait(rsp_valid);repeat(4)@(negedge clk);if(rsp_status!=status)$fatal(1,"capture status %d expected %d",rsp_status,status);
   rsp_ready=1;@(negedge clk);rsp_ready=0;end
 endtask
 initial begin
  repeat(5)@(negedge clk);rst_n=1;
  launch();frame(0);wr_ready=1;finish_job(3);if(writes>=12)$fatal(1,"overflow not exercised");
  writes=0;launch();frame(30);finish_job(0);
  for(i=0;i<12;i=i+1)if({mem[i*2+1],mem[i*2]}!==16'h1000+i)$fatal(1,"recovery pixel %d",i);
  $display("PASS capture DMA: forced FIFO overflow invalidates frame, drain and next-frame recovery");$finish;
 end
 initial begin #200000;$fatal(1,"capture timeout");end
endmodule
