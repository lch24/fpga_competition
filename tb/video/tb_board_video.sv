`timescale 1ns/1ps
// Actual burst reader + CDC bridge at 125 MHz / 37.125 MHz, 720p raster.
// DDR is a behavioral delayed-response model, not a vendor PHY simulation.
module tb_board_video;
 reg finished=0;
 reg clk=0,pclk=0,rst=0,de=0,vs=0,enable=0;
 reg [31:0] frame_base=32'h200000;
 wire idle,video_core_rst_n,video_pixel_rst_n,bridge_ready;
 reset_sync cr(.clk(clk),.arst_n(rst&&enable),.rst_n(video_core_rst_n));
 reset_sync pr(.clk(pclk),.arst_n(rst&&enable),.rst_n(video_pixel_rst_n));
 always #4 clk=~clk;always #13.468 pclk=~pclk;
 wire valid,ready,first,last,error;
 assign ready=bridge_ready&&video_core_rst_n;wire [15:0] pixel;
 wire [27:0] addr;wire [3:0] len,id;wire av;
 reg ar=0,rv=0,rl=0;reg [255:0] data=0;
 wire [7:0] r,g,b;wire underflow;
 board_display_hmic #(.DYNAMIC_BASE(1)) reader(.clk(clk),.rst_n(rst),.enable(enable),.frame_base(frame_base),.idle(idle),.pixel_valid(valid),.pixel_ready(ready),.pixel(pixel),.pixel_first(first),.pixel_last(last),.error(error),
 .axi_araddr(addr),.axi_arlen(len),.axi_aruser_id(id),.axi_arvalid(av),.axi_arready(ar),.axi_rdata(data),.axi_rvalid(rv),.axi_rlast(rl),.axi_rid(4'd0));
 hdmi_pixel_bridge #(.FIFO_BITS(11)) bridge(.clk(clk),.rst_n(video_core_rst_n),.in_valid(valid&&video_core_rst_n),.in_ready(bridge_ready),.in_pixel(pixel),.in_first(first),.in_last(last),
 .pclk(pclk),.prst_n(video_pixel_rst_n),.de(de),.vsync(vs),.clear_error(!enable),.r(r),.g(g),.b(b),.underflow(underflow));
 integer remaining=0,base=0,beat=0,i,x,y,checked=0;
 reg [15:0] expected;
 function [15:0] mem(input integer a);
  begin mem=(a>=32'h01000000) ? (16'h8000^((a-32'h01000000)/2)) : (a/2);end
 endfunction
 always @(negedge clk)begin
  ar=0;rv=0;rl=0;
  if(rst)begin
   if(remaining>0)begin
    if($urandom_range(0,3)!=0)begin
     rv=1;rl=remaining==1;
     for(i=0;i<16;i=i+1)data[i*16+:16]=mem(base+beat*32+i*2);
     remaining=remaining-1;beat=beat+1;
    end
   end else if(av&&$urandom_range(0,3)!=0)begin ar=1;base=addr*4;remaining=len+1;beat=0;end
  end
 end
 task show_frame;
  begin
   vs=1;repeat(1650*5)@(negedge pclk);vs=0;
   repeat(1650*20)@(negedge pclk);
   for(y=0;y<720;y=y+1)begin
    repeat(260)@(negedge pclk);
    for(x=0;x<1280;x=x+1)begin
     de=1;expected=mem(frame_base+2*(y*1280+x));
     #1;if({r,g,b}!=={expected[15:11],3'b0,expected[10:5],2'b0,expected[4:0],3'b0})
      $fatal(1,"HDMI mismatch x=%0d y=%0d got %h pixel %h",x,y,{r,g,b},expected);
     checked=checked+1;@(negedge pclk);
    end
    de=0;repeat(110)@(negedge pclk);
   end
   if(error||underflow)$fatal(1,"video failed");
  end
 endtask
 initial begin
  repeat(8)@(negedge pclk);rst=1;enable=1;
  show_frame();
  @(negedge clk);enable=0;wait(idle);repeat(10)@(negedge clk);
  // Change content as well as address: second frame is corrected DDR region.
  frame_base=32'h1000000;enable=1;
  @(negedge pclk);show_frame();
  if(checked!=1843200)$fatal(1,"pixel count");
  finished=1;$display("PASS tb_board_video 1843200 HDMI pixels, raw/corrected switch, FIFO flush, no underflow");$finish;
 end
 initial begin #100000000;$fatal(1,"video timeout");end
endmodule
