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
 wire scale_ready,scale_valid,scale_first,scale_last,scale_error;wire [15:0] scale_pixel;
 assign ready=scale_ready&&video_core_rst_n;wire [15:0] pixel;
 wire [27:0] addr;wire [3:0] len,id;wire av;
 reg ar=0,rv=0,rl=0;reg [255:0] data=0;
 wire [7:0] r,g,b;wire underflow;
 board_display_hmic #(.DYNAMIC_BASE(1),.WIDTH(960),.HEIGHT(540)) reader(.clk(clk),.rst_n(rst),.enable(enable),.frame_base(frame_base),.idle(idle),.pixel_valid(valid),.pixel_ready(ready),.pixel(pixel),.pixel_first(first),.pixel_last(last),.error(error),
 .axi_araddr(addr),.axi_arlen(len),.axi_aruser_id(id),.axi_arvalid(av),.axi_arready(ar),.axi_rdata(data),.axi_rvalid(rv),.axi_rlast(rl),.axi_rid(4'd0));
 video_scale_4_3 #(.WIDTH(960),.HEIGHT(540)) scale(.clk(clk),.rst_n(video_core_rst_n),.in_valid(valid&&video_core_rst_n),.in_ready(scale_ready),.in_pixel(pixel),.in_first(first),.in_last(last),
 .out_valid(scale_valid),.out_ready(bridge_ready),.out_pixel(scale_pixel),.out_first(scale_first),.out_last(scale_last),.error(scale_error),.frames(),.phase());
 hdmi_pixel_bridge #(.FIFO_BITS(11)) bridge(.clk(clk),.rst_n(video_core_rst_n),.in_valid(scale_valid),.in_ready(bridge_ready),.in_pixel(scale_pixel),.in_first(scale_first),.in_last(scale_last),
 .pclk(pclk),.prst_n(video_pixel_rst_n),.de(de),.vsync(vs),.clear_error(!enable),.r(r),.g(g),.b(b),.underflow(underflow));
 integer remaining=0,base=0,beat=0,i,x,y,checked=0;
 reg [15:0] expected;
 function [15:0] mem(input integer a);
  begin mem=(a>=32'h01000000) ? (16'h8000^((a-32'h01000000)/2)) : (a/2);end
 endfunction
 function integer channel(input [15:0] p,input integer c);
  case(c)0:channel={p[4:0],p[4:2]};1:channel={p[10:5],p[10:9]};2:channel={p[15:11],p[15:13]};endcase
 endfunction
 function [15:0] enlarged(input integer x,y);
  integer a,b,c,d,u,v,k,s;reg [23:0] rgb;
  begin a=x*3/4;b=y*3/4;c=a<959?a+1:a;d=b<539?b+1:b;u=x*3%4;v=y*3%4;
   for(k=0;k<3;k=k+1)begin
    s=((4-u)*(4-v)*channel(mem(frame_base+2*(b*960+a)),k)+u*(4-v)*channel(mem(frame_base+2*(b*960+c)),k)+
       (4-u)*v*channel(mem(frame_base+2*(d*960+a)),k)+u*v*channel(mem(frame_base+2*(d*960+c)),k)+8)/16;
    rgb[k*8+:8]=s;
   end
   enlarged={rgb[23:19],rgb[15:10],rgb[7:3]};
  end
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
     de=1;expected=enlarged(x,y);
     #1;if({r,g,b}!=={expected[15:11],3'b0,expected[10:5],2'b0,expected[4:0],3'b0})
      $fatal(1,"HDMI mismatch x=%0d y=%0d got %h pixel %h",x,y,{r,g,b},expected);
     checked=checked+1;@(negedge pclk);
    end
    de=0;repeat(110)@(negedge pclk);
   end
   if(error||scale_error||underflow)$fatal(1,"video failed");
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
  finished=1;$display("PASS tb_board_video 960x540 DDR -> 1843200 checked 720p HDMI pixels, bilinear scaling, raw/corrected switch, no underflow");$finish;
 end
 initial begin #100000000;$fatal(1,"video timeout");end
endmodule
