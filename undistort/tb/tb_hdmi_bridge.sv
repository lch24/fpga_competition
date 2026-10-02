`timescale 1ns/1ps
module tb_hdmi_bridge;
 reg clk=0,pclk=0;always #5 clk=~clk;always #7 pclk=~pclk;reg rst_n=0;
 reg in_valid=0,in_first=0,in_last=0,de=0,vsync=0,clear_error=0;
 reg [15:0] in_pixel;wire in_ready;wire [7:0] r,g,b;wire underflow;
 hdmi_pixel_bridge #(.FIFO_BITS(3)) dut(.clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_ready(in_ready),
 .in_pixel(in_pixel),.in_first(in_first),.in_last(in_last),.pclk(pclk),.prst_n(rst_n),.de(de),.vsync(vsync),
 .clear_error(clear_error),.r(r),.g(g),.b(b),.underflow(underflow));
 task token(input integer first,input integer last,input [15:0] pixel);
  begin @(negedge clk);in_valid=1;in_first=first;in_last=last;in_pixel=pixel;
   do @(posedge clk);while(!in_ready);@(negedge clk);in_valid=0;end
 endtask
 task sync_frame;
  begin @(negedge pclk);de=0;vsync=1;repeat(2)@(negedge pclk);vsync=0;end
 endtask
 task sample(input [23:0] expected);
  begin de=1;#1;if({r,g,b}!==expected)$fatal(1,"HDMI got %h expected %h",{r,g,b},expected);@(negedge pclk);end
 endtask
 initial begin
  repeat(4)@(negedge clk);rst_n=1;
  token(1,0,16'hf800);repeat(5)@(negedge pclk);sync_frame();sample(24'hf80000);sample(0);
  if(!underflow)$fatal(1,"underflow flag missing");de=0;
  token(0,0,16'h07e0);token(0,1,16'h001f);token(1,0,16'hffff);token(0,1,16'hf800);
  repeat(8)@(negedge pclk);sample(0);de=0; // no restart mid-frame
  sync_frame();sample(24'hf8fcf8);sample(24'hf80000);sample(0);de=0;
  clear_error=1;@(negedge pclk);clear_error=0;if(underflow)$fatal(1,"clear flag");
  $display("PASS HDMI bridge: underflow black, discard old tail, SOF/VSYNC resynchronization");$finish;
 end
 initial begin #100000;$fatal(1,"bridge timeout");end
endmodule
