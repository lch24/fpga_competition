`timescale 1ns/1ps
module tb_camera_capture_chain;
 parameter W=1280;localparam H=2;
 reg clk=0,pclk=0,rst=0,vs=0,href=0,cmd=0;
 reg [7:0] byte_data=0;
 always #4 clk=~clk;always #5 pclk=~pclk;
 wire pv,fvs;wire [15:0] pixel;wire [5:0] bad;wire [3:0] frame_error;
 camera_byte_unpack #(.WIDTH(W),.HEIGHT(H)) unpack(.pclk(pclk),.rst_n(rst),.vsync(vs),.href(href),.data(byte_data),.pixel_valid(pv),.pixel(pixel),.line_bad(bad),.frame_error(frame_error),.capture_vsync(fvs));
 wire ready,rsp,av;wire [7:0] status;wire [27:0] addr;
 wire [255:0] data;wire [31:0] strb;wire [3:0] len,id;
 wire [191:0] diagnostic;
 reg busy=0,complete=0;integer cycles=0,writes=0,responses=0,j;
 reg [7:0] result=0;reg [15:0] mem[0:W*H-1];reg [27:0] a;
 wire aq=!busy&&!complete&&(cycles%5!=0),wr=busy&&(cycles%3!=0);
 video_capture_hmic #(.WIDTH(W),.HEIGHT(H),.FIFO_BITS(4)) cap(
 .clk(clk),.rst_n(rst),.pclk(pclk),.prst_n(rst),.vsync(fvs),.pixel_valid(pv),.pixel(pixel),.line_short(bad),
 .source_frame_error(frame_error),
 .cmd_valid(cmd),.cmd_ready(ready),.cmd_base(32'd0),.rsp_valid(rsp),.rsp_ready(1'b1),.rsp_status(status),.rsp_base(),.skipped_frames(),.diagnostic(diagnostic),
 .axi_awaddr(addr),.axi_awlen(len),.axi_awuser_id(id),.axi_awvalid(av),.axi_awready(aq),
 .axi_wdata(data),.axi_wstrb(strb),.axi_wready(wr),.axi_wusero_last(complete),.axi_wusero_id(4'd0));
 always @(posedge clk)if(rst)begin
  cycles<=cycles+1;
  if(av&&aq)begin a<=addr;busy<=1;end
  if(wr)begin
   for(j=0;j<16;j=j+1)mem[a*2+j]<=data[j*16+:16];
   writes<=writes+1;busy<=0;complete<=1;
  end
  if(complete)complete<=0;
  if(rsp)begin responses<=responses+1;result<=status;end
 end
 function [15:0] swap(input [15:0] v);swap={v[4:0],v[10:5],v[15:11]};endfunction
 task start_frame;
 begin
  @(negedge clk);cmd=1;do @(posedge clk);while(!ready);@(negedge clk);cmd=0;
  repeat(8)@(negedge pclk);#1;vs=1;
  repeat(4)@(negedge pclk);#1;vs=0;
 end endtask
 task send_frame(input integer short_first,input integer blank_end,input integer seed);
 integer r,c,n;reg [15:0] v;
 begin
  for(r=0;r<H;r=r+1)begin
   n=((r==0&&short_first==1)||short_first==2)?W-1:W;
   for(c=0;c<n;c=c+1)begin
    v=seed+r*W+c;
    @(negedge pclk);#1;href=1;byte_data=v[15:8];
    @(negedge pclk);#1;byte_data=v[7:0];
   end
   @(negedge pclk);#1;href=0;
   if(r<H-1)repeat(8)@(negedge pclk);
  end
  // zero blank clocks stresses the last byte immediately followed by VSYNC.
  repeat(blank_end)@(negedge pclk);#1;vs=1;
  repeat(4)@(negedge pclk);#1;vs=0;
 end endtask
 integer i,k,target;
 initial begin
  repeat(5)@(negedge clk);rst=1;
  for(k=0;k<7;k=k+1)begin
   target=responses+1;start_frame;send_frame(k==2?1:((k==4||k==5)?2:0),k==0?0:5,16'h1200+k*64);
   wait(responses==target);#1;
   if(result!==((k==2||k==4||k==5)?8'h8b:8'h00))$fatal(1,"case %0d status %h",k,result);
   if(diagnostic[31:0]!==32'h43415031||diagnostic[63:32]!=W*H-(k==2?1:((k==4||k==5)?2:0))||!diagnostic[118]||diagnostic[191:160]!=0)
    $fatal(1,"capture diagnostic mismatch %h",diagnostic);
   for(i=0;i<W*H;i=i+1)if(result==0)begin
    if(mem[i]!==swap(16'h1200+k*64+i))
     $fatal(1,"case %0d pixel %0d got %h",k,i,mem[i]);
   end
   if(bad!==(k==2?6'd1:((k==4||k==5)?6'd2:6'd0)))$fatal(1,"short census case %0d bad=%0d",k,bad);
  end
  // An END whose count complement was damaged must not be published.
  target=responses+1;start_frame;
  fork
   send_frame(0,5,16'h5600);
   begin
    wait(cap.qv&&cap.qdata[287:286]==2);
    @(negedge clk);force cap.qdata[32]=cap.qdata[0];
    @(posedge clk);#1;release cap.qdata[32];
   end
  join
  wait(responses==target);#1;
  if(result!=8'h87||diagnostic[191:160]!=1||diagnostic[118])$fatal(1,"corrupt END accepted");
  $display("PASS camera chain: zero-gap VSYNC, RGB565 DDR data, damaged frames rejected, census, recovery, DDR stalls");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
