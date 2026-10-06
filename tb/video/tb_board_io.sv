`timescale 1ns/1ps
module tb_board_io;
 reg clk=0,pclk=0,rst=0;
 always #5 clk=~clk;always #7 pclk=~pclk;
 reg cmd=0,vs=0,pv=0;reg [15:0] pix=0;
 wire cr,rv;reg rr=0;wire [7:0] status;
 wire [27:0] awaddr;wire [3:0] awlen,awid;wire awvalid;
 reg awready=0,wready=0,wlast=0;reg [3:0] wid=0;
 wire [255:0] wdata;wire [31:0] mask;
 reg stall=0;integer writes=0,delay_count=0,writer_state=0,k,j,expected=0;
 reg [27:0] saved_addr;
 board_capture_hmic #(.WIDTH(64),.HEIGHT(2),.BASE(32'h100),.FIFO_BITS(2)) cap(
  .clk(clk),.rst_n(rst),.pclk(pclk),.prst_n(rst),.vsync(vs),.pixel_valid(pv),.pixel(pix),
  .cmd_valid(cmd),.cmd_ready(cr),.rsp_valid(rv),.rsp_ready(rr),.rsp_status(status),
  .axi_awaddr(awaddr),.axi_awlen(awlen),.axi_awuser_id(awid),.axi_awvalid(awvalid),.axi_awready(awready),
  .axi_wdata(wdata),.axi_wstrb(mask),.axi_wready(wready),.axi_wusero_last(wlast),.axi_wusero_id(wid));
 always @(negedge clk)begin
  awready=0;wready=0;wlast=0;
  if(!rst)begin writer_state=0;writes=0;end
  else case(writer_state)
   0:if(awvalid&&!stall&&$urandom_range(0,2)!=0)begin
    awready=1;saved_addr=awaddr;
    if(awlen!=0||awid!=0||mask!==32'hffffffff)$fatal(1,"capture descriptor");
    if(awaddr!==28'h40+8*writes)$fatal(1,"capture address");
    writer_state=1;
   end
   1:if($urandom_range(0,2)!=0)begin
    wready=1;
    if(expected==0)for(j=0;j<16;j=j+1)
     if(wdata[j*16+:16]!==16'(writes*16+j))$fatal(1,"capture pixel %0d",writes*16+j);
    writes=writes+1;delay_count=3;writer_state=2;
   end
   2:if(delay_count==0)begin wlast=1;writer_state=0;end else delay_count=delay_count-1;
  endcase
 end
 task start_capture;
  begin
   @(negedge clk);cmd=1;wait(cr);@(negedge clk);cmd=0;
   repeat(8)@(negedge pclk);vs=1;repeat(3)@(negedge pclk);vs=0;
  end
 endtask
 task frame(input integer n);
  integer i;begin
   for(i=0;i<n;i=i+1)begin @(negedge pclk);pv=1;pix=i;end
   @(negedge pclk);pv=0;repeat(4)@(negedge pclk);vs=1;
   repeat(3)@(negedge pclk);vs=0;
  end
 endtask
 task take_response(input integer code);
  begin wait(rv);if(status!=code)$fatal(1,"capture status expected %0d got %0d",code,status);
   repeat(5)begin @(negedge clk);if(!rv||status!=code)$fatal(1,"response hold");end
   rr=1;@(negedge clk);rr=0;
  end
 endtask
 // Display model supports true multi-beat read bursts and randomized stalls.
 reg display_enable=0,pr=0;wire dv,df,dl,de;wire [15:0] dp;
 wire [27:0] araddr;wire [3:0] arlen,arid;wire arvalid;
 reg arready=0,rvalid=0,rlast=0;reg [3:0] rid=0;reg [255:0] rdata=0;
 integer left=0,beat=0,read_base=0,outputs=0,col,row,index,ex,frames=0;
 board_display_hmic #(.WIDTH(1280),.HEIGHT(2),.RAW_BASE(0),.DST_BASE(32'h10000)) disp(
  .clk(clk),.rst_n(rst),.enable(display_enable),.pixel_valid(dv),.pixel_ready(pr),.pixel(dp),.pixel_first(df),.pixel_last(dl),.error(de),
  .axi_araddr(araddr),.axi_arlen(arlen),.axi_aruser_id(arid),.axi_arvalid(arvalid),.axi_arready(arready),
  .axi_rdata(rdata),.axi_rvalid(rvalid),.axi_rlast(rlast),.axi_rid(rid));
 function [15:0] memory_pixel(input integer address);
  begin memory_pixel=address>=65536?16'h8000+((address-65536)/2):address/2;end
 endfunction
 always @(negedge clk)begin
  arready=0;rvalid=0;rlast=0;pr=$urandom_range(0,3)!=0;
  if(rst)begin
   if(left>0)begin
    if($urandom_range(0,3)!=0)begin
     rvalid=1;rlast=left==1;
     for(k=0;k<16;k=k+1)rdata[k*16+:16]=memory_pixel(read_base+beat*32+k*2);
     left=left-1;beat=beat+1;
    end
   end else if(arvalid&&$urandom_range(0,2)!=0)begin
    arready=1;left=arlen+1;read_base=araddr*4;beat=0;
    if(arid!=0||read_base%32!=0)$fatal(1,"display descriptor");
   end
  end
 end
 always @(posedge clk)if(rst&&dv&&pr)begin
  col=outputs%1280;row=(outputs/1280)%2;
  ex=col<640?row*1280+col:16'h8000+row*1280+col-640;
  if(col==640)ex=65535;
  if(dp!==16'(ex)||df!==(outputs%2560==0)||dl!==(outputs%2560==2559))$fatal(1,"display pixel %0d got %h expected %h",outputs,dp,ex);
  outputs=outputs+1;
 end
 initial begin
  repeat(6)@(negedge clk);rst=1;
  start_capture();frame(128);take_response(0);
  if(writes!=8)$fatal(1,"capture writes");
  @(negedge clk);writes=0;expected=1;
  start_capture();frame(117);take_response(3);
  @(negedge clk);writes=0;stall=1;
  start_capture();frame(128);stall=0;take_response(3);
  @(negedge clk);display_enable=1;
  wait(outputs>=5120);if(de)$fatal(1,"display error");
  // Malformed response must drain to RLAST then stop, without publishing it.
  rid=1;wait(de);repeat(600)@(negedge clk);
  if(dv||arvalid)$fatal(1,"display did not stop after bad RID");
  $display("PASS tb_board_io packed capture, overflow, partial frame, delayed completion, 5120 display pixels, burst error");$finish;
 end
 initial begin #3000000;$fatal(1,"timeout board IO");end
endmodule
