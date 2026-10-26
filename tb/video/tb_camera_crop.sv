`timescale 1ns/1ps
module tb_camera_crop;
 parameter W=64,H=16;
 localparam CW=W*3/4,CH=H*3/4,X0=(W-CW)/2,Y0=(H-CH)/2;
 reg clk=0,pclk=0,rst=0,vs=0,href=0;reg [7:0] data=0;
 always #4 clk=~clk;always #5 pclk=~pclk;
 wire pv,cv;wire [15:0] pixel;wire [5:0] short_lines;wire [3:0] frame_error;
 camera_byte_unpack #(.WIDTH(W),.HEIGHT(H),.CROP_X(X0),.CROP_Y(Y0),.CROP_WIDTH(CW),.CROP_HEIGHT(CH)) front(
 .pclk(pclk),.rst_n(rst),.vsync(vs),.href(href),.data(data),.pixel_valid(pv),.pixel(pixel),.line_bad(short_lines),.frame_error(frame_error),.capture_vsync(cv));
 reg cmd=0;wire ready,rsp;wire [7:0] status;wire [191:0] diag;
 wire [27:0] addr;wire [3:0] len,id;wire av;wire [255:0] wd;wire [31:0] strb;
 reg busy=0,done=0;reg [27:0] saved_addr;integer cycle=0,words=0,responses=0,j;
 wire ar=!busy&&!done&&cycle%7!=0,wr=busy&&cycle%5!=0;
 video_capture_hmic #(.WIDTH(CW),.HEIGHT(CH)) cap(
 .clk(clk),.rst_n(rst),.pclk(pclk),.prst_n(rst),.vsync(cv),.pixel_valid(pv),.pixel(pixel),.line_short(short_lines),
 .source_frame_error(frame_error),
 .cmd_valid(cmd),.cmd_ready(ready),.cmd_base(32'h200000),.rsp_valid(rsp),.rsp_ready(1'b1),.rsp_status(status),.rsp_base(),.skipped_frames(),.diagnostic(diag),
 .axi_awaddr(addr),.axi_awlen(len),.axi_awuser_id(id),.axi_awvalid(av),.axi_awready(ar),.axi_wdata(wd),.axi_wstrb(strb),
 .axi_wready(wr),.axi_wusero_last(done),.axi_wusero_id(4'd0));
 function [15:0] source(input integer x,y);
  reg [15:0] v;begin v=(y*W+x)^16'hb439;source={v[4:0],v[10:5],v[15:11]};end
 endfunction
 integer n=0,index;reg score=0;
 always @(posedge clk)if(rst)begin
  cycle<=cycle+1;
  if(av&&ar)begin saved_addr<=addr;busy<=1;end
  if(wr)begin
   if(saved_addr!=32'h80000+words*8||len!=0||strb!=32'hffffffff)$fatal(1,"DDR crop layout");
   for(j=0;j<16;j=j+1)begin
    index=words*16+j;
    if(wd[j*16+:16]!==source(X0+index%CW,Y0+index/CW))$fatal(1,"crop DDR pixel %0d",index);
   end
   words<=words+1;busy<=0;done<=1;
  end else if(done)done<=0;
  if(rsp)begin
   if(status!=0||diag[63:32]!=CW*CH||diag[95:64]!=CW*CH/16)$fatal(1,"crop status/count %h",status);
   responses<=responses+1;
  end
 end
 always @(negedge pclk)if(score&&pv)begin
  if(pixel!==source(X0+n%CW,Y0+n/CW))$fatal(1,"crop stream index %0d",n);
  n=n+1;
 end
 integer x,y,f,limit;reg [15:0] raw;
 initial begin
  repeat(5)@(negedge clk);rst=1;
  for(f=0;f<2;f=f+1)begin
   words=0;n=0;
   @(negedge clk);cmd=1;do @(posedge clk);while(!ready);@(negedge clk);cmd=0;
   repeat(8)@(negedge pclk);#1;vs=1;repeat(4)@(negedge pclk);#1;vs=0;score=1;
   for(y=0;y<H;y=y+1)begin
    // Short rows end outside the kept ROI; next source row must not shift.
    limit=W-((f==1&&(y==0||y==Y0+1))?1:0);
    for(x=0;x<limit;x=x+1)begin
     raw=(y*W+x)^16'hb439;
     @(negedge pclk);#1;href=1;data=raw[15:8];
     @(negedge pclk);#1;data=raw[7:0];
    end
    @(negedge pclk);#1;href=0;repeat(8)@(negedge pclk);
   end
   #1;vs=1;repeat(4)@(negedge pclk);#1;vs=0;
   wait(responses==f+1);#1;score=0;
   if(n!=CW*CH||words!=CW*CH/16)$fatal(1,"crop length n=%0d words=%0d",n,words);
  end
  $display("PASS camera crop %0dx%0d -> %0dx%0d, exact DDR data, source short rows, stalls",W,H,CW,CH);$finish;
 end
 initial begin #100000000;$fatal(1,"crop timeout");end
endmodule
