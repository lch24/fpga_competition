`timescale 1ns/1ps
// DVP faults can leave exactly the expected number of cropped pixels. Frame
// integrity must still reject them, and the next clean frame must recover.
module tb_camera_integrity;
 localparam W=64,H=8,CW=48,CH=6,X0=8,Y0=1;
 reg clk=0,pclk=0,rst=0,vs=0,href=0,cmd=0;
 reg [7:0] data=0;
 always #4 clk=~clk;
 always #5 pclk=~pclk;
 wire pv,cv,ready,rsp,av;wire [15:0] pixel;wire [5:0] short_lines;
 wire [3:0] faults;wire [7:0] status;wire [191:0] diagnostic;
 wire [255:0] wd;wire [27:0] addr;wire [31:0] strb;wire [3:0] len,id;
 camera_byte_unpack #(.WIDTH(W),.HEIGHT(H),.CROP_X(X0),.CROP_Y(Y0),.CROP_WIDTH(CW),.CROP_HEIGHT(CH)) front(
  .pclk(pclk),.rst_n(rst),.vsync(vs),.href(href),.data(data),
  .pixel_valid(pv),.pixel(pixel),.line_bad(short_lines),.frame_error(faults),.capture_vsync(cv));
 reg writing=0,complete=0;integer cycles=0,responses=0,words=0,j;
 reg [7:0] result;reg [27:0] saved_addr;
 reg [15:0] memory[0:CW*CH-1];
 wire awready=!writing&&!complete&&(cycles%7!=0);
 wire wready=writing&&(cycles%5!=0);
 video_capture_hmic #(.WIDTH(CW),.HEIGHT(CH),.FIFO_BITS(4)) cap(
  .clk(clk),.rst_n(rst),.pclk(pclk),.prst_n(rst),.vsync(cv),.pixel_valid(pv),.pixel(pixel),
  .line_short(short_lines),.source_frame_error(faults),
  .cmd_valid(cmd),.cmd_ready(ready),.cmd_base(32'd0),.rsp_valid(rsp),.rsp_ready(1'b1),
  .rsp_status(status),.rsp_base(),.skipped_frames(),.diagnostic(diagnostic),
  .axi_awaddr(addr),.axi_awlen(len),.axi_awuser_id(id),.axi_awvalid(av),.axi_awready(awready),
  .axi_wdata(wd),.axi_wstrb(strb),.axi_wready(wready),.axi_wusero_last(complete),.axi_wusero_id(4'd0));
 always @(posedge clk)if(rst)begin
  cycles<=cycles+1;
  if(av&&awready)begin writing<=1;saved_addr<=addr;end
  if(wready)begin
   for(j=0;j<16;j=j+1)memory[saved_addr*2+j]<=wd[j*16+:16];
   words<=words+1;writing<=0;complete<=1;
  end else complete<=0;
  if(rsp)begin result<=status;responses<=responses+1;end
 end
 task send_frame(input integer kind);
  integer row,b,bytes,rows;reg [15:0] raw;
  begin
   @(negedge clk);cmd=1;do @(posedge clk);while(!ready);@(negedge clk);cmd=0;
   repeat(8)@(negedge pclk);#1;vs=1;repeat(4)@(negedge pclk);#1;vs=0;
   rows=kind==5?H-1:H;
   for(row=0;row<rows;row=row+1)begin
    bytes=2*W;
    if(row==2)case(kind)
     1:bytes=2*(W-1); // missing tail lies outside retained columns
     2:bytes=2*(X0+CW-1); // one missing ROI pixel
     3:bytes=2*W-1; // odd byte count, ROI pixel count still complete
     4:bytes=2*(W+1); // extra pixel, cropped count still complete
    endcase
    for(b=0;b<bytes;b=b+1)begin
     raw=16'h1234+row*W+b/2;
     @(negedge pclk);#1;href=1;data=b%2?raw[7:0]:raw[15:8];
    end
    @(negedge pclk);#1;href=0;repeat(5)@(negedge pclk);
   end
   #1;vs=1;repeat(4)@(negedge pclk);#1;vs=0;
  end
 endtask
 integer k,expect_fault,expect_status,target,i;reg [15:0] raw;
 initial begin
  repeat(5)@(negedge clk);rst=1;
  for(k=0;k<7;k=k+1)begin
   target=responses+1;send_frame(k==6?0:k);wait(responses==target);#1;
   expect_fault=k==2?1:k==3?2:k==4?4:k==5?8:0;
   expect_status=k==2?'h8b:(expect_fault!=0?'h86:0);
   if(result!=expect_status||faults!=expect_fault||diagnostic[122:119]!=expect_fault)
    $fatal(1,"frame %0d status=%h faults=%h diag=%h",k,result,faults,diagnostic);
   if(k!=2 && diagnostic[63:32]!=CW*CH)$fatal(1,"unexpected cropped length");
   if(result==0)for(i=0;i<CW*CH;i=i+1)begin
    raw=16'h1234+(Y0+i/CW)*W+X0+i%CW;
    if(memory[i]!=={raw[4:0],raw[10:5],raw[15:11]})$fatal(1,"pixel %0d",i);
   end
  end
  $display("PASS camera integrity: clean/crop-tail/short/odd/long/missing-row/recovery");$finish;
 end
 initial begin #2000000;$fatal(1,"camera integrity timeout");end
endmodule
