`timescale 1ns/1ps
module tb_camera_system;
 reg clk=0,pclk=0,hclk=0;always #5 clk=~clk;always #7 pclk=~pclk;always #11 hclk=~hclk;
 reg rst_n=0,cv=0,ch=0;reg [7:0] cd=0;reg de=0,vs=0;
 reg cmd_valid=0,rsp_ready=0;wire cmd_ready,rsp_valid;reg [2:0] opcode=0;
 reg [31:0] job=1;reg [15:0] width=4,height=3;wire [7:0] status;
 wire [7:0] hr,hg,hb;wire underflow,ownership_error,map_valid;
 reg pin_ready=0,release_valid=0;reg [3:0] release_slot=0;
 wire pin_valid,release_ready;wire [3:0] pin_slot;wire [31:0] pin_base;
 wire [15:0] pin_width,pin_height;
 wire [27:0] aa,wa;wire [3:0] aid,wid,alen,wlen;
 wire av,wav,ap,wap;reg ar=0,awr=0,rv=0,wr=0,wdone=0;
 reg [255:0] rd;wire [255:0] wd;wire [31:0] mask;
 reg ext_valid=0,ext_ready_data=0;wire ext_ready,ext_rv;wire [31:0] ext_data;
 wire [3:0] ext_keep;wire [15:0] ext_tag;wire ext_last,ext_error;
 camera_system_top #(.RAW_BASE(0),.OUTPUT_BASE(1024),.SLOT_BYTES(256),.RAW_SLOTS(3),.OUTPUT_SLOTS(2),
 .MAP_X_BASE(2048),.MAP_Y_BASE(2560),.MAP_CAPACITY(512)) dut(
 .clk(clk),.rst_n(rst_n),.ddr_ready(1'b1),
 .camera_pclk(pclk),.camera_rst_n(rst_n),.camera_vsync(cv),.camera_href(ch),.camera_data(cd),
 .hdmi_pclk(hclk),.hdmi_rst_n(rst_n),.hdmi_de(de),.hdmi_vsync(vs),.clear_hdmi_error(1'b0),
 .hdmi_r(hr),.hdmi_g(hg),.hdmi_b(hb),.hdmi_underflow(underflow),
 .cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_opcode(opcode),.cmd_job_id(job),.cmd_calib_id(32'd42),
 .cmd_camera_valid(1'b1),.cmd_params({224'd0,32'h40800000,32'h40800000}),.cmd_width(width),.cmd_height(height),
 .cmd_border_replicate(1'b0),.rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(status),
 .map_valid(map_valid),.ownership_error(ownership_error),
 .raw_pin_valid(pin_valid),.raw_pin_ready(pin_ready),.raw_pin_slot(pin_slot),.raw_pin_base(pin_base),
 .raw_pin_width(pin_width),.raw_pin_height(pin_height),.raw_release_valid(release_valid),.raw_release_ready(release_ready),.raw_release_slot(release_slot),
 .ext_rd_valid(ext_valid),.ext_rd_ready(ext_ready),.ext_rd_addr(32'd0),.ext_rd_len(32'd2),.ext_rd_tag(16'hbeef),
 .ext_r_valid(ext_rv),.ext_r_ready(ext_ready_data),.ext_r_data(ext_data),.ext_r_keep(ext_keep),.ext_r_tag(ext_tag),.ext_r_last(ext_last),.ext_r_error(ext_error),
 .ext_wr_valid(1'b0),.ext_wr_addr(32'd0),.ext_wr_len(32'd0),.ext_wr_tag(16'd0),.ext_w_valid(1'b0),.ext_w_data(32'd0),.ext_w_keep(4'd0),.ext_w_last(1'b0),.ext_b_ready(1'b1),
 .axi_araddr(aa),.axi_awaddr(wa),.axi_aruser_id(aid),.axi_awuser_id(wid),.axi_arlen(alen),.axi_awlen(wlen),
 .axi_aruser_ap(ap),.axi_awuser_ap(wap),.axi_arvalid(av),.axi_awvalid(wav),.axi_arready(ar),.axi_awready(awr),
 .axi_rdata(rd),.axi_rvalid(rv),.axi_rlast(1'b1),.axi_rid(4'd0),.axi_wdata(wd),.axi_wstrb(mask),
 .axi_wready(wr),.axi_wusero_last(wdone),.axi_wusero_id(4'd0));
 reg [7:0] mem[0:4095];reg [15:0] pixels[0:11];reg [15:0] random_state=16'hac43;
 integer ar_delay=-1,aw_delay=-1,done_delay=-1,raddr,waddr,k,i;
 always @(posedge clk)if(rst_n)begin
  random_state<={random_state[14:0],random_state[15]^random_state[13]^random_state[12]^random_state[10]};
  ar<=random_state[0]&&ar_delay<0;awr<=random_state[1]&&aw_delay<0;
  rv<=0;wr<=0;wdone<=0;
  if(av&&ar)begin if(alen!=0)$fatal(1,"unexpected burst");raddr=aa*4;ar_delay<=3;end
  if(ar_delay>0)ar_delay<=ar_delay-1;
  if(ar_delay==0)begin for(k=0;k<32;k=k+1)rd[k*8+:8]<=mem[raddr+k];rv<=1;ar_delay<=-1;end
  if(wav&&awr)begin if(wlen!=0)$fatal(1,"unexpected write burst");waddr=wa*4;aw_delay<=3;end
  if(aw_delay>0)aw_delay<=aw_delay-1;
  if(aw_delay==0)begin wr<=1;aw_delay<=-1;end
  if(wr)begin for(k=0;k<32;k=k+1)if(mask[k])mem[waddr+k]<=wd[k*8+:8];done_delay<=3;end
  if(done_delay>0)done_delay<=done_delay-1;
  if(done_delay==0)begin wdone<=1;done_delay<=-1;end
 end
 task command(input integer op);
  begin @(negedge clk);opcode=op;job=job+1;cmd_valid=1;do @(posedge clk);while(!cmd_ready);@(negedge clk);cmd_valid=0;end
 endtask
 task response(input integer expected);
  begin wait(rsp_valid);repeat(3)@(negedge clk);if(status!==expected)$fatal(1,"command %d status %d expected %d",opcode,status,expected);
   rsp_ready=1;@(negedge clk);rsp_ready=0;end
 endtask
 task camera_frame;
  reg [15:0] incoming;integer n;
  begin
   repeat(8)@(negedge pclk);cv=1;repeat(3)@(negedge pclk);cv=0;
   for(n=0;n<12;n=n+1)begin
    incoming={pixels[n][4:0],pixels[n][10:5],pixels[n][15:11]};
    ch=1;cd=incoming[15:8];@(negedge pclk);cd=incoming[7:0];@(negedge pclk);
    if(n%4==3)begin ch=0;repeat(2)@(negedge pclk);end
   end
   ch=0;repeat(8)@(negedge pclk);cv=1;repeat(3)@(negedge pclk);cv=0;
  end
 endtask
 initial begin
  for(i=0;i<4096;i=i+1)mem[i]=8'ha5;
  for(i=0;i<12;i=i+1)pixels[i]=16'h1234+i*16'h1041;
  repeat(5)@(negedge clk);rst_n=1;
  command(3);response(3); // no raw frame
  command(1);response(0);if(!map_valid)$fatal(1,"map publication");
  command(2);camera_frame();response(0);
  for(i=0;i<12;i=i+1)if({mem[i*2+1],mem[i*2]}!==pixels[i])$fatal(1,"camera packing %d",i);
  if(!pin_valid||pin_width!=4||pin_height!=3)$fatal(1,"frame descriptor");
  // Hold external DDR response back while remap is submitted: the arbiter must
  // route its tag/data only to the detector and then allow correction to resume.
  @(negedge clk);ext_valid=1;do @(posedge clk);while(!ext_ready);@(negedge clk);ext_valid=0;
  command(3);wait(ext_rv);repeat(4)@(negedge clk);
  if(ext_tag!=16'hbeef||ext_data[15:0]!==pixels[0]||ext_keep!=3||!ext_last||ext_error)$fatal(1,"external client route");
  ext_ready_data=1;@(negedge clk);ext_ready_data=0;response(0);
  for(i=0;i<12;i=i+1)if({mem[1024+i*2+1],mem[1024+i*2]}!==pixels[i])$fatal(1,"corrected image %d",i);
  command(4);response(0);repeat(6)@(negedge hclk);vs=1;repeat(2)@(negedge hclk);vs=0;
  // All twelve pixels fit in the FIFO before display starts in this test mode.
  for(i=0;i<12;i=i+1)begin
   de=1;#1;
   if({hr,hg,hb}!=={pixels[i][15:11],3'd0,pixels[i][10:5],2'd0,pixels[i][4:0],3'd0})$fatal(1,"HDMI pixel %d",i);
   @(negedge hclk);if(i%4==3)begin de=0;repeat(2)@(negedge hclk);end
  end
  de=0;if(underflow||ownership_error)$fatal(1,"video/ownership error");
  // Detector pins another raw frame. A correction cannot reuse its READING slot.
  command(2);camera_frame();response(0);wait(pin_valid);release_slot=pin_slot;
  @(negedge clk);pin_ready=1;@(negedge clk);pin_ready=0;
  command(3);response(3);
  @(negedge clk);release_valid=1;do @(posedge clk);while(!release_ready);@(negedge clk);release_valid=0;
  command(2);camera_frame();response(0);width=3;command(3);response(1);width=4;
  if(ownership_error)$fatal(1,"ownership after pin/release");
  $display("PASS camera system: camera CDC -> HMIC -> map -> correction -> HDMI, detector arbitration and frame ownership");$finish;
 end
 initial begin #5000000;$fatal(1,"camera system timeout state %d",dut.state);end
endmodule
