`timescale 1ns/1ps
// No algorithm mocks: camera bytes -> HMIC -> gray -> real detector -> actual
// calibration collection failure -> response, followed by a fresh job.
module tb_vision_camera;
 reg clk=0,camera_pclk=0,rst_n=0;always #5 clk=~clk;always #7 camera_pclk=~camera_pclk;
 wire camera_rst_n=rst_n;reg ddr_ready=1,camera_vsync=0,camera_href=0;reg [7:0] camera_data=0;
 reg start_valid=0,capture_valid=0,rsp_ready=0;wire start_ready,capture_ready,rsp_valid,busy,ownership_error;
 reg [63:0] square_size_fp64=64'h3ff0000000000000;
 wire [7:0] rsp_status,debug_view;wire [31:0] debug_job,result_base,result_stride;
 wire [15:0] result_width,result_height;wire [5:0] debug_phase;wire [287:0] result_params;wire [63:0] result_rms;
 wire [27:0] axi_araddr,axi_awaddr;wire [3:0] axi_aruser_id,axi_awuser_id,axi_arlen,axi_awlen;
 wire axi_aruser_ap,axi_awuser_ap,axi_arvalid,axi_awvalid;
 reg axi_arready=0,axi_awready=0,axi_rvalid=0,axi_wready=0,axi_wusero_last=0;
 reg [255:0] axi_rdata;wire [255:0] axi_wdata;wire [31:0] axi_wstrb;
 wire axi_rlast=1'b1;wire [3:0] axi_rid=0,axi_wusero_id=0;
 vision_camera_top #(.WIDTH(32),.HEIGHT(32),.DEPTH(1),.RAW_BASE(0),.SLOT_BYTES(4096),
  .DST_BASE(8192),.GRAY_BASE(12288),.MAP_X_BASE(16384),.MAP_Y_BASE(20480)) dut(.*);
 reg [7:0] mem[0:32767];integer rdelay=-1,wdelay=-1,bdelay=-1,ra,wa,k,i,n,jobn,reads=0,writes=0;
 reg [15:0] random_state=16'h8231;
 always @(posedge clk)if(rst_n)begin
  random_state<={random_state[14:0],random_state[15]^random_state[13]^random_state[12]^random_state[10]};
  axi_arready<=random_state[0]&&rdelay<0;axi_awready<=random_state[1]&&wdelay<0;
  axi_rvalid<=0;axi_wready<=0;axi_wusero_last<=0;
  if(axi_arvalid&&axi_arready)begin ra=axi_araddr*4;rdelay<=3;reads<=reads+1;if(axi_arlen!=0||ra>=32768)$fatal(1,"physical read");end
  if(rdelay>0)rdelay<=rdelay-1;
  if(rdelay==0)begin for(k=0;k<32;k=k+1)axi_rdata[k*8+:8]<=mem[ra+k];axi_rvalid<=1;rdelay<=-1;end
  if(axi_awvalid&&axi_awready)begin wa=axi_awaddr*4;wdelay<=3;writes<=writes+1;if(axi_awlen!=0||wa>=32768)$fatal(1,"physical write");end
  if(wdelay>0)wdelay<=wdelay-1;
  if(wdelay==0)begin axi_wready<=1;wdelay<=-1;end
  if(axi_wready)begin for(k=0;k<32;k=k+1)if(axi_wstrb[k])mem[wa+k]<=axi_wdata[k*8+:8];bdelay<=3;end
  if(bdelay>0)bdelay<=bdelay-1;
  if(bdelay==0)begin axi_wusero_last<=1;bdelay<=-1;end
 end
 task launch;
  begin @(negedge clk);start_valid=1;do @(posedge clk);while(!start_ready);@(negedge clk);start_valid=0;end
 endtask
 initial begin
  for(i=0;i<32768;i=i+1)mem[i]=8'ha5;
  repeat(8)@(negedge clk);rst_n=1;
  for(jobn=0;jobn<2;jobn=jobn+1)begin
   launch();wait(capture_ready);@(negedge clk);capture_valid=1;@(negedge clk);capture_valid=0;
   repeat(20)@(negedge camera_pclk);camera_vsync=1;repeat(4)@(negedge camera_pclk);camera_vsync=0;
   for(n=0;n<1024;n=n+1)begin
    camera_href=1;camera_data=0;repeat(2)@(negedge camera_pclk);camera_href=0;
    // Slow test camera allows the real serial physical DDR adapter to drain.
    repeat(20)@(negedge camera_pclk);
   end
   camera_vsync=1;repeat(4)@(negedge camera_pclk);camera_vsync=0;
   wait(rsp_valid);repeat(8)@(negedge clk);
   if(rsp_status!==2||ownership_error||capture_ready)$fatal(1,"real blank frame result status=%d phase=%d",rsp_status,debug_phase);
   for(i=0;i<2048;i=i+1)if(mem[i]!==0)$fatal(1,"camera raw pixel");
   for(i=0;i<1024;i=i+1)if(mem[12288+i]!==0)$fatal(1,"gray pixel");
   for(i=0;i<2048;i=i+1)if(mem[8192+i]!==8'ha5)$fatal(1,"failed calibration wrote output");
   rsp_ready=1;@(negedge clk);rsp_ready=0;repeat(4)@(negedge clk);
  end
  if(reads==0||writes==0)$fatal(1,"no physical DDR traffic");
  $display("PASS vision camera: two actual captured blank frames, physical DDR, real detection, NO_BOARD, release and retry (no algorithm mocks)");$finish;
 end
 initial begin #20000000;
  $display("DEBUG phase=%d cap=%d det=%d hm=%d rdv/rdy=%b%b rv/rdy=%b%b c_req=%b c_ret=%b",debug_phase,dut.state,dut.pipeline.detector.u_frame.st,dut.storage.adapter.state,
   dut.pipeline.rd_valid,dut.pipeline.rd_ready,dut.pipeline.r_valid,dut.pipeline.r_ready,dut.pipeline.c_rd_valid,dut.pipeline.c_r_valid);
  $display("DEBUG physical av/ar=%b%b rv=%b addr=%h reads=%d writes=%d",axi_arvalid,axi_arready,axi_rvalid,axi_araddr,reads,writes);
  $display("DEBUG fetch busy/done=%b%b dma busy/done=%b%b rows=%d bytes=%d y=%d readstate=%d ar=%b",
   dut.pipeline.detector.u_fetch.busy,dut.pipeline.detector.u_fetch.done,dut.pipeline.detector.u_fetch.u_dma.busy,dut.pipeline.detector.u_fetch.u_dma.done,
   dut.pipeline.detector.u_fetch.u_dma.cfg_rows_r,dut.pipeline.detector.u_fetch.u_dma.cfg_row_bytes_r,dut.pipeline.detector.u_fetch.u_dma.y_r,dut.pipeline.detector.u_fetch.u_dma.rd_state,dut.pipeline.ar);
  $fatal(1,"camera timeout");end
endmodule
