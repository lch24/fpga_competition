`timescale 1ns/1ps
module tb_board_flow_cdc;
reg core_clk=0,algorithm_clk=0,core_rst_n=0,algorithm_rst_n=0;
always #4 core_clk=~core_clk;always #12.5 algorithm_clk=~algorithm_clk;
reg finished=0;integer captures=0,n;reg cap_rsp_valid=0;
wire cap_cmd_valid,cap_rsp_ready,capture_owner,display_enable;
wire cap_cmd_ready=1;
wire [7:0] status;wire waiting;wire [3:0] phase;
wire a_start_valid;
wire start_valid;
reg a_start_ready=0;
wire start_ready;
wire a_frame_valid;
wire frame_valid;
reg a_frame_ready=0;
wire frame_ready;
wire [7:0] a_frame_status;
wire [7:0] frame_status;
reg a_release_valid=0;
wire release_valid;
wire a_release_ready;
wire release_ready;
reg a_rsp_valid=0;
wire rsp_valid;
wire a_rsp_ready;
wire rsp_ready;
reg [7:0] a_rsp_status=0;
wire [7:0] rsp_status;
reg a_rd_valid=0;
wire rd_valid;
wire a_rd_ready;
reg rd_ready=0;
reg [31:0] a_rd_addr=0;
wire [31:0] rd_addr;
reg [31:0] a_rd_len=0;
wire [31:0] rd_len;
reg [15:0] a_rd_tag=0;
wire [15:0] rd_tag;
wire a_r_valid;
reg r_valid=0;
reg a_r_ready=0;
wire r_ready;
wire [31:0] a_r_data;
reg [31:0] r_data=0;
wire [3:0] a_r_keep;
reg [3:0] r_keep=0;
wire [15:0] a_r_tag;
reg [15:0] r_tag=0;
wire a_r_last;
reg r_last=0;
wire a_r_error;
reg r_error=0;
reg a_wr_valid=0;
wire wr_valid;
wire a_wr_ready;
reg wr_ready=0;
reg [31:0] a_wr_addr=0;
wire [31:0] wr_addr;
reg [31:0] a_wr_len=0;
wire [31:0] wr_len;
reg [15:0] a_wr_tag=0;
wire [15:0] wr_tag;
reg a_w_valid=0;
wire w_valid;
wire a_w_ready;
reg w_ready=0;
reg [31:0] a_w_data=0;
wire [31:0] w_data;
reg [3:0] a_w_keep=0;
wire [3:0] w_keep;
reg a_w_last=0;
wire w_last;
wire a_b_valid;
reg b_valid=0;
reg a_b_ready=0;
wire b_ready;
wire [15:0] a_b_tag;
reg [15:0] b_tag=0;
wire a_b_error;
reg b_error=0;
vision_clock_bridge dut(.*);
board_flow #(.CAPTURE_WAIT_CYCLES(3)) flow(.clk(core_clk),.rst_n(core_rst_n),.camera_ready(1'b1),
 .start_valid(start_valid),.start_ready(start_ready),.frame_valid(frame_valid),.frame_ready(frame_ready),.frame_status(frame_status),
 .frame_release_valid(release_valid),.frame_release_ready(release_ready),
 .cap_cmd_valid(cap_cmd_valid),.cap_cmd_ready(cap_cmd_ready),.cap_rsp_valid(cap_rsp_valid),.cap_rsp_ready(cap_rsp_ready),.cap_status(8'd0),
 .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),
 .capture_owner(capture_owner),.display_enable(display_enable),.status(status),.waiting(waiting),.phase(phase));
initial forever begin
 @(posedge core_clk);
 if(core_rst_n && cap_cmd_valid && cap_cmd_ready)begin
  captures=captures+1;repeat(7)@(negedge core_clk);cap_rsp_valid=1;
  do @(posedge core_clk);while(!cap_rsp_ready);
  @(negedge core_clk);cap_rsp_valid=0;
 end
end
initial begin
 repeat(5)@(negedge core_clk);core_rst_n=1;
 repeat(3)@(negedge algorithm_clk);algorithm_rst_n=1;a_start_ready=1;
 do @(posedge algorithm_clk);while(!a_start_valid);
 @(negedge algorithm_clk);a_start_ready=0;
 for(n=1;n<=3;n=n+1)begin
  repeat(20)@(negedge algorithm_clk);
  if(captures!=n-1)$fatal(1,"capture without algorithm frame demand");
  a_frame_ready=1;
  do @(posedge algorithm_clk);while(!a_frame_valid);
  if(a_frame_status!=0)$fatal(1,"frame status corrupted");
  @(negedge algorithm_clk);a_frame_ready=0;
  repeat(20)@(negedge algorithm_clk);
  if(captures!=n)$fatal(1,"lease failed: unexpected capture");
  a_release_valid=1;
  do @(posedge algorithm_clk);while(!a_release_ready);
  @(negedge algorithm_clk);a_release_valid=0;
 end
 a_rsp_status=0;a_rsp_valid=1;
 do @(posedge algorithm_clk);while(!a_rsp_ready);
 @(negedge algorithm_clk);a_rsp_valid=0;
 wait(display_enable);repeat(20)@(negedge core_clk);
 if(captures!=3 || capture_owner || status!=0)$fatal(1,"ownership handoff failed");
 finished=1;$display("PASS board flow CDC: three leases, demand gating, display handoff");$finish;
end
initial begin #100000;$fatal(1,"flow timeout");end
endmodule
