`timescale 1ns/1ps
module tb_vision_clock_bridge;
 reg core_clk=0,algorithm_clk=0,core_rst_n=0,algorithm_rst_n=0;
 always #4 core_clk=~core_clk;
 initial begin #1.3;forever #12.5 algorithm_clk=~algorithm_clk;end
 integer checked=0;reg finished=0;
wire a_start_valid;
reg start_valid=0;
reg a_start_ready=0;
wire start_ready;
wire a_frame_valid;
reg frame_valid=0;
reg a_frame_ready=0;
wire frame_ready;
wire [7:0] a_frame_status;
reg [7:0] frame_status=0;
reg a_release_valid=0;
wire release_valid;
wire a_release_ready;
reg release_ready=0;
reg a_rsp_valid=0;
wire rsp_valid;
wire a_rsp_ready;
reg rsp_ready=0;
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
integer sent_start=0,got_start=0; reg take_start=0; reg [0:0] expect_start[0:255];
always @(negedge core_clk) begin
 if(!core_rst_n)begin start_valid<=0;sent_start=0;end
 else if(!start_valid || take_start)begin
  start_valid <= sent_start<200 && ($random & 3)!=0;
 end
end
always @(posedge core_clk) take_start <= core_rst_n && start_valid && start_ready;
always @(posedge core_clk)if(core_rst_n && start_valid && start_ready)begin
 expect_start[sent_start]=1'b0;sent_start=sent_start+1;
end
always @(negedge algorithm_clk)begin
 if(!algorithm_rst_n)a_start_ready<=0;
 else a_start_ready<=($random & 7)!=0;
end
always @(posedge algorithm_clk)begin
 if(!algorithm_rst_n)got_start=0;
 else if(a_start_valid && a_start_ready)begin
 got_start=got_start+1;checked=checked+1; if(got_start>sent_start)$fatal(1,"CDC start duplicate");
 end
end
integer sent_frame=0,got_frame=0; reg take_frame=0; reg [7:0] expect_frame[0:255];
always @(negedge core_clk) begin
 if(!core_rst_n)begin frame_valid<=0;sent_frame=0;end
 else if(!frame_valid || take_frame)begin
  frame_valid <= sent_frame<200 && ($random & 3)!=0;
  frame_status <= {$random,$random,$random};
 end
end
always @(posedge core_clk) take_frame <= core_rst_n && frame_valid && frame_ready;
always @(posedge core_clk)if(core_rst_n && frame_valid && frame_ready)begin
 expect_frame[sent_frame]=frame_status;sent_frame=sent_frame+1;
end
always @(negedge algorithm_clk)begin
 if(!algorithm_rst_n)a_frame_ready<=0;
 else a_frame_ready<=($random & 7)!=0;
end
always @(posedge algorithm_clk)begin
 if(!algorithm_rst_n)got_frame=0;
 else if(a_frame_valid && a_frame_ready)begin
 if(a_frame_status !== expect_frame[got_frame])$fatal(1,"CDC frame data/order mismatch %0d",got_frame);
 got_frame=got_frame+1;checked=checked+1; if(got_frame>sent_frame)$fatal(1,"CDC frame duplicate");
 end
end
integer sent_release=0,got_release=0; reg take_release=0; reg [0:0] expect_release[0:255];
always @(negedge algorithm_clk) begin
 if(!algorithm_rst_n)begin a_release_valid<=0;sent_release=0;end
 else if(!a_release_valid || take_release)begin
  a_release_valid <= sent_release<200 && ($random & 3)!=0;
 end
end
always @(posedge algorithm_clk) take_release <= algorithm_rst_n && a_release_valid && a_release_ready;
always @(posedge algorithm_clk)if(algorithm_rst_n && a_release_valid && a_release_ready)begin
 expect_release[sent_release]=1'b0;sent_release=sent_release+1;
end
always @(negedge core_clk)begin
 if(!core_rst_n)release_ready<=0;
 else release_ready<=($random & 7)!=0;
end
always @(posedge core_clk)begin
 if(!core_rst_n)got_release=0;
 else if(release_valid && release_ready)begin
 got_release=got_release+1;checked=checked+1; if(got_release>sent_release)$fatal(1,"CDC release duplicate");
 end
end
integer sent_result=0,got_result=0; reg take_result=0; reg [7:0] expect_result[0:255];
always @(negedge algorithm_clk) begin
 if(!algorithm_rst_n)begin a_rsp_valid<=0;sent_result=0;end
 else if(!a_rsp_valid || take_result)begin
  a_rsp_valid <= sent_result<200 && ($random & 3)!=0;
  a_rsp_status <= {$random,$random,$random};
 end
end
always @(posedge algorithm_clk) take_result <= algorithm_rst_n && a_rsp_valid && a_rsp_ready;
always @(posedge algorithm_clk)if(algorithm_rst_n && a_rsp_valid && a_rsp_ready)begin
 expect_result[sent_result]=a_rsp_status;sent_result=sent_result+1;
end
always @(negedge core_clk)begin
 if(!core_rst_n)rsp_ready<=0;
 else rsp_ready<=($random & 7)!=0;
end
always @(posedge core_clk)begin
 if(!core_rst_n)got_result=0;
 else if(rsp_valid && rsp_ready)begin
 if(rsp_status !== expect_result[got_result])$fatal(1,"CDC result data/order mismatch %0d",got_result);
 got_result=got_result+1;checked=checked+1; if(got_result>sent_result)$fatal(1,"CDC result duplicate");
 end
end
integer sent_read_cmd=0,got_read_cmd=0; reg take_read_cmd=0; reg [79:0] expect_read_cmd[0:255];
always @(negedge algorithm_clk) begin
 if(!algorithm_rst_n)begin a_rd_valid<=0;sent_read_cmd=0;end
 else if(!a_rd_valid || take_read_cmd)begin
  a_rd_valid <= sent_read_cmd<200 && ($random & 3)!=0;
  {a_rd_addr,a_rd_len,a_rd_tag} <= {$random,$random,$random};
 end
end
always @(posedge algorithm_clk) take_read_cmd <= algorithm_rst_n && a_rd_valid && a_rd_ready;
always @(posedge algorithm_clk)if(algorithm_rst_n && a_rd_valid && a_rd_ready)begin
 expect_read_cmd[sent_read_cmd]={a_rd_addr,a_rd_len,a_rd_tag};sent_read_cmd=sent_read_cmd+1;
end
always @(negedge core_clk)begin
 if(!core_rst_n)rd_ready<=0;
 else rd_ready<=($random & 7)!=0;
end
always @(posedge core_clk)begin
 if(!core_rst_n)got_read_cmd=0;
 else if(rd_valid && rd_ready)begin
 if({rd_addr,rd_len,rd_tag} !== expect_read_cmd[got_read_cmd])$fatal(1,"CDC read_cmd data/order mismatch %0d",got_read_cmd);
 got_read_cmd=got_read_cmd+1;checked=checked+1; if(got_read_cmd>sent_read_cmd)$fatal(1,"CDC read_cmd duplicate");
 end
end
integer sent_read_data=0,got_read_data=0; reg take_read_data=0; reg [53:0] expect_read_data[0:255];
always @(negedge core_clk) begin
 if(!core_rst_n)begin r_valid<=0;sent_read_data=0;end
 else if(!r_valid || take_read_data)begin
  r_valid <= sent_read_data<200 && ($random & 3)!=0;
  {r_data,r_keep,r_tag,r_last,r_error} <= {$random,$random,$random};
 end
end
always @(posedge core_clk) take_read_data <= core_rst_n && r_valid && r_ready;
always @(posedge core_clk)if(core_rst_n && r_valid && r_ready)begin
 expect_read_data[sent_read_data]={r_data,r_keep,r_tag,r_last,r_error};sent_read_data=sent_read_data+1;
end
always @(negedge algorithm_clk)begin
 if(!algorithm_rst_n)a_r_ready<=0;
 else a_r_ready<=($random & 7)!=0;
end
always @(posedge algorithm_clk)begin
 if(!algorithm_rst_n)got_read_data=0;
 else if(a_r_valid && a_r_ready)begin
 if({a_r_data,a_r_keep,a_r_tag,a_r_last,a_r_error} !== expect_read_data[got_read_data])$fatal(1,"CDC read_data data/order mismatch %0d",got_read_data);
 got_read_data=got_read_data+1;checked=checked+1; if(got_read_data>sent_read_data)$fatal(1,"CDC read_data duplicate");
 end
end
integer sent_write_cmd=0,got_write_cmd=0; reg take_write_cmd=0; reg [79:0] expect_write_cmd[0:255];
always @(negedge algorithm_clk) begin
 if(!algorithm_rst_n)begin a_wr_valid<=0;sent_write_cmd=0;end
 else if(!a_wr_valid || take_write_cmd)begin
  a_wr_valid <= sent_write_cmd<200 && ($random & 3)!=0;
  {a_wr_addr,a_wr_len,a_wr_tag} <= {$random,$random,$random};
 end
end
always @(posedge algorithm_clk) take_write_cmd <= algorithm_rst_n && a_wr_valid && a_wr_ready;
always @(posedge algorithm_clk)if(algorithm_rst_n && a_wr_valid && a_wr_ready)begin
 expect_write_cmd[sent_write_cmd]={a_wr_addr,a_wr_len,a_wr_tag};sent_write_cmd=sent_write_cmd+1;
end
always @(negedge core_clk)begin
 if(!core_rst_n)wr_ready<=0;
 else wr_ready<=($random & 7)!=0;
end
always @(posedge core_clk)begin
 if(!core_rst_n)got_write_cmd=0;
 else if(wr_valid && wr_ready)begin
 if({wr_addr,wr_len,wr_tag} !== expect_write_cmd[got_write_cmd])$fatal(1,"CDC write_cmd data/order mismatch %0d",got_write_cmd);
 got_write_cmd=got_write_cmd+1;checked=checked+1; if(got_write_cmd>sent_write_cmd)$fatal(1,"CDC write_cmd duplicate");
 end
end
integer sent_write_data=0,got_write_data=0; reg take_write_data=0; reg [36:0] expect_write_data[0:255];
always @(negedge algorithm_clk) begin
 if(!algorithm_rst_n)begin a_w_valid<=0;sent_write_data=0;end
 else if(!a_w_valid || take_write_data)begin
  a_w_valid <= sent_write_data<200 && ($random & 3)!=0;
  {a_w_data,a_w_keep,a_w_last} <= {$random,$random,$random};
 end
end
always @(posedge algorithm_clk) take_write_data <= algorithm_rst_n && a_w_valid && a_w_ready;
always @(posedge algorithm_clk)if(algorithm_rst_n && a_w_valid && a_w_ready)begin
 expect_write_data[sent_write_data]={a_w_data,a_w_keep,a_w_last};sent_write_data=sent_write_data+1;
end
always @(negedge core_clk)begin
 if(!core_rst_n)w_ready<=0;
 else w_ready<=($random & 7)!=0;
end
always @(posedge core_clk)begin
 if(!core_rst_n)got_write_data=0;
 else if(w_valid && w_ready)begin
 if({w_data,w_keep,w_last} !== expect_write_data[got_write_data])$fatal(1,"CDC write_data data/order mismatch %0d",got_write_data);
 got_write_data=got_write_data+1;checked=checked+1; if(got_write_data>sent_write_data)$fatal(1,"CDC write_data duplicate");
 end
end
integer sent_write_rsp=0,got_write_rsp=0; reg take_write_rsp=0; reg [16:0] expect_write_rsp[0:255];
always @(negedge core_clk) begin
 if(!core_rst_n)begin b_valid<=0;sent_write_rsp=0;end
 else if(!b_valid || take_write_rsp)begin
  b_valid <= sent_write_rsp<200 && ($random & 3)!=0;
  {b_tag,b_error} <= {$random,$random,$random};
 end
end
always @(posedge core_clk) take_write_rsp <= core_rst_n && b_valid && b_ready;
always @(posedge core_clk)if(core_rst_n && b_valid && b_ready)begin
 expect_write_rsp[sent_write_rsp]={b_tag,b_error};sent_write_rsp=sent_write_rsp+1;
end
always @(negedge algorithm_clk)begin
 if(!algorithm_rst_n)a_b_ready<=0;
 else a_b_ready<=($random & 7)!=0;
end
always @(posedge algorithm_clk)begin
 if(!algorithm_rst_n)got_write_rsp=0;
 else if(a_b_valid && a_b_ready)begin
 if({a_b_tag,a_b_error} !== expect_write_rsp[got_write_rsp])$fatal(1,"CDC write_rsp data/order mismatch %0d",got_write_rsp);
 got_write_rsp=got_write_rsp+1;checked=checked+1; if(got_write_rsp>sent_write_rsp)$fatal(1,"CDC write_rsp duplicate");
 end
end
initial begin
 repeat(5)@(negedge core_clk);core_rst_n=1;
 repeat(3)@(negedge algorithm_clk);algorithm_rst_n=1;
 wait(got_start==200 && got_frame==200 && got_release==200 && got_result==200 && got_read_cmd==200 && got_read_data==200 && got_write_cmd==200 && got_write_data==200 && got_write_rsp==200);
 // Global abort and skewed synchronous release must not replay stale tokens.
 @(negedge core_clk);core_rst_n=0;algorithm_rst_n=0;
 repeat(8)@(negedge algorithm_clk);algorithm_rst_n=1;
 repeat(4)@(negedge core_clk);core_rst_n=1;
 // Abort partially transferred transactions; no old data may survive restart.
 repeat(20)@(negedge core_clk);core_rst_n=0;algorithm_rst_n=0;
 repeat(6)@(negedge core_clk);core_rst_n=1;
 repeat(3)@(negedge algorithm_clk);algorithm_rst_n=1;
 wait(got_start==200 && got_frame==200 && got_release==200 && got_result==200 && got_read_cmd==200 && got_read_data==200 && got_write_cmd==200 && got_write_data==200 && got_write_rsp==200);
 $display("PASS CDC nine channels, 3600 completed transfers plus an aborted epoch, backpressure and reset");finished=1;$finish;
end
initial begin #1000000;$fatal(1,"CDC timeout");end
endmodule
