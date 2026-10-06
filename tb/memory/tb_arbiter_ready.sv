`timescale 1ns/1ps
// Downstream deliberately waits for VALID before raising READY. A newly arriving
// higher-priority client must not change a stalled request's address/tag.
module tb_arbiter_ready;
 reg clk=0,rst_n=0;always #5 clk=~clk;
 reg rd_req_valid[0:1],rd_ret_ready[0:1],wr_req_valid[0:1],wr_dat_valid[0:1],wr_dat_last[0:1],wr_done_ready[0:1];
 reg [31:0] rd_req_addr[0:1],rd_req_len[0:1],wr_req_addr[0:1],wr_req_len[0:1],wr_dat_data[0:1];
 reg [15:0] rd_req_tag[0:1],wr_req_tag[0:1];reg [3:0] wr_dat_keep[0:1];
 wire rd_req_ready[0:1],rd_ret_valid[0:1],rd_ret_last[0:1],rd_ret_error[0:1],wr_req_ready[0:1],wr_dat_ready[0:1],wr_done_valid[0:1],wr_done_error[0:1];
 wire [31:0] rd_ret_data[0:1];wire [3:0] rd_ret_keep[0:1];wire [15:0] rd_ret_tag[0:1],wr_done_tag[0:1];
 wire m_rd_req_valid,m_rd_ret_ready,m_wr_req_valid,m_wr_dat_valid,m_wr_dat_last,m_wr_cplt_ready;
 wire [31:0] m_rd_req_addr,m_rd_req_len_bytes,m_wr_req_addr,m_wr_req_len_bytes,m_wr_dat_data;
 wire [15:0] m_rd_req_tag,m_wr_req_tag;wire [3:0] m_wr_dat_keep;
 reg m_rd_req_ready=0,m_rd_ret_valid=0,m_rd_ret_last=1,m_rd_ret_error=0,m_wr_req_ready=0,m_wr_dat_ready=0,m_wr_cplt_valid=0,m_wr_cplt_error=0;
 reg [31:0] m_rd_ret_data=0;reg [3:0] m_rd_ret_keep=15;reg [15:0] m_rd_ret_tag=0,m_wr_cplt_tag=0;
 ddr_port_arbiter dut(.*);
 integer i;
 initial begin
  for(i=0;i<2;i=i+1)begin
   rd_req_valid[i]=0;rd_ret_ready[i]=1;rd_req_addr[i]=100+i*100;rd_req_len[i]=4;rd_req_tag[i]=10+i;
   wr_req_valid[i]=0;wr_dat_valid[i]=0;wr_dat_last[i]=1;wr_done_ready[i]=1;
   wr_req_addr[i]=300+i*100;wr_req_len[i]=4;wr_req_tag[i]=20+i;wr_dat_data[i]=i;wr_dat_keep[i]=15;
  end
  repeat(4)@(negedge clk);rst_n=1;
  rd_req_valid[1]=1;wr_req_valid[1]=1;
  repeat(3)@(negedge clk);
  if(!m_rd_req_valid||!m_wr_req_valid)$fatal(1,"VALID waits for READY");
  rd_req_valid[0]=1;wr_req_valid[0]=1;
  repeat(7)begin @(negedge clk);
   if(!m_rd_req_valid||m_rd_req_addr!=200||m_rd_req_tag!=11||!m_wr_req_valid||m_wr_req_addr!=400||m_wr_req_tag!=21)$fatal(1,"stalled selection changed");
  end
  m_rd_req_ready=1;m_wr_req_ready=1;
  @(negedge clk);rd_req_valid[1]=0;wr_req_valid[1]=0;m_rd_req_ready=0;m_wr_req_ready=0;
  m_rd_ret_valid=1;m_rd_ret_tag=11;wr_dat_valid[1]=1;m_wr_dat_ready=1;
  @(negedge clk);m_rd_ret_valid=0;wr_dat_valid[1]=0;m_wr_dat_ready=0;m_wr_cplt_valid=1;m_wr_cplt_tag=21;
  @(negedge clk);m_wr_cplt_valid=0;
  repeat(3)@(negedge clk);
  if(!m_rd_req_valid||m_rd_req_addr!=100||!m_wr_req_valid||m_wr_req_addr!=300)$fatal(1,"queued client not serviced");
  m_rd_req_ready=1;m_wr_req_ready=1;
  @(negedge clk);rd_req_valid[0]=0;wr_req_valid[0]=0;m_rd_req_ready=0;m_wr_req_ready=0;
  m_rd_ret_valid=1;m_rd_ret_tag=10;wr_dat_valid[0]=1;m_wr_dat_ready=1;
  @(negedge clk);m_rd_ret_valid=0;wr_dat_valid[0]=0;m_wr_dat_ready=0;m_wr_cplt_valid=1;m_wr_cplt_tag=20;
  @(negedge clk);m_wr_cplt_valid=0;repeat(3)@(negedge clk);
  // Zero-length errors are local and must work with READY permanently low.
  rd_req_len[0]=0;wr_req_len[0]=0;rd_req_valid[0]=1;wr_req_valid[0]=1;
  @(negedge clk);rd_req_valid[0]=0;wr_req_valid[0]=0;
  if(!rd_ret_valid[0]||!rd_ret_error[0]||!wr_done_valid[0]||!wr_done_error[0]||m_rd_req_valid||m_wr_req_valid)$fatal(1,"local zero length");
  $display("PASS arbiter ready: valid independent of ready, stalled selection lock, queued clients, local zero-length errors");$finish;
 end
 initial begin #20000;$fatal(1,"arbiter ready timeout");end
endmodule
