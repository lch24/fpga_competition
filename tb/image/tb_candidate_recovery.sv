`timescale 1ns/1ps
// Two outstanding cache transactions: a gray read fails while the candidate
// write completion is deliberately delayed. Frame ownership must not escape.
module tb_candidate_recovery;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,process_frame=0,allow_completion=0;
 wire busy,done;wire [1:0] status;
 `include "logical_bus.vh"
 wire cache_bready;
 `include "logical_memory.vh"
 assign b_ready=cache_bready&&allow_completion;
 corner_detect_ddr_top #(.W0(64),.H0(48),.DEPTH(1),.GRAY_ADDR_W(13),
  .DDR_GRAY(1),.DDR_CANDIDATES(1),.CANDIDATE_BASE(524288),.REPLAY_RESP(1)) dut(
  .clk(clk),.rst_n(rst_n),.process_frame(process_frame),.busy(busy),.done(done),.status(status),
  .cfg_gray_base(4096),.cfg_gray_stride(64),.cfg_gray_w(16'd64),.cfg_gray_h(16'd48),.cfg_ram_base(13'd0),
  .cfg_resp_base(0),.cfg_resp_dump_en(1'b0),.cfg_pyr_en(1'b0),.out_ready(1'b1),
  .m_rd_req_valid(rd_valid),.m_rd_req_ready(rd_ready),.m_rd_req_addr(rd_addr),.m_rd_req_len_bytes(rd_len),.m_rd_req_tag(rd_tag),
  .m_rd_ret_valid(r_valid),.m_rd_ret_ready(r_ready),.m_rd_ret_data(r_data),.m_rd_ret_keep(r_keep),.m_rd_ret_tag(r_tag),.m_rd_ret_last(r_last),.m_rd_ret_error(r_error),
  .m_wr_req_valid(wr_valid),.m_wr_req_ready(wr_ready),.m_wr_req_addr(wr_addr),.m_wr_req_len_bytes(wr_len),.m_wr_req_tag(wr_tag),
  .m_wr_dat_valid(w_valid),.m_wr_dat_ready(w_ready),.m_wr_dat_data(w_data),.m_wr_dat_keep(w_keep),.m_wr_dat_last(w_last),
  .m_wr_cplt_valid(b_valid&&allow_completion),.m_wr_cplt_ready(cache_bready),.m_wr_cplt_tag(b_tag),.m_wr_cplt_error(b_error),
  .ext_rd_req_valid(1'b0),.ext_rd_req_addr(0),.ext_rd_req_len(0),.ext_rd_req_tag(16'd0),.ext_rd_ret_ready(1'b1),
  .ext_wr_req_valid(1'b0),.ext_wr_req_addr(0),.ext_wr_req_len(0),.ext_wr_req_tag(16'd0),
  .ext_wr_dat_valid(1'b0),.ext_wr_dat_data(0),.ext_wr_dat_keep(4'd0),.ext_wr_dat_last(1'b0),.ext_wr_done_ready(1'b1));
 reg finished=0;integer checked=0,i;
 initial begin
  for(i=0;i<3072;i=i+1)memory.mem[4096+i]=17;
  repeat(4)@(negedge clk);rst_n=1;process_frame=1;
  @(negedge clk);process_frame=0;
  // Drive the RAM-side requests directly; this test covers memory lifetime,
  // not corner numerics. Let both DDR services actually execute transactions.
  force dut.det_busy=1;force dut.scratch_wr_en=1;
  force dut.scratch_wr_addr=15'd9;force dut.scratch_wr_data=64'h123456789abcdef0;
  force dut.scratch_rd_en=0;
  wait(b_valid);
  memory.inject_error(4096,8191);
  force dut.gray_rd_en=1;force dut.gray_rd_addr=13'd0;
  wait(dut.gray_error);
  repeat(20)begin @(negedge clk);if(done||!busy)$fatal(1,"released frame before other DDR transaction drained");end
  checked=1;
  allow_completion=1;
  wait(done);@(negedge clk);
  if(busy||status!=2||!dut.scratch_idle||violations)$fatal(1,"drain failed");
  release dut.det_busy;release dut.scratch_wr_en;release dut.scratch_wr_addr;
  release dut.scratch_wr_data;release dut.scratch_rd_en;
  release dut.gray_rd_en;release dut.gray_rd_addr;
  memory.clear_error_injection();process_frame=1;
  @(negedge clk);process_frame=0;
  if(done||!busy||dut.cache_error)$fatal(1,"retry failed to clear both caches");
  wait(done);@(negedge clk);
  if(status!=1||busy||dut.cache_error||violations)$fatal(1,"retry failed");
  checked=2;finished=1;$finish;
 end
 initial begin #20000000;$fatal(1,"candidate recovery timeout");end
endmodule
