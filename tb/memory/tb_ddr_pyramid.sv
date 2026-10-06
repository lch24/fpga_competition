`timescale 1ns/1ps
// Full two-level DDR pyramid: real downsampling, cache stalls and both native
// detector passes. A constant image has no corners; every downsampled byte is 17.
module tb_ddr_pyramid;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,process_frame=0;
 wire busy,done;wire [1:0] status;wire out_valid;wire [31:0] out_x,out_y;wire [15:0] out_total;wire out_grid_ok;
 `include "logical_bus.vh"
 `include "logical_memory.vh"
 corner_detect_ddr_top #(.W0(1280),.H0(64),.DEPTH(2),.GRAY_ADDR_W(17),.DDR_GRAY(1),.REPLAY_RESP(1)) dut(
  .clk(clk),.rst_n(rst_n),.process_frame(process_frame),.busy(busy),.done(done),.status(status),
  .cfg_gray_base(32'd4096),.cfg_gray_stride(32'd1280),.cfg_gray_w(16'd1280),.cfg_gray_h(16'd64),.cfg_ram_base(17'd0),
  .cfg_resp_base(32'd0),.cfg_resp_dump_en(1'b0),.cfg_pyr_en(1'b1),
  .out_valid(out_valid),.out_ready(1'b1),.out_x(out_x),.out_y(out_y),.out_total(out_total),.out_grid_ok(out_grid_ok),
  .m_rd_req_valid(rd_valid),.m_rd_req_ready(rd_ready),.m_rd_req_addr(rd_addr),.m_rd_req_len_bytes(rd_len),.m_rd_req_tag(rd_tag),
  .m_rd_ret_valid(r_valid),.m_rd_ret_ready(r_ready),.m_rd_ret_data(r_data),.m_rd_ret_keep(r_keep),.m_rd_ret_tag(r_tag),.m_rd_ret_last(r_last),.m_rd_ret_error(r_error),
  .m_wr_req_valid(wr_valid),.m_wr_req_ready(wr_ready),.m_wr_req_addr(wr_addr),.m_wr_req_len_bytes(wr_len),.m_wr_req_tag(wr_tag),
  .m_wr_dat_valid(w_valid),.m_wr_dat_ready(w_ready),.m_wr_dat_data(w_data),.m_wr_dat_keep(w_keep),.m_wr_dat_last(w_last),
  .m_wr_cplt_valid(b_valid),.m_wr_cplt_ready(b_ready),.m_wr_cplt_tag(b_tag),.m_wr_cplt_error(b_error),
  .ext_rd_req_valid(1'b0),.ext_rd_req_addr(32'd0),.ext_rd_req_len(32'd0),.ext_rd_req_tag(16'd0),.ext_rd_ret_ready(1'b1),
  .ext_wr_req_valid(1'b0),.ext_wr_req_addr(32'd0),.ext_wr_req_len(32'd0),.ext_wr_req_tag(16'd0),
  .ext_wr_dat_valid(1'b0),.ext_wr_dat_data(32'd0),.ext_wr_dat_keep(4'd0),.ext_wr_dat_last(1'b0),.ext_wr_done_ready(1'b1));
 integer i,cycles=0,checked=0,native_passes=0;
 reg finished=0;
 always @(posedge clk)if(rst_n)begin
  cycles<=cycles+1;
  if(out_valid)$fatal(1,"constant image produced a corner");
  if(dut.engine_ce && dut.u_det.nat_entry && dut.u_det.stage==1)native_passes<=native_passes+1;
 end
 initial begin
  for(i=0;i<81920;i=i+1)memory.mem[4096+i]=17;
  for(i=81920;i<102464;i=i+1)memory.mem[4096+i]=8'ha5;
  repeat(4)@(negedge clk);rst_n=1;process_frame=1;@(negedge clk);process_frame=0;
  wait(done);@(negedge clk);
  // Frame transport succeeds even when the detector reports no grid.
  if(status!=1 || dut.det_status!=2 || out_grid_ok || dut.cache_error || violations || native_passes!=2)$fatal(1,"DDR pyramid status/pass count");
  for(i=0;i<102400;i=i+1)begin
   if(memory.mem[4096+i]!==8'd17)$fatal(1,"pyramid byte %0d",i);
   checked=checked+1;
  end
  for(i=102400;i<102464;i=i+1)if(memory.mem[4096+i]!==8'ha5)$fatal(1,"pyramid overwrite");
  finished=1;$finish;
 end
 initial begin #300000000;$fatal(1,"pyramid timeout");end
endmodule
