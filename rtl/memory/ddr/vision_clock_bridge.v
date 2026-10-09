// Nine atomic ready/valid channels between algorithm and board DDR domains.
// Reset both sides together; clocks may start/stop independently during reset.
module vision_clock_bridge(
 input wire core_clk,core_rst_n,algorithm_clk,algorithm_rst_n,
 output wire a_start_valid,
 input wire start_valid,
 input wire a_start_ready,
 output wire start_ready,
 output wire a_frame_valid,
 input wire frame_valid,
 input wire a_frame_ready,
 output wire frame_ready,
 output wire [7:0] a_frame_status,
 input wire [7:0] frame_status,
 input wire a_release_valid,
 output wire release_valid,
 output wire a_release_ready,
 input wire release_ready,
 input wire a_rsp_valid,
 output wire rsp_valid,
 output wire a_rsp_ready,
 input wire rsp_ready,
 input wire [7:0] a_rsp_status,
 output wire [7:0] rsp_status,
 input wire a_rd_valid,
 output wire rd_valid,
 output wire a_rd_ready,
 input wire rd_ready,
 input wire [31:0] a_rd_addr,
 output wire [31:0] rd_addr,
 input wire [31:0] a_rd_len,
 output wire [31:0] rd_len,
 input wire [15:0] a_rd_tag,
 output wire [15:0] rd_tag,
 output wire a_r_valid,
 input wire r_valid,
 input wire a_r_ready,
 output wire r_ready,
 output wire [31:0] a_r_data,
 input wire [31:0] r_data,
 output wire [3:0] a_r_keep,
 input wire [3:0] r_keep,
 output wire [15:0] a_r_tag,
 input wire [15:0] r_tag,
 output wire a_r_last,
 input wire r_last,
 output wire a_r_error,
 input wire r_error,
 input wire a_wr_valid,
 output wire wr_valid,
 output wire a_wr_ready,
 input wire wr_ready,
 input wire [31:0] a_wr_addr,
 output wire [31:0] wr_addr,
 input wire [31:0] a_wr_len,
 output wire [31:0] wr_len,
 input wire [15:0] a_wr_tag,
 output wire [15:0] wr_tag,
 input wire a_w_valid,
 output wire w_valid,
 output wire a_w_ready,
 input wire w_ready,
 input wire [31:0] a_w_data,
 output wire [31:0] w_data,
 input wire [3:0] a_w_keep,
 output wire [3:0] w_keep,
 input wire a_w_last,
 output wire w_last,
 output wire a_b_valid,
 input wire b_valid,
 input wire a_b_ready,
 output wire b_ready,
 output wire [15:0] a_b_tag,
 input wire [15:0] b_tag,
 output wire a_b_error,
 input wire b_error
);
wire unused_start,unused_release,frame_fifo_ready;
reg [1:0] frame_demand_sync /* synthesis PAP_ASYNC_REG=1 */;
always @(posedge core_clk or negedge core_rst_n)
 if(!core_rst_n) frame_demand_sync<=0; else frame_demand_sync<={frame_demand_sync[0],a_frame_ready};
// WAIT must see algorithm demand, not merely free space in the frame FIFO.
assign frame_ready=frame_fifo_ready && (frame_valid || frame_demand_sync[1]);
async_fifo #(.WIDTH(1),.ADDR_BITS(2)) cdc_start(
 .wclk(core_clk),.wrst_n(core_rst_n),.wvalid(start_valid),.wready(start_ready),.wdata(1'b0),
 .rclk(algorithm_clk),.rrst_n(algorithm_rst_n),.rvalid(a_start_valid),.rready(a_start_ready),.rdata(unused_start));
async_fifo #(.WIDTH(8),.ADDR_BITS(2)) cdc_frame(
 .wclk(core_clk),.wrst_n(core_rst_n),.wvalid(frame_valid),.wready(frame_fifo_ready),.wdata(frame_status),
 .rclk(algorithm_clk),.rrst_n(algorithm_rst_n),.rvalid(a_frame_valid),.rready(a_frame_ready),.rdata(a_frame_status));
async_fifo #(.WIDTH(1),.ADDR_BITS(2)) cdc_release(
 .wclk(algorithm_clk),.wrst_n(algorithm_rst_n),.wvalid(a_release_valid),.wready(a_release_ready),.wdata(1'b0),
 .rclk(core_clk),.rrst_n(core_rst_n),.rvalid(release_valid),.rready(release_ready),.rdata(unused_release));
async_fifo #(.WIDTH(8),.ADDR_BITS(2)) cdc_result(
 .wclk(algorithm_clk),.wrst_n(algorithm_rst_n),.wvalid(a_rsp_valid),.wready(a_rsp_ready),.wdata(a_rsp_status),
 .rclk(core_clk),.rrst_n(core_rst_n),.rvalid(rsp_valid),.rready(rsp_ready),.rdata(rsp_status));
async_fifo #(.WIDTH(80),.ADDR_BITS(2)) cdc_read_cmd(
 .wclk(algorithm_clk),.wrst_n(algorithm_rst_n),.wvalid(a_rd_valid),.wready(a_rd_ready),.wdata({a_rd_addr,a_rd_len,a_rd_tag}),
 .rclk(core_clk),.rrst_n(core_rst_n),.rvalid(rd_valid),.rready(rd_ready),.rdata({rd_addr,rd_len,rd_tag}));
async_fifo #(.WIDTH(54),.ADDR_BITS(2)) cdc_read_data(
 .wclk(core_clk),.wrst_n(core_rst_n),.wvalid(r_valid),.wready(r_ready),.wdata({r_data,r_keep,r_tag,r_last,r_error}),
 .rclk(algorithm_clk),.rrst_n(algorithm_rst_n),.rvalid(a_r_valid),.rready(a_r_ready),.rdata({a_r_data,a_r_keep,a_r_tag,a_r_last,a_r_error}));
async_fifo #(.WIDTH(80),.ADDR_BITS(2)) cdc_write_cmd(
 .wclk(algorithm_clk),.wrst_n(algorithm_rst_n),.wvalid(a_wr_valid),.wready(a_wr_ready),.wdata({a_wr_addr,a_wr_len,a_wr_tag}),
 .rclk(core_clk),.rrst_n(core_rst_n),.rvalid(wr_valid),.rready(wr_ready),.rdata({wr_addr,wr_len,wr_tag}));
async_fifo #(.WIDTH(37),.ADDR_BITS(2)) cdc_write_data(
 .wclk(algorithm_clk),.wrst_n(algorithm_rst_n),.wvalid(a_w_valid),.wready(a_w_ready),.wdata({a_w_data,a_w_keep,a_w_last}),
 .rclk(core_clk),.rrst_n(core_rst_n),.rvalid(w_valid),.rready(w_ready),.rdata({w_data,w_keep,w_last}));
async_fifo #(.WIDTH(17),.ADDR_BITS(2)) cdc_write_rsp(
 .wclk(core_clk),.wrst_n(core_rst_n),.wvalid(b_valid),.wready(b_ready),.wdata({b_tag,b_error}),
 .rclk(algorithm_clk),.rrst_n(algorithm_rst_n),.rvalid(a_b_valid),.rready(a_b_ready),.rdata({a_b_tag,a_b_error}));
endmodule
