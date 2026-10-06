`timescale 1ns/1ps
// One restoring division step per cycle. Full IEEE FP32 RNE, including
// subnormals and special values. CE freezes the core; reset cancels work.
// Callers must honor valid/ready, never assume the former FIFO latency.
module fp32_div #(parameter USE_CE=0)(
 input wire ce,clk,rst_n,in_valid, output wire in_ready,
 input wire [31:0] in_a,in_b, output wire out_valid,
 input wire out_ready, output wire [31:0] out_r
);
 fp_divsqrt #(.FP_W(32),.USE_CE(USE_CE)) core(
  .ce(ce),.clk(clk),.rst_n(rst_n),.req_valid(in_valid),.req_ready(in_ready),
  .req_sqrt(1'b0),.req_a(in_a),.req_b(in_b),
  .rsp_valid(out_valid),.rsp_ready(out_ready),.rsp_result(out_r),.rsp_flags());
endmodule
