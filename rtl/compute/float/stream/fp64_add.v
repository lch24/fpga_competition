`timescale 1ns/1ps
// RNE IEEE-754 adder. Narrow guard/round/sticky arithmetic, throughput one
// request per enabled clock. Default keeps the legacy two-clock latency.
// CE freezes all stages and FIFO; reset cancels queued/in-flight operations.
module fp64_add #(parameter USE_CE=0)(
 input wire ce,clk,rst_n,in_valid, output wire in_ready,
 input wire [63:0] in_a,in_b, output wire out_valid,
 input wire out_ready, output wire [63:0] out_r
);
 fp_add_pipeline #(.BITS(64),.USE_CE(USE_CE)) core(.ce(ce),.clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_ready(in_ready),.in_a(in_a),.in_b(in_b),.out_valid(out_valid),.out_ready(out_ready),.out_r(out_r));
endmodule
