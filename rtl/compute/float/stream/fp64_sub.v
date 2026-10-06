`timescale 1ns/1ps
// IEEE FP64 subtraction uses the same narrow RNE adder as addition.
// Flip the second sign before special-value handling and rounding.
module fp64_sub #(parameter USE_CE=0)(
 input wire ce,clk,rst_n,in_valid,output wire in_ready,
 input wire [63:0] in_a,in_b,output wire out_valid,
 input wire out_ready,output wire [63:0] out_r
);
 fp64_add #(.USE_CE(USE_CE)) core(.ce(ce),.clk(clk),.rst_n(rst_n),
  .in_valid(in_valid),.in_ready(in_ready),.in_a(in_a),.in_b({~in_b[63],in_b[62:0]}),
  .out_valid(out_valid),.out_ready(out_ready),.out_r(out_r));
endmodule
