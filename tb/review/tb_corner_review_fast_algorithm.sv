`timescale 1ns/1ps
// Reverse the board clock ratio to exercise full corner FIFOs / CDC latency.
module tb_corner_review_fast_algorithm;
 tb_corner_review_integration #(.CORE_HALF(7),.ALG_HALF(3),.ALG_OFFSET(1)) test();
endmodule
