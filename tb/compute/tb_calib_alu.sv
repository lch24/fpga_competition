`timescale 1ns/1ps
// Reuse independent numerical vectors and the established protocol checker.
module tb_calib_alu;
 wire done; wire [31:0] errors,checked;
 fp_checker #(64,1,0,0,0,0,1) check_alu(done,errors,checked);
endmodule
