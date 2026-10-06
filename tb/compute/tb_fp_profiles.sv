`timescale 1ns/1ps
// Every retained operation is checked against the original independent vectors.
// Profiles match the five feature combinations used by the calibration RTL.
module tb_fp_profiles;
    wire [5:0] completed;
    wire [31:0] failures[0:5], counts[0:5];
    fp_checker #(64,1,0,0,0,0) basic(completed[0],failures[0],counts[0]);
    fp_checker #(64,2,1,0,0,0) exponential(completed[1],failures[1],counts[1]);
    fp_checker #(64,3,0,1,0,0) logarithm(completed[2],failures[2],counts[2]);
    fp_checker #(64,4,0,0,1,1) rotation(completed[3],failures[3],counts[3]);
    fp_checker #(64,5,1,0,0,1) validation(completed[4],failures[4],counts[4]);
    fp_checker #(32,1,0,0,0,0) remap(completed[5],failures[5],counts[5]);
    wire done=&completed;
    wire [31:0] errors=failures[0]+failures[1]+failures[2]+failures[3]+failures[4]+failures[5];
    wire [31:0] checked=counts[0]+counts[1]+counts[2]+counts[3]+counts[4]+counts[5];
endmodule
