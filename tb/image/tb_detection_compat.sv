`timescale 1ns/1ps
// Actual detector only, stopping after all views commit, before lengthy LM.
module tb_detection_compat;
 tb_vision_numeric #(.DETECT_ONLY(1),.FIXED_INTERPOLATION(0)) test();
endmodule
