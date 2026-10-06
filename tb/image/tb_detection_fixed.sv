`timescale 1ns/1ps
module tb_detection_fixed;
 tb_vision_numeric #(.DETECT_ONLY(1),.FIXED_INTERPOLATION(1)) test();
endmodule
