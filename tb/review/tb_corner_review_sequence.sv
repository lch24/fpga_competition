`timescale 1ns/1ps
module tb_corner_review_sequence;
 tb_vision_manual #(.BUILD_ONLY(1),.ENABLE_CORNER_REVIEW(1)) test();
endmodule
