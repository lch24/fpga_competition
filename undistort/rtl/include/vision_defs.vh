`ifndef VISION_DEFS_VH
`define VISION_DEFS_VH

// Status values are fixed by main/README.md.
`define VISION_STATUS_OK               8'd0
`define VISION_STATUS_BAD_CONFIG       8'd1
`define VISION_STATUS_NO_BOARD         8'd2
`define VISION_STATUS_BUFFER_OVERFLOW  8'd3
`define VISION_STATUS_CALIB_INVALID    8'd4
`define VISION_STATUS_MEM_ERROR        8'd5
`define VISION_STATUS_TIMEOUT          8'd6

`define VISION_PIXEL_FORMAT_RGB565     8'd1

// IEEE 754 single-precision bit patterns used by the remap path.
`define VISION_FP32_ZERO               32'h0000_0000
`define VISION_FP32_ONE                32'h3f80_0000

// Abstract FP32 service operations. The RTL service owns their latency
// and initiation interval; clients use ready/valid and do not assume either.
`define VISION_FP_OP_ADD               3'd0
`define VISION_FP_OP_SUB               3'd1
`define VISION_FP_OP_MUL               3'd2
`define VISION_FP_OP_DIV               3'd3
`define VISION_FP_OP_U32_TO_FP32       3'd4

`endif
