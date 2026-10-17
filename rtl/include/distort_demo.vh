`ifndef DISTORT_DEMO_VH
`define DISTORT_DEMO_VH
// Private command sent over the existing atomic start FIFO; outside DDR range.
`define DEMO_BUILD_COMMAND 32'hffffff00
`define DEMO_CLEAR_COMMAND 32'hffffff04
`define DEMO_MAP_BASE 32'h03800000
`define DEMO_PHOTO_BASE 32'h01400000
`define DEMO_MIDDLE_BASE 32'h01600000
`endif
