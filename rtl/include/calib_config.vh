// 全工程统一的综合期配置；也可用编译器 +define+NAME=value 覆盖。
// 修改后必须重新编译/综合所有模块；接口两端必须使用同一配置。
`ifndef CALIB_CONFIG_VH
`define CALIB_CONFIG_VH
`ifndef PAR_VIEWS
`define PAR_VIEWS 3
`endif
`ifndef PAR_BOARD_ROWS
`define PAR_BOARD_ROWS 5
`endif
`ifndef PAR_BOARD_COLS
`define PAR_BOARD_COLS 8
`endif
`ifndef PAR_LM_MAX_ITERS
`define PAR_LM_MAX_ITERS 150
`endif
`ifndef PAR_LM_MAX_TRIES
`define PAR_LM_MAX_TRIES 16
`endif
// 支持范围：3..16视图；行列均>=2，每图<=256内角点；迭代1..255，重试1..256。
// 容量范围不代表目标器件一定放得下，资源和时序仍需综合确认。
`endif
