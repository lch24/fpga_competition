// 仅为 parameter 子系统定义常量，尚未与公共 vision_defs.vh 合并。
`ifndef CALIB_DEFS_VH
`define CALIB_DEFS_VH
`include "calib_config.vh"
`define PAR_POINTS (`PAR_BOARD_ROWS*`PAR_BOARD_COLS)
`define PAR_TOTAL_POINTS (`PAR_VIEWS*`PAR_POINTS)
`define PAR_RESIDUALS (2*`PAR_TOTAL_POINTS)
`define PAR_STATE_N (9+6*`PAR_VIEWS)
`define PAR_STAGE0_N (4+6*`PAR_VIEWS)
`define PAR_STAGE1_N (`PAR_STAGE0_N+1)
`define PAR_ACTIVE_N (`PAR_STAGE0_N+4)
`define PAR_STATE_W (64*`PAR_STATE_N)
`define PAR_VIEW_BITS $clog2(`PAR_VIEWS+1)
`define PAR_POINT_BITS $clog2(`PAR_POINTS+1)
`define PAR_RES_BITS $clog2(`PAR_RESIDUALS)
`define PAR_COL_BITS $clog2(`PAR_ACTIVE_N+1)
`define PAR_ADDR_BITS $clog2(`PAR_TOTAL_POINTS)
`define PAR_GAUSS_SIZE (`PAR_ACTIVE_N*(`PAR_ACTIVE_N+1))
`define PAR_GAUSS_ADDR_BITS $clog2(`PAR_GAUSS_SIZE)
`define PAR_TRIANGLE_SIZE (`PAR_ACTIVE_N*(`PAR_ACTIVE_N+1)/2)
`define PAR_VIEW_RMS_W (64*`PAR_VIEWS)
`define PAR_CAMERA_W 288
`define PAR_H_W 576
`define PAR_H_ALL_W (576*`PAR_VIEWS)
`define PAR_K_W 256
`define PAR_SCALE_W (64*`PAR_ACTIVE_N)
`define PAR_POSES_W (768*`PAR_VIEWS)
// 状态：PAR_STATE_N=(9+6*PAR_VIEWS) 个 FP64；第 i 项占 [64*i +: 64]。
// 0/1=log(fx/fy), 2/3=cx/W,cy/H, 4/5/6/7/8=k1,k2,p1,p2,k3。
// 9+6*v 起：rx,ry,rz,tx,ty,log(tz)。k3=0；对象点采用中心化单位格。
// camera：9 个 FP32，低位起 fx,fy,cx,cy,k1,k2,k3,p1,p2。
// H/R：行优先 FP64；H_ALL：按view递增拼接，view0在低位。
// K：低位起 fx,fy,cx,cy，均为解码后的 FP64，不是 log 或归一化值。
// POSES：每视图R的9项(行优先)加t的3项，共12*PAR_VIEWS个FP64。
// stage0 活动索引：0,1,2,3,9..PAR_STATE_N-1；stage1追加4；stage2追加5,6,7。
// J列、scale/delta按上述顺序，容量PAR_ACTIVE_N，未用高槽置零。
`define PAR_OK 8'd0
`define PAR_BAD_CONFIG 8'd1
`define PAR_NO_BOARD 8'd2
`define PAR_BUFFER_OVERFLOW 8'd3
`define PAR_CALIB_INVALID 8'd4
`define PAR_MEM_ERROR 8'd5
`define PAR_TIMEOUT 8'd6
// fp_operator 操作码；所有输入/输出都是位模式，不是 Verilog real。
`define PAR_FP_ADD 5'd0
`define PAR_FP_SUB 5'd1
`define PAR_FP_MUL 5'd2
`define PAR_FP_DIV 5'd3
`define PAR_FP_SQRT 5'd4
`define PAR_FP_SIN 5'd5
`define PAR_FP_COS 5'd6
`define PAR_FP_ATAN2 5'd7
`define PAR_FP_EXP 5'd8
`define PAR_FP_LOG 5'd9
`define PAR_FP_ACOS 5'd10
`define PAR_FP_F32_TO_F64 5'd11
`define PAR_FP_F64_TO_F32 5'd12
`define PAR_FP_COMPARE 5'd13
`endif
