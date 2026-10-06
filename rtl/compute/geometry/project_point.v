`include "calib_defs.vh"
// 接口适配层：锁存、串行访存、指令执行和错误处理统一由 geometry_engine 完成。
// project_point 的第二个历史浮点槽固定空闲，保留总线宽度以兼容调用方。
// 一次一项任务；valid/ready 背压保持，复位取消；失败结果清零。
module project_point #(parameter FP_SHARED=0) (
    // Optional shared FP64 service; local mode keeps standalone compatibility.
    output wire [1:0] shared_req_valid,
    input wire [1:0] shared_req_ready,
    output wire [9:0] shared_req_op,
    output wire [127:0] shared_req_a,
    output wire [127:0] shared_req_b,
    output wire [1:0] shared_active,
    input wire [1:0] shared_rsp_valid,
    output wire [1:0] shared_rsp_ready,
    input wire [63:0] shared_rsp_result,
    input wire [4:0] shared_rsp_flags,

    input wire clk, // core_clk，同一时钟域
    input wire rst_n, // 低有效复位，同步释放；取消全部在途事务
    input wire cmd_valid, // 命令有效
    output wire cmd_ready, // 可接受命令
    input wire [63:0] cmd_x_fp64, // 中心化单位格X
    input wire [63:0] cmd_y_fp64, // 中心化单位格Y
    input wire [575:0] cmd_r_fp64, // 3x3 R，行优先
    input wire [191:0] cmd_t_fp64, // tx,ty,tz，实际tz非log
    input wire [`PAR_K_W-1:0] cmd_k_fp64, // fx,fy,cx,cy
    input wire [319:0] cmd_dist_fp64, // 低位起k1,k2,k3,p1,p2
    output wire rsp_valid, // 完成响应有效
    input wire rsp_ready, // 接收完成响应
    output wire [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [63:0] rsp_u_fp64, // 预测像素u
    output wire [63:0] rsp_v_fp64 // 预测像素v
);

    assign shared_req_valid[1 +: 1] = 0;
    assign shared_req_op[5 +: 5] = 0;
    assign shared_req_a[64 +: 64] = 0;
    assign shared_req_b[64 +: 64] = 0;
    assign shared_active[1 +: 1] = 0;
    assign shared_rsp_ready[1 +: 1] = 0;
    geometry_engine #(.FP_W(64),.FP_SHARED(FP_SHARED),.PROJECTION(1)) engine(
        .clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),
        .cmd_payload({cmd_dist_fp64,cmd_k_fp64,cmd_t_fp64,cmd_r_fp64,cmd_y_fp64,cmd_x_fp64}),.rsp_valid(rsp_valid),.rsp_ready(rsp_ready),
        .rsp_status(rsp_status),.rsp_x(rsp_u_fp64),.rsp_y(rsp_v_fp64),
        .shared_req_valid(shared_req_valid[0 +: 1]),
        .shared_req_ready(shared_req_ready[0 +: 1]),
        .shared_req_op(shared_req_op[0 +: 5]),
        .shared_req_a(shared_req_a[0 +: 64]),
        .shared_req_b(shared_req_b[0 +: 64]),
        .shared_active(shared_active[0 +: 1]),
        .shared_rsp_valid(shared_rsp_valid[0 +: 1]),
        .shared_rsp_ready(shared_rsp_ready[0 +: 1]),
        .shared_rsp_result(shared_rsp_result),
        .shared_rsp_flags(shared_rsp_flags));
endmodule
