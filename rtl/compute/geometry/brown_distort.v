`include "calib_defs.vh"
// 接口适配层：锁存、串行访存、指令执行和错误处理统一由 geometry_engine 完成。
// project_point 的第二个历史浮点槽固定空闲，保留总线宽度以兼容调用方。
// 一次一项任务；valid/ready 背压保持，复位取消；失败结果清零。
module brown_distort #(parameter FP_SHARED=0,
    parameter FP_W = 64
) (
    // Optional shared FP64 service; local mode keeps standalone compatibility.
    output wire [0:0] shared_req_valid,
    input wire [0:0] shared_req_ready,
    output wire [4:0] shared_req_op,
    output wire [63:0] shared_req_a,
    output wire [63:0] shared_req_b,
    output wire [0:0] shared_active,
    input wire [0:0] shared_rsp_valid,
    output wire [0:0] shared_rsp_ready,
    input wire [63:0] shared_rsp_result,
    input wire [4:0] shared_rsp_flags,

    input wire clk, // core_clk，同一时钟域
    input wire rst_n, // 低有效复位，同步释放；取消全部在途事务
    input wire cmd_valid, // 命令有效
    output wire cmd_ready, // 可接受命令
    input wire [FP_W-1:0] cmd_x, // 归一化x
    input wire [FP_W-1:0] cmd_y, // 归一化y
    input wire [5*FP_W-1:0] cmd_dist, // 低位起k1,k2,k3,p1,p2，各FP_W位
    output wire rsp_valid, // 完成响应有效
    input wire rsp_ready, // 接收完成响应
    output wire [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [FP_W-1:0] rsp_xd, // 畸变归一化x
    output wire [FP_W-1:0] rsp_yd // 畸变归一化y
);

    geometry_engine #(.FP_W(FP_W),.FP_SHARED(FP_SHARED),.PROJECTION(0)) engine(
        .clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),
        .cmd_payload({cmd_dist,cmd_y,cmd_x}),.rsp_valid(rsp_valid),.rsp_ready(rsp_ready),
        .rsp_status(rsp_status),.rsp_x(rsp_xd),.rsp_y(rsp_yd),
        .shared_req_valid(shared_req_valid),
        .shared_req_ready(shared_req_ready),
        .shared_req_op(shared_req_op),
        .shared_req_a(shared_req_a),
        .shared_req_b(shared_req_b),
        .shared_active(shared_active),
        .shared_rsp_valid(shared_rsp_valid),
        .shared_rsp_ready(shared_rsp_ready),
        .shared_rsp_result(shared_rsp_result),
        .shared_rsp_flags(shared_rsp_flags));
endmodule
