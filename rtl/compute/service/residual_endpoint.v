`include "calib_defs.vh"
// Residual service endpoint: standalone private mode or phase-owned shared mode.
// External reset cancels in-flight work; caller owns the service until response.
module residual_endpoint #(parameter FP_SHARED=0, EXTERNAL=0) (
    output wire  ext_rst_n,
    output wire  ext_cmd_valid,
    input wire  ext_cmd_ready,
    output wire [15:0] ext_cmd_width,
    output wire [15:0] ext_cmd_height,
    output wire [`PAR_STATE_W-1:0] ext_cmd_state,
    input wire  ext_point_rd_en,
    input wire [`PAR_VIEW_BITS-1:0] ext_point_rd_view_id,
    input wire [`PAR_POINT_BITS-1:0] ext_point_rd_index,
    output wire  ext_point_rd_valid,
    output wire [31:0] ext_point_rd_x_fp32,
    output wire [31:0] ext_point_rd_y_fp32,
    input wire  ext_data_valid,
    output wire  ext_data_ready,
    input wire [`PAR_RES_BITS-1:0] ext_data_index,
    input wire [63:0] ext_data_fp64,
    input wire  ext_data_last,
    input wire  ext_rsp_valid,
    output wire  ext_rsp_ready,
    input wire [7:0] ext_rsp_status,
    input wire [63:0] ext_rsp_cost_fp64,
    // Optional shared FP64 service; local mode keeps standalone compatibility.
    output wire [3:0] shared_req_valid,
    input wire [3:0] shared_req_ready,
    output wire [19:0] shared_req_op,
    output wire [255:0] shared_req_a,
    output wire [255:0] shared_req_b,
    output wire [3:0] shared_active,
    input wire [3:0] shared_rsp_valid,
    output wire [3:0] shared_rsp_ready,
    input wire [63:0] shared_rsp_result,
    input wire [4:0] shared_rsp_flags,

    input wire clk, // core_clk，同一时钟域
    input wire rst_n, // 低有效复位，同步释放；取消全部在途事务
    input wire cmd_valid, // 命令有效
    output wire cmd_ready, // 可接受命令
    input wire [15:0] cmd_width, // 图像宽度，>=2
    input wire [15:0] cmd_height, // 图像高度，>=2
    input wire [`PAR_STATE_W-1:0] cmd_state, // PAR_STATE_N 项 FP64 状态快照，命令握手时锁存
    output wire point_rd_en, // 固定1拍角点读使能，无ready，发起前预留接收空间
    output wire [`PAR_VIEW_BITS-1:0] point_rd_view_id, // 0..PAR_VIEWS-1
    output wire [`PAR_POINT_BITS-1:0] point_rd_index, // 图内角点0..PAR_POINTS-1
    input wire point_rd_valid, // 固定1拍返回，必须当拍消费；无返回背压
    input wire [31:0] point_rd_x_fp32, // 原图x
    input wire [31:0] point_rd_y_fp32, // 原图y
    output wire data_valid, // 残差流有效
    input wire data_ready, // 下游可接收
    output wire [`PAR_RES_BITS-1:0] data_index, // 0..239
    output wire [63:0] data_fp64, // 预测减观测
    output wire data_last, // 仅239为1
    output wire rsp_valid, // 完成响应有效
    input wire rsp_ready, // 接收完成响应
    output wire [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [63:0] rsp_cost_fp64 // 平方和；失败时+Inf
);
 generate if(EXTERNAL)begin : external_service
 assign ext_rst_n=rst_n;
 assign ext_cmd_valid=cmd_valid;
 assign cmd_ready=ext_cmd_ready;
 assign ext_cmd_width=cmd_width;
 assign ext_cmd_height=cmd_height;
 assign ext_cmd_state=cmd_state;
 assign point_rd_en=ext_point_rd_en;
 assign point_rd_view_id=ext_point_rd_view_id;
 assign point_rd_index=ext_point_rd_index;
 assign ext_point_rd_valid=point_rd_valid;
 assign ext_point_rd_x_fp32=point_rd_x_fp32;
 assign ext_point_rd_y_fp32=point_rd_y_fp32;
 assign data_valid=ext_data_valid;
 assign ext_data_ready=data_ready;
 assign data_index=ext_data_index;
 assign data_fp64=ext_data_fp64;
 assign data_last=ext_data_last;
 assign rsp_valid=ext_rsp_valid;
 assign ext_rsp_ready=rsp_ready;
 assign rsp_status=ext_rsp_status;
 assign rsp_cost_fp64=ext_rsp_cost_fp64;
 assign shared_req_valid=0;
 assign shared_req_op=0;
 assign shared_req_a=0;
 assign shared_req_b=0;
 assign shared_active=0;
 assign shared_rsp_ready=0;
 end else begin : private_service
 assign ext_rst_n=0;
 assign ext_cmd_valid=0;
 assign ext_cmd_width=0;
 assign ext_cmd_height=0;
 assign ext_cmd_state=0;
 assign ext_point_rd_valid=0;
 assign ext_point_rd_x_fp32=0;
 assign ext_point_rd_y_fp32=0;
 assign ext_data_ready=0;
 assign ext_rsp_ready=0;
 residual_engine #(.FP_SHARED(FP_SHARED)) core(.shared_req_valid(shared_req_valid),.shared_req_ready(shared_req_ready),.shared_req_op(shared_req_op),.shared_req_a(shared_req_a),.shared_req_b(shared_req_b),.shared_active(shared_active),.shared_rsp_valid(shared_rsp_valid),.shared_rsp_ready(shared_rsp_ready),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_width(cmd_width),.cmd_height(cmd_height),.cmd_state(cmd_state),.point_rd_en(point_rd_en),.point_rd_view_id(point_rd_view_id),.point_rd_index(point_rd_index),.point_rd_valid(point_rd_valid),.point_rd_x_fp32(point_rd_x_fp32),.point_rd_y_fp32(point_rd_y_fp32),.data_valid(data_valid),.data_ready(data_ready),.data_index(data_index),.data_fp64(data_fp64),.data_last(data_last),.rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.rsp_cost_fp64(rsp_cost_fp64));
 end endgenerate
endmodule
