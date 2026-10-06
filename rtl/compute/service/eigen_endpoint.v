`include "calib_defs.vh"
// Phase-owned eigen service; h/zhang do not execute concurrently.
module eigen_endpoint #(parameter EXTERNAL=0, parameter FP_SHARED=0, parameter MAX_SEARCH_ROUNDS=0) (
    output wire  eigen_rst_n,
    output wire  eigen_cmd_valid,
    input wire  eigen_cmd_ready,
    output wire [3:0] eigen_cmd_n,
    output wire  eigen_matrix_valid,
    input wire  eigen_matrix_ready,
    output wire [63:0] eigen_matrix_fp64,
    output wire  eigen_matrix_last,
    input wire  eigen_rsp_valid,
    output wire  eigen_rsp_ready,
    input wire [7:0] eigen_rsp_status,
    input wire [575:0] eigen_rsp_min_vector_fp64,
    input wire [63:0] eigen_rsp_min_value_fp64,
    input wire [63:0] eigen_rsp_second_value_fp64,
    input wire [63:0] eigen_rsp_max_value_fp64,
    input wire [13:0] eigen_rsp_rotations,
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

    input wire clk,
    input wire rst_n,
    input wire cmd_valid,
    output wire cmd_ready,
    input wire [3:0] cmd_n,
    input wire matrix_valid,
    output wire matrix_ready,
    input wire [63:0] matrix_fp64,
    input wire matrix_last,
    output wire rsp_valid,
    input wire rsp_ready,
    output wire [7:0] rsp_status,
    output wire [575:0] rsp_min_vector_fp64,
    output wire [63:0] rsp_min_value_fp64,
    output wire [63:0] rsp_second_value_fp64,
    output wire [63:0] rsp_max_value_fp64,
    output wire [13:0] rsp_rotations
);
 generate if(EXTERNAL)begin : external_service
 assign eigen_rst_n=rst_n;
 assign eigen_cmd_valid=cmd_valid;
 assign cmd_ready=eigen_cmd_ready;
 assign eigen_cmd_n=cmd_n;
 assign eigen_matrix_valid=matrix_valid;
 assign matrix_ready=eigen_matrix_ready;
 assign eigen_matrix_fp64=matrix_fp64;
 assign eigen_matrix_last=matrix_last;
 assign rsp_valid=eigen_rsp_valid;
 assign eigen_rsp_ready=rsp_ready;
 assign rsp_status=eigen_rsp_status;
 assign rsp_min_vector_fp64=eigen_rsp_min_vector_fp64;
 assign rsp_min_value_fp64=eigen_rsp_min_value_fp64;
 assign rsp_second_value_fp64=eigen_rsp_second_value_fp64;
 assign rsp_max_value_fp64=eigen_rsp_max_value_fp64;
 assign rsp_rotations=eigen_rsp_rotations;
 assign shared_req_valid=0;
 assign shared_req_op=0;
 assign shared_req_a=0;
 assign shared_req_b=0;
 assign shared_active=0;
 assign shared_rsp_ready=0;
 end else begin : private_service
 assign eigen_rst_n=0;
 assign eigen_cmd_valid=0;
 assign eigen_cmd_n=0;
 assign eigen_matrix_valid=0;
 assign eigen_matrix_fp64=0;
 assign eigen_matrix_last=0;
 assign eigen_rsp_ready=0;
 jacobi_eigen #(.FP_SHARED(FP_SHARED),.MAX_SEARCH_ROUNDS(MAX_SEARCH_ROUNDS)) core(.shared_req_valid(shared_req_valid),.shared_req_ready(shared_req_ready),.shared_req_op(shared_req_op),.shared_req_a(shared_req_a),.shared_req_b(shared_req_b),.shared_active(shared_active),.shared_rsp_valid(shared_rsp_valid),.shared_rsp_ready(shared_rsp_ready),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_n(cmd_n),.matrix_valid(matrix_valid),.matrix_ready(matrix_ready),.matrix_fp64(matrix_fp64),.matrix_last(matrix_last),.rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.rsp_min_vector_fp64(rsp_min_vector_fp64),.rsp_min_value_fp64(rsp_min_value_fp64),.rsp_second_value_fp64(rsp_second_value_fp64),.rsp_max_value_fp64(rsp_max_value_fp64),.rsp_rotations(rsp_rotations));
 end endgenerate
endmodule
