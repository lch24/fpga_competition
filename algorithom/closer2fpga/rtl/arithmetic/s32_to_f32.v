`timescale 1ns / 1ps
//==============================================================================
// s32_to_f32.v — 有符号 32 位整数 → IEEE754 fp32（精确，无舍入）
//------------------------------------------------------------------------------
// 用途：把 tensor_window_sum 的整数张量和（s32 位模式，如 17=0x11）转换为
//   min_eigen_core 要求的 fp32 位模式（17=0x41880000）。
// 数值前提：|in_data| < 2^24（本域 |Σxx|,|Σyy|≤9363600、|Σxy|≤1040400），
//   整数在 fp32 中可精确表示 → 本模块始终精确，无舍入。
//
// 转换：mag=|s|；e = mag 最高置位索引（0..23）；mag>0 时
//   结果 = {sign, e+127, (mag << (23-e))[22:0]}；mag==0 → 正零 0x00000000。
//   （隐式 1 位由 e 选中的 mag[23] 充当；无被舍掉的低位，因 mag≤2^24-1。）
//
// 时序：1 拍寄存（同 bgr_to_gray/sobel_core 模式），in_ready = out_ready 透传。
//==============================================================================
module s32_to_f32 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [31:0] in_data,
    output reg         out_valid,
    input  wire        out_ready,
    output reg  [31:0] out_r
);

    assign in_ready = out_ready;

    wire [31:0] mag    = in_data[31] ? (~in_data + 32'd1) : in_data;
    wire        is_neg = in_data[31];

    // 最高置位索引（0..30；mag==0 时取 0，结果走零路径被掩掉）
    reg [4:0] e;
    integer k;
    always @(*) begin
        e = 5'd0;
        for (k = 0; k < 31; k = k + 1)
            if (mag[k])
                e = k[4:0];
    end

    wire [31:0] shifted = mag << (5'd23 - e);
    // exp8 = 127+e（e≤23 → ≤150，用 8 位避免溢出；与 verilog 位宽规则小心）
    wire [7:0]  exp8    = 8'd127 + e;
    wire [31:0] r = {is_neg, exp8, shifted[22:0]};
    wire [31:0] r_out = (mag == 32'd0) ? 32'h0000_0000 : r;

    always @(posedge clk) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            out_r     <= 32'd0;
        end else if (in_valid && in_ready) begin
            out_valid <= 1'b1;
            out_r     <= r_out;
        end else if (out_ready) begin
            out_valid <= 1'b0;
        end
    end

endmodule