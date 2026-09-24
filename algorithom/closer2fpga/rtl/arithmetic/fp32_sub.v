`timescale 1ns / 1ps
//==============================================================================
// fp32_sub.v — 单精度浮点减法（RNE）
//------------------------------------------------------------------------------
// 薄封装：fp32_sub(a, b) == fp32_add(a, -b)，即翻转 in_b 的符号位后复用 fp32_add。
// 弹性流接口与延迟同 fp32_add（接受拍→FIFO 出拍 2 拍，水位冻结）。
// 无独立算术；其余契约（RNE、输入假设、冻结语义）见 fp32_add.v。
//==============================================================================
module fp32_sub (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [31:0] in_a,
    input  wire [31:0] in_b,
    output wire        out_valid,
    input  wire        out_ready,
    output wire [31:0] out_r
);

    wire [31:0] negb = {~in_b[31], in_b[30:0]};

    fp32_add u_sub (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (in_valid),
        .in_ready (in_ready),
        .in_a     (in_a),
        .in_b     (negb),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_r    (out_r)
    );

endmodule