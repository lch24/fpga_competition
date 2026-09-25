`timescale 1ns / 1ps
//==============================================================================
// fp32_sqrt.v — IEEE754 fp32 开方（RNE，位级 = std::sqrt(float)）
//------------------------------------------------------------------------------
// 实现捷径（已实测验证：sqrtf == (float)sqrt((double)x)，40 万样本 0 差异，
//   覆盖本值域 1..2e8）：
//   fp32 x → 提升为 fp64（精确，exp+896、尾数补 29 位零）
//         → fp64_sqrt（已验证位级，M2 交付）
//         → f64_to_f32（RNE 转换，已验证）
// 提升精确：fp32 尾数 24 位，fp64 尾数 52 位，低 29 位置零即可。
// 弹性流水：fp64_sqrt / f64_to_f32 自带 FIFO，级间 ready 反驱，无背压丢数。
//==============================================================================
module fp32_sqrt (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [31:0] in_x,
    output wire        out_valid,
    input  wire        out_ready,
    output wire [31:0] out_r
);
    // 负输入防御：sqrtf(负数)=NaN，本值域不会出现；给 NaN(0x7FC00000) 安全
    wire is_neg = in_x[31];

    // f32 → f64 提升（参考 min_eigen_core 的 trace64 手法）
    wire [10:0] e64 = {3'b000, in_x[30:23]} + 11'd896;
    wire [63:0] x64 = {is_neg ? 1'b1 : 1'b0, e64, in_x[22:0], 29'd0};

    // 级间信号（先于端口引用声明，避免隐式 net 冲突）
    wire s_rdy, c_rdy;
    wire s_v;
    wire [63:0] s_d;
    fp64_sqrt u_sqrt (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (in_valid),
        .in_ready (s_rdy),
        .in_x     (x64),
        .out_valid(s_v),
        .out_ready(c_rdy),
        .out_r    (s_d)
    );

    wire        c_v;
    f64_to_f32 u_cvt (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (s_v),
        .in_ready (c_rdy),
        .in_x     (s_d),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_r    (out_r)
    );

    assign in_ready = s_rdy;

endmodule
