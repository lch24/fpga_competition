`timescale 1ns / 1ps
//==============================================================================
// fp32_hypot.v — 欧氏距离核（位级 = std::hypot(float,float)）
//------------------------------------------------------------------------------
// 实测：std::hypot(float,float) == (float)hypot((double)a,(double)b)（全部 SAME）。
//   hypotf 内部走 double 精确计算再转 fp32；与 fp32 sqrt 链（mul+add+sqrtf）
//   在个别输入差 1 ulp（dx=9678,dy=-2210：hypot=0x461b1c7f，sqrt 链=0x461b1c7e）。
// 因此本实现走 double 路径：
//   f32→f64 提升（精确）→ fp64_mul(a,a) / fp64_mul(b,b) → fp64_add（用 fp64_sub
//   翻转符号，因 a²+b²≥0 无正负零歧义）→ fp64_sqrt → f64_to_f32。
//   dx²+dy² ≤ ~2.9e8 < 2^53：double 域乘加均精确；sqrt 正确舍入 → 位级一致。
//
// 弹性流水：级间 ready 反驱；join 用组合 &&（m1_v && m2_v），输入锁步。
//==============================================================================
module fp32_hypot (
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
    // f32 → f64 提升（精确：尾数补 29 位零，指数 +896）
    wire [63:0] a64 = {in_a[31], {3'b000, in_a[30:23]} + 11'd896, in_a[22:0], 29'd0};
    wire [63:0] b64 = {in_b[31], {3'b000, in_b[30:23]} + 11'd896, in_b[22:0], 29'd0};

    // 级间信号（先于端口引用声明，避免隐式 net）
    wire m1_rdy, m2_rdy, add_rdy, s_rdy, c_rdy;
    wire m1_v, m2_v, add_v, s_v;
    wire [63:0] m1_d, m2_d, add_d, s_d;

    fp64_mul u_m1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_ready(m1_rdy),
        .in_a(a64), .in_b(a64),
        .out_valid(m1_v), .out_ready(add_rdy), .out_r(m1_d)
    );
    fp64_mul u_m2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_ready(m2_rdy),
        .in_a(b64), .in_b(b64),
        .out_valid(m2_v), .out_ready(add_rdy), .out_r(m2_d)
    );
    // fp64_add = fp64_sub(a, -b)：翻转 b 符号
    wire [63:0] m2_neg = {~m2_d[63], m2_d[62:0]};
    fp64_sub u_add (
        .clk(clk), .rst_n(rst_n),
        .in_valid(m1_v && m2_v), .in_ready(add_rdy),
        .in_a(m1_d), .in_b(m2_neg),
        .out_valid(add_v), .out_ready(s_rdy), .out_r(add_d)
    );
    fp64_sqrt u_sqrt (
        .clk(clk), .rst_n(rst_n),
        .in_valid(add_v), .in_ready(s_rdy),
        .in_x(add_d),
        .out_valid(s_v), .out_ready(c_rdy), .out_r(s_d)
    );
    f64_to_f32 u_cvt (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s_v), .in_ready(c_rdy),
        .in_x(s_d),
        .out_valid(out_valid), .out_ready(out_ready), .out_r(out_r)
    );

    assign in_ready = m1_rdy && m2_rdy;

endmodule
