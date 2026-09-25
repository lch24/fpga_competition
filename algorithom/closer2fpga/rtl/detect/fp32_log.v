`timescale 1ns / 1ps
//==============================================================================
// fp32_log.v — 自定义 fp32 log（位级 = export_m4.cpp::log_ref）
//------------------------------------------------------------------------------
// 权威公式（Horner 嵌套，每次运算逐次 fp32 舍入，与 C++ 同序）：
//   z  = (x - 1) / (x + 1)            （fp32_sub / fp32_add 并行 → fp32_div）
//   z2 = z * z
//   p  = 1/15 + z2*(1/13 + z2*(1/11 + z2*(1/9 + z2*(1/7 + z2*(1/5 + z2*(1/3 + z2*1))))))
//   return 2.0f * (z * p)             （先 z*p 再 ×2，严格按 C++ 运算顺序）
// 每级：mul(z2, c) 先 → add(c', mul) 后；位模式常量由 g++ 同机打印：
//   c13=3eaaaaab c15=3e4ccccd c17=3e124925 c19=3de38e39
//   c111=3dba2e8c c113=3d9d89d9 c115=3d888889 one=3f800000 two=40000000
// 说明：实测 libm logf ≠ (float)log(double)（1ulp 差），本项目以 log_ref 为
//   位级权威（M4_REPORT 已记录），RTL 与 log_ref 逐位一致即可，不追求 libm。
//
// 实现：弹性握手级联流水（同 fp32_hypot 风格），z/z2 因被多次消费
//   （z: z2 与最终 z*p；z2: Horner 7 级）锁存到寄存器，单飞行（z2_rv）——
//   一个输入在整条链完成（m9 输出被接受）前不接受新输入，避免数据竞争。
// 延迟：接受拍 → out_valid 约 40 拍（各级模块 2 拍 + 级联）。
//==============================================================================
module fp32_log (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [31:0] in_x,
    output wire        out_valid,
    input  wire        out_ready,
    output wire [31:0] out_r
);

    // 位模式常量（g++ 同机打印，勿自算）
    localparam [31:0] ONE  = 32'h3f800000;
    localparam [31:0] TWO  = 32'h40000000;
    localparam [31:0] C13  = 32'h3eaaaaab;
    localparam [31:0] C15  = 32'h3e4ccccd;
    localparam [31:0] C17  = 32'h3e124925;
    localparam [31:0] C19  = 32'h3de38e39;
    localparam [31:0] C111 = 32'h3dba2e8c;
    localparam [31:0] C113 = 32'h3d9d89d9;
    localparam [31:0] C115 = 32'h3d888889;

    //====================================================================
    // 级间信号（先声明后引用，避免隐式 net）
    //====================================================================
    wire sub_v, add_v, div_v, z2_v, m1_v, a1_v, m2_v, a2_v, m3_v, a3_v;
    wire m4_v, a4_v, m5_v, a5_v, m6_v, a6_v, m7_v, a7_v, m8_v, m9_v;
    wire sub_rdy, add_rdy, div_irdy, z2_irdy, z2_rdy;
    wire m1_rdy, a1_rdy, m2_rdy, a2_rdy, m3_rdy, a3_rdy, m4_rdy, a4_rdy;
    wire m5_rdy, a5_rdy, m6_rdy, a6_rdy, m7_rdy, a7_rdy, m8_rdy, m9_rdy;
    wire [31:0] sub_d, add_d, div_d, z2_d, m1_d, a1_d, m2_d, a2_d, m3_d, a3_d;
    wire [31:0] m4_d, a4_d, m5_d, a5_d, m6_d, a6_d, m7_d, a7_d, m8_d, m9_d;

    //--------------------------------------------------------------------
    // 级 1：x-1 与 x+1（并行）
    //--------------------------------------------------------------------
    fp32_sub u_sub (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_ready(sub_rdy),
        .in_a(in_x), .in_b(ONE),
        .out_valid(sub_v), .out_ready(div_irdy), .out_r(sub_d)
    );
    fp32_add u_add (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_ready(add_rdy),
        .in_a(in_x), .in_b(ONE),
        .out_valid(add_v), .out_ready(div_irdy), .out_r(add_d)
    );

    //--------------------------------------------------------------------
    // 级 2：z = (x-1)/(x+1)
    //--------------------------------------------------------------------
    fp32_div u_div (
        .clk(clk), .rst_n(rst_n),
        .in_valid(sub_v && add_v), .in_ready(div_irdy),
        .in_a(sub_d), .in_b(add_d),
        .out_valid(div_v), .out_ready(z2_irdy), .out_r(div_d)
    );

    //--------------------------------------------------------------------
    // 级 3：z2 = z*z；同时锁存 z（供链尾 z*p）
    //--------------------------------------------------------------------
    fp32_mul u_z2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(div_v), .in_ready(z2_irdy),
        .in_a(div_d), .in_b(div_d),
        .out_valid(z2_v), .out_ready(z2_rdy), .out_r(z2_d)
    );

    // 单飞行控制：z2 锁存完成标志（链进行中保持，m9 输出接受后清零）
    reg z2_rv;
    reg [31:0] z_r, z2_r;
    reg m1_fired;

    assign z2_rdy = !z2_rv;                       // z2 仅当 z2_r 空闲时出队
    wire z_latch  = div_v && z2_irdy;             // z 在 div 出队拍锁存
    wire z2_latch = z2_v && !z2_rv;               // z2 锁存
    wire m1_go    = z2_rv && !m1_fired;           // m1 单次触发
    wire m9_acc   = m9_v && out_ready;            // 链完成（输出被接受）

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            z2_rv    <= 1'b0;
            m1_fired <= 1'b0;
            z_r      <= 32'd0;
            z2_r     <= 32'd0;
        end else begin
            if (z_latch)  z_r  <= div_d;
            if (z2_latch) z2_r <= z2_d;
            if (z2_latch)      z2_rv <= 1'b1;
            else if (m9_acc)   z2_rv <= 1'b0;
            if (!z2_rv)        m1_fired <= 1'b0;
            else if (m1_go)    m1_fired <= 1'b1;
        end
    end

    //--------------------------------------------------------------------
    // Horner 链：每级 mul(z2_r, c) → add(c', mul)
    //--------------------------------------------------------------------
    fp32_mul u_m1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(m1_go), .in_ready(m1_rdy),
        .in_a(z2_r), .in_b(C113),
        .out_valid(m1_v), .out_ready(a1_rdy), .out_r(m1_d)
    );
    fp32_add u_a1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(m1_v), .in_ready(a1_rdy),
        .in_a(C115), .in_b(m1_d),
        .out_valid(a1_v), .out_ready(m2_rdy), .out_r(a1_d)
    );
    fp32_mul u_m2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(a1_v), .in_ready(m2_rdy),
        .in_a(z2_r), .in_b(a1_d),
        .out_valid(m2_v), .out_ready(a2_rdy), .out_r(m2_d)
    );
    fp32_add u_a2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(m2_v), .in_ready(a2_rdy),
        .in_a(C111), .in_b(m2_d),
        .out_valid(a2_v), .out_ready(m3_rdy), .out_r(a2_d)
    );
    fp32_mul u_m3 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(a2_v), .in_ready(m3_rdy),
        .in_a(z2_r), .in_b(a2_d),
        .out_valid(m3_v), .out_ready(a3_rdy), .out_r(m3_d)
    );
    fp32_add u_a3 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(m3_v), .in_ready(a3_rdy),
        .in_a(C19), .in_b(m3_d),
        .out_valid(a3_v), .out_ready(m4_rdy), .out_r(a3_d)
    );
    fp32_mul u_m4 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(a3_v), .in_ready(m4_rdy),
        .in_a(z2_r), .in_b(a3_d),
        .out_valid(m4_v), .out_ready(a4_rdy), .out_r(m4_d)
    );
    fp32_add u_a4 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(m4_v), .in_ready(a4_rdy),
        .in_a(C17), .in_b(m4_d),
        .out_valid(a4_v), .out_ready(m5_rdy), .out_r(a4_d)
    );
    fp32_mul u_m5 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(a4_v), .in_ready(m5_rdy),
        .in_a(z2_r), .in_b(a4_d),
        .out_valid(m5_v), .out_ready(a5_rdy), .out_r(m5_d)
    );
    fp32_add u_a5 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(m5_v), .in_ready(a5_rdy),
        .in_a(C15), .in_b(m5_d),
        .out_valid(a5_v), .out_ready(m6_rdy), .out_r(a5_d)
    );
    fp32_mul u_m6 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(a5_v), .in_ready(m6_rdy),
        .in_a(z2_r), .in_b(a5_d),
        .out_valid(m6_v), .out_ready(a6_rdy), .out_r(m6_d)
    );
    fp32_add u_a6 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(m6_v), .in_ready(a6_rdy),
        .in_a(C13), .in_b(m6_d),
        .out_valid(a6_v), .out_ready(m7_rdy), .out_r(a6_d)
    );
    fp32_mul u_m7 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(a6_v), .in_ready(m7_rdy),
        .in_a(z2_r), .in_b(a6_d),
        .out_valid(m7_v), .out_ready(a7_rdy), .out_r(m7_d)
    );
    fp32_add u_a7 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(m7_v), .in_ready(a7_rdy),
        .in_a(ONE), .in_b(m7_d),
        .out_valid(a7_v), .out_ready(m8_rdy), .out_r(a7_d)
    );

    //--------------------------------------------------------------------
    // 链尾：z*p 先，再 ×2（C++: 2.0f * (z * p)）
    //--------------------------------------------------------------------
    fp32_mul u_m8 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(a7_v), .in_ready(m8_rdy),
        .in_a(z_r), .in_b(a7_d),
        .out_valid(m8_v), .out_ready(m9_rdy), .out_r(m8_d)
    );
    fp32_mul u_m9 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(m8_v), .in_ready(m9_rdy),
        .in_a(TWO), .in_b(m8_d),
        .out_valid(m9_v), .out_ready(out_ready), .out_r(m9_d)
    );

    assign out_valid = m9_v;
    assign out_r     = m9_d;
    assign in_ready  = sub_rdy && add_rdy;

endmodule
