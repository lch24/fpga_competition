`timescale 1ns / 1ps
//==============================================================================
// min_eigen_core.v — 2×2 张量最小特征值响应核（逐位复刻 kernels::min_eigenvalue）
//------------------------------------------------------------------------------
// 公式（gradient.h，运算语义必须逐级一致）：
//   trace = fp32(RNE(a+c))；det = fp32(RNE(RNE(a×c) − RNE(b×b)))
//   discr = double(trace)·trace − 4.0·det      （fp64；精确整数域，无新舍入）
//   s     = fp64 RNE sqrt( max(0.0, discr) )
//   tmp   = fp64 RNE (trace − s)
//   out   = fp32(RNE( tmp )) × 0.5f            （先转 f32 再 ×0.5）
// 输入值域：a,c∈[0,9363600]，b∈[−1040400,1040400]（fp32 精确整数）。
//
// 端口（冻结契约，集成方依赖）：in_valid/in_ready、in_a/in_b/in_c（fp32 位模式）、
//   out_valid/out_ready、out_resp（fp32 位模式，逐位 = 官方响应）。
//
// 结构：按 7 步串接 fp 库弹性模块；多路汇合用组合 join（两输入同时就绪才前移），
//   单流拆分用 stream_fork2（两输出各带缓冲、独立伸缩）。每个算术/连接模块内部都
//   有 sync_fifo 吸收背压，join 的"等待"即对其上游 FIFO 施加背压，逐级冻结不丢数。
//
// 时序方向性：所有 backpressure 只沿数据反方向传播，无 ready 组合环，因此任意
//   下游反压（out_ready 乱波）下不丢、不死锁、保序。
// 延迟：in 拍→out 拍含各模块 FIFO 抖动与组合 isqrt，端到端 ~20 拍量级。
//==============================================================================

//------------------------------------------------------------------------------
// stream_fork2：单流→双流，两输出各自带缓冲、可独立伸缩（解除上下游互斥）
//------------------------------------------------------------------------------
module stream_fork2 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [63:0] in_data,
    output wire        out1_valid,
    input  wire        out1_ready,
    output wire [63:0] out1_data,
    output wire        out2_valid,
    input  wire        out2_ready,
    output wire [63:0] out2_data
);
    localparam AW = 5;
    localparam THRESH = 31;
    wire [AW:0] c1, c2;
    wire w1, w2;
    assign in_ready = (c1 < THRESH) && (c2 < THRESH);
    wire do_w = in_valid && in_ready;

    sync_fifo #(.DATA_WIDTH(64), .ADDR_WIDTH(AW)) u_f1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(do_w), .in_ready(w1), .in_data(in_data),
        .out_valid(out1_valid), .out_ready(out1_ready), .out_data(out1_data),
        .count(c1), .empty(), .full()
    );
    sync_fifo #(.DATA_WIDTH(64), .ADDR_WIDTH(AW)) u_f2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(do_w), .in_ready(w2), .in_data(in_data),
        .out_valid(out2_valid), .out_ready(out2_ready), .out_data(out2_data),
        .count(c2), .empty(), .full()
    );
endmodule

//------------------------------------------------------------------------------
// min_eigen_core：顶层响应核
//------------------------------------------------------------------------------
module min_eigen_core (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [31:0] in_a,
    input  wire [31:0] in_b,
    input  wire [31:0] in_c,
    output wire        out_valid,
    input  wire        out_ready,
    output wire [31:0] out_resp
);

    //================== 输入三分叉：trace / a*c / b*b ==========================
    // 三个输入模块各自带 FIFO 冻结，若直接用 in_valid 作为各自 in_valid，则一方
    //   backpressure（in_ready=0）期间另一方可就绪的模块会在 TB 保持 in_valid
    //   的多个周期里重复采样同一 token（oversample），导致三路逆代错位。
    // 修正：以全局 all_ready 同时门控三口 in_valid —— 三者只在本拍同时可接收时
    //   才各自 assert，保证一拍一 token、三路严格同帧进（lockstep capture）。
    wire        add_rdy, mac_rdy, mbb_rdy;
    wire        trace_v, ac_v, bb_v;
    wire [31:0] trace_d, ac_d, bb_d;
    wire all_rdy = add_rdy && mac_rdy && mbb_rdy;
    wire g_in    = in_valid && all_rdy;
    assign in_ready = all_rdy;

    fp32_add u_trace (.clk(clk),.rst_n(rst_n),.in_valid(g_in),.in_ready(add_rdy),
                      .in_a(in_a),.in_b(in_c),.out_valid(trace_v),.out_ready(fk_in_rdy),.out_r(trace_d));
    fp32_mul u_ac    (.clk(clk),.rst_n(rst_n),.in_valid(g_in),.in_ready(mac_rdy),
                      .in_a(in_a),.in_b(in_c),.out_valid(ac_v),.out_ready(join_ab_rdy),.out_r(ac_d));
    fp32_mul u_bb    (.clk(clk),.rst_n(rst_n),.in_valid(g_in),.in_ready(mbb_rdy),
                      .in_a(in_b),.in_b(in_b),.out_valid(bb_v),.out_ready(join_ab_rdy),.out_r(bb_d));

    //================== det = fp32_sub(ac, bb)（组合 join） ======================
    wire det_in_rdy; wire det_v; wire [31:0] det_d;
    assign join_ab_rdy = (ac_v && bb_v) ? det_in_rdy : 1'b0;
    assign det_v       = ac_v && bb_v;

    fp32_sub u_det (.clk(clk),.rst_n(rst_n),.in_valid(det_v),.in_ready(det_in_rdy),
                    .in_a(ac_d),.in_b(bb_d),.out_valid(det_o_v),.out_ready(four_in_rdy),.out_r(det_d));

    //================== f32→f64（trace/det 提升均精确） =========================
    wire [10:0] tre64 = {3'b000, trace_d[30:23]} + 11'd896;
    wire [63:0] trace64 = {trace_d[31], tre64, trace_d[22:0], 29'd0};
    wire [10:0] dre64 = {3'b000, det_d[30:23]} + 11'd896;
    wire [63:0] det64 = {det_d[31], dre64, det_d[22:0], 29'd0};

    //================== trace 拆分：t² 路径 / 末段相减路径 ======================
    wire f1_v, f1_rdy, f2_v, f2_rdy; wire [63:0] f1_d, f2_d;
    stream_fork2 u_fk (
        .clk(clk),.rst_n(rst_n),
        .in_valid(trace_v),.in_ready(fk_in_rdy),.in_data(trace64),
        .out1_valid(f1_v),.out1_ready(f1_rdy),.out1_data(f1_d),
        .out2_valid(f2_v),.out2_ready(join_tms_rdy),.out2_data(f2_d)
    );

    // t² = trace64×trace64（fp64）
    wire t2_v, t2_rdy; wire [63:0] t2_d;
    fp64_mul u_t2 (.clk(clk),.rst_n(rst_n),.in_valid(f1_v),.in_ready(f1_rdy),
                   .in_a(f1_d),.in_b(f1_d),.out_valid(t2_v),.out_ready(join_disc_rdy),.out_r(t2_d));

    // 4·det（fp64，4.0 = 0x4010000000000000）
    wire four_v, four_rdy; wire [63:0] four_d;
    fp64_mul u_4  (.clk(clk),.rst_n(rst_n),.in_valid(det_o_v),.in_ready(four_in_rdy),
                   .in_a(det64),.in_b(64'h4010000000000000),.out_valid(four_v),.out_ready(join_disc_rdy),.out_r(four_d));

    //================== discr = t² − 4det（组合 join） ==========================
    wire disc_in_rdy; wire disc_v, disc_o_v; wire [63:0] disc_d;
    assign join_disc_rdy = (t2_v && four_v) ? disc_in_rdy : 1'b0;
    assign disc_v        = t2_v && four_v;

    fp64_sub u_disc (.clk(clk),.rst_n(rst_n),.in_valid(disc_v),.in_ready(disc_in_rdy),
                     .in_a(t2_d),.in_b(four_d),.out_valid(disc_o_v),.out_ready(sqrt_in_rdy),.out_r(disc_d));

    //================== s = RNE sqrt( max(0, discr) ) ===========================
    wire [63:0] sq_in = disc_d[63] ? 64'd0 : disc_d;
    wire s_v, s_rdy; wire [63:0] s_d;
    fp64_sqrt u_sqrt (.clk(clk),.rst_n(rst_n),.in_valid(disc_o_v),.in_ready(sqrt_in_rdy),
                      .in_x(sq_in),.out_valid(s_v),.out_ready(join_tms_rdy),.out_r(s_d));

    //================== tmp = trace − s（组合 join，trace 走 fork2 分支2）=========
    wire tms_in_rdy; wire tms_v, tms_o_v; wire [63:0] tms_d;
    assign join_tms_rdy = (f2_v && s_v) ? tms_in_rdy : 1'b0;
    assign tms_v        = f2_v && s_v;

    fp64_sub u_tms (.clk(clk),.rst_n(rst_n),.in_valid(tms_v),.in_ready(tms_in_rdy),
                    .in_a(f2_d),.in_b(s_d),.out_valid(tms_o_v),.out_ready(tms_out_rdy),.out_r(tms_d));

    //================== r32 = f32(RNE(trace−s))，再 × 0.5f ======================
    wire r32_v, r32_rdy; wire [31:0] r32_d;
    f64_to_f32 u_cvt (.clk(clk),.rst_n(rst_n),.in_valid(tms_o_v),.in_ready(tms_out_rdy),
                      .in_x(tms_d),.out_valid(r32_v),.out_ready(r32_rdy),.out_r(r32_d));
    fp32_mul u_half (.clk(clk),.rst_n(rst_n),.in_valid(r32_v),.in_ready(r32_rdy),
                     .in_a(r32_d),.in_b(32'h3F000000),.out_valid(out_valid),.out_ready(out_ready),.out_r(out_resp));

endmodule