`timescale 1ns / 1ps
//==============================================================================
// tensor_solve.v — double 2×2 求解核（逐位复刻 kernels::solve_tensor）
//------------------------------------------------------------------------------
// 语义（与 C++ 权威逐位一致，运算顺序严格，全部 FP64 RNE）：
//   det   = a*c - b*b                    （fp64_mul → fp64_mul → fp64_sub）
//   trace = a + c                        （fp64_add）
//   if (trace < 1e-8 || det <= 1e-5*trace*trace) → ok=0
//     注：1e-5*trace*trace 为左结合（(1e-5*trace)*trace，两次 fp64_mul）
//   dx = (c*bx - b*by) / det             （mul → mul → sub → div）
//   dy = (a*by - b*bx) / det
// 常量：1e-5/1e-8 位模式取 tests/build/vectors/m5_const.txt。
//   比较用 IEEE-754 补码序：无 NaN 时 s64 位序 == 实数序。
//
// 时序：start/busy/done 状态机，子单元弹性握手（sync_fifo 水位冻结）；
//   每例串行，busy 全程拉高，done 置位后回 IDLE（TB 见 done 再发下一例）。
//==============================================================================
module tensor_solve (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    output reg         busy,
    output reg         done,
    input  wire [63:0] in_a,
    input  wire [63:0] in_b,
    input  wire [63:0] in_c,
    input  wire [63:0] in_bx,
    input  wire [63:0] in_by,
    output reg         out_ok,
    output reg  [63:0] out_dx,
    output reg  [63:0] out_dy
);

    localparam [63:0] C_1E5 = 64'h3ee4f8b588e368f1;   // 1e-5（m5_const.txt）
    localparam [63:0] C_1E8 = 64'h3e45798ee2308c3a;   // 1e-8

    localparam S_IDLE = 4'd0, S1 = 4'd1, S2 = 4'd2, S3 = 4'd3, S4 = 4'd4,
               S5 = 4'd5, S6 = 4'd6, S7 = 4'd7, S8 = 4'd8, S9 = 4'd9;
    reg [3:0] state;

    reg [63:0] a_r, b_r, c_r, bx_r, by_r;
    reg [63:0] trace_reg, det_reg;
    reg        ok_flag;

    //--------------------------------------------------------------------
    // 发射/消费信号（声明先于例化端口使用，避免隐式 net）
    //--------------------------------------------------------------------
    wire s1_fire, s2_fire, s3_fire, s4_fire, s5_fire,
         s6_fire, s7_fire, s8_fire, done_fire;

    //--------------------------------------------------------------------
    // 子单元互连
    //--------------------------------------------------------------------
    wire ac_rdy, bb_rdy, tr_rdy, sd_rdy, k1_rdy, k2_rdy;
    wire ac_v, bb_v, tr_v, sd_v, k1_v, k2_v;
    wire [63:0] ac_r, bb_r, tr_r, sd_r, k1_r, k2_r;
    wire cbx_rdy, bby_rdy, aby_rdy, bbx_rdy, n1_rdy, n2_rdy, ddx_rdy, ddy_rdy;
    wire cbx_v, bby_v, aby_v, bbx_v, n1_v, n2_v, ddx_v, ddy_v;
    wire [63:0] cbx_r, bby_r, aby_r, bbx_r, n1_r, n2_r, ddx_r, ddy_r;

    assign s1_fire  = (state == S1) && ac_rdy && bb_rdy && tr_rdy;
    assign s2_fire  = (state == S2) && ac_v && bb_v && sd_rdy;
    assign s3_fire  = (state == S3) && tr_v && k1_rdy;
    assign s4_fire  = (state == S4) && k1_v && k2_rdy;
    assign s5_fire  = (state == S5) && k2_v && sd_v;
    assign s6_fire  = (state == S6) && cbx_rdy && bby_rdy && aby_rdy && bbx_rdy;
    assign s7_fire  = (state == S7) && cbx_v && bby_v && aby_v && bbx_v
                      && n1_rdy && n2_rdy;
    assign s8_fire  = (state == S8) && n1_v && n2_v && ddx_rdy && ddy_rdy;
    assign done_fire = (state == S9) && ddx_v && ddy_v;

    fp64_mul u_mul_ac (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s1_fire), .in_ready(ac_rdy),
        .in_a(a_r), .in_b(c_r),
        .out_valid(ac_v), .out_ready(s2_fire), .out_r(ac_r)
    );
    fp64_mul u_mul_bb (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s1_fire), .in_ready(bb_rdy),
        .in_a(b_r), .in_b(b_r),
        .out_valid(bb_v), .out_ready(s2_fire), .out_r(bb_r)
    );
    fp64_add u_add_tr (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s1_fire), .in_ready(tr_rdy),
        .in_a(a_r), .in_b(c_r),
        .out_valid(tr_v), .out_ready(s3_fire), .out_r(tr_r)
    );
    fp64_sub u_sub_det (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s2_fire), .in_ready(sd_rdy),
        .in_a(ac_r), .in_b(bb_r),
        .out_valid(sd_v), .out_ready(s5_fire), .out_r(sd_r)
    );
    fp64_mul u_mul_k1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s3_fire), .in_ready(k1_rdy),
        .in_a(C_1E5), .in_b(tr_r),
        .out_valid(k1_v), .out_ready(s4_fire), .out_r(k1_r)
    );
    fp64_mul u_mul_k2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s4_fire), .in_ready(k2_rdy),
        .in_a(k1_r), .in_b(trace_reg),
        .out_valid(k2_v), .out_ready(s5_fire), .out_r(k2_r)
    );
    fp64_mul u_mul_cbx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s6_fire), .in_ready(cbx_rdy),
        .in_a(c_r), .in_b(bx_r),
        .out_valid(cbx_v), .out_ready(s7_fire), .out_r(cbx_r)
    );
    fp64_mul u_mul_bby (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s6_fire), .in_ready(bby_rdy),
        .in_a(b_r), .in_b(by_r),
        .out_valid(bby_v), .out_ready(s7_fire), .out_r(bby_r)
    );
    fp64_mul u_mul_aby (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s6_fire), .in_ready(aby_rdy),
        .in_a(a_r), .in_b(by_r),
        .out_valid(aby_v), .out_ready(s7_fire), .out_r(aby_r)
    );
    fp64_mul u_mul_bbx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s6_fire), .in_ready(bbx_rdy),
        .in_a(b_r), .in_b(bx_r),
        .out_valid(bbx_v), .out_ready(s7_fire), .out_r(bbx_r)
    );
    fp64_sub u_sub_n1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s7_fire), .in_ready(n1_rdy),
        .in_a(cbx_r), .in_b(bby_r),
        .out_valid(n1_v), .out_ready(s8_fire), .out_r(n1_r)
    );
    fp64_sub u_sub_n2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s7_fire), .in_ready(n2_rdy),
        .in_a(aby_r), .in_b(bbx_r),
        .out_valid(n2_v), .out_ready(s8_fire), .out_r(n2_r)
    );
    fp64_div u_div_dx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s8_fire), .in_ready(ddx_rdy),
        .in_a(n1_r), .in_b(det_reg),
        .out_valid(ddx_v), .out_ready(done_fire), .out_r(ddx_r)
    );
    fp64_div u_div_dy (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s8_fire), .in_ready(ddy_rdy),
        .in_a(n2_r), .in_b(det_reg),
        .out_valid(ddy_v), .out_ready(done_fire), .out_r(ddy_r)
    );

    // 比较（IEEE-754 位模式按补码序 == 实数序；trace 用 S3 已锁存的 reg，
    //   det/k2 在 s5_fire 拍组合使用并消费）
    wire trace_lt = ($signed(trace_reg) < $signed(C_1E8));
    wire det_le   = ($signed(sd_r) <= $signed(k2_r));

    //--------------------------------------------------------------------
    // 状态机
    //--------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            busy    <= 1'b0;
            done    <= 1'b0;
            out_ok  <= 1'b0;
            out_dx  <= 64'd0;
            out_dy  <= 64'd0;
            ok_flag <= 1'b0;
            a_r <= 64'd0; b_r <= 64'd0; c_r <= 64'd0; bx_r <= 64'd0; by_r <= 64'd0;
            trace_reg <= 64'd0;
            det_reg   <= 64'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        busy <= 1'b1;
                        done <= 1'b0;
                        a_r <= in_a; b_r <= in_b; c_r <= in_c;
                        bx_r <= in_bx; by_r <= in_by;
                        state <= S1;
                    end
                end
                S1: if (s1_fire)  state <= S2;
                S2: if (s2_fire)  state <= S3;
                S3: begin
                    if (s3_fire) begin
                        trace_reg <= tr_r;
                        state <= S4;
                    end
                end
                S4: if (s4_fire)  state <= S5;
                S5: begin
                    if (s5_fire) begin
                        det_reg <= sd_r;
                        if (trace_lt || det_le) begin
                            ok_flag <= 1'b0;
                            out_ok  <= 1'b0;
                            done    <= 1'b1;
                            busy    <= 1'b0;
                            state   <= S_IDLE;
                        end else begin
                            ok_flag <= 1'b1;
                            state   <= S6;
                        end
                    end
                end
                S6: if (s6_fire)  state <= S7;
                S7: if (s7_fire)  state <= S8;
                S8: if (s8_fire)  state <= S9;
                S9: begin
                    if (done_fire) begin
                        out_ok  <= 1'b1;
                        out_dx  <= ddx_r;
                        out_dy  <= ddy_r;
                        done    <= 1'b1;
                        busy    <= 1'b0;
                        state   <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule