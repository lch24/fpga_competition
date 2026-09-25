`timescale 1ns / 1ps
//==============================================================================
// subpixel_accum.v — M5 亚像素：高斯加权梯度五项 FP64 累加核
//------------------------------------------------------------------------------
// 语义（位级权威 tests/rtl/export_m5.cpp::export_acc，逐行一致）：
//   窗口样本按 y 外循环、x 内循环逐项喂入（顺序即 m5_acc.bin 段内顺序）。
//     xx = w*gx*gx   xy = w*gx*gy   yy = w*gy*gy   （左结合；共享 w*gx、w*gy）
//     a += xx;  b += xy;  c += yy;
//     bx += xx*x + xy*y   （t1=xx*x; t2=xy*y; t3=t1+t2; bx+=t3，先乘后加再累加）
//     by += xy*x + yy*y   （同构）
//   x/y 为窗口偏移整数，以 fp64 位模式直通（不做转换）。
//
// 数据通路：3×fp64_mul + 2×fp64_add（弹性握手，算术块输出侧 out_ready 恒 1）。
//   逐样本流水：每个处理阶段"同拍发出、同拍等齐 out_valid"后推进；五个累加器
//   有反馈依赖（文档规划 §6.2 首版语义：等加法写回再消费下一样本）。
//
// 接口：start/busy/done；start 清零五项并锁存 n_win；SAMPLE_WAIT 态 in_ready=1
//   逐样本握手；收齐 n_win 项后 out_valid 一次给出 {a,b,c,bx,by}，out_ready 应答
//   后 done 单拍脉冲、busy 拉低。
//==============================================================================
module subpixel_accum #(
    parameter N_ADDR_W = 8        // 预留：窗口样本寻址位宽（subpixel_ctrl 可选）
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    output reg         busy,
    output reg         done,
    input  wire [15:0] n_win,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [63:0] in_x,
    input  wire [63:0] in_y,
    input  wire [63:0] in_w,
    input  wire [63:0] in_gx,
    input  wire [63:0] in_gy,
    output reg         out_valid,
    input  wire        out_ready,
    output wire [63:0] out_a,
    output wire [63:0] out_b,
    output wire [63:0] out_c,
    output wire [63:0] out_bx,
    output wire [63:0] out_by
);

    localparam ST_IDLE = 4'd0;
    localparam ST_WAIT = 4'd1;   // 等样本（in_valid&&in_ready 接受）
    localparam ST_WG   = 4'd2;   // w*gx -> m1；w*gy -> m2
    localparam ST_OP   = 4'd3;   // m1*gx -> xx；m1*gy -> xy；m2*gy -> yy
    localparam ST_ACC  = 4'd4;   // a+=xx；b+=xy；t1=xx*x；t2=xy*y；t4=xy*x
    localparam ST_C    = 4'd5;   // c+=yy；t5=yy*y；t3=t1+t2
    localparam ST_BX   = 4'd6;   // t6=t4+t5；bx+=t3
    localparam ST_BY   = 4'd7;   // by+=t6
    localparam ST_OUT  = 4'd8;   // 输出 5 项，等 out_ready

    reg  [3:0]  state;
    reg  [3:0]  pstate;
    reg  [15:0] n_win_r;
    reg  [15:0] samples_done;

    reg  [63:0] in_x_r, in_y_r, in_w_r, in_gx_r, in_gy_r;
    reg  [63:0] m1, m2;                 // w*gx, w*gy
    reg  [63:0] xx, xy, yy;
    reg  [63:0] t1, t2, t3, t4, t5, t6;
    reg  [63:0] acc_a, acc_b, acc_c, acc_bx, acc_by;

    assign in_ready = (state == ST_WAIT);
    assign out_a  = acc_a;
    assign out_b  = acc_b;
    assign out_c  = acc_c;
    assign out_bx = acc_bx;
    assign out_by = acc_by;

    // 状态刚进入的那一拍（entry=1）：算术块只在该拍发出，避免整阶段重复发出
    wire entry = (state != pstate);

    //----------------------------------------------------------------------
    // 算术块操作数选择（每阶段各自的乘/加）
    //----------------------------------------------------------------------
    wire [63:0] mu1_a = (state == ST_WG)  ? in_w_r  :
                        (state == ST_OP)  ? m1      :
                        (state == ST_ACC) ? xx      :
                        (state == ST_C)   ? yy      : 64'd0;
    wire [63:0] mu1_b = (state == ST_WG)  ? in_gx_r :
                        (state == ST_OP)  ? in_gx_r :
                        (state == ST_ACC) ? in_x_r  :
                        (state == ST_C)   ? in_y_r  : 64'd0;

    wire [63:0] mu2_a = (state == ST_WG)  ? in_w_r  :
                        (state == ST_OP)  ? m1      :
                        (state == ST_ACC) ? xy      : 64'd0;
    wire [63:0] mu2_b = (state == ST_WG)  ? in_gy_r :
                        (state == ST_OP)  ? in_gy_r :
                        (state == ST_ACC) ? in_y_r  : 64'd0;

    wire [63:0] mu3_a = (state == ST_OP)  ? m2      :
                        (state == ST_ACC) ? xy      : 64'd0;
    wire [63:0] mu3_b = (state == ST_OP)  ? in_gy_r :
                        (state == ST_ACC) ? in_x_r  : 64'd0;

    wire [63:0] ad1_a = (state == ST_ACC) ? acc_a  :
                        (state == ST_C)   ? acc_c  :
                        (state == ST_BX)  ? acc_bx :
                        (state == ST_BY)  ? acc_by : 64'd0;
    wire [63:0] ad1_b = (state == ST_ACC) ? xx :
                        (state == ST_C)   ? yy :
                        (state == ST_BX)  ? t3 :
                        (state == ST_BY)  ? t6 : 64'd0;

    wire [63:0] ad2_a = (state == ST_ACC) ? acc_b :
                        (state == ST_C)   ? t1    :
                        (state == ST_BX)  ? t4    : 64'd0;
    wire [63:0] ad2_b = (state == ST_ACC) ? xy :
                        (state == ST_C)   ? t2 :
                        (state == ST_BX)  ? t5 : 64'd0;

    // 发出拍：仅状态刚进入时 1 拍
    wire mu1_iv = entry && (state == ST_WG || state == ST_OP || state == ST_ACC || state == ST_C);
    wire mu2_iv = entry && (state == ST_WG || state == ST_OP || state == ST_ACC);
    wire mu3_iv = entry && (state == ST_OP  || state == ST_ACC);
    wire ad1_iv = entry && (state == ST_ACC || state == ST_C || state == ST_BX || state == ST_BY);
    wire ad2_iv = entry && (state == ST_ACC || state == ST_C || state == ST_BX);

    wire        mu1_v, mu2_v, mu3_v, ad1_v, ad2_v;
    wire [63:0] mu1_r, mu2_r, mu3_r, ad1_r, ad2_r;

    fp64_mul u_mu1 (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (mu1_iv),
        .in_ready (),
        .in_a     (mu1_a),
        .in_b     (mu1_b),
        .out_valid(mu1_v),
        .out_ready(1'b1),
        .out_r    (mu1_r)
    );
    fp64_mul u_mu2 (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (mu2_iv),
        .in_ready (),
        .in_a     (mu2_a),
        .in_b     (mu2_b),
        .out_valid(mu2_v),
        .out_ready(1'b1),
        .out_r    (mu2_r)
    );
    fp64_mul u_mu3 (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (mu3_iv),
        .in_ready (),
        .in_a     (mu3_a),
        .in_b     (mu3_b),
        .out_valid(mu3_v),
        .out_ready(1'b1),
        .out_r    (mu3_r)
    );
    fp64_add u_ad1 (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (ad1_iv),
        .in_ready (),
        .in_a     (ad1_a),
        .in_b     (ad1_b),
        .out_valid(ad1_v),
        .out_ready(1'b1),
        .out_r    (ad1_r)
    );
    fp64_add u_ad2 (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (ad2_iv),
        .in_ready (),
        .in_a     (ad2_a),
        .in_b     (ad2_b),
        .out_valid(ad2_v),
        .out_ready(1'b1),
        .out_r    (ad2_r)
    );

    //----------------------------------------------------------------------
    // 主 FSM
    //----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            pstate       <= ST_IDLE;
            busy         <= 1'b0;
            done         <= 1'b0;
            out_valid    <= 1'b0;
            n_win_r      <= 16'd0;
            samples_done <= 16'd0;
            in_x_r       <= 64'd0;
            in_y_r       <= 64'd0;
            in_w_r       <= 64'd0;
            in_gx_r      <= 64'd0;
            in_gy_r      <= 64'd0;
            m1           <= 64'd0;
            m2           <= 64'd0;
            xx           <= 64'd0;
            xy           <= 64'd0;
            yy           <= 64'd0;
            t1           <= 64'd0;
            t2           <= 64'd0;
            t3           <= 64'd0;
            t4           <= 64'd0;
            t5           <= 64'd0;
            t6           <= 64'd0;
            acc_a        <= 64'd0;
            acc_b        <= 64'd0;
            acc_c        <= 64'd0;
            acc_bx       <= 64'd0;
            acc_by       <= 64'd0;
        end else begin
            done   <= 1'b0;              // done 单拍脉冲
            pstate <= state;
            case (state)
                ST_IDLE: begin
                    if (start) begin
                        busy      <= 1'b1;
                        n_win_r   <= n_win;
                        samples_done <= 16'd0;
                        acc_a     <= 64'd0;
                        acc_b     <= 64'd0;
                        acc_c     <= 64'd0;
                        acc_bx    <= 64'd0;
                        acc_by    <= 64'd0;
                        if (n_win == 16'd0) begin
                            out_valid <= 1'b1;
                            state     <= ST_OUT;
                        end else
                            state <= ST_WAIT;
                    end
                end

                ST_WAIT: begin
                    if (in_valid && in_ready) begin
                        in_x_r  <= in_x;
                        in_y_r  <= in_y;
                        in_w_r  <= in_w;
                        in_gx_r <= in_gx;
                        in_gy_r <= in_gy;
                        state   <= ST_WG;
                    end
                end

                ST_WG: begin
                    if (mu1_v && mu2_v) begin
                        m1    <= mu1_r;
                        m2    <= mu2_r;
                        state <= ST_OP;
                    end
                end

                ST_OP: begin
                    if (mu1_v && mu2_v && mu3_v) begin
                        xx    <= mu1_r;
                        xy    <= mu2_r;
                        yy    <= mu3_r;
                        state <= ST_ACC;
                    end
                end

                ST_ACC: begin
                    if (ad1_v && ad2_v && mu1_v && mu2_v && mu3_v) begin
                        acc_a <= ad1_r;
                        acc_b <= ad2_r;
                        t1    <= mu1_r;
                        t2    <= mu2_r;
                        t4    <= mu3_r;
                        state <= ST_C;
                    end
                end

                ST_C: begin
                    if (ad1_v && mu1_v && ad2_v) begin
                        acc_c <= ad1_r;
                        t5    <= mu1_r;
                        t3    <= ad2_r;
                        state <= ST_BX;
                    end
                end

                ST_BX: begin
                    if (ad2_v && ad1_v) begin
                        t6     <= ad2_r;
                        acc_bx <= ad1_r;
                        state  <= ST_BY;
                    end
                end

                ST_BY: begin
                    if (ad1_v) begin
                        acc_by       <= ad1_r;
                        samples_done <= samples_done + 16'd1;
                        if (samples_done + 16'd1 == n_win_r) begin
                            out_valid <= 1'b1;
                            state     <= ST_OUT;
                        end else
                            state <= ST_WAIT;
                    end
                end

                ST_OUT: begin
                    if (out_valid && out_ready) begin
                        done      <= 1'b1;
                        busy      <= 1'b0;
                        out_valid <= 1'b0;
                        state     <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
