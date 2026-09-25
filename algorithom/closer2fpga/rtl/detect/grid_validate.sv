`timescale 1ns / 1ps
//==============================================================================
// grid_validate.sv — 网格代价判定（逐位复刻 export_m4.cpp::cost_ref）
//------------------------------------------------------------------------------
// cost_ref 语义（运算顺序与 C++ 完全一致，每次运算逐次 fp32 舍入）：
//   对每格点 p（顺序 r*cols+c）：
//     双轴（axis=0 步 1、axis=1 步 cols，pos+2>=count 跳过）：
//       dx1=a.x-p.x dy1=a.y-p.y dx2=b.x-a.x dy2=b.y-a.y
//       l1=hypot(dx1,dy1) l2=hypot(dx2,dy2)
//       l1<4 || l2<4 || l2/l1<0.55 || l2/l1>1.8 → 无效(1e30)
//       cosine=(dx1*dx2+dy1*dy2)/(l1*l2)（分子：dx1*dx2 先、dy1*dy2 再、相加）
//       cosine<0.90f → 无效
//       change=log_ref(l2/l1)（fp32_log；l2/l1 复用 AX_DIV1 结果，位级一致）
//       cost += (1-cosine) + change*change（1-cosine 先、change² 再、相加、累加）
//     单格（r+1<rows && c+1<cols）：q[4]={p,idx+1,idx+cols+1,idx+cols}，k=0..3：
//       a=q[k] b=q[k+1] d=q[k+2]（模 4）
//       cross=(b.x-a.x)*(d.y-b.y)-(b.y-a.y)*(d.x-b.x)
//       lengths=dist(a,b)*dist(b,d)（hypot×hypot，fp32_mul）
//       lengths<16 || fabs(cross)<lengths*0.2f → 无效
//       sign==0 → sign=cross；cross*sign<=0 → 无效
//   返回 cost 或 1e30f(0x7149f2ca)
// 实现：顺序扫描状态机 + 算术模块复用（4×fp32_sub、2×fp32_hypot、3×fp32_mul、
//   2×fp32_add、2×fp32_div、1×fp32_log），每阶段"驱动+等待 out_valid"，
//   失败立即进入 FAIL 输出 1e30f。网格点由外部 RAM 提供（1 拍读延迟）。
//==============================================================================
module grid_validate #(
    parameter ROWS     = 5,
    parameter COLS     = 8,
    parameter N_ADDR_W = 6
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    output reg                busy,
    output reg                done,
    input  wire [15:0]        n_in,
    output reg                rd_en,
    output reg  [N_ADDR_W-1:0] rd_addr,
    input  wire [31:0]        rd_x,
    input  wire [31:0]        rd_y,
    output reg                valid_out,
    output reg  [31:0]        cost_out
);

    // 位模式常量（g++ 同机打印，勿自算）
    localparam [31:0] ONE    = 32'h3f800000;  // 1.0f
    localparam [31:0] C_4    = 32'h40800000;  // 4.0f
    localparam [31:0] C_055  = 32'h3f0ccccd;  // 0.55f
    localparam [31:0] C_18   = 32'h3fe66666;  // 1.8f
    localparam [31:0] C_090  = 32'h3f666666;  // 0.90f
    localparam [31:0] C_16   = 32'h41800000;  // 16.0f
    localparam [31:0] C_02   = 32'h3e4ccccd;  // 0.2f
    localparam [31:0] C_1E30 = 32'h7149f2ca;  // 1e30f

    // fp32 数值比较 a < b（含符号与 ±0，无 NaN）
    function automatic logic cmp_lt(input logic [31:0] a, input logic [31:0] b);
        if (a[31] && !b[31])      cmp_lt = 1'b1;   // a 负 b 正
        else if (!a[31] && b[31]) cmp_lt = 1'b0;   // a 正 b 负
        else if (a[31])           cmp_lt = (a > b); // 同负：位模式大者更小
        else                      cmp_lt = (a < b); // 同正
    endfunction

    //--------------------------------------------------------------------
    // 状态
    //--------------------------------------------------------------------
    localparam [7:0]
        IDLE     = 8'd0,
        P_RD     = 8'd1,  P_LAT = 8'd2,  P_W      = 8'd3,
        A0_RD1   = 8'd4,  A0_L1 = 8'd5,  A0_W1    = 8'd6,
        A0_RD2   = 8'd7,  A0_L2 = 8'd8,  A0_W2    = 8'd9,
        A1_RD1   = 8'd10, A1_L1 = 8'd11, A1_W1    = 8'd12,
        A1_RD2   = 8'd13, A1_L2 = 8'd14, A1_W2    = 8'd15,
        Q_RD1    = 8'd16, Q_L1  = 8'd17, Q_W1     = 8'd18,
        Q_RD2    = 8'd19, Q_L2  = 8'd20, Q_W2     = 8'd21,
        Q_RD3    = 8'd22, Q_L3  = 8'd23, Q_W3     = 8'd24,
        AX_SUB   = 8'd25, AX_HYP = 8'd26, AX_LCHK = 8'd27, AX_DIV1 = 8'd28,
        AX_RCHK  = 8'd29, AX_MUL = 8'd30, AX_ADD  = 8'd31, AX_DIV2 = 8'd32,
        AX_CCHK  = 8'd33, AX_LOG = 8'd34, AX_LOG_W = 8'd35, AX_U = 8'd36,
        AX_ADD1  = 8'd37, AX_ADD2  = 8'd38, AX_DONE = 8'd39,
        CC_SUB   = 8'd40, CC_MH  = 8'd41, CC_CROSS = 8'd42, CC_LEN = 8'd43,
        CC_LCHK  = 8'd44, CC_LB  = 8'd45, CC_ABS   = 8'd46, CC_SIGN = 8'd47,
        CC_CS    = 8'd48, CC_CCHK = 8'd49,
        ENTRY    = 8'd50, NEXT_P  = 8'd51, FINISH  = 8'd52, FAIL   = 8'd53;

    reg [7:0] state;

    //--------------------------------------------------------------------
    // 数据寄存器
    //--------------------------------------------------------------------
    reg [N_ADDR_W-1:0] idx_r;
    reg [3:0]  r_r, c_r;                 // 当前行列（r=c/COLS 计数维护）
    reg        cur_axis;                 // 0=轴0(step1) 1=轴1(stepCOLS)
    reg [2:0]  k_r;                      // 单格 k 循环
    reg        ax0_done, ax1_done, cell_done;  // 本点各检查完成标志
    reg [31:0] cost_r, sign_r;

    reg [31:0] p_xr, p_yr;
    reg [31:0] a0_xr, a0_yr, b0_xr, b0_yr;
    reg [31:0] a1_xr, a1_yr, b1_xr, b1_yr;
    reg [31:0] q1_xr, q1_yr, q2_xr, q2_yr, q3_xr, q3_yr;

    // 轴检查中间量
    reg [31:0] dx1r, dy1r, dx2r, dy2r;
    reg [31:0] l1r, l2r, ratior, t1r, t2r, denr, numr, cosiner, changer, ur, vr, sr;
    // 单格检查中间量
    reg [31:0] sbxr, sdyr, sbyr, sdxr, m1vr, m2vr, h1vr, h2vr, crossr, lengthsr, lbr, csr;

    //--------------------------------------------------------------------
    // 条件
    //--------------------------------------------------------------------
    wire axis0_ok = (c_r + 4'd2 < COLS[3:0]);
    wire axis1_ok = (r_r + 4'd2 < ROWS[3:0]);
    wire cell_ok  = (r_r + 4'd1 < ROWS[3:0]) && (c_r + 4'd1 < COLS[3:0]);

    // 轴检查输入选择（cur_axis）
    wire [31:0] ax_px = p_xr;
    wire [31:0] ax_py = p_yr;
    wire [31:0] ax_ax = cur_axis ? a1_xr : a0_xr;
    wire [31:0] ax_ay = cur_axis ? a1_yr : a0_yr;
    wire [31:0] ax_bx = cur_axis ? b1_xr : b0_xr;
    wire [31:0] ax_by = cur_axis ? b1_yr : b0_yr;

    // 单格 k 循环输入选择：a=q[k] b=q[k+1] d=q[k+2]（模 4）
    reg [31:0] cc_ax, cc_ay, cc_bx, cc_by, cc_dx, cc_dy;
    always @(*) begin
        case (k_r)
            3'd0: begin
                cc_ax = p_xr;  cc_ay = p_yr;
                cc_bx = q1_xr; cc_by = q1_yr;
                cc_dx = q2_xr; cc_dy = q2_yr;
            end
            3'd1: begin
                cc_ax = q1_xr; cc_ay = q1_yr;
                cc_bx = q2_xr; cc_by = q2_yr;
                cc_dx = q3_xr; cc_dy = q3_yr;
            end
            3'd2: begin
                cc_ax = q2_xr; cc_ay = q2_yr;
                cc_bx = q3_xr; cc_by = q3_yr;
                cc_dx = p_xr;  cc_dy = p_yr;
            end
            default: begin
                cc_ax = q3_xr; cc_ay = q3_yr;
                cc_bx = p_xr;  cc_by = p_yr;
                cc_dx = q1_xr; cc_dy = q1_yr;
            end
        endcase
    end

    //--------------------------------------------------------------------
    // 算术模块实例（out_ready 恒 1；in_ready 悬空）
    //--------------------------------------------------------------------
    wire [31:0] s0_d, s1_d, s2_d, s3_d;
    wire        s0_v, s1_v, s2_v, s3_v;
    wire [31:0] h0_d, h1_d;
    wire        h0_v, h1_v;
    wire [31:0] m0_d, m1_d, m2_d;
    wire        m0_v, m1_v, m2_v;
    wire [31:0] a0_d, a1_d;
    wire        a0_v, a1_v;
    wire [31:0] d0_d, d1_d;
    wire        d0_v, d1_v;
    wire [31:0] lg_d;
    wire        lg_v, log_irdy;

    // 驱动使能（状态相关）
    wire ax_sub_go = (state == AX_SUB);
    wire ax_hyp_go = (state == AX_HYP);
    wire ax_dv1_go = (state == AX_DIV1);
    wire ax_mul_go = (state == AX_MUL);
    wire ax_add_go = (state == AX_ADD);
    wire ax_dv2_go = (state == AX_DIV2);
    wire ax_log_go = (state == AX_LOG);
    wire ax_u_go   = (state == AX_U);
    wire ax_a1_go  = (state == AX_ADD1);
    wire ax_a2_go  = (state == AX_ADD2);
    wire cc_sub_go = (state == CC_SUB);
    wire cc_mh_go  = (state == CC_MH);
    wire cc_cr_go  = (state == CC_CROSS);
    wire cc_len_go = (state == CC_LEN);
    wire cc_lb_go  = (state == CC_LB);
    wire cc_cs_go  = (state == CC_CS);

    fp32_sub u_s0 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ax_sub_go || cc_sub_go || ax_u_go || cc_cr_go),
        .in_ready(),
        .in_a(ax_sub_go ? ax_ax : cc_sub_go ? cc_bx : ax_u_go ? ONE : m1vr),
        .in_b(ax_sub_go ? ax_px : cc_sub_go ? cc_ax : ax_u_go ? cosiner : m2vr),
        .out_valid(s0_v), .out_ready(1'b1), .out_r(s0_d)
    );
    fp32_sub u_s1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ax_sub_go || cc_sub_go),
        .in_ready(),
        .in_a(ax_sub_go ? ax_ay : cc_dy),
        .in_b(ax_sub_go ? ax_py : cc_by),
        .out_valid(s1_v), .out_ready(1'b1), .out_r(s1_d)
    );
    fp32_sub u_s2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ax_sub_go || cc_sub_go),
        .in_ready(),
        .in_a(ax_sub_go ? ax_bx : cc_by),
        .in_b(ax_sub_go ? ax_ax : cc_ay),
        .out_valid(s2_v), .out_ready(1'b1), .out_r(s2_d)
    );
    fp32_sub u_s3 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ax_sub_go || cc_sub_go),
        .in_ready(),
        .in_a(ax_sub_go ? ax_by : cc_dx),
        .in_b(ax_sub_go ? ax_ay : cc_bx),
        .out_valid(s3_v), .out_ready(1'b1), .out_r(s3_d)
    );

    fp32_hypot u_h0 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ax_hyp_go || cc_mh_go),
        .in_ready(),
        .in_a(ax_hyp_go ? dx1r : sbxr),
        .in_b(ax_hyp_go ? dy1r : sbyr),
        .out_valid(h0_v), .out_ready(1'b1), .out_r(h0_d)
    );
    fp32_hypot u_h1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ax_hyp_go || cc_mh_go),
        .in_ready(),
        .in_a(ax_hyp_go ? dx2r : sdxr),
        .in_b(ax_hyp_go ? dy2r : sdyr),
        .out_valid(h1_v), .out_ready(1'b1), .out_r(h1_d)
    );

    fp32_mul u_m0 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ax_mul_go || ax_u_go || cc_mh_go || cc_lb_go),
        .in_ready(),
        .in_a(ax_mul_go ? dx1r : ax_u_go ? changer : cc_mh_go ? sbxr : lengthsr),
        .in_b(ax_mul_go ? dx2r : ax_u_go ? changer : cc_mh_go ? sdyr : C_02),
        .out_valid(m0_v), .out_ready(1'b1), .out_r(m0_d)
    );
    fp32_mul u_m1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ax_mul_go || cc_mh_go || cc_cs_go),
        .in_ready(),
        .in_a(ax_mul_go ? dy1r : cc_mh_go ? sbyr : crossr),
        .in_b(ax_mul_go ? dy2r : cc_mh_go ? sdxr : sign_r),
        .out_valid(m1_v), .out_ready(1'b1), .out_r(m1_d)
    );
    fp32_mul u_m2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ax_mul_go || cc_len_go),
        .in_ready(),
        .in_a(ax_mul_go ? l1r : h1vr),
        .in_b(ax_mul_go ? l2r : h2vr),
        .out_valid(m2_v), .out_ready(1'b1), .out_r(m2_d)
    );

    fp32_add u_a0 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ax_add_go || ax_a1_go),
        .in_ready(),
        .in_a(ax_add_go ? t1r : ur),
        .in_b(ax_add_go ? t2r : vr),
        .out_valid(a0_v), .out_ready(1'b1), .out_r(a0_d)
    );
    fp32_add u_a1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ax_a2_go),
        .in_ready(),
        .in_a(cost_r), .in_b(sr),
        .out_valid(a1_v), .out_ready(1'b1), .out_r(a1_d)
    );

    fp32_div u_d0 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ax_dv1_go),
        .in_ready(),
        .in_a(l2r), .in_b(l1r),
        .out_valid(d0_v), .out_ready(1'b1), .out_r(d0_d)
    );
    fp32_div u_d1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ax_dv2_go),
        .in_ready(),
        .in_a(numr), .in_b(denr),
        .out_valid(d1_v), .out_ready(1'b1), .out_r(d1_d)
    );

    fp32_log u_log (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ax_log_go),
        .in_ready(log_irdy),
        .in_x(ratior),
        .out_valid(lg_v), .out_ready(1'b1), .out_r(lg_d)
    );

    //--------------------------------------------------------------------
    // 主状态机
    //--------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            idx_r     <= 0;
            r_r       <= 0;
            c_r       <= 0;
            cur_axis  <= 0;
            k_r       <= 0;
            ax0_done  <= 0;
            ax1_done  <= 0;
            cell_done <= 0;
            cost_r    <= 0;
            sign_r    <= 0;
            rd_en     <= 0;
            rd_addr   <= 0;
            busy      <= 0;
            done      <= 0;
            valid_out <= 0;
            cost_out  <= 0;
            p_xr <= 0; p_yr <= 0;
            a0_xr <= 0; a0_yr <= 0; b0_xr <= 0; b0_yr <= 0;
            a1_xr <= 0; a1_yr <= 0; b1_xr <= 0; b1_yr <= 0;
            q1_xr <= 0; q1_yr <= 0; q2_xr <= 0; q2_yr <= 0; q3_xr <= 0; q3_yr <= 0;
            dx1r <= 0; dy1r <= 0; dx2r <= 0; dy2r <= 0;
            l1r <= 0; l2r <= 0; ratior <= 0; t1r <= 0; t2r <= 0; denr <= 0;
            numr <= 0; cosiner <= 0; changer <= 0; ur <= 0; vr <= 0; sr <= 0;
            sbxr <= 0; sdyr <= 0; sbyr <= 0; sdxr <= 0;
            m1vr <= 0; m2vr <= 0; h1vr <= 0; h2vr <= 0;
            crossr <= 0; lengthsr <= 0; lbr <= 0; csr <= 0;
        end else begin
            rd_en     <= 0;
            done      <= 0;
            valid_out <= 0;

            case (state)
                //--------------------------------------------------------
                IDLE: begin
                    if (start) begin
                        busy      <= 1'b1;
                        idx_r     <= 0;
                        r_r       <= 0;
                        c_r       <= 0;
                        cost_r    <= 0;
                        sign_r    <= 0;
                        state     <= P_RD;
                    end
                end

                //------------------ 读取阶段（请求→空转→锁存，2 拍读延迟） --
                P_RD: begin
                    rd_en   <= 1'b1;
                    rd_addr <= idx_r;
                    state   <= P_LAT;
                end
                P_LAT: begin
                    state <= P_W;
                end
                P_W: begin
                    p_xr <= rd_x;
                    p_yr <= rd_y;
                    if (axis0_ok)            state <= A0_RD1;
                    else if (axis1_ok)       state <= A1_RD1;
                    else if (cell_ok)        state <= Q_RD1;
                    else                     state <= ENTRY;
                end

                A0_RD1: begin rd_en <= 1'b1; rd_addr <= idx_r + 1;      state <= A0_L1; end
                A0_L1:  begin                                             state <= A0_W1; end
                A0_W1:  begin
                    a0_xr <= rd_x; a0_yr <= rd_y;
                    rd_en <= 1'b1; rd_addr <= idx_r + 2;
                    state <= A0_L2;
                end
                A0_L2:  begin                                             state <= A0_W2; end
                A0_W2:  begin
                    b0_xr <= rd_x; b0_yr <= rd_y;
                    if (axis1_ok)            state <= A1_RD1;
                    else if (cell_ok)        state <= Q_RD1;
                    else                     state <= ENTRY;
                end

                A1_RD1: begin rd_en <= 1'b1; rd_addr <= idx_r + COLS[5:0];  state <= A1_L1; end
                A1_L1:  begin                                                state <= A1_W1; end
                A1_W1:  begin
                    a1_xr <= rd_x; a1_yr <= rd_y;
                    rd_en <= 1'b1; rd_addr <= idx_r + 2*COLS[5:0];
                    state <= A1_L2;
                end
                A1_L2:  begin                                                state <= A1_W2; end
                A1_W2:  begin
                    b1_xr <= rd_x; b1_yr <= rd_y;
                    if (cell_ok)             state <= Q_RD1;
                    else                     state <= ENTRY;
                end

                Q_RD1: begin rd_en <= 1'b1; rd_addr <= idx_r + 1;          state <= Q_L1; end
                Q_L1:  begin                                                 state <= Q_W1; end
                Q_W1:  begin
                    q1_xr <= rd_x; q1_yr <= rd_y;
                    rd_en <= 1'b1; rd_addr <= idx_r + COLS[5:0] + 1;
                    state <= Q_L2;
                end
                Q_L2:  begin                                                 state <= Q_W2; end
                Q_W2:  begin
                    q2_xr <= rd_x; q2_yr <= rd_y;
                    rd_en <= 1'b1; rd_addr <= idx_r + COLS[5:0];
                    state <= Q_L3;
                end
                Q_L3:  begin                                                 state <= Q_W3; end
                Q_W3:  begin
                    q3_xr <= rd_x; q3_yr <= rd_y;
                    state <= ENTRY;
                end

                //------------------ 轴检查链（axis0/axis1 共用） ---------
                AX_SUB: begin
                    if (s0_v && s1_v && s2_v && s3_v) begin
                        dx1r <= s0_d; dy1r <= s1_d; dx2r <= s2_d; dy2r <= s3_d;
                        state <= AX_HYP;
                    end
                end
                AX_HYP: begin
                    if (h0_v && h1_v) begin
                        l1r <= h0_d; l2r <= h1_d;
                        state <= AX_LCHK;
                    end
                end
                AX_LCHK: begin
                    if (cmp_lt(l1r, C_4) || cmp_lt(l2r, C_4)) state <= FAIL;
                    else                                       state <= AX_DIV1;
                end
                AX_DIV1: begin
                    if (d0_v) begin
                        ratior <= d0_d;
                        state  <= AX_RCHK;
                    end
                end
                AX_RCHK: begin
                    if (cmp_lt(ratior, C_055) || cmp_lt(C_18, ratior)) state <= FAIL;
                    else                                               state <= AX_MUL;
                end
                AX_MUL: begin
                    if (m0_v && m1_v && m2_v) begin
                        t1r <= m0_d; t2r <= m1_d; denr <= m2_d;
                        state <= AX_ADD;
                    end
                end
                AX_ADD: begin
                    if (a0_v) begin
                        numr  <= a0_d;
                        state <= AX_DIV2;
                    end
                end
                AX_DIV2: begin
                    if (d1_v) begin
                        cosiner <= d1_d;
                        state   <= AX_CCHK;
                    end
                end
                AX_CCHK: begin
                    if (cmp_lt(cosiner, C_090)) state <= FAIL;
                    else                        state <= AX_LOG;
                end
                AX_LOG: begin
                    // 脉冲握手：接受 1 次后立即撤 in_valid，防止流水连续接受产生残留
                    if (ax_log_go && log_irdy) state <= AX_LOG_W;
                end
                AX_LOG_W: begin
                    if (lg_v) begin
                        changer <= lg_d;
                        state   <= AX_U;
                    end
                end
                AX_U: begin
                    if (s0_v && m0_v) begin
                        ur <= s0_d; vr <= m0_d;
                        state <= AX_ADD1;
                    end
                end
                AX_ADD1: begin
                    if (a0_v) begin
                        sr    <= a0_d;
                        state <= AX_ADD2;
                    end
                end
                AX_ADD2: begin
                    if (a1_v) begin
                        cost_r <= a1_d;
                        state  <= AX_DONE;
                    end
                end
                AX_DONE: begin
                    if (!cur_axis) ax0_done <= 1'b1;
                    else           ax1_done <= 1'b1;
                    state <= ENTRY;
                end

                //------------------ 计算入口：按优先级选下一检查 ---------
                ENTRY: begin
                    if (!ax0_done && axis0_ok) begin
                        cur_axis <= 1'b0;
                        state    <= AX_SUB;
                    end else if (!ax1_done && axis1_ok) begin
                        cur_axis <= 1'b1;
                        state    <= AX_SUB;
                    end else if (!cell_done && cell_ok) begin
                        k_r   <= 0;
                        state <= CC_SUB;
                    end else
                        state <= NEXT_P;
                end

                //------------------ 单格检查链（k=0..3） -----------------
                CC_SUB: begin
                    if (s0_v && s1_v && s2_v && s3_v) begin
                        sbxr <= s0_d; sdyr <= s1_d; sbyr <= s2_d; sdxr <= s3_d;
                        state <= CC_MH;
                    end
                end
                CC_MH: begin
                    if (m0_v && m1_v && h0_v && h1_v) begin
                        m1vr <= m0_d; m2vr <= m1_d; h1vr <= h0_d; h2vr <= h1_d;
                        state <= CC_CROSS;
                    end
                end
                CC_CROSS: begin
                    if (s0_v) begin
                        crossr <= s0_d;
                        state  <= CC_LEN;
                    end
                end
                CC_LEN: begin
                    if (m2_v) begin
                        lengthsr <= m2_d;
                        state    <= CC_LCHK;
                    end
                end
                CC_LCHK: begin
                    if (cmp_lt(lengthsr, C_16)) state <= FAIL;
                    else                        state <= CC_LB;
                end
                CC_LB: begin
                    if (m0_v) begin
                        lbr   <= m0_d;
                        state <= CC_ABS;
                    end
                end
                CC_ABS: begin
                    if (cmp_lt({1'b0, crossr[30:0]}, lbr)) state <= FAIL;
                    else                                   state <= CC_SIGN;
                end
                CC_SIGN: begin
                    if (sign_r == 32'd0 || sign_r == 32'h80000000)
                        sign_r <= crossr;
                    state <= CC_CS;
                end
                CC_CS: begin
                    if (m1_v) begin
                        csr   <= m1_d;
                        state <= CC_CCHK;
                    end
                end
                CC_CCHK: begin
                    if (csr == 32'd0 || csr == 32'h80000000 || csr[31])
                        state <= FAIL;
                    else if (k_r == 3'd3) begin
                        cell_done <= 1'b1;
                        state <= ENTRY;
                    end else begin
                        k_r   <= k_r + 1'b1;
                        state <= CC_SUB;
                    end
                end

                //------------------ 推进 / 结束 --------------------------
                NEXT_P: begin
                    ax0_done  <= 1'b0;
                    ax1_done  <= 1'b0;
                    cell_done <= 1'b0;
                    if (idx_r + 1 >= n_in[5:0]) begin
                        state <= FINISH;
                    end else begin
                        idx_r <= idx_r + 1'b1;
                        if (c_r == COLS[3:0] - 1) begin
                            c_r <= 0;
                            r_r <= r_r + 1'b1;
                        end else
                            c_r <= c_r + 1'b1;
                        state <= P_RD;
                    end
                end
                FINISH: begin
                    busy      <= 0;
                    done      <= 1'b1;
                    valid_out <= 1'b1;
                    cost_out  <= cost_r;
                    state     <= IDLE;
                end
                FAIL: begin
                    busy      <= 0;
                    done      <= 1'b1;
                    valid_out <= 1'b1;
                    cost_out  <= C_1E30;
                    state     <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end

endmodule
