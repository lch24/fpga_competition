`timescale 1ns / 1ps
//==============================================================================
// ring_check.sv — M3 环判定核（逐位复刻 candidates.cpp::alternating_ring /
//   export_m3.cpp::ring_detail_，单半径调用）
//------------------------------------------------------------------------------
// 语义（与 C++ 权威逐位一致，不得改动）：
//   边界检查：p.x < r+1 || p.y < r+1 || p.x >= w-r-1 || p.y >= h-r-1 → 全 0 记录。
//   32 采样：sx = lround(p.x + radius*cos(2πk/32))，sy 同；cos/sin 取 ROM
//     （ring_cos_sin.mem：0 基行 2k=cos、2k+1=sin，g++ libm 位模式）。
//   smooth[k]=(v[k-1]+2v[k]+v[k+1])/4（整数域算分子，fp32 精确，位级一致）。
//   lo/hi = smooth 的 min/max；hi-lo < 20 → 全 0 记录。
//   thr = (hi+lo)*0.5f；transition 判 (s[k]>thr)!=(s[k-1]>thr)；
//   opp_err = Σ fabs(s[k]-s[k+16])，k=0..31 链式 fp32_add 增序；
//   limit = 32*(hi-lo)*0.28f（与 opp 链并行）。
//   判定 A：ntrans==4 且 !(opp_err > limit) → sector_ok=1；
//   判定 B：sector 长度 (tr[(j+1)%4]-tr[j]+32)%32 均 ∈[3,13] → pass=1。
//   输出 {hi, lo, thr, ntrans, opp_err, sector_ok, pass}（与 m3_ring.bin 同序）。
//   边界/对比度失败时中间量全 0（与 ring_detail_ 的 d={} 一致）。
//
// 流水：mul(radius,cos/sin)→add(x/y,·)→lround→gray 锁步 1 采样/拍；
//   opp_err 串行链（sub→fabs→add，k 增序，保证 C++ 逐次舍入序）。
//------------------------------------------------------------------------------
module ring_check #(
    parameter W          = 32,
    parameter H          = 24,
    parameter GRAY_ADDR_W = 14,
    parameter ROM_FILE   = "ring_cos_sin.mem"
) (
    input  wire                    clk,
    input  wire                    rst_n,
    // 输入点（流式握手，一个点一次调用）
    input  wire                    in_valid,
    output wire                    in_ready,
    input  wire [31:0]             in_x,
    input  wire [31:0]             in_y,
    input  wire [31:0]             in_radius,
    // 灰度读口（外部 RAM，1 拍延迟：rd_en 下一拍 rd_data 有效）
    output reg                     rd_en,
    output reg  [GRAY_ADDR_W-1:0]  rd_addr,
    input  wire [7:0]              rd_data,
    // 结果输出
    output reg                     out_valid,
    input  wire                    out_ready,
    output reg  [31:0]             out_hi,
    output reg  [31:0]             out_lo,
    output reg  [31:0]             out_thr,
    output reg  [31:0]             out_ntrans,
    output reg  [31:0]             out_opp_err,
    output reg                     out_sector_ok,
    output reg                     out_pass
);

    import lround_pkg::*;

    //--------------------------------------------------------------------
    // 常量（fp32 位模式）
    //--------------------------------------------------------------------
    localparam C_1F    = 32'h3f800000;   // 1.0f
    localparam C_05F   = 32'h3f000000;   // 0.5f
    localparam C_20F   = 32'h41a00000;   // 20.0f
    localparam C_32F   = 32'h42000000;   // 32.0f
    localparam C_028F  = 32'h3e8f5c29;   // 0.28f

    // img.w / img.h 的 fp32 位模式（参数化常量，精确：W,H ≤ 4096）
    function automatic [31:0] fbits_int(input integer v);
        integer i;
        integer mag, e;
        integer k;
        reg [7:0]  exp8;
        reg [22:0] man23;
        begin
            mag = (v < 0) ? (-v) : v;
            e = 0;
            for (k = 0; k < 31; k = k + 1)
                if ((mag >> k) & 1) e = k;
            if (mag == 0) begin
                fbits_int = 32'h00000000;
            end else if (e <= 23) begin
                // 显式中间 reg（避免内联拼接的位宽推断异常）
                exp8  = 8'd127 + e[4:0];
                man23 = (mag << (23 - e)) & 23'h7FFFFF;
                fbits_int = {1'b0, exp8, man23};
            end else begin
                fbits_int = 32'h7f7fffff;   // 域外防御（本场景不触发）
            end
        end
    endfunction
    localparam W_F = fbits_int(W);
    localparam H_F = fbits_int(H);

    // fp32 比较（无 NaN/Inf 域；符号感知，兼容负值）
    function automatic fp32_lt(input [31:0] a, input [31:0] b);
        if (a[31] != b[31]) fp32_lt = a[31] & ~b[31];   // 仅 a 负 b 正
        else if (a[31])     fp32_lt = (a > b);          // 双负：无符号大者数值小
        else                fp32_lt = (a < b);          // 双正
    endfunction
    function automatic fp32_gt(input [31:0] a, input [31:0] b);
        if (a[31] != b[31]) fp32_gt = ~a[31] & b[31];   // 仅 a 正 b 负
        else if (a[31])     fp32_gt = (a < b);
        else                fp32_gt = (a > b);
    endfunction

    // smooth[k] = (v[k-1]+2v[k]+v[k+1])/4 的 fp32 位（整数域精确）
    function automatic [31:0] smooth_bits(input [7:0] vm1, input [7:0] v0, input [7:0] vp1);
        integer mol;
        integer e;
        integer k;
        reg [7:0]  exp8;
        reg [22:0] man23;
        begin
            mol = vm1 + 2 * v0 + vp1;
            if (mol == 0) begin
                smooth_bits = 32'h00000000;
            end else begin
                e = 0;
                for (k = 0; k < 12; k = k + 1)
                    if ((mol >> k) & 1) e = k;
                // mol/4 精确：指数域 = e - 2（偏置 127 + e - 2 = 125 + e）
                exp8  = 8'd125 + e;
                man23 = (mol << (23 - e)) & 23'h7FFFFF;
                smooth_bits = {1'b0, exp8, man23};
            end
        end
    endfunction
    // smooth_of(idx) = smooth_bits(values[idx-1], values[idx], values[idx+1])，
    //   5 位索引回绕即 mod 32（0→31、31→0 自然成立）
    function automatic [31:0] smooth_of(input [4:0] idx);
        smooth_of = smooth_bits(values[idx - 5'd1], values[idx], values[idx + 5'd1]);
    endfunction

    //--------------------------------------------------------------------
    // cos/sin ROM（ring_cos_sin.mem：0 基行 2k=cos(2πk/32)，2k+1=sin）
    //--------------------------------------------------------------------
    reg [31:0] rom [0:63];
    initial $readmemh(ROM_FILE, rom);

    //--------------------------------------------------------------------
    // 状态
    //--------------------------------------------------------------------
    localparam S_IDLE   = 4'd0;
    localparam S_BOUND  = 4'd1;   // 边界检查
    localparam S_SAMP   = 4'd2;   // 32 采样流水
    localparam S_LOHI   = 4'd3;   // smooth lo/hi（组合）
    localparam S_CTRST  = 4'd4;   // 对比度：hi-lo < 20
    localparam S_THR    = 4'd5;   // thr + transitions
    localparam S_OPPERR = 4'd6;   // opp_err 串行链 + limit 并行
    localparam S_JUDGE  = 4'd7;   // 判定 A/B
    localparam S_OUT    = 4'd8;
    reg [3:0] state;

    reg [31:0] x_reg, y_reg, radius_reg;
    reg [31:0] r1_reg, wr_reg, hr_reg;
    reg [7:0]  values[0:31];
    reg [31:0] lo_reg, hi_reg, thr_reg;
    reg [4:0]  ntrans_reg;
    reg [4:0]  trk[0:3];         // transitions 位置（ntrans==4 时有效）
    reg [31:0] opp_acc;
    reg [31:0] limit_reg;
    reg        sector_ok_reg, pass_reg;
    reg        rec_zero;         // 边界/对比度失败 → 全 0 记录

    reg [4:0]  bst;              // S_BOUND 步进
    reg [5:0]  sc;               // S_SAMP 经过拍数（每拍递增；锁步推导采样 k）
    reg [2:0]  cst;              // S_CTRST 步进
    reg [3:0]  thc;              // S_THR 步进
    reg [4:0]  opk;              // S_OPPERR 链 k
    reg [1:0]  opph;             // S_OPPERR 相位（0 发 sub / 1 收 sub / 2 发 add / 3 收 add）
    reg [3:0]  ltc;              // S_OPPERR limit 计时
    reg [31:0] sdiff;            // opp_err 项（fabs(sub 结果)）

    //--------------------------------------------------------------------
    // 子模块互连
    //--------------------------------------------------------------------
    wire mxr_rdy, myr_rdy, mxr_v, myr_v;
    wire [31:0] mxr_r, myr_r;
    wire axr_rdy, ayr_rdy, axr_v, ayr_v;
    wire [31:0] axr_r, ayr_r;
    wire sxr_rdy, syr_rdy, sxr_v, syr_v;
    wire [31:0] sxr_r, syr_r;

    // 采样流水（锁步 1 采样/拍）：mul(radius,cos/sin) → add(x/y,·) → lround → gray
    //   sc 为经过拍数：mul 喂 k=sc(0..31)，add 喂 k=sc-2，gray issue k=sc-4，
    //   rd_data 存 k=sc-6；sc 每拍递增（无背压，见头注锁步论证）。
    wire samp_mul_fire = (state == S_SAMP) && (sc <= 31) && mxr_rdy && myr_rdy;
    wire samp_add_fire = (state == S_SAMP) && (sc >= 2) && (sc <= 33) &&
                         mxr_v && myr_v && axr_rdy && ayr_rdy;
    wire samp_gray_on  = (state == S_SAMP) && (sc >= 4) && (sc <= 35) &&
                         axr_v && ayr_v;

    // 边界：bst=0 发 add(r,1)+sub(w_f,r)+sub(h_f,r)；bst=2 发 sub(-1)（wr/hr）
    wire bnd_fire0 = (state == S_BOUND) && (bst == 0) &&
                     axr_rdy && sxr_rdy && syr_rdy;
    wire bnd_fire1 = (state == S_BOUND) && (bst == 2) &&
                     sxr_v && syr_v && sxr_rdy && syr_rdy;
    // 对比度：sub(hi,lo)
    wire ctr_fire  = (state == S_CTRST) && (cst == 0) && sxr_rdy;
    // thr：add(hi,lo) → mul(·,0.5f)
    wire thr_fire1 = (state == S_THR) && (thc == 0) && axr_rdy;
    wire thr_fire2 = (state == S_THR) && (thc == 2) && axr_v && mxr_rdy;
    // opp_err 链：sub(s[k],s[k+16]) → fabs → add 链
    wire opp_sub_fire = (state == S_OPPERR) && (opph == 0) && (opk <= 31) && sxr_rdy;
    wire opp_add_fire = (state == S_OPPERR) && (opph == 2) && ayr_rdy;
    // limit：sub(hi,lo) → ×32 → ×0.28（ltc 每拍递增）
    wire lim_fire0 = (state == S_OPPERR) && (ltc == 0) && syr_rdy;
    wire lim_fire1 = (state == S_OPPERR) && (ltc == 2) && syr_v && mxr_rdy;
    wire lim_fire2 = (state == S_OPPERR) && (ltc == 4) && mxr_v && mxr_rdy;

    // smooth 广播数组（generate：genvar 为 32 位整数，需显式回绕）
    wire [31:0] smooth_arr[0:31];
    genvar gk;
    generate
        for (gk = 0; gk < 32; gk = gk + 1) begin : g_smooth
            assign smooth_arr[gk] =
                smooth_bits(values[(gk == 0) ? 31 : gk - 1],
                            values[gk],
                            values[(gk == 31) ? 0 : gk + 1]);
        end
    endgenerate

    //--------------------------------------------------------------------
    // 子模块例化（分时复用；各状态内触发互斥）
    //--------------------------------------------------------------------
    fp32_mul u_mulx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(samp_mul_fire || thr_fire2 || lim_fire1 || lim_fire2),
        .in_ready(mxr_rdy),
        .in_a(samp_mul_fire ? radius_reg :
              thr_fire2     ? axr_r :
              lim_fire1     ? syr_r : mxr_r),
        .in_b(samp_mul_fire ? rom[2 * sc] :
              thr_fire2     ? C_05F :
              lim_fire1     ? C_32F : C_028F),
        .out_valid(mxr_v), .out_ready(1'b1), .out_r(mxr_r)
    );
    fp32_mul u_muly (
        .clk(clk), .rst_n(rst_n),
        .in_valid(samp_mul_fire),
        .in_ready(myr_rdy),
        .in_a(radius_reg),
        .in_b(rom[2 * sc + 1]),
        .out_valid(myr_v), .out_ready(1'b1), .out_r(myr_r)
    );
    fp32_add u_addx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(bnd_fire0 || samp_add_fire || thr_fire1),
        .in_ready(axr_rdy),
        .in_a(bnd_fire0 ? radius_reg : samp_add_fire ? x_reg : hi_reg),
        .in_b(bnd_fire0 ? C_1F : samp_add_fire ? mxr_r : lo_reg),
        .out_valid(axr_v), .out_ready(1'b1), .out_r(axr_r)
    );
    fp32_add u_addy (
        .clk(clk), .rst_n(rst_n),
        .in_valid(samp_add_fire || opp_add_fire),
        .in_ready(ayr_rdy),
        .in_a(samp_add_fire ? y_reg : opp_acc),
        .in_b(samp_add_fire ? myr_r : sdiff),
        .out_valid(ayr_v), .out_ready(1'b1), .out_r(ayr_r)
    );
    fp32_sub u_subx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(bnd_fire0 || bnd_fire1 || ctr_fire || opp_sub_fire),
        .in_ready(sxr_rdy),
        .in_a(bnd_fire0 ? W_F :
              bnd_fire1 ? sxr_r :
              ctr_fire  ? hi_reg : smooth_of(opk)),
        .in_b(bnd_fire0 ? radius_reg :
              bnd_fire1 ? C_1F :
              ctr_fire  ? lo_reg : smooth_of(opk + 5'd16)),
        .out_valid(sxr_v), .out_ready(1'b1), .out_r(sxr_r)
    );
    fp32_sub u_suby (
        .clk(clk), .rst_n(rst_n),
        .in_valid(bnd_fire0 || bnd_fire1 || lim_fire0),
        .in_ready(syr_rdy),
        .in_a(bnd_fire0 ? H_F : bnd_fire1 ? syr_r : hi_reg),
        .in_b(bnd_fire0 ? radius_reg : bnd_fire1 ? C_1F : lo_reg),
        .out_valid(syr_v), .out_ready(1'b1), .out_r(syr_r)
    );

    //--------------------------------------------------------------------
    // 主状态机
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            x_reg         <= 32'd0;
            y_reg         <= 32'd0;
            radius_reg    <= 32'd0;
            lo_reg        <= 32'd0;
            hi_reg        <= 32'd0;
            thr_reg       <= 32'd0;
            ntrans_reg    <= 5'd0;
            opp_acc       <= 32'd0;
            limit_reg     <= 32'd0;
            sector_ok_reg <= 1'b0;
            pass_reg      <= 1'b0;
            rec_zero      <= 1'b0;
            rd_en         <= 1'b0;
            rd_addr       <= {GRAY_ADDR_W{1'b0}};
            out_valid     <= 1'b0;
            out_hi        <= 32'd0;
            out_lo        <= 32'd0;
            out_thr       <= 32'd0;
            out_ntrans    <= 32'd0;
            out_opp_err   <= 32'd0;
            out_sector_ok <= 1'b0;
            out_pass      <= 1'b0;
        end else begin
            rd_en <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (in_valid) begin
                        x_reg      <= in_x;
                        y_reg      <= in_y;
                        radius_reg <= in_radius;
                        rec_zero   <= 1'b0;
                        bst        <= 5'd0;
                        state      <= S_BOUND;
                    end
                end

                S_BOUND: begin
                    if (bst == 2)
                        r1_reg <= axr_r;         // radius+1（axr_v 锁步成立）
                    if (bst == 4) begin
                        // wr = sxr_r / hr = syr_r 本拍有效（第二次 sub 的输出）
                        wr_reg <= sxr_r;
                        hr_reg <= syr_r;
                        if (fp32_lt(x_reg, r1_reg) || fp32_lt(y_reg, r1_reg) ||
                            !fp32_lt(x_reg, sxr_r) || !fp32_lt(y_reg, syr_r)) begin
                            rec_zero <= 1'b1;
                            state    <= S_OUT;
                        end else begin
                            sc    <= 6'd0;
                            state <= S_SAMP;
                        end
                    end
                    bst <= bst + 5'd1;
                end

                S_SAMP: begin
                    // gray 数据于 gray issue 后 2 拍到达（rd_en 寄存 1 拍 + RAM 1 拍）
                    if (sc >= 6 && sc <= 37)
                        values[sc[5:0] - 6'd6] <= rd_data;
                    if (samp_gray_on) begin
                        rd_en   <= 1'b1;
                        rd_addr <= lround_f32(axr_r) + lround_f32(ayr_r) * W;
                    end
                    sc <= sc + 6'd1;            // 每拍递增（锁步）
                    if (sc == 38)
                        state <= S_LOHI;
                end

                S_LOHI: begin
                    lo_reg <= lo_comb();
                    hi_reg <= hi_comb();
                    cst    <= 3'd0;
                    state  <= S_CTRST;
                end

                S_CTRST: begin
                    if (cst == 2) begin
                        // sxr_r = hi-lo（ctr_fire 于 cst=0 发出）
                        if (fp32_lt(sxr_r, C_20F)) begin
                            rec_zero <= 1'b1;
                            state    <= S_OUT;
                        end else begin
                            thc <= 4'd0;
                            state <= S_THR;
                        end
                    end
                    cst <= cst + 3'd1;
                end

                S_THR: begin
                    if (thc == 4)
                        thr_reg <= mxr_r;        // (hi+lo)*0.5（mxr_v 锁步成立）
                    if (thc == 5) begin
                        ntrans_reg <= trans_comb()[24:20];
                        trk[0]     <= trans_comb()[19:15];
                        trk[1]     <= trans_comb()[14:10];
                        trk[2]     <= trans_comb()[9:5];
                        trk[3]     <= trans_comb()[4:0];
                        opk   <= 5'd0;
                        opph  <= 2'd0;
                        ltc   <= 4'd0;
                        opp_acc <= 32'd0;
                        state <= S_OPPERR;
                    end
                    thc <= thc + 4'd1;
                end

                S_OPPERR: begin
                    // limit 链（ltc 每拍递增；fires 按 ltc 步进，锁步）
                    if (ltc < 8)
                        ltc <= ltc + 4'd1;
                    if (lim_fire0) ltc <= 4'd1;   // 提前跨步防重复
                    if (ltc == 6 && mxr_v)
                        limit_reg <= mxr_r;       // 32*(hi-lo)*0.28

                    // opp_err 串行链（相位 0..3）
                    case (opph)
                        2'd0: begin
                            if (opp_sub_fire) opph <= 2'd1;
                        end
                        2'd1: begin
                            if (sxr_v) begin
                                sdiff <= {1'b0, sxr_r[30:0]};   // fabs
                                opph  <= 2'd2;
                            end
                        end
                        2'd2: begin
                            if (opp_add_fire) opph <= 2'd3;
                        end
                        2'd3: begin
                            if (ayr_v) begin
                                opp_acc <= ayr_r;
                                opph  <= 2'd0;
                                if (opk == 31)
                                    state <= S_JUDGE;
                                else
                                    opk <= opk + 5'd1;
                            end
                        end
                    endcase
                end

                S_JUDGE: begin
                    sector_ok_reg <= judge_sector_ok();
                    pass_reg      <= judge_pass();
                    state         <= S_OUT;
                end

                S_OUT: begin
                    out_valid <= 1'b1;
                    if (rec_zero) begin
                        out_hi        <= 32'd0;
                        out_lo        <= 32'd0;
                        out_thr       <= 32'd0;
                        out_ntrans    <= 32'd0;
                        out_opp_err   <= 32'd0;
                        out_sector_ok <= 1'b0;
                        out_pass      <= 1'b0;
                    end else begin
                        out_hi        <= hi_reg;
                        out_lo        <= lo_reg;
                        out_thr       <= thr_reg;
                        out_ntrans    <= {27'd0, ntrans_reg};
                        out_opp_err   <= opp_acc;
                        out_sector_ok <= sector_ok_reg;
                        out_pass      <= pass_reg;
                    end
                    if (out_valid && out_ready) begin
                        out_valid <= 1'b0;
                        state     <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    assign in_ready = (state == S_IDLE);

    //--------------------------------------------------------------------
    // 组合函数：lo/hi、transitions、判定
    //--------------------------------------------------------------------
    function automatic [31:0] lo_comb();
        integer k;
        reg [31:0] m;
        begin
            m = smooth_arr[0];
            for (k = 1; k < 32; k = k + 1)
                if (smooth_arr[k] < m) m = smooth_arr[k];
            lo_comb = m;
        end
    endfunction
    function automatic [31:0] hi_comb();
        integer k;
        reg [31:0] m;
        begin
            m = smooth_arr[0];
            for (k = 1; k < 32; k = k + 1)
                if (smooth_arr[k] > m) m = smooth_arr[k];
            hi_comb = m;
        end
    endfunction
    // trans_comb：{ntrans[4:0], trk0[4:0], trk1, trk2, trk3}（25 位）
    function automatic [24:0] trans_comb();
        integer k;
        reg [24:0] r;
        reg [4:0]  n;
        begin
            r = 25'd0;
            n = 5'd0;
            for (k = 0; k < 32; k = k + 1) begin
                if ((smooth_arr[k] > thr_reg) != (smooth_arr[(k + 31) % 32] > thr_reg)) begin
                    if (n < 4) r[19 - 5 * n -: 5] = k[4:0];
                    n = n + 1;
                end
            end
            r[24:20] = n;
            trans_comb = r;
        end
    endfunction

    // 判定 A：sector_ok = ntrans==4 && !(opp_err > limit)
    function automatic judge_sector_ok();
        judge_sector_ok = (ntrans_reg == 4) && !fp32_gt(opp_acc, limit_reg);
    endfunction
    // 判定 B：sector 长度 ∈ [3,13]（仅判定 A 通过时检查）
    function automatic judge_pass();
        reg [4:0] l0, l1, l2, l3;
        begin
            if ((ntrans_reg != 4) || fp32_gt(opp_acc, limit_reg)) begin
                judge_pass = 1'b0;
            end else begin
                l0 = (trk[1] - trk[0] + 32) & 31;
                l1 = (trk[2] - trk[1] + 32) & 31;
                l2 = (trk[3] - trk[2] + 32) & 31;
                l3 = (trk[0] - trk[3] + 32) & 31;
                judge_pass = (l0 >= 3 && l0 <= 13) && (l1 >= 3 && l1 <= 13) &&
                             (l2 >= 3 && l2 <= 13) && (l3 >= 3 && l3 <= 13);
            end
        end
    endfunction

endmodule
