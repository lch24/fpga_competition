`timescale 1ns / 1ps
//==============================================================================
// fp64_add.v — 双精度 IEEE-754 加法（RNE，位级 = C++ double 加法）
//------------------------------------------------------------------------------
// 语义：out_r = RNE( in_a + in_b )（fp64）。完整 IEEE-754 binary64 加法：
//   ±0（含符号规范化）、±subnormal、±normal、±Inf、NaN（统一输出规范 QNaN
//   0x7FF8_0000_0000_0000）；结果次正规/零/无穷均正确舍入，与 C++ double 逐位一致。
//
// 实现（fp64_sub 守卫位方案扩展为通用）：
//   1) 输入解码 value = M·2^(e-1075)：normal M={1,frac}, e=ea(1..2046)；
//      subnormal M=frac, e=1；zero M=0, e=0。Inf/NaN 走特殊路径。
//   2) magnitude 对齐：较大者(lm,ler)为基准，较小者按指数差 diff 右移并入
//      sticky（守卫位 G=52：diff≤52 精确，diff>52 截断入 st_all）。
//   3) 同号加/异号减得 106 位 C；msb n；efield = n + ler - 104（含尾数进位）。
//   4) 正常结果（efield≥1）：53 位尾数 + round/sticky 做 RNE。
//   5) 次正规结果（efield≤0）：f = mtop·2^(efield-1)，右移 sh=1-efield 位，
//      在新尺度重取 guard/round/sticky 做 RNE，进位成最小正规。
//   6) 零规范化：相消→+0；(-0)+(-0)→-0；其余同号零保持符号。
//
// 弹性流水：输入拍寄存 + 组合计算 + 输出 sync_fifo（水位冻结）。延迟 2 拍。
//==============================================================================
module fp64_add (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [63:0] in_a,
    input  wire [63:0] in_b,
    output wire        out_valid,
    input  wire        out_ready,
    output wire [63:0] out_r
);

    localparam AW     = 5;
    localparam THRESH = 31;

    wire [AW:0] fcnt;
    wire        wfull;
    assign in_ready = (fcnt < THRESH);
    wire acc = in_valid && in_ready;

    reg [63:0] a_r, b_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin a_r <= 64'd0; b_r <= 64'd0; end
        else if (acc)  begin a_r <= in_a; b_r <= in_b; end
    end

    //------------------------------------------------------------------------
    // 解码与特殊值识别：value = M·2^(e-1075)
    //------------------------------------------------------------------------
    wire        sa = a_r[63];
    wire        sb = b_r[63];
    wire [10:0] ea = a_r[62:52];
    wire [10:0] eb = b_r[62:52];
    wire [51:0] fa = a_r[51:0];
    wire [51:0] fb = b_r[51:0];
    wire a_nan = (ea == 11'h7FF) && (fa != 52'd0);
    wire b_nan = (eb == 11'h7FF) && (fb != 52'd0);
    wire a_inf = (ea == 11'h7FF) && (fa == 52'd0);
    wire b_inf = (eb == 11'h7FF) && (fb == 52'd0);
    wire az = (ea == 11'h0) && (fa == 52'd0);
    wire bz = (eb == 11'h0) && (fb == 52'd0);
    wire inf_nan = a_inf && b_inf && (sa != sb);        // Inf + (-Inf) → NaN

    // normal: M={1,frac}, e=ea；subnormal: M=frac, e=1；zero: M=0, e=0
    wire [52:0] ma  = {1'b1, fa};
    wire [52:0] mb  = {1'b1, fb};
    wire [52:0] maz = (ea == 11'h0) ? {1'b0, fa} : ma;
    wire [52:0] mbz = (eb == 11'h0) ? {1'b0, fb} : mb;
    wire [10:0] eaz = (ea == 11'h0) ? ((fa == 52'd0) ? 11'd0 : 11'd1) : ea;
    wire [10:0] ebz = (eb == 11'h0) ? ((fb == 52'd0) ? 11'd0 : 11'd1) : eb;

    wire bigger_a = (eaz == ebz) ? (maz >= mbz) : (eaz > ebz);

    wire [52:0] lm  = bigger_a ? maz   : mbz;
    wire [52:0] sm  = bigger_a ? mbz   : maz;
    wire [10:0] ler = bigger_a ? eaz   : ebz;
    wire [10:0] ser = bigger_a ? ebz   : eaz;
    wire        ss  = bigger_a ? sa : sb;              // 结果符号 = 量值较大操作数的符号
    wire same_sign = (sa == sb);
    wire [10:0] diff = ler - ser;                      // ≥0

    // G=52 守卫小数位：diff≤52 时较小操作数完整保留（C 精确），diff>52 截断入 sticky
    localparam G  = 52;
    localparam W  = 53 + G;                            // 105
    wire [W-1:0] man_big  = {lm,  {G{1'b0}}};          // lm<<G
    wire [W-1:0] t4       = {sm,  {G{1'b0}}};          // sm<<G
    wire [W-1:0] man_small;
    wire         st_all;
    wire big_shift = (diff >= W);
    assign man_small = big_shift            ? {W{1'b0}}
                     : (diff == 0)          ? t4
                     :                       (t4 >> diff);
    assign st_all   = big_shift             ? (|sm)
                    : (diff == 0)           ? 1'b0
                    :                         (|(t4 & ((105'h1 << diff) - 105'h1)));

    wire [W:0] C = same_sign ? ({1'b0, man_big} + {1'b0, man_small})
                             : ({1'b0, man_big} - {1'b0, man_small});

    function automatic [7:0] msb106;
        input [W:0] v;
        integer i;
        begin
            msb106 = 8'd0;
            for (i = W; i >= 0; i = i - 1)
                if (v[i]) begin msb106 = i[7:0]; i = -1; end
        end
    endfunction
    wire [7:0] n = msb106(C);

    wire czero = (C == 106'd0);
    wire signed [9:0] drop = $signed({3'b000, n}) - 10'sd52;

    wire [52:0] mtop = (drop > 0) ? (C >> (drop[5:0])) : (C << (-drop[5:0]));
    wire        rbit = (drop > 0) ? C[drop-1] : 1'b0;
    wire [105:0] m_rst = (drop >= 2) ? ((106'h1 << (drop - 10'sd1)) - 106'h1) : 106'd0;
    wire        sbit = (|(C & m_rst));
    wire        inc  = rbit && (sbit || st_all || mtop[0]);

    wire [53:0] madd = {1'b0, mtop} + {53'd0, inc};
    wire        mcar = madd[53];
    wire [52:0] mant = mcar ? 53'h10000000000000 : madd[52:0];   // 进位 → 尾数回 2^52（frac=0）

    // 结果指数（含进位），14 位有符号：efield = ler + n - 104
    wire signed [13:0] ebase = $signed({3'b000, ler}) + $signed({6'd0, n}) - 14'sd104;
    wire signed [13:0] efield = ebase + (mcar ? 14'sd1 : 14'sd0);

    // 正常路径（efield ∈ [1,2046]）
    wire sign0 = czero ? 1'b0 : ss;
    wire ovf = !czero && (efield >= 14'sd2047);
    wire [63:0] res_norm = {sign0, efield[10:0], mant[51:0]};
    wire [63:0] res_ovf  = {sign0, 11'h7FF, 52'd0};

    // 次正规结果路径（efield ≤ 0）：f = mtop·2^(efield-1)，右移 sh=1-efield
    wire signed [13:0] sh = 14'sd1 - efield;           // ∈ [1,52]
    wire [51:0] f_hi = (sh >= 53) ? 52'd0 : (mtop >> sh[5:0]);
    wire        gb  = mtop[sh-1];                      // sh∈[1,52] → 索引安全
    wire        rb  = (sh >= 2) ? mtop[sh-2] : rbit;
    // bits [sh-3:0]（OR）以常量宽度掩码等价实现（变量界定的部分选择不可综合）
    wire        stb = (sh >= 3) ? (|(mtop & ((53'h1 << (sh[5:0] - 6'd2)) - 53'h1))) : 1'b0;
    wire        st2 = stb || ((sh == 2) ? rbit : 1'b0) || sbit || st_all;
    wire        inc2 = gb && (rb || st2 || f_hi[0]);
    wire [52:0] f_round = {1'b0, f_hi} + {52'd0, inc2};
    wire [63:0] res_sub = f_round[52]    ? {sign0, 11'd1, 52'd0}    // 进位成最小正规
                        : (f_round == 0) ? {sign0, 11'd0, 52'd0}    // 下溢→±0
                        :                  {sign0, 11'd0, f_round[51:0]};

    // 零规范化：相消→+0；(-0)+(-0)→-0
    wire czero_sign = (az && bz && sa && sb);
    wire [63:0] res_zero = {czero_sign, 11'd0, 52'd0};

    // 特殊值：NaN 统一输出规范 QNaN
    wire has_special = a_nan || b_nan || a_inf || b_inf || inf_nan;
    wire [63:0] res_special =
        (a_nan || b_nan || inf_nan) ? 64'h7FF8_0000_0000_0000
      : (a_inf)                     ? {sa, 11'h7FF, 52'd0}
      :                               {sb, 11'h7FF, 52'd0};

    wire [63:0] res =
        has_special ? res_special
      : (czero)     ? res_zero
      : (ovf)       ? res_ovf
      : (efield <= 14'sd0) ? res_sub
      :                      res_norm;

    reg psh;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) psh <= 1'b0;
        else        psh <= acc;
    end

    sync_fifo #(.DATA_WIDTH(64), .ADDR_WIDTH(AW)) u_fifo (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (psh),
        .in_ready (wfull),
        .in_data  (res),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_data (out_r),
        .count    (fcnt),
        .empty    (),
        .full     ()
    );

endmodule
