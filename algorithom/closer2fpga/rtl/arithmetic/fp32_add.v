`timescale 1ns / 1ps
//==============================================================================
// fp32_add.v — 单精度 IEEE-754 加减法（RNE：round-to-nearest-even）
//------------------------------------------------------------------------------
// 语义：out_r = RNE( in_a + in_b )，与 C++ float 加法逐次舍入逐位一致。
// fp32_sub 即本模块的薄封装（队友签反号）。
//
// 输入假设（本子系统值域）：in_a / in_b 为 normal fp32 精确整数，无 NaN/Inf。
//   支持 ±0 与同号/异号大数相消（守卫位方案，覆盖 2^25±1 类边界）。
//
// 实现（magnitude 对齐 + 守卫位 RNE）：
//   1) 取较大操作数（指数优先，次尾数）为基准，较小者按指数差 diff 右移对齐，
//      丢弃低位并入 sticky（st_all）。
//   2) 同号相加 / 异号相减得 26 位 C；因基准已取较大者，异号相减 C≥0。
//   3) 找 C 最高位 n；efield = n + ler - 23（ler=较大者偏置指数），归一化 24 位尾数。
//   4) 向下取整位做 RNE：round 位 + sticky（含 st_all）+ LSB。
//   5) 尾数进位时右移一位、指数 +1。
//
// 弹性流水：同 fp32_mul（输入拍寄存 + 组合计算 + 输出 sync_fifo，水位冻结）。
// 延迟：接受拍→FIFO 出拍 2 拍。
//==============================================================================
module fp32_add (
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

    localparam AW     = 5;
    localparam THRESH = 31;

    wire [AW:0] fcnt;
    wire        wfull;
    assign in_ready = (fcnt < THRESH);
    wire acc = in_valid && in_ready;

    reg [31:0] a_r, b_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin a_r <= 32'd0; b_r <= 32'd0; end
        else if (acc)  begin a_r <= in_a; b_r <= in_b; end
    end

    //--------------------------------------------------------------------
    // 组合计算：RNE 加减
    //--------------------------------------------------------------------
    wire        sa = a_r[31];
    wire        sb = b_r[31];
    wire [7:0]  ea = a_r[30:23];
    wire [7:0]  eb = b_r[30:23];
    wire [23:0] ma = {1'b1, a_r[22:0]};
    wire [23:0] mb = {1'b1, b_r[22:0]};
    wire za = (ea == 8'h0);
    wire zb = (eb == 8'h0);
    wire [23:0] maz = za ? 24'd0 : ma;
    wire [23:0] mbz = zb ? 24'd0 : mb;
    wire [7:0]  eaz = za ? 8'd0  : ea;
    wire [7:0]  ebz = zb ? 8'd0  : eb;

    wire bigger_a = (eaz == ebz) ? (maz >= mbz) : (eaz > ebz);

    wire [23:0] lm  = bigger_a ? maz   : mbz;
    wire [23:0] sm  = bigger_a ? mbz   : maz;
    wire [7:0]  ler = bigger_a ? eaz   : ebz;
    wire [7:0]  ser = bigger_a ? ebz   : eaz;
    wire        ss  = bigger_a ? sa : sb;        // 结果符号 = 量值较大操作数的符号

    wire same_sign = (sa == sb);
    wire [7:0] diff = ler - ser;                 // ≥0

    // G=52 守卫小数位：本域尾数以 24 位、操作数可达 2^48、指数差 ≤~48，令 G 足够大
    //   使较小操作数右移对齐时绝不截断（diff≤G 完整保留），C 精确，RNE 逐位正确。
    //   （若截断进 sticky，减法方向会令 RNE 误向上取整；大 G 从根上消除该风险。）
    localparam G = 52;
    localparam W = 24 + G;                        // 76
    wire [W-1:0] man_big  = {lm,  {G{1'b0}}};         // lm<<G
    wire [W-1:0] t3       = {sm,  {G{1'b0}}};         // sm<<G
    wire [W-1:0] man_small;
    wire         st_all;
    wire big_shift = (diff >= W);
    assign man_small = big_shift            ? {W{1'b0}}
                     : (diff == 0)          ? t3
                     :                       (t3 >> diff);
    assign st_all   = big_shift             ? (|sm)
                    : (diff == 0)           ? 1'b0
                    :                         (|(t3 & ((76'h1 << diff) - 76'h1)));

    wire [W:0] C = same_sign ? ({1'b0, man_big} + {1'b0, man_small})
                             : ({1'b0, man_big} - {1'b0, man_small});

    function automatic [6:0] msb77;
        input [W:0] v;
        integer i;
        begin
            msb77 = 7'd0;
            for (i = W; i >= 0; i = i - 1)
                if (v[i]) begin msb77 = i[6:0]; i = -1; end
        end
    endfunction
    wire [6:0] n = msb77(C);

    wire czero = (C == 77'd0);
    wire signed [8:0] drop = $signed({3'b000, n}) - 9'sd23;

    // C 为整型（低 G 位即守卫位）。右移保留 24 位尾数，round/sticky 取自被丢弃位+st
    wire [23:0] mtop = (drop > 0) ? (C >> drop[5:0]) : (C << (-drop[5:0]));
    wire        rbit = (drop > 0) ? C[drop-1] : 1'b0;
    wire [76:0] m_rst = (drop >= 2) ? ((77'h1 << (drop - 9'sd1)) - 77'h1) : 77'd0;   // 位 [drop-2:0]
    wire        sbit = (|(C & m_rst));
    wire        inc  = rbit && (sbit || st_all || mtop[0]);

    wire [24:0] madd = {1'b0, mtop} + {24'd0, inc};
    wire        mcar = madd[24];
    wire [23:0] mant = mcar ? 24'h800000 : madd[23:0];

    // 指数 = n + ler - (M-1) - G；尾数进位 +1
    wire [8:0] ebase = {1'b0, ler} + {7'd0, n} - 9'd75;
    wire [8:0] efield = ebase + (mcar ? 9'd1 : 9'd0);

    // 结果符号：较大者符号；若相消为 0 则 RNE 得 +0
    wire sign0 = czero ? 1'b0 : ss;
    wire ovf = (efield >= 9'd255) && !czero;
    wire ufl = (efield == 9'd0)  && !czero;

    wire [31:0] res =
        czero                 ? 32'd0
      : (ovf)                 ? {sign0, 8'hFF, 23'd0}   // ±Inf（本域不触发）
      : (ufl)                 ? {sign0, 8'd0, 23'd0}    // 下溢→±0（本域不触发）
      :                         {sign0, efield[7:0], mant[22:0]};

    reg psh;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) psh <= 1'b0;
        else        psh <= acc;
    end

    sync_fifo #(.DATA_WIDTH(32), .ADDR_WIDTH(AW)) u_fifo (
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