`timescale 1ns / 1ps
//==============================================================================
// fp64_sub.v — 双精度 IEEE-754 减法（RNE）
//------------------------------------------------------------------------------
// 语义：out_r = RNE( in_a - in_b )（fp64）。
//   · discriminant = trace² - 4·det：两真值均为精确整数（<2^50），本步无新舍入。
//   · trace - sqrt(disc)：sqrt 为无理数，需正确 RNE，与 C++ std::sqrt 链对拍。
// 通用实现处理 ±0 与大数相消（守卫位方法，同 fp32_add 的 53 位版本）。
//
// 实现：同 fp32_add（magnitude 对齐 + 守卫位 RNE），尾数 53 位、指数 11 位
//   （偏置 1023）。efield = n + ler - 52。
//
// 弹性流水：输入拍寄存 + 组合计算 + 输出 sync_fifo（水位冻结）。延迟 2 拍。
//==============================================================================
module fp64_sub (
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

    wire        sa = a_r[63];
    wire        sb = b_r[63];
    wire [10:0] ea = a_r[62:52];
    wire [10:0] eb = b_r[62:52];
    wire [52:0] ma = {1'b1, a_r[51:0]};
    wire [52:0] mb = {1'b1, b_r[51:0]};
    wire za = (ea == 11'h0);
    wire zb = (eb == 11'h0);
    wire [52:0] maz = za ? 53'd0 : ma;
    wire [52:0] mbz = zb ? 53'd0 : mb;
    wire [10:0] eaz = za ? 11'd0 : ea;
    wire [10:0] ebz = zb ? 11'd0 : eb;

    wire bigger_a = (eaz == ebz) ? (maz >= mbz) : (eaz > ebz);

    wire [52:0] lm  = bigger_a ? maz   : mbz;
    wire [52:0] sm  = bigger_a ? mbz   : maz;
    wire [10:0] ler = bigger_a ? eaz   : ebz;
    wire [10:0] ser = bigger_a ? ebz   : eaz;
    wire        sb_sub = ~b_r[63];              // 减法 = a + (-b)
    wire        ss  = bigger_a ? sa : sb_sub;   // 结果符号 = 量值较大者的符号(a 或 -b)
    wire same_sign = (sa == sb_sub);
    wire [10:0] diff = ler - ser;

    // G=52 守卫小数位：任意 diff≤52（小操作数落入结果 53 显著位窗口）时小操作数
    //   完整保留，配合 sticky，保证"精确整数相减得精确整数(判别式 trace²-4det)"
    //   逐位精确；diff>52 时小操作数完全低于结果 LSB，仅作 sticky，不影响舍入。
    localparam G  = 52;
    localparam W  = 53 + G;                          // 105
    wire [W-1:0] man_big  = {lm,  {G{1'b0}}};            // lm<<G
    wire [W-1:0] t4       = {sm,  {G{1'b0}}};            // sm<<G
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
    wire [105:0] m_rst = (drop >= 2) ? ((106'h1 << (drop - 10'sd1)) - 106'h1) : 106'd0;   // 位 [drop-2:0]
    wire        sbit = (|(C & m_rst));
    wire        inc  = rbit && (sbit || st_all || mtop[0]);

    wire [53:0] madd = {1'b0, mtop} + {53'd0, inc};
    wire        mcar = madd[53];
    wire [52:0] mant = mcar ? 53'h8000000000000 : madd[52:0];

    wire [11:0] ebase = {1'b0, ler} + {8'd0, n} - 12'd104;
    wire [11:0] efield = ebase + (mcar ? 12'd1 : 12'd0);

    wire sign0 = czero ? 1'b0 : ss;
    wire ovf = (efield >= 12'h7FF) && !czero;
    wire ufl = (efield == 12'h0)   && !czero;

    wire [63:0] res =
        czero                 ? 64'd0
      : (ovf)                 ? {sign0, 11'h7FF, 52'd0}
      : (ufl)                 ? {sign0, 11'h0, 52'd0}
      :                         {sign0, efield[10:0], mant[51:0]};

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