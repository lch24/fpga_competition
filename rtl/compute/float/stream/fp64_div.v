`timescale 1ns / 1ps
//==============================================================================
// fp64_div.v — 双精度 IEEE-754 除法（RNE，位级 = C++ double 除法）
//------------------------------------------------------------------------------
// 语义：out_r = RNE( in_a / in_b )（fp64）。完整 IEEE-754 binary64 除法：
//   恢复余数长除（radix-2，54 步，得 54 位商 = 52 尾数 + guard + round），
//   sticky = 精确余数非零，按 RNE 舍入；subnormal 输入先规格化，subnormal
//   结果在次正规尺度重取 guard/round/sticky；±0、±Inf、NaN（统一规范
//   0x7FF8_0000_0000_0000）齐备，与 C++ double 逐位一致。
//
// 数值：输入规格化为 Ma/Mb ∈ [1,2)（ageb）或 [0.5,1)（!ageb），
//   N = ageb ? Ma-Mb : Ma（恒 N < Mb），q = floor(N·2^54/Mb)（54 位），
//   mantissa=q[53:2]，g=q[1]，r=q[0]；inc = g&&(r||sticky||LSB)，进位尾数回零。
//   结果指数 e0 = Ea-Eb+1023-(ageb?0:1)：
//     e0≥1 正常（e1 = e0+进位，≥2047 → ±Inf）；
//     e0≤0 次正规：f = W·2^(3-e0-adj)，W = ageb ? 2^54+q : q，RNE 进位成最小正规。
//
// Serial restoring division: 54 steps plus preparation and rounding.
// Ordinary input latency 56 cycles, special values 2; reset cancels in-flight work.
//==============================================================================
module fp64_div #(parameter USE_CE=0) (
    // Global synchronous stall for variable-latency backing memory.
    input wire ce,

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

    // One restoring division bit per cycle; no wide multiplier for sticky.
    localparam IDLE=0, PREPARE=1, DIVIDE=2, PACK=3, RESPONSE=4;
    reg [2:0] state;
    reg [63:0] a_r,b_r,result_q;
    reg [53:0] quotient_q;
    reg [54:0] remainder_q, denominator_q;
    reg [5:0] step_q;
    wire [54:0] brought=remainder_q<<1;
    wire take=brought>=denominator_q;
    wire [54:0] rest=take ? brought-denominator_q : brought;
    assign in_ready=rst_n && state==IDLE;
    assign out_valid=rst_n && state==RESPONSE;
    assign out_r=result_q;

    //------------------------------------------------------------------------
    // 解码与特殊值识别
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
    wire a_sub = (ea == 11'h0) && (fa != 52'd0);
    wire b_sub = (eb == 11'h0) && (fb != 52'd0);
    wire sign = sa ^ sb;
    wire special = a_nan || b_nan || (az && bz) || (a_inf && b_inf);

    // subnormal 输入规格化：value = frac·2^-1074 = M·2^(E-1075)，M∈[2^52,2^53)
    function automatic [5:0] msb52;
        input [51:0] v;
        integer i;
        begin
            msb52 = 6'd0;
            // Fixed-bound highest-set-bit scan, compatible with PDS unrolling.
            for (i = 0; i <= 51; i = i + 1)
                if (v[i]) msb52 = i[5:0];
        end
    endfunction
    wire [5:0] msa = a_sub ? msb52(fa) : 6'd0;
    wire [5:0] msb_ = b_sub ? msb52(fb) : 6'd0;
    wire [5:0] ka = a_sub ? (52 - msa) : 6'd0;         // M = frac<<ka，E = 1-ka
    wire [5:0] kb = b_sub ? (52 - msb_) : 6'd0;

    wire [53:0] Ma54 = a_sub ? ({2'b0, fa} << ka) : {2'b0, 1'b1, fa};
    wire [53:0] Mb54 = b_sub ? ({2'b0, fb} << kb) : {2'b0, 1'b1, fb};
    wire [52:0] Ma = Ma54[52:0];
    wire [52:0] Mb = Mb54[52:0];
    // subnormal 输入规格化后 E ∈ [-51, 0]（有符号），正常输入 E = ea ∈ [1,2046]
    wire signed [13:0] Ea = a_sub ? (14'sd1 - {8'd0, ka}) : {3'b000, ea};
    wire signed [13:0] Eb = b_sub ? (14'sd1 - {8'd0, kb}) : {3'b000, eb};

    //------------------------------------------------------------------------
    // 恢复余数除法：q = floor(N·2^54/Mb)，54 步
    //------------------------------------------------------------------------
    wire ageb = (Ma >= Mb);
    wire adj  = ageb ? 1'b0 : 1'b1;                    // Q∈[1,2) → 0；Q∈[0.5,1) → 1
    wire [52:0] N = ageb ? (Ma - Mb) : Ma;             // 恒 N < Mb
    wire [53:0] q = quotient_q;
    wire sticky = |remainder_q;

    // RNE：mant=q[53:2]，g=q[1]，r=q[0]，sticky=余数
    // !ageb（Q∈[0.5,1)）：商=q/2^54·2^(Ea-Eb) 规格化后尾数=(q>>1)-2^52，
    //   故尾数域=q[52:1]、guard=q[0]、round 无（补 0），sticky 由精确余数提供。
    wire [51:0] mant_raw = ageb ? q[53:2] : q[52:1];
    wire        g = ageb ? q[1] : q[0];
    wire        r = ageb ? q[0] : 1'b0;
    wire        inc = g && (r || sticky || mant_raw[0]);
    wire [52:0] m2 = {1'b0, mant_raw} + {52'd0, inc};
    wire        qcar = m2[52];
    wire [51:0] qmant = m2[51:0];

    // 结果指数：e0 = Ea-Eb+1023-adj；e1 = e0+进位
    wire signed [13:0] e0 = Ea - Eb + 14'sd1023 - $signed({13'd0, adj});
    wire signed [13:0] e1 = e0 + (qcar ? 14'sd1 : 14'sd0);

    // 次正规结果（e0≤0）：f = W·2^-kk，kk = 3-e0-adj（舍入在次正规尺度重做，不计 qcar）
    wire signed [13:0] kk = 14'sd3 - e0 - $signed({13'd0, adj});
    wire [54:0] W55 = ageb ? (55'h40000000000000 + {1'b0, q}) : {1'b0, q};   // 2^54+q 或 q
    wire [51:0] f_hi = (kk >= 55) ? 52'd0 : (W55 >> kk[5:0]);
    wire        gb  = (kk >= 1 && kk <= 55) ? ((W55 >> (kk-1)) & 1'b1) : 1'b0;
    wire        rb  = (kk >= 2 && kk <= 56) ? ((W55 >> (kk-2)) & 1'b1) : 1'b0;
    wire        stb = (kk >= 3 && kk <= 56) ? (|(W55 & ((56'h1 << (kk - 2)) - 56'h1))) : 1'b0;
    wire        st2 = stb || sticky;
    wire        inc2 = gb && (rb || st2 || f_hi[0]);
    wire [52:0] f_round = {1'b0, f_hi} + {52'd0, inc2};
    wire [63:0] res_sub = f_round[52]    ? {sign, 11'd1, 52'd0}    // 进位成最小正规
                        : (f_round == 0) ? {sign, 11'd0, 52'd0}    // 下溢→±0
                        :                  {sign, 11'd0, f_round[51:0]};

    // 组装（含防御：NaN/0/0/Inf/Inf → QNaN；0/x、x/Inf → ±0；x/0、Inf/x → ±Inf）
    wire [63:0] res =
        special              ? 64'h7FF8_0000_0000_0000
      : (a_inf)              ? {sign, 11'h7FF, 52'd0}
      : (b_inf)              ? {sign, 11'd0, 52'd0}
      : (az)                 ? {sign, 11'd0, 52'd0}
      : (bz)                 ? {sign, 11'h7FF, 52'd0}
      : (e1 >= 14'sd2047)    ? {sign, 11'h7FF, 52'd0}     // 溢出 → ±Inf
      : (e0 <= 14'sd0)       ? res_sub
      :                        {sign, e1[10:0], qmant};

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE; a_r<=0; b_r<=0; result_q<=0;
            quotient_q<=0; remainder_q<=0; denominator_q<=0; step_q<=0;
        end else if(!USE_CE || ce) begin case(state)
            IDLE: if(in_valid) begin a_r<=in_a; b_r<=in_b; state<=PREPARE; end
            PREPARE: begin
                quotient_q<=0; remainder_q<={2'd0,N};
                denominator_q<={2'd0,Mb}; step_q<=0;
                if(special || a_inf || b_inf || az || bz) state<=PACK;
                else state<=DIVIDE;
            end
            DIVIDE: begin
                remainder_q<=rest; quotient_q<={quotient_q[52:0],take};
                if(step_q==53) state<=PACK; else step_q<=step_q+1'b1;
            end
            PACK: begin result_q<=res; state<=RESPONSE; end
            RESPONSE: if(out_ready) state<=IDLE;
            default: state<=IDLE;
        endcase
    end // synchronous clock enable
    end
endmodule
