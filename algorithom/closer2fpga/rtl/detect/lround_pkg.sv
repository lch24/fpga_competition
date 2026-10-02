//==============================================================================
// lround_pkg.sv — fp32 → int32 四舍五入（round half away from zero）
//------------------------------------------------------------------------------
// 语义：本函数 == C++ std::lround(float)（对有限值，值域 |x| < 2^31）：
//   · 整数部分直接取；小数部分 ≥ 0.5 进位（.5 向远离零方向）。
//   · 域外（|x| ≥ 2^31）饱和到 ±(2^31-1)（C++ 未定义，防御性处理）。
//
// 实现（整数域精确，无浮点加 0.5 的精度风险）：
//   值 = m24 × 2^(e-23)，m24 = {1,尾数}，e = exp-127。
//   · e < -1（|x|<0.5）→ 0；e == -1（0.5≤|x|<1）→ ±1。
//   · e ≥ 0：小数位个数 shift = 23-e；shift ≤ 0 时 |x| 为整数，
//     shift ≥ 1 时整数部分 = m24>>shift，frac = 低 shift 位，
//     frac ≥ 2^(shift-1) 则整数部分 +1（half away from zero）。
//
// 用法：ring_check.sv 采样坐标转换、tb_ring.sv 独立对拍共用本函数，
//   保证被测路径与验证路径同源。
//==============================================================================
package lround_pkg;

    function automatic signed [31:0] lround_f32(input [31:0] v);
        reg        sgn;
        reg [7:0]  e8;
        reg [22:0] man;
        integer    m24;          // 24 位尾数（隐含 1）
        integer    shift;        // 小数位个数（可为负）
        integer    intp, frac, half;
        integer    mag;
        integer    k;
        begin
            sgn = v[31];
            e8  = v[30:23];
            man = v[22:0];
            if (e8 == 8'd0) begin
                // ±0 / 次正规 → 0
                lround_f32 = 32'sd0;
            end else if (e8 < 8'd126) begin
                // |x| < 0.5 → 0
                lround_f32 = 32'sd0;
            end else if (e8 == 8'd126) begin
                // |x| ∈ [0.5, 1.0) → ±1（0.5 本身向远离零 → 1）
                lround_f32 = sgn ? -32'sd1 : 32'sd1;
            end else if (e8 >= 8'd158) begin
                // |x| ≥ 2^31 → 饱和
                lround_f32 = sgn ? 32'h80000000 : 32'h7FFFFFFF;
            end else begin
                m24   = (1 << 23) | man;
                shift = 23 - (e8 - 127);
                if (shift <= 0) begin
                    // |x| 为整数（≥ 2^23）
                    mag = m24 << (-shift);
                end else begin
                    intp = m24 >> shift;
                    frac = m24 & ((1 << shift) - 1);
                    half = 1 << (shift - 1);
                    if (frac >= half)
                        intp = intp + 1;
                    mag = intp;
                end
                if (mag >= 32'h80000000) begin
                    lround_f32 = sgn ? 32'h80000000 : 32'h7FFFFFFF;
                end else begin
                    lround_f32 = sgn ? (-mag) : mag;
                end
            end
        end
    endfunction

endpackage
