// Shared fp32 round-half-away-from-zero function body.
// Include inside a module or package. Intentionally no global include guard:
// each enclosing scope needs its own declaration. Keep one implementation here.
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
            m24=0; shift=0; intp=0; frac=0; half=0; mag=0; k=0;
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
