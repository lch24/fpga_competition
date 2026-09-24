`timescale 1ns / 1ps
//==============================================================================
// fp64_sqrt.v — 双精度 IEEE-754 开方（RNE），与 C++ std::sqrt 逐位一致
//------------------------------------------------------------------------------
// 语义：out_r = RNE( √in_x )（fp64）。
//
// 输入假设：in_x 为 fp32 提升或精确乘积（非负精确整数，2^52 ≤ x ≤ 2^54 量级），
//   √x ≤ ~2^27。负数（防御）与 0 分别返回 0（min_eigen 已先做 max(0,·) 钳位）。
//
// 实现（整数牛顿 isqrt + fp64 尾数 RNE）：
//   1) x = m·2^E（m∈[1,2)，E 无偏指数）。寄生的二次根精度按 2^54 放大：
//        E 偶：N = A·2^56（A={1'b1,尾数}），ee = E/2；
//        E 奇：N = A·2^57，                                ee = (E-1)/2。
//      √x 的 54 位整数近似 g = isqrt(N)（逐位二分，精确 floor）。
//   2) 结果 = (g/2^54)·2^ee；取 52 位尾数 g[53:2]，guard=g[1:0]，sticky=|g[0]|余数。
//   3) RNE：inc = g[1] && (g[0]||余数||g[2])；进位时尾数回零、指数 +1。
//   isqrt 用逐位二分（每次试 (res+bit)²≤N），收敛精确，不受初始猜影响。
//
// 弹性流水：输入拍寄存 + 组合计算 + 输出 sync_fifo（水位冻结）。延迟 2 拍
//   （isqrt 为组合函数，综合会展开为逻辑网络；本任务以仿真对拍为准）。
//==============================================================================
module fp64_sqrt (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [63:0] in_x,
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

    reg [63:0] x_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) x_r <= 64'd0;
        else if (acc)  x_r <= in_x;
    end

    //--------------------------------------------------------------------------------
    // 精确整数平方根（逐位二分，floor），N 最高到 2^110，g 最高到 2^55
    //--------------------------------------------------------------------------------
    function automatic [54:0] isqrt55;
        input [119:0] n;
        integer i;
        reg [119:0] bt, cand, sq;
        reg [54:0]  res;
        begin
            res = 55'd0;
            bt  = 120'h1 << 54;
            for (i = 54; i >= 0; i = i - 1) begin
                cand = res + bt;
                sq   = cand * cand;
                if (sq <= n) res = cand;
                bt = bt >> 1;
            end
            isqrt55 = res;
        end
    endfunction

    // Origin 组合计算
    wire        s   = x_r[63];
    wire [10:0] xe  = x_r[62:52];
    wire [51:0] xm  = x_r[51:0];
    wire iszero = (xe == 11'd0) && (xm == 52'd0);

    wire signed [11:0] E = $signed({1'b0, xe}) - 12'sd1023;
    wire even = ~E[0];

    wire [52:0] A = {1'b1, xm};
    wire [119:0] Neven = {11'd0, A, 56'd0};   // A·2^56（A 53 位 + 56 位 = 109 位）
    wire [119:0] Nodd  = {10'd0, A, 57'd0};   // A·2^57（110 位）
    wire [119:0] Nval  = even ? Neven : Nodd;

    wire signed [12:0] ee = even ? (E >> 1) : ((E - 12'sd1) >> 1);
    wire [54:0] g = isqrt55(Nval);

    // 取尾数与 RNE 舍入
    wire [51:0] mant_c = g[53:2];
    wire hasRem = (g * g != Nval);
    wire incq   = g[1] && (g[0] || hasRem || g[2]);

    wire [52:0] mm   = {1'b0, mant_c} + {52'd0, incq};
    wire        mcar = mm[52];
    wire [51:0] mant = mm[51:0];   // mcar 时高位进位、低 52 位置零（2^52）

    wire signed [12:0] ee2 = ee + (mcar ? 13'sd1 : 13'sd0);
    wire [10:0] efield = ee2[10:0] + 11'd1023;   // ee ≥ 0（本域），直接用低 11 位

    wire [63:0] res = (iszero || s) ? 64'd0
                    : {1'b0, efield, mant};

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