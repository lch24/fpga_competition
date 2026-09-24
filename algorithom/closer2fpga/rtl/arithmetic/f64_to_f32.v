`timescale 1ns / 1ps
//==============================================================================
// f64_to_f32.v — fp64 → fp32 IEEE-754 转换（RNE）
//------------------------------------------------------------------------------
// 语义：out_r = RNE_f32( in_x )。本域值适中（0 ~ ~2^25），正常舍入为主；
//   另按 IEEE 兜底：溢出→±Inf、下溢→次正规或 ±0（均正确舍入）。
//
// 实现：
//   保持 fp64 尾数（52 位），右移 29 位得 fp32 的 23 位尾数：
//     · 正常值：round 位 = f[28]，sticky = |f[27:0]，LSB = f[29]；尾数进位则指数+1。
//     · 次正规（目标指数 < -126）：把 v·2^149 取 23 位整再 RNE（进位成最小正规）。
//   指数域 = E + 127。
//
// 弹性流水：输入拍寄存 + 组合计算 + 输出 sync_fifo（水位冻结）。延迟 2 拍。
//==============================================================================
module f64_to_f32 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [63:0] in_x,
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

    reg [63:0] x_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) x_r <= 64'd0;
        else if (acc)  x_r <= in_x;
    end

    wire        s   = x_r[63];
    wire [10:0] xe  = x_r[62:52];
    wire [51:0] xf  = x_r[51:0];
    wire [52:0] m53 = {1'b1, xf};
    wire signed [11:0] E = $signed({1'b0, xe}) - 12'sd1023;
    wire signed [12:0] e32 = E + 13'sd127;          // fp32 有偏指数（含 -126..-102 次正规）
    wire iszero = (xe == 11'h0) && (xf == 52'd0);

    //----- 正常路径：右移 29 取 23 位尾数 -----
    wire [22:0] mn  = m53[51:29];
    wire        rbit = m53[28];
    wire        sbit = |m53[27:0];
    wire        inc  = rbit && (sbit || mn[0]);
    wire [23:0] madd = {1'b0, mn} + {23'd0, inc};
    wire        mcar = madd[23];
    wire [22:0] mnormal = madd[22:0];
    wire [12:0] efin  = e32 + (mcar ? 13'sd1 : 13'sd0);   // 尾数进位指数+1

    //----- 次正规路径：v·2^149 取 23 位整，RNE，进位成最小正规 -----
    wire signed [6:0]shift = -E - 7'sd97;             // = -(E+97)，次正规 E∈[-149,-127]→shift∈[30,52]
    wire [52:0] sshifted = (shift > 0) ? (m53 >> shift[5:0]) : m53;
    wire        srub = (shift > 0) ? m53[shift-1] : 1'b0;      // 单比特变址选择合法
    wire [52:0] smask = (shift > 1) ? ((53'h1 << (shift - 7'sd1)) - 53'h1) : 53'd0;
    wire        srst = (shift > 1) ? (|(m53 & smask)) : 1'b0;
    wire        srinc = srub && (srst || sshifted[0]);
    wire [23:0] sr_full = {1'b0, sshifted} + {23'd0, srinc};
    wire        sr_normcarry = sr_full[23];           // 进位成 2^23 = 最小正规
    wire [22:0] msub = sr_full[22:0];

    // fp32 有偏指数 ≥1 → 正常；e32∈(0,1]边界与次正规对照
    wire want_normal = (e32 >= 13'sd1);

    wire [7:0]  ef_n = want_normal ? efin[7:0] : 8'd0;
    wire [22:0] nt23 = want_normal ? mnormal : (sr_normcarry ? 23'd0 : msub);
    wire [7:0]  expn = want_normal ? (efin >= 13'sd255 ? 8'hFF : ef_n)
                                   : (sr_normcarry ? 8'h01 : 8'd0);

    wire [31:0] res = iszero ? 32'd0
                    : {s, expn, nt23};

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