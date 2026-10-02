`timescale 1ns / 1ps
//==============================================================================
// fp64_mul.v — 双精度 IEEE-754 乘法（RNE）
//------------------------------------------------------------------------------
// 语义：out_r = RNE( in_a × in_b )（fp64）。本子系统输入为 fp32 提升/精确整数
//   （trace、det、常量 4.0），乘积 < 2^53+ 精确，普通 fp64 RNE 乘法即可。
// 通用实现仍处理 ±0、±正常数、指数溢出/下溢。
//
// 实现（同 fp32_mul，53 位尾数、11 位指数、偏置 1023）：
//   1) 53×53 尾数相乘得 106 位 P；依 P[105]（进位）选两套归一化。
//   2) efield（进位）= ea+eb-1022；否则 = ea+eb-1023。
//   3) 取 53 位尾数，round/sticky 做 RNE，尾数进位再右移+指数+1。
//
// 弹性流水：输入拍寄存 + 组合计算 + 输出 sync_fifo（水位冻结）。
// 延迟：接受拍→FIFO 出拍 2 拍。
//==============================================================================
module fp64_mul (
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

    // 106 位乘积
    wire [105:0] p = {53'd0, ma} * {53'd0, mb};
    wire         carry = p[105];
    wire [52:0]  mtop = carry ? p[105:53] : p[104:52];
    wire         rbit = carry ? p[52]     : p[51];
    wire         sbit = carry ? (|p[51:0]) : (|p[50:0]);
    wire         inc  = rbit && (sbit || mtop[0]);

    wire [53:0]  madd  = {1'b0, mtop} + {53'd0, inc};
    wire         mcar  = madd[53];
    wire [52:0]  mant  = mcar ? 53'h8000000000000 : madd[52:0];

    wire [11:0] esum = {1'b0, ea} + {1'b0, eb};
    wire [11:0] efield0 = carry ? (esum - 12'd1022) : (esum - 12'd1023);
    wire [11:0] efield  = efield0 + (mcar ? 12'd1 : 12'd0);

    wire ovf = (efield >= 12'h7FF) && !(za || zb);
    wire ufl = (efield == 12'h0)   && !(za || zb);

    wire [63:0] res =
        (za || zb)         ? {sa ^ sb, 11'h0, 52'd0}
      : (ovf)              ? {sa ^ sb, 11'h7FF, 52'd0}
      : (ufl)              ? {sa ^ sb, 11'h0, 52'd0}
      :                       {sa ^ sb, efield[10:0], mant[51:0]};

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