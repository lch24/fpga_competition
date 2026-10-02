`timescale 1ns / 1ps
//==============================================================================
// fp32_mul.v — 单精度 IEEE-754 乘法（RNE：round-to-nearest-even）
//------------------------------------------------------------------------------
// 语义：out_r = RNE( in_a × in_b )，与 C++ float 乘法逐次舍入逐位一致。
//
// 输入假设（本子系统值域）：
//   in_a / in_b 为普通（normal）fp32 精确整数；本域无 NaN/Inf/次正规数输入。
//   仍按 IEEE 兜底处理：±0、指数溢出→±Inf、指数下溢→±0（本域不触发）。
//
// 实现（RNE 五步）：
//   1) 24×24 位尾数相乘得 48 位乘积 P（结果尾数需 24 位）。
//   2) 依 P 的 MSB 位（P[47]）决定归一化及指数（进位/不进位两分支）。
//   3) 截短到 24 位，round 位 = 被丢弃最高位，sticky = |其余丢弃位。
//   4) RNE 增量 = round && (sticky || LSB)。
//   5) 尾数进位列进位时再右移一位、指数 +1。
//
// 弹性流水（window3x3 定型同款）：
//   内部无反压直出：输入拍寄存 + 组合计算；结果经输出同步 sync_fifo 吸收下游
//   背压；FIFO 水位 ≥ THRESH 时冻结输入（in_ready=0），in_valid&&!in_ready 时
//   payload 由内部寄存器保持。
// 延迟：接受拍→FIFO 出拍为 2 拍（1 拍寄存 + 1 拍写）；FIFO 满水位冻结。
//==============================================================================
module fp32_mul (
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

    localparam AW     = 5;                 // FIFO 深度 32
    localparam THRESH = 31;                // 水位≥31 冻结（在途≤1，永不写满）

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
    // 组合计算：RNE 乘法
    //--------------------------------------------------------------------
    wire        sa   = a_r[31];
    wire        sb   = b_r[31];
    wire [7:0]  ea   = a_r[30:23];
    wire [7:0]  eb   = b_r[30:23];
    wire [23:0] ma   = {1'b1, a_r[22:0]};
    wire [23:0] mb   = {1'b1, b_r[22:0]};
    wire za = (ea == 8'h0);
    wire zb = (eb == 8'h0);

    // 48 位乘积：操作数扩展到 48 位以获得满位宽乘积
    wire [47:0] p = {24'b0, ma} * {24'b0, mb};
    wire        carry = p[47];
    wire [23:0] mtop  = carry ? p[47:24] : p[46:23];
    wire        rbit  = carry ? p[23]    : p[22];
    wire        sbit  = carry ? (|p[22:0]) : (|p[21:0]);
    wire        inc   = rbit && (sbit || mtop[0]);

    wire [24:0] madd  = {1'b0, mtop} + {24'd0, inc};
    wire        mcar  = madd[24];
    wire [23:0] mant  = mcar ? 24'h800000 : madd[23:0];

    // 指数（带偏置）。永无进位: carry-> ea+eb-126；否则 ea+eb-127
    wire [8:0]  esum  = {1'b0, ea} + {1'b0, eb};
    wire [8:0]  efield0 = carry ? (esum - 9'd126) : (esum - 9'd127);
    wire [8:0]  efield  = efield0 + (mcar ? 9'd1 : 9'd0);

    wire ovf = (efield >= 9'd255);
    wire ufl = (efield == 9'd0);

    wire [31:0] res =
        (za || zb)         ? {sa ^ sb, 8'd0, 23'd0}
      : (ovf)              ? {sa ^ sb, 8'hFF, 23'd0}               // ±Inf（本域不触发）
      : (ufl)              ? {sa ^ sb, 8'd0, 23'd0}                // 下溢→±0（本域不触发）
      :                       {sa ^ sb, efield[7:0], mant[22:0]};

    //--------------------------------------------------------------------
    // 出拍：push = acc 延迟 1 拍；结果写 F IO
    //--------------------------------------------------------------------
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