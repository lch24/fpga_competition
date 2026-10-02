`timescale 1ns / 1ps
//==============================================================================
// fp32_div.v — IEEE754 fp32 除法（RNE，位级 = C++ float 除法）
//------------------------------------------------------------------------------
// 算法：恢复余数长除（radix-2，25 步，得 25 位商 = 24 尾数 + guard），
//   round 位 = 第 26 位商，sticky = 余数非零；按 RNE 舍入。
//   数值：|a/b| ≤ ~2^24（本域：坐标均值 分子<2^32 分母≤4096），无下溢。
//   尾数规格化：A={1,a_m}，B={1,b_m}（24 位，∈[2^23,2^24)），A<2B 保证商<2。
//
// RNE：guard 为第 25 位商（bit24），round 为 bit23；inc = round && (guard|sticky)
//   —— 或等价：商再算 1 位做 round，sticky=余数。
//
// 端口：弹性流（输入寄存 + 组合计算 + 输出 sync_fifo 水位冻结），
//   延迟 ~2 拍；in_ready 由 FIFO 水位驱动。
//==============================================================================
module fp32_div (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [31:0] in_a,
    input  wire [31:0] in_b,
    output reg         out_valid,
    input  wire        out_ready,
    output reg  [31:0] out_r
);

    localparam AW = 5, THRESH = 31;

    //---- 输入级：寄存 + FIFO 水位冻结 ----
    wire [AW:0] fcnt;
    assign in_ready = (fcnt < THRESH);
    wire acc = in_valid && in_ready;

    reg [31:0] a_r, b_r;
    always @(posedge clk) begin
        if (!rst_n) begin a_r <= 0; b_r <= 0; end
        else if (acc) begin a_r <= in_a; b_r <= in_b; end
    end

    //---- 组合：恢复余数除法 ----
    wire sa = a_r[31], sb = b_r[31];
    wire [7:0] ea = a_r[30:23], eb = b_r[30:23];
    wire [22:0] ma = a_r[22:0], mb = b_r[22:0];
    wire az = (ea == 8'd0) && (ma == 23'd0);   // a==0
    wire bz = (eb == 8'd0) && (mb == 23'd0);   // b==0

    // 尾数域（隐含 1；非规格化按 0 处理，本域不会出现）
    wire [23:0] A = {1'b1, ma};
    wire [23:0] B = {1'b1, mb};
    wire ageb = (A >= B);              // 商∈[1,2) 还是 [0.5,1)
    wire [23:0] N = ageb ? (A - B) : A;   // 归一化被除数，恒 N < B

    // 恢复余数除法：28 位商 q = floor(N·2^28/B)，rem 初值 = N（不预左移）
    function automatic [27:0] divq;
        input [23:0] a_, b_;
        reg [27:0] q;
        reg [55:0] rem, bd;
        integer i;
        begin
            q  = 28'd0;
            rem = {32'd0, a_};
            bd  = {32'd0, b_};
            for (i = 27; i >= 0; i = i - 1) begin
                rem = rem << 1;
                if (rem >= bd) begin
                    rem = rem - bd;
                    q[i] = 1'b1;
                end
            end
            divq = q;
        end
    endfunction

    wire [27:0] q = divq(N, B);
    // 精确余数（含全部 q 低位与余数信息）：N·2^28 - q·B
    wire [55:0] num  = {32'd0, N} << 28;
    wire [55:0] prod = ({28'd0, B}) * ({28'd0, q});
    wire        sticky = (num != prod);

    // 尾数/舍入（ageb 分支）：
    //   ageb=1: 商=1+q·2^-28，mant=q[27:5]，g=q[4]，r=q[3]
    //   ageb=0: 商=q·2^-28 ∈[0.5,1)，mant=q[26:4]，g=q[3]，r=q[2]
    // RNE: inc = g && (r || sticky || mant_lsb)
    wire [22:0] mant_raw = ageb ? q[27:5] : q[26:4];
    wire        g = ageb ? q[4] : q[3];
    wire        r = ageb ? q[3] : q[2];
    wire inc = g && (r || sticky || mant_raw[0]);

    wire [23:0] m2 = {1'b0, mant_raw} + {23'd0, inc};     // 可能进位到 bit23
    wire        qcar = m2[23];
    wire [22:0] qmant = qcar ? 23'd0 : m2[22:0];
    wire [8:0]  qe = ({1'b0, ea} - {1'b0, eb}) + 9'd127
                     - {8'd0, (ageb ? 1'b0 : 1'b1)}
                     + {8'd0, (qcar ? 1'b1 : 1'b0)};

    // 组装（含防御：a==0→0；b==0→inf(0x7F800000)）
    wire [31:0] res = az ? 32'h0000_0000
                   : bz ? (sa ^ sb ? 32'hFF80_0000 : 32'h7F80_0000)
                   : {sa ^ sb, qe[7:0], qmant};

    //---- 输出 FIFO（同 fp64_sqrt 弹性模式）----
    reg psh;
    always @(posedge clk) begin
        if (!rst_n) psh <= 1'b0;
        else        psh <= acc;
    end
    sync_fifo #(.DATA_WIDTH(32), .ADDR_WIDTH(AW)) u_fifo (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (psh),
        .in_ready (),
        .in_data  (res),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_data (out_r),
        .count    (fcnt),
        .empty    (),
        .full     ()
    );

endmodule
