`timescale 1ns / 1ps
//==============================================================================
// bilinear_core.v — float 双线性插值核（逐位复刻 kernels::bilinear）
//------------------------------------------------------------------------------
// 语义（与 C++ 权威逐位一致，运算顺序与舍入次数严格一致，无融合/重排）：
//   top    = (1-dx)*p00 + dx*p10      （fp32_sub → 2×fp32_mul → fp32_add）
//   bottom = (1-dx)*p01 + dx*p11      （独立第二路，同上）
//   out    = (1-dy)*top + dy*bottom   （fp32_sub → 2×fp32_mul → fp32_add）
// 每次运算都是独立 fp32 RNE 单元逐次舍入（export_m5.cpp bilinear 逐行复刻）。
// 常量：1.0f = 32'h3F800000（m5_const.txt ONE_F）。
//
// 时序：串行状态机（一次一个样例）；子单元弹性握手（sync_fifo 水位冻结，
//   out_valid 保持直到 out_ready）；输出带背压。
// 吞吐：每例 ~11 拍（sub→mul→add→mul→add 五级依赖 + 输入/输出各 1 拍）。
//==============================================================================
module bilinear_core (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [31:0] in_p00,
    input  wire [31:0] in_p10,
    input  wire [31:0] in_p01,
    input  wire [31:0] in_p11,
    input  wire [31:0] in_dx,
    input  wire [31:0] in_dy,
    output reg         out_valid,
    input  wire        out_ready,
    output reg  [31:0] out_r
);

    localparam ONE = 32'h3F800000;

    localparam S_IDLE = 3'd0, S_SUB = 3'd1, S_MUL1 = 3'd2, S_ADD = 3'd3,
               S_MUL2 = 3'd4, S_OUT = 3'd5;
    reg [2:0] state;

    reg [31:0] p00_r, p10_r, p01_r, p11_r, dx_r, dy_r;

    //--------------------------------------------------------------------
    // 发射/消费信号（声明先于例化端口使用，避免隐式 net）
    //--------------------------------------------------------------------
    wire sub_fire, muls_fire, adds_fire, muls2_fire, addout_fire, ao_out_rdy;

    //--------------------------------------------------------------------
    // 子单元互连
    //--------------------------------------------------------------------
    wire sdx_rdy, sdy_rdy, sdx_v, sdy_v;
    wire [31:0] sdx_r, sdy_r;
    wire m1_rdy, m2_rdy, m3_rdy, m4_rdy, m1_v, m2_v, m3_v, m4_v;
    wire [31:0] m1_r, m2_r, m3_r, m4_r;
    wire at_rdy, ab_rdy, at_v, ab_v;
    wire [31:0] at_r, ab_r;
    wire m5_rdy, m6_rdy, m5_v, m6_v;
    wire [31:0] m5_r, m6_r;
    wire ao_rdy, ao_v;
    wire [31:0] ao_r;

    fp32_sub u_subdx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(sub_fire), .in_ready(sdx_rdy),
        .in_a(ONE), .in_b(in_dx),
        .out_valid(sdx_v), .out_ready(muls_fire), .out_r(sdx_r)
    );
    fp32_sub u_subdy (
        .clk(clk), .rst_n(rst_n),
        .in_valid(sub_fire), .in_ready(sdy_rdy),
        .in_a(ONE), .in_b(in_dy),
        .out_valid(sdy_v), .out_ready(muls2_fire), .out_r(sdy_r)
    );
    fp32_mul u_mul1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(muls_fire), .in_ready(m1_rdy),
        .in_a(sdx_r), .in_b(p00_r),
        .out_valid(m1_v), .out_ready(adds_fire), .out_r(m1_r)
    );
    fp32_mul u_mul2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(muls_fire), .in_ready(m2_rdy),
        .in_a(dx_r), .in_b(p10_r),
        .out_valid(m2_v), .out_ready(adds_fire), .out_r(m2_r)
    );
    fp32_mul u_mul3 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(muls_fire), .in_ready(m3_rdy),
        .in_a(sdx_r), .in_b(p01_r),
        .out_valid(m3_v), .out_ready(adds_fire), .out_r(m3_r)
    );
    fp32_mul u_mul4 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(muls_fire), .in_ready(m4_rdy),
        .in_a(dx_r), .in_b(p11_r),
        .out_valid(m4_v), .out_ready(adds_fire), .out_r(m4_r)
    );
    fp32_add u_addtop (
        .clk(clk), .rst_n(rst_n),
        .in_valid(adds_fire), .in_ready(at_rdy),
        .in_a(m1_r), .in_b(m2_r),
        .out_valid(at_v), .out_ready(muls2_fire), .out_r(at_r)
    );
    fp32_add u_addbot (
        .clk(clk), .rst_n(rst_n),
        .in_valid(adds_fire), .in_ready(ab_rdy),
        .in_a(m3_r), .in_b(m4_r),
        .out_valid(ab_v), .out_ready(muls2_fire), .out_r(ab_r)
    );
    fp32_mul u_mul5 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(muls2_fire), .in_ready(m5_rdy),
        .in_a(sdy_r), .in_b(at_r),
        .out_valid(m5_v), .out_ready(addout_fire), .out_r(m5_r)
    );
    fp32_mul u_mul6 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(muls2_fire), .in_ready(m6_rdy),
        .in_a(dy_r), .in_b(ab_r),
        .out_valid(m6_v), .out_ready(addout_fire), .out_r(m6_r)
    );
    fp32_add u_addout (
        .clk(clk), .rst_n(rst_n),
        .in_valid(addout_fire), .in_ready(ao_rdy),
        .in_a(m5_r), .in_b(m6_r),
        .out_valid(ao_v), .out_ready(ao_out_rdy), .out_r(ao_r)
    );

    //--------------------------------------------------------------------
    // 组合发射 / 消费信号（fire 拍组合使用输出值并消费）
    //--------------------------------------------------------------------
    assign in_ready   = (state == S_IDLE);
    assign sub_fire    = (state == S_IDLE) && in_valid;
    assign muls_fire   = (state == S_SUB) && sdx_v && sdy_v
                         && m1_rdy && m2_rdy && m3_rdy && m4_rdy;
    assign adds_fire   = (state == S_MUL1) && m1_v && m2_v && m3_v && m4_v
                         && at_rdy && ab_rdy;
    assign muls2_fire  = (state == S_ADD) && at_v && ab_v && sdy_v
                         && m5_rdy && m6_rdy;
    assign addout_fire = (state == S_MUL2) && m5_v && m6_v && ao_rdy;
    assign ao_out_rdy  = (state == S_OUT) && ao_v && !out_valid;

    //--------------------------------------------------------------------
    // 状态机
    //--------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            out_valid <= 1'b0;
            out_r     <= 32'd0;
            p00_r <= 32'd0; p10_r <= 32'd0; p01_r <= 32'd0; p11_r <= 32'd0;
            dx_r  <= 32'd0; dy_r  <= 32'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (sub_fire) begin
                        p00_r <= in_p00; p10_r <= in_p10;
                        p01_r <= in_p01; p11_r <= in_p11;
                        dx_r  <= in_dx;  dy_r  <= in_dy;
                        state <= S_SUB;
                    end
                end
                S_SUB:   if (muls_fire)   state <= S_MUL1;
                S_MUL1:  if (adds_fire)   state <= S_ADD;
                S_ADD:   if (muls2_fire)  state <= S_MUL2;
                S_MUL2:  if (addout_fire) state <= S_OUT;
                S_OUT: begin
                    if (ao_v && !out_valid) begin
                        out_valid <= 1'b1;
                        out_r     <= ao_r;
                    end
                    if (out_valid && out_ready) begin
                        out_valid <= 1'b0;
                        state     <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule