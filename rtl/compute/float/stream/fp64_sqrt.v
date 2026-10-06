`timescale 1ns / 1ps
// 检测链 FP64 平方根：保持原正规正数 RNE 结果，负数及零返回 +0。
// 非正规数、Inf、NaN 保留旧核位运算行为，不承诺完整 IEEE 特殊值语义。
// 每拍处理两位，55 拍求根 + 1 拍舍入；无乘法器、除法器。
// 一次一个请求，背压时保持结果；复位取消运算及待取结果。
// 输入握手后第 56 个上升沿输出 valid，消费后才能接受下一项。
module fp64_sqrt #(parameter USE_CE=0) (
    // Global synchronous stall for variable-latency backing memory.
    input wire ce,

    input wire clk, input wire rst_n,
    input wire in_valid, output wire in_ready,
    input wire [63:0] in_x,
    output wire out_valid, input wire out_ready,
    output wire [63:0] out_r
);
    localparam IDLE=2'd0, ROOT=2'd1, ROUND=2'd2, HOLD=2'd3;
    reg [1:0] state;
    reg [109:0] radicand;
    reg [54:0] root;
    reg [57:0] remainder;
    reg [5:0] count;
    reg [10:0] half_exp;
    reg force_zero;
    reg [63:0] result;
    assign in_ready = rst_n && state==IDLE;
    assign out_valid = rst_n && state==HOLD;
    assign out_r = result;
    wire signed [12:0] exponent = $signed({2'b0,in_x[62:52]})-13'sd1023;
    wire signed [12:0] exponent_half = exponent >>> 1;
    // 前缀 P=root^2+remainder。新前缀 4P+pair，试探根 2root+1。
    // 两个候选平方之差是 4root+1，只需移位、比较和减法。
    wire [57:0] shifted_rem = {remainder[55:0],radicand[109:108]};
    wire [57:0] trial = {1'b0,root,2'b01};
    wire take_bit = shifted_rem >= trial;
    wire [57:0] next_rem = take_bit ? shifted_rem-trial : shifted_rem;
    wire [54:0] next_root = {root[53:0],take_bit};
    // 精确余数直接给出 sticky，省去 root*root。
    wire increment = root[1] && (root[0] || (remainder!=0) || root[2]);
    wire [52:0] rounded = {1'b0,root[53:2]} + {52'd0,increment};
    wire [10:0] result_exp = half_exp + 11'd1023 + {10'd0,rounded[52]};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE; radicand<=0; root<=0; remainder<=0;
            count<=0; half_exp<=0; force_zero<=0; result<=0;
        end else if(!USE_CE || ce) begin case(state)
            IDLE: if(in_valid) begin
                radicand <= exponent[0] ? {1'b1,in_x[51:0],57'd0}
                                        : {1'b0,1'b1,in_x[51:0],56'd0};
                half_exp<=exponent_half[10:0];
                force_zero<=in_x[63] || (in_x[62:0]==0);
                root<=0; remainder<=0; count<=0; state<=ROOT;
            end
            ROOT: begin
                root<=next_root; remainder<=next_rem;
                radicand<={radicand[107:0],2'b0};
                if(count==54) state<=ROUND;
                else count<=count+1'b1;
            end
            ROUND: begin
                result<=force_zero ? 64'd0 : {1'b0,result_exp,rounded[51:0]};
                state<=HOLD;
            end
            HOLD: if(out_ready) state<=IDLE;
            default: state<=IDLE;
        endcase
    end // synchronous clock enable
    end
endmodule
