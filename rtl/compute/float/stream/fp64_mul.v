`timescale 1ns / 1ps
//==============================================================================
// fp64_mul.v — 检测链有限数值范围内的双精度乘法（RNE）
//------------------------------------------------------------------------------
// 语义：out_r = RNE( in_a × in_b )（fp64）。本子系统输入为 fp32 提升/精确整数
//   （trace、det、常量 4.0），乘积 < 2^53+ 精确，普通 fp64 RNE 乘法即可。
// 沿用检测链原有数值约定：支持带符号零和正常数乘积；不是完整 IEEE-754
// 特殊值/非规格化数运算器。通用浮点运算请使用 rtl/compute/float/fp_operator.v。
//
// 实现（同 fp32_mul，53 位尾数、11 位指数、偏置 1023）：
//   1) 53×53 尾数相乘得 106 位 P；依 P[105]（进位）选两套归一化。
//   2) efield（进位）= ea+eb-1022；否则 = ea+eb-1023。
//   3) 取 53 位尾数，round/sticky 做 RNE，尾数进位再右移+指数+1。
//
// Exact radix-four product: ordinary inputs return after 29 enabled cycles.
// Result is held under backpressure; clock enable freezes every pipeline state.
//==============================================================================
module fp64_mul #(parameter USE_CE=0) (
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

    // Radix-four exact integer product: 27 iterations, no DSP blocks.
    // Preserve the detector's existing rounding and finite-input contract.
    localparam IDLE=0,PREPARE=1,MULTIPLY=2,PACK=3,RESPONSE=4;
    reg [2:0] state;
    reg [63:0] a_r,b_r,result_q;
    reg [105:0] product_q,shift_a,shift_triple;
    reg [52:0] bits_b;
    reg [4:0] iteration;
    wire [105:0] addend=bits_b[1:0]==0 ? 106'd0 :
        bits_b[1:0]==1 ? shift_a : bits_b[1:0]==2 ? (shift_a<<1) : shift_triple;
    assign in_ready=rst_n && state==IDLE;
    assign out_valid=rst_n && state==RESPONSE;
    assign out_r=result_q;

    wire        sa = a_r[63];
    wire        sb = b_r[63];
    wire [10:0] ea = a_r[62:52];
    wire [10:0] eb = b_r[62:52];
    wire [52:0] ma = {1'b1, a_r[51:0]};
    wire [52:0] mb = {1'b1, b_r[51:0]};
    wire za = (ea == 11'h0);
    wire zb = (eb == 11'h0);

    // 106 位乘积
    wire [105:0] p = product_q;
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

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;a_r<=0;b_r<=0;result_q<=0;product_q<=0;
            shift_a<=0;shift_triple<=0;bits_b<=0;iteration<=0;
        end else if(!USE_CE || ce) case(state)
            IDLE: if(in_valid) begin a_r<=in_a;b_r<=in_b;state<=PREPARE;end
            PREPARE: begin
                product_q<=0;shift_a<={53'd0,ma};
                shift_triple<=({53'd0,ma}<<1)+{53'd0,ma};bits_b<=mb;iteration<=0;
                state<=(za||zb)?PACK:MULTIPLY;
            end
            MULTIPLY: begin
                product_q<=product_q+addend;
                shift_a<=shift_a<<2;shift_triple<=shift_triple<<2;bits_b<=bits_b>>2;
                if(iteration==26)state<=PACK;else iteration<=iteration+1'b1;
            end
            PACK: begin result_q<=res;state<=RESPONSE;end
            RESPONSE: if(out_ready)state<=IDLE;
            default:state<=IDLE;
        endcase
    end
endmodule
