// Public arithmetic interface. FP64 complex functions execute ROM microcode;
// the FP32-only detector profile retains its existing narrow implementation.
module fp_operator #(parameter FP_W=64, parameter ENABLE_EXP=1,ENABLE_LOG=1,
 parameter ENABLE_SINCOS=1,ENABLE_ATAN_ACOS=1)(
 input wire clk,rst_n,req_valid,output wire req_ready,
 input wire [4:0] req_op,input wire [FP_W-1:0] req_a,req_b,
 output wire rsp_valid,input wire rsp_ready,output wire [FP_W-1:0] rsp_result,
 output wire [4:0] rsp_flags,output wire rsp_less,rsp_equal,rsp_unordered
);
 generate if(FP_W==64)begin : g_program
  fp_math_program #(.ENABLE_EXP(ENABLE_EXP),.ENABLE_LOG(ENABLE_LOG),
   .ENABLE_SINCOS(ENABLE_SINCOS),.ENABLE_ATAN_ACOS(ENABLE_ATAN_ACOS)) core(
   .clk(clk),.rst_n(rst_n),.req_valid(req_valid),.req_ready(req_ready),
   .req_op(req_op),.req_a(req_a),.req_b(req_b),.rsp_valid(rsp_valid),
   .rsp_ready(rsp_ready),.rsp_result(rsp_result),.rsp_flags(rsp_flags),
   .rsp_less(rsp_less),.rsp_equal(rsp_equal),.rsp_unordered(rsp_unordered));
 end else begin : g_narrow
  fp_operator_narrow #(.ENABLE_EXP(ENABLE_EXP),.ENABLE_LOG(ENABLE_LOG),
   .ENABLE_SINCOS(ENABLE_SINCOS),.ENABLE_ATAN_ACOS(ENABLE_ATAN_ACOS)) core(
   .clk(clk),.rst_n(rst_n),.req_valid(req_valid),.req_ready(req_ready),
   .req_op(req_op),.req_a(req_a),.req_b(req_b),.rsp_valid(rsp_valid),
   .rsp_ready(rsp_ready),.rsp_result(rsp_result),.rsp_flags(rsp_flags),
   .rsp_less(rsp_less),.rsp_equal(rsp_equal),.rsp_unordered(rsp_unordered));
 end endgenerate
endmodule
`include "calib_defs.vh"
/*
自主浮点运算器：FP_W仅32或64，所有操作均由可综合整数/位运算实现。
基础运算/转换采用最近偶数舍入，支持非规格化数、带符号零、Inf和NaN。
超越函数用Q128范围缩减、级数和CORDIC近似；不是正确舍入的数学库保证。
sin/cos用1280位2/pi做全有限输入范围缩减；输入幅度不限制为小角度。
flags[0..4]=invalid,divide_by_zero,overflow,underflow,inexact。
超越函数除明确精确特例外置inexact；近似舍入精度及实测见sim说明。
atan2(a,b)=atan2(y,x)。COMPARE仅对sNaN置invalid，任意NaN置unordered。
FP32/64转换命令只允许FP_W=64；FP32位模式放低32位，高位清零。
一个请求在途；结果背压时保持，结果被接收前不接下一请求。
复位取消运算。DIV/SQRT及内部浮点除法/开方均调用fp_divsqrt逐拍执行。
Internal fixed-point divides keep a 256-bit numerator and 144-bit remainder;
small divisors use 64 radix-16 cycles. Internal products use five 32-bit limbs
(30 cycles), sharing a 64x32 multiplier with full-range trig reduction.
All Q128 fraction bits and the 1280-bit 2/pi table are retained.
FP64 basic operations share calib_alu; result packing and CORDIC shifts are
iterative. Angle constants use synchronous ROM. No fixed response latency.
*/
// Compile-time features: default preserves the complete public operator.
// Disabled opcodes return invalid + quiet NaN. Gate BOTH entry and state bodies
// so synthesis does not first build unused wide multipliers/dividers.
module fp_operator_narrow #(parameter ENABLE_EXP=1, ENABLE_LOG=1,
    parameter ENABLE_SINCOS=1, ENABLE_ATAN_ACOS=1) (
    input wire clk,
    input wire rst_n,
    input wire req_valid,
    output wire req_ready,
    input wire [4:0] req_op,
    input wire [31:0] req_a,
    input wire [31:0] req_b,
    output reg rsp_valid,
    input wire rsp_ready,
    output reg [31:0] rsp_result,
    output reg [4:0] rsp_flags,
    output reg rsp_less,
    output reg rsp_equal,
    output reg rsp_unordered
);
    localparam FP_W=32; // This implementation is exclusively binary32.
    `include "fp_bits.vh"
    `include "fp_constants.vh"
    function integer leading_mantissa;
        input [63:0] value;
        reg [63:0] probe;
        integer n;
        begin
            probe=value; n=0;
            if (|probe[63:32]) begin probe=probe>>32; n=n+32; end
            if (|probe[31:16]) begin probe=probe>>16; n=n+16; end
            if (|probe[15:8]) begin probe=probe>>8; n=n+8; end
            if (|probe[7:4]) begin probe=probe>>4; n=n+4; end
            if (|probe[3:2]) begin probe=probe>>2; n=n+2; end
            if (probe[1]) n=n+1;
            leading_mantissa=n;
        end
    endfunction
    localparam F=(FP_W==32)?23:52;
    localparam BIAS=(FP_W==32)?127:1023;
    localparam [63:0] FRAC_MASK=(64'd1<<F)-1;
    localparam [63:0] EXP_MASK=(64'd1<<(FP_W-F-1))-1;
    localparam [63:0] SIGN_MASK=64'd1<<(FP_W-1);
    localparam [63:0] INF_BITS=EXP_MASK<<F;
    localparam [63:0] NAN_BITS=INF_BITS|(64'd1<<(F-1));
    localparam [63:0] ONE_BITS=BIAS<<F;
    localparam IDLE=0, EXP_ITER=1, LOG_INIT=2, LOG_ITER=3,
               TRIG_ITER=4, ATAN_SETUP=5, ATAN_ITER=6,
               ACOS_SUB=7, ACOS_ADD=8, ACOS_MUL=9, ACOS_SQRT=10,
               LOG_SUB=11, LOG_ADD=12, LOG_DIV=13, LOG_SQUARE=14,
               LOG_CUBE=15, LOG_THIRD=16, LOG_SUM=17, LOG_DOUBLE=18,
               SEQ_REQ=19, SEQ_WAIT=20, INT_DIV=21, INT_DONE=22,
               EXP_REDUCE=23, EXP_ACC=24, LOG_RATIO=25, LOG_ACC=26,
               TRIG_MUL=27, TRIG_SHIFT=28, TRIG_INIT=29, TRIG_MUL_END=30,
               FIX_MUL=31, FIX_DONE=32, EXP_RANGE_DONE=33, EXP_TERM_DONE=34,
               LOG_SQUARE_DONE=35, LOG_TERM_DONE=36, LOG_SCALE_DONE=37, TRIG_ANGLE_DONE=38, PACK_FLOAT=39, FIX_CARRY=40, CORDIC_STEP=41, BASIC_REQ=42, BASIC_WAIT=43, PACK_NORMALIZE=44, PACK_ROUND=45, CORDIC_SHIFT=46;
    localparam ATAN_ALIGN=47;
    reg [5:0] state;
    reg [7:0] atan_shift_x,atan_shift_y;
    reg [6:0] iteration;
    reg [4:0] operation;
    reg [63:0] saved_a, saved_b, atan_y, atan_x, temp_float;
    // Q16.128 is sufficient for these bounded working values: EXP input is
    // screened to |x|<4096, reduced EXP/LOG terms stay below 4, and CORDIC
    // normalized coordinates/angles stay below 8. Keep all 128 fraction bits.
    // General products, range reduction and packing retain their full widths.
    reg signed [143:0] fx_x,fx_y,fx_z,term,sum,power2;
    reg signed [191:0] work_q,work_next;
    reg signed [383:0] product;
    // Only bits [shift+129:shift] of the 1344-bit product are consumed.
    // Capture six selected little-endian words as multiplication streams out;
    // discarded words still propagate their carry into subsequent words.
    reg [191:0] phase_product;
    reg [5:0] phase_base;
    integer phase_lane;
    reg [63:0] trig_mantissa, trig_carry;
    reg [5:0] trig_word;
    reg [10:0] trig_shift;
    // Only one 64x32 multiply, one 96-bit carry add and a 40-word constant mux.
    wire [31:0] trig_constant = TWO_OVER_PI >> {trig_word,5'b0};
    // Range proof: LOG's numerator (m-1)*2^256 is <2^256;
    // EXP/series numerators are smaller. Largest denominator m+1 is <3*2^128.
    // Keep all Q128 bits, with a 256-bit quotient and 144-bit remainder.
    reg [255:0] int_quotient;
    reg [143:0] int_denominator, int_remainder;
    reg int_negative;
    reg [8:0] int_count;
    reg [5:0] int_return;
    wire [144:0] int_trial = {int_remainder,int_quotient[255]};
    wire int_take = int_trial >= {1'b0,int_denominator};
    wire [144:0] int_sub = int_trial - {1'b0,int_denominator};
    wire signed [255:0] int_result = int_negative ? -int_quotient : int_quotient;
    reg [319:0] fix_acc;
    reg [159:0] fix_shift;
    reg [159:0] fix_bits;
    reg fix_negative;
    reg [2:0] fix_row,fix_column;
    reg [31:0] fix_carry;
    reg [5:0] fix_return;
    wire signed [319:0] fix_result = fix_negative ? -fix_acc : fix_acc;
    // Fixed-point products reuse the existing trigonometric 64x32 multiplier.
    // Five by five 32-bit limbs + one carry word per row = 30 cycles, exact 320 bits.
    wire [63:0] wide_mul_a=(state==FIX_MUL)?{32'd0,fix_shift[31:0]}:trig_mantissa;
    wire [31:0] wide_mul_b=(state==FIX_MUL)?fix_bits[31:0]:trig_constant;
    wire [95:0] trig_partial=wide_mul_a*wide_mul_b;
    wire [95:0] trig_total=trig_partial+{32'd0,trig_carry};
    // Rotate the ten-word accumulator so the addressed word is always low.
    // This removes its variable read mux and variable part-select write decoder.
    wire [63:0] fix_word_sum=trig_partial[63:0]+{32'd0,fix_acc[31:0]}+{32'd0,fix_carry};
    wire [319:0] fix_with_carry={fix_acc[319:32],fix_carry};
    reg int_small;
    function [11:0] divide_four_bits;
        input [7:0] remainder;
        input [3:0] digit;
        input [6:0] divisor;
        reg [7:0] work;
        reg [3:0] quotient;
        integer bit_number;
        begin
            work=remainder;quotient=0;
            for(bit_number=3;bit_number>=0;bit_number=bit_number-1)begin
                work={work[6:0],digit[bit_number]};
                if(work>={1'b0,divisor})begin work=work-{1'b0,divisor};quotient[bit_number]=1;end
            end
            divide_four_bits={work,quotient};
        end
    endfunction
    wire [11:0] int_four=divide_four_bits(int_remainder[7:0],int_quotient[255:252],int_denominator[6:0]);

    // Rotation and vectoring are mutually exclusive. Share the 144-bit
    // shifters and three add/sub datapaths; register shifts before additions.
    // The angle table is a synchronous ROM, not a wide combinational mux.
    reg [143:0] angle_rom[0:127];
    integer angle_index;
    initial for(angle_index=0;angle_index<128;angle_index=angle_index+1)
        angle_rom[angle_index]=cordic_angle(angle_index);
    reg [6:0] cordic_shift_count;
    reg cordic_vector,cordic_sub;
    reg signed [143:0] cordic_dx,cordic_dy,cordic_da;
    wire signed [143:0] cordic_x_next = fx_x + (cordic_dx ^ {144{cordic_sub}}) + {{143{1'b0}},cordic_sub};
    wire signed [143:0] cordic_y_next = fx_y + (cordic_dy ^ {144{!cordic_sub}}) + {{143{1'b0}},!cordic_sub};
    wire signed [143:0] cordic_z_next = fx_z + (cordic_da ^ {144{cordic_sub}}) + {{143{1'b0}},cordic_sub};
    reg [129:0] phase;
    reg [1:0] quadrant;
    reg sin_negative, atan_negative_x, atan_negative_y;
    integer scale, ex, ey, largest, shift, j;
    reg [63:0] ma,mb,a,b;
    reg [68:0] computed, intermediate;
    reg a_nan,b_nan,a_snan,b_snan,a_inf,b_inf,a_zero,b_zero;
    reg a_sign,b_sign;
    reg seq_sqrt;
    reg [FP_W-1:0] seq_a,seq_b;
    reg [2:0] seq_destination;
    wire seq_req_ready,seq_rsp_valid;
    wire [FP_W-1:0] seq_result;
    wire [4:0] seq_flags;

    // One arithmetic request mux for external instructions and LOG/ACOS.
    // FP64 uses the compact multi-cycle ALU; FP32 keeps its small existing
    // combinational basic path. Never instantiate one ALU per calling state.
    reg [4:0] eval_op;
    reg [63:0] eval_a,eval_b;
    always @* begin
        eval_op=req_op;eval_a=0;eval_b=0;
        eval_a[FP_W-1:0]=req_a;eval_b[FP_W-1:0]=req_b;
        case(state)
            BASIC_REQ, BASIC_WAIT: begin eval_op=operation;eval_a=saved_a;eval_b=saved_b;end
            LOG_SUB: if(ENABLE_LOG) begin eval_op=`PAR_FP_SUB;eval_a=saved_a;eval_b=ONE_BITS;end
            LOG_ADD: if(ENABLE_LOG) begin eval_op=`PAR_FP_ADD;eval_a=saved_a;eval_b=ONE_BITS;end
            LOG_SQUARE: if(ENABLE_LOG) begin eval_op=`PAR_FP_MUL;eval_a=atan_x;eval_b=atan_x;end
            LOG_CUBE: if(ENABLE_LOG) begin eval_op=`PAR_FP_MUL;eval_a=temp_float;eval_b=atan_x;end
            LOG_SUM: if(ENABLE_LOG) begin eval_op=`PAR_FP_ADD;eval_a=atan_x;eval_b=temp_float;end
            LOG_DOUBLE: if(ENABLE_LOG) begin eval_op=`PAR_FP_MUL;eval_a=temp_float;eval_b=((BIAS+1)*64'd1)<<F;end
            ACOS_SUB: if(ENABLE_ATAN_ACOS) begin eval_op=`PAR_FP_SUB;eval_a=ONE_BITS;eval_b=saved_a;end
            ACOS_ADD: if(ENABLE_ATAN_ACOS) begin eval_op=`PAR_FP_ADD;eval_a=ONE_BITS;eval_b=saved_a;end
            ACOS_MUL: if(ENABLE_ATAN_ACOS) begin eval_op=`PAR_FP_MUL;eval_a=temp_float;eval_b=atan_y;end
            default: begin end
        endcase
    end
    wire [68:0] eval_result;
    wire eval_valid;
    wire basic_ready;
    wire internal_basic = (ENABLE_LOG && (state==LOG_SUB || state==LOG_ADD ||
        state==LOG_SQUARE || state==LOG_CUBE || state==LOG_SUM || state==LOG_DOUBLE)) ||
        (ENABLE_ATAN_ACOS && (state==ACOS_SUB || state==ACOS_ADD || state==ACOS_MUL));
    reg signed [191:0] pack_value;
    integer pack_scale;
    reg pack_inexact,pack_tiny;
    reg [191:0] pack_mag;
    reg pack_sign;
    reg signed [15:0] pack_exp;
    wire [F+1:0] pack_rounded={1'b0,pack_mag[F+3:3]} +
        {{(F+1){1'b0}},(pack_mag[2] && (pack_mag[1] || pack_mag[0] || pack_mag[3]))};
    wire [F+1:0] pack_mantissa=pack_rounded[F+1] ? pack_rounded>>1 : pack_rounded;
    wire signed [15:0] pack_final_exp=pack_exp+(pack_rounded[F+1]?16'sd1:16'sd0);
    wire [15:0] pack_biased=pack_final_exp+BIAS;
    wire pack_normal=pack_mantissa[F];
    wire pack_overflow=(pack_mag!=0) && pack_final_exp>BIAS;

    task start_pack;
        input signed [191:0] value;
        input integer exponent;
        input inexact_flag,tiny_flag;
        begin
            pack_value<=value;pack_scale<=exponent;pack_inexact<=inexact_flag;
            pack_tiny<=tiny_flag;state<=PACK_FLOAT;
        end
    endtask
    generate if(FP_W==64) begin: g_compact_basic
        wire valid; wire [63:0] result; wire [4:0] flags;
        calib_alu arithmetic(
            .ce(1'b1),.clk(clk),.rst_n(rst_n),
            .req_valid(rst_n && (state==SEQ_REQ || state==BASIC_REQ || internal_basic)),
            .req_ready(basic_ready),
            .req_op(state==SEQ_REQ ? (seq_sqrt ? `PAR_FP_SQRT : `PAR_FP_DIV) : eval_op),
            .req_a(state==SEQ_REQ ? seq_a : eval_a),
            .req_b(state==SEQ_REQ ? seq_b : eval_b),
            .rsp_valid(valid),.rsp_ready(state==SEQ_WAIT || state==BASIC_WAIT || internal_basic),
            .rsp_result(result),.rsp_flags(flags),.rsp_less(),.rsp_equal(),.rsp_unordered());
        assign eval_result={flags,result};
        assign eval_valid=valid;
        assign seq_req_ready=basic_ready;
        assign seq_rsp_valid=valid;
        assign seq_result=result;
        assign seq_flags=flags;
    end else begin: g_fp32_basic
        assign eval_result=fp_eval(eval_op,eval_a,eval_b,FP_W);
        assign eval_valid=1'b1;
        assign basic_ready=1'b0;
        fp_divsqrt #(.FP_W(FP_W)) divsqrt (
            .ce(1'b1),.clk(clk),.rst_n(rst_n),
            .req_valid(rst_n && state==SEQ_REQ),.req_ready(seq_req_ready),
            .req_sqrt(seq_sqrt),.req_a(seq_a),.req_b(seq_b),
            .rsp_valid(seq_rsp_valid),.rsp_ready(rst_n && state==SEQ_WAIT),
            .rsp_result(seq_result),.rsp_flags(seq_flags));
    end endgenerate


    // destination: 0=外部结果，1=log的t，2=log的三次项/3，3=acos的平方根，4=极小atan2。
    task start_divsqrt;
        input sqrt_operation;
        input [63:0] operand_a,operand_b;
        input [2:0] destination;
        begin
            seq_sqrt<=sqrt_operation;seq_a<=operand_a[FP_W-1:0];seq_b<=operand_b[FP_W-1:0];
            seq_destination<=destination;state<=SEQ_REQ;
        end
    endtask

    // Internal divisors are nonzero. Divide magnitudes then restore sign,
    // preserving Verilog signed / truncation toward zero (including negatives).
    task start_integer_div;
        input signed [383:0] numerator, denominator;
        input [5:0] destination;
        reg [383:0] denominator_magnitude;
        begin
            denominator_magnitude=denominator[383] ? -denominator : denominator;
            int_small<=(denominator_magnitude[383:7]==0 && denominator_magnitude[6:0]!=0);
            int_quotient <= numerator[383] ? -numerator : numerator;
            int_denominator <= denominator[383] ? -denominator : denominator;
            int_remainder <= 0;
            int_negative <= numerator[383] ^ denominator[383];
            int_count <= (denominator_magnitude[383:7]==0 && denominator_magnitude[6:0]!=0)?63:255; int_return <= destination; state <= INT_DIV;
        end
    endtask

    // Exact internal signed product. Every caller fits signed144: reduced
    // Q128 terms/angles <8, EXP input <4096, integer scales <8192.
    // Five limbs retain 160x160 bits; external FP inputs remain unrestricted.
    task start_fixed_mul;
        input signed [191:0] lhs,rhs;
        input [5:0] destination;
        reg [191:0] lhs_magnitude,rhs_magnitude;
        begin
            lhs_magnitude=lhs[191] ? -lhs : lhs;
            rhs_magnitude=rhs[191] ? -rhs : rhs;
            fix_shift<=lhs_magnitude;fix_bits<=rhs_magnitude;
            fix_acc<=0;fix_negative<=lhs[191]^rhs[191];fix_row<=0;fix_column<=0;fix_carry<=0;
            fix_return<=destination;state<=FIX_MUL;
        end
    endtask

    assign req_ready=rst_n && (state==IDLE) && !rsp_valid;

    // 此任务只生成寄存器赋值；不是软件调用，也不含仿真语句。
    task finish;
        input [68:0] value;
        begin
            rsp_result<=value[FP_W-1:0];
            rsp_flags<=value[68:64];
            rsp_valid<=1'b1;
            state<=IDLE;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cordic_shift_count<=0;
            state<=IDLE; rsp_valid<=0; rsp_result<=0; rsp_flags<=0;
            rsp_less<=0; rsp_equal<=0; rsp_unordered<=0;
            iteration<=0; operation<=0; saved_a<=0; saved_b<=0; atan_y<=0; atan_x<=0;
            temp_float<=0; fx_x<=0; fx_y<=0; fx_z<=0; term<=0; sum<=0; power2<=0;
            quadrant<=0; sin_negative<=0; atan_negative_x<=0; atan_negative_y<=0; scale<=0;
            seq_sqrt<=0;seq_a<=0;seq_b<=0;seq_destination<=0;
            phase_product<=0;phase_base<=0;trig_mantissa<=0;trig_carry<=0;trig_word<=0;trig_shift<=0;
            atan_shift_x<=0;atan_shift_y<=0;
            int_quotient<=0;int_denominator<=0;int_remainder<=0;
            int_negative<=0;int_count<=0;int_return<=IDLE;
            fix_acc<=0;fix_shift<=0;fix_bits<=0;fix_negative<=0;fix_row<=0;fix_column<=0;fix_carry<=0;int_small<=0;fix_return<=IDLE;
            pack_mag<=0;pack_sign<=0;pack_exp<=0;
            pack_value<=0;pack_scale<=0;pack_inexact<=0;pack_tiny<=0;
        end else begin
            if (rsp_valid && rsp_ready) rsp_valid<=0;
            case (state)
            IDLE: if (req_valid && req_ready) begin
                a=0; b=0; a[FP_W-1:0]=req_a; b[FP_W-1:0]=req_b;
                a_sign=(a&SIGN_MASK)!=0; b_sign=(b&SIGN_MASK)!=0;
                a_nan=((a&INF_BITS)==INF_BITS)&&((a&FRAC_MASK)!=0);
                b_nan=((b&INF_BITS)==INF_BITS)&&((b&FRAC_MASK)!=0);
                a_snan=a_nan && !(a&(64'd1<<(F-1)));
                b_snan=b_nan && !(b&(64'd1<<(F-1)));
                a_inf=((a&~SIGN_MASK)==INF_BITS); b_inf=((b&~SIGN_MASK)==INF_BITS);
                a_zero=((a&~SIGN_MASK)==0); b_zero=((b&~SIGN_MASK)==0);
                rsp_less<=0; rsp_equal<=0; rsp_unordered<=0;
                operation<=req_op; saved_a<=a; saved_b<=b;
                if ((!ENABLE_EXP && req_op==`PAR_FP_EXP) ||
                    (!ENABLE_LOG && req_op==`PAR_FP_LOG) ||
                    (!ENABLE_SINCOS && (req_op==`PAR_FP_SIN || req_op==`PAR_FP_COS)) ||
                    (!ENABLE_ATAN_ACOS && (req_op==`PAR_FP_ATAN2 || req_op==`PAR_FP_ACOS)))
                    finish({5'b00001,NAN_BITS});
                else if (req_op==`PAR_FP_DIV || req_op==`PAR_FP_SQRT)
                    start_divsqrt(req_op==`PAR_FP_SQRT,a,b,0);
                else if (req_op<=`PAR_FP_MUL || req_op>=`PAR_FP_F32_TO_F64) begin
                    computed=eval_result;
                    if (req_op==`PAR_FP_COMPARE) begin
                        rsp_unordered<=a_nan||b_nan;
                        rsp_equal<=!(a_nan||b_nan) && ((a==b)||(a_zero&&b_zero));
                        rsp_less<=!(a_nan||b_nan) && !(a_zero&&b_zero) &&
                                  ((a_sign!=b_sign)?a_sign:(a_sign?(a>b):(a<b)));
                    end
                    if(FP_W==64) state<=BASIC_REQ; else finish(computed);
                end else if (a_nan || ((req_op==`PAR_FP_ATAN2)&&b_nan)) begin
                    finish({4'd0,(a_snan||((req_op==`PAR_FP_ATAN2)&&b_snan)),NAN_BITS});
                end else case (req_op)
                `PAR_FP_EXP: if (ENABLE_EXP) begin
                    if (a_inf) finish({5'd0,(a_sign?64'd0:INF_BITS)});
                    else if (a_zero) finish({5'd0,ONE_BITS});
                    else begin
                        ex=((a>>F)&EXP_MASK)-BIAS;
                        if (ex>11) begin
                            if (a_sign) finish({5'b11000,64'd0});
                            else finish({5'b10100,INF_BITS});
                        end else begin
                            work_q=fp_fixed(a,FP_W);
                            // 整数k取最近值，余项r约束到[-ln2/2,ln2/2]。
                            fx_z<=work_q;
                            if (work_q>=0) work_next=work_q+(FX_LN2>>>1);
                            else work_next=work_q-(FX_LN2>>>1);
                            start_integer_div(work_next,FX_LN2,EXP_REDUCE);
                        end
                    end
                end
                `PAR_FP_LOG: if (ENABLE_LOG) begin
                    if (a_zero) finish({5'b00010,(SIGN_MASK|INF_BITS)});
                    else if (a_sign) finish({5'b00001,NAN_BITS});
                    else if (a_inf) finish({5'd0,INF_BITS});
                    else if (a==ONE_BITS) finish(69'd0);
                    // 靠近1时log非常小，改用浮点atanh短级数避免固定小数绝对误差。
                    else if (((a>ONE_BITS)?(a-ONE_BITS):(ONE_BITS-a)) < (64'd1<<(F-20)))
                        state<=LOG_SUB;
                    else begin
                        ma=a&FRAC_MASK; ex=(a>>F)&EXP_MASK;
                        if (ex!=0) begin ma[F]=1; ex=ex-BIAS; end
                        else ex=1-BIAS;
                        shift=(ma==0)?52:(F-leading_mantissa(ma));
                        ma=ma<<shift; ex=ex-shift;
                        work_q=$signed({64'd0,ma})<<(128-F);
                        product=(work_q-FX_ONE); product=product<<<128;
                        scale<=ex;
                        work_next=work_q+FX_ONE;
                        start_integer_div(product,work_next,LOG_RATIO);
                    end
                end
                `PAR_FP_SIN, `PAR_FP_COS: if (ENABLE_SINCOS) begin
                    if (a_inf) finish({5'b00001,NAN_BITS});
                    else if (a_zero) finish({5'd0,((req_op==`PAR_FP_SIN)?a:ONE_BITS)});
                    else begin
                        ex=((a>>F)&EXP_MASK)-BIAS;
                        if (ex < -30) begin
                            computed={5'b10000,((req_op==`PAR_FP_SIN)?a:ONE_BITS)};
                            if ((req_op==`PAR_FP_SIN)&&((a&INF_BITS)==0)) computed[67]=1;
                            finish(computed);
                        end else begin
                            ma=(a&FRAC_MASK)|(64'd1<<F);
                            trig_mantissa<=ma; trig_carry<=0; trig_word<=0;
                            phase_product<=0;
                            shift=1280-(ex-F)-128;
                            phase_base<=shift[10:5];
                            trig_shift<={6'd0,shift[4:0]};
                            sin_negative<=a_sign; state<=TRIG_MUL;
                        end
                    end
                end
                `PAR_FP_ATAN2: if (ENABLE_ATAN_ACOS) begin atan_y<=a; atan_x<=b; state<=ATAN_SETUP; end
                `PAR_FP_ACOS: if (ENABLE_ATAN_ACOS) begin
                    if (a_inf || ((a&~SIGN_MASK)>ONE_BITS)) finish({5'b00001,NAN_BITS});
                    else if (a==ONE_BITS) finish(69'd0);
                    else if (a==(ONE_BITS|SIGN_MASK)) begin
                        start_pack(FX_PI,0,1,0);
                    end else begin
                        atan_x<=a; state<=ACOS_SUB;
                    end
                end
                default: finish({5'b00001,NAN_BITS});
                endcase
            end
            EXP_ITER: if (ENABLE_EXP) begin
                start_fixed_mul(term,fx_z,EXP_TERM_DONE);
            end
            EXP_TERM_DONE: if (ENABLE_EXP) begin
                work_q=fix_result>>>128;
                start_integer_div(work_q,$signed({377'd0,iteration}),EXP_ACC);
            end
            EXP_REDUCE: if (ENABLE_EXP) begin
                ex=int_result[31:0];
                scale<=ex;
                start_fixed_mul(ex,FX_LN2,EXP_RANGE_DONE);
            end
            EXP_RANGE_DONE: if (ENABLE_EXP) begin
                fx_z<=fx_z-$signed(fix_result[191:0]);
                term<=FX_ONE; sum<=FX_ONE; iteration<=1; state<=EXP_ITER;
            end
            EXP_ACC: if (ENABLE_EXP) begin
                work_next=int_result[191:0];
                term<=work_next; sum<=sum+work_next;
                if (iteration==32) begin
                    start_pack(sum+work_next,scale,1,1);
                end else begin iteration<=iteration+1'b1; state<=EXP_ITER; end
            end
            LOG_SUB: if (ENABLE_LOG && eval_valid) begin
                computed=eval_result;
                temp_float<=computed[63:0]; state<=LOG_ADD;
            end
            LOG_ADD: if (ENABLE_LOG && eval_valid) begin
                computed=eval_result;
                atan_y<=computed[63:0]; state<=LOG_DIV;
            end
            LOG_DIV: if (ENABLE_LOG) begin
                start_divsqrt(0,temp_float,atan_y,1);
            end
            LOG_SQUARE: if (ENABLE_LOG && eval_valid) begin
                computed=eval_result;
                temp_float<=computed[63:0]; state<=LOG_CUBE;
            end
            LOG_CUBE: if (ENABLE_LOG && eval_valid) begin
                computed=eval_result;
                temp_float<=computed[63:0]; state<=LOG_THIRD;
            end
            LOG_THIRD: if (ENABLE_LOG) begin
                start_divsqrt(0,temp_float,(((BIAS+1)*64'd1)<<F)|(64'd1<<(F-1)),2);
            end
            LOG_SUM: if (ENABLE_LOG && eval_valid) begin
                computed=eval_result;
                temp_float<=computed[63:0]; state<=LOG_DOUBLE;
            end
            LOG_DOUBLE: if (ENABLE_LOG && eval_valid) begin
                computed=eval_result;
                computed[68]=1; finish(computed);
            end
            LOG_INIT: if (ENABLE_LOG) begin
                start_fixed_mul(fx_z,fx_z,LOG_SQUARE_DONE);
            end
            LOG_SQUARE_DONE: if (ENABLE_LOG) begin
                power2<=fix_result>>>128;
                term<=fx_z; sum<=fx_z; iteration<=3; state<=LOG_ITER;
            end
            LOG_ITER: if (ENABLE_LOG) begin
                start_fixed_mul(term,power2,LOG_TERM_DONE);
            end
            LOG_TERM_DONE: if (ENABLE_LOG) begin
                work_q=fix_result>>>128;
                term<=work_q;
                start_integer_div(work_q,$signed({377'd0,iteration}),LOG_ACC);
            end
            LOG_RATIO: if (ENABLE_LOG) begin
                fx_z<=int_result[191:0]; state<=LOG_INIT;
            end
            LOG_ACC: if (ENABLE_LOG) begin
                work_next=sum+$signed(int_result[191:0]);
                sum<=work_next;
                if (iteration==79) begin
                    start_fixed_mul(scale,FX_LN2,LOG_SCALE_DONE);
                end else begin iteration<=iteration+2; state<=LOG_ITER; end
            end
            LOG_SCALE_DONE: if (ENABLE_LOG) begin
                work_next=(sum<<<1)+$signed(fix_result[191:0]);
                start_pack(work_next,0,1,0);
            end
            TRIG_MUL: if (ENABLE_SINCOS) begin
                for(phase_lane=0;phase_lane<6;phase_lane=phase_lane+1)
                    if(trig_word==phase_base+phase_lane)
                        phase_product[32*phase_lane+:32]<=trig_total[31:0];
                trig_carry<=trig_total[95:32];
                trig_word<=trig_word+1'b1;
                if(trig_word==39) state<=TRIG_MUL_END;
            end
            TRIG_MUL_END: if (ENABLE_SINCOS) begin
                for(phase_lane=0;phase_lane<6;phase_lane=phase_lane+1)
                    if(trig_word==phase_base+phase_lane)
                        phase_product[32*phase_lane+:32]<=trig_carry[31:0];
                trig_carry<=trig_carry>>32;
                trig_word<=trig_word+1'b1;
                if(trig_word==41) state<=TRIG_SHIFT;
            end
            TRIG_SHIFT: if (ENABLE_SINCOS) begin
                // The word offset was handled during capture; only 0..31 remain.
                if(trig_shift>=8) begin phase_product<=phase_product>>8;trig_shift<=trig_shift-8;end
                else if(trig_shift!=0) begin phase_product<=phase_product>>1;trig_shift<=trig_shift-1'b1;end
                else state<=TRIG_INIT;
            end
            TRIG_INIT: if (ENABLE_SINCOS) begin
                phase=phase_product[129:0];
                work_q=$signed({32'd0,phase[127:0]});
                if(phase[127]) begin work_q=work_q-FX_ONE;quadrant<=phase[129:128]+2'd1;end
                else quadrant<=phase[129:128];
                start_fixed_mul(work_q,FX_HALF_PI,TRIG_ANGLE_DONE);
            end
            TRIG_ANGLE_DONE: if (ENABLE_SINCOS) begin
                fx_z<=fix_result>>>128;
                fx_x<=FX_GAIN;fx_y<=0;iteration<=0;state<=TRIG_ITER;
            end
            TRIG_ITER: if (ENABLE_SINCOS) begin
                cordic_vector<=0;cordic_sub<=fx_z>=0;
                cordic_dx<=fx_y;cordic_dy<=fx_x;cordic_shift_count<=iteration;
                cordic_da<=angle_rom[iteration];state<=CORDIC_SHIFT;
            end
            ACOS_SUB: if (ENABLE_ATAN_ACOS && eval_valid) begin
                computed=eval_result;
                temp_float<=computed[63:0]; state<=ACOS_ADD;
            end
            ACOS_ADD: if (ENABLE_ATAN_ACOS && eval_valid) begin
                computed=eval_result;
                atan_y<=computed[63:0]; state<=ACOS_MUL;
            end
            ACOS_MUL: if (ENABLE_ATAN_ACOS && eval_valid) begin
                computed=eval_result;
                temp_float<=computed[63:0]; state<=ACOS_SQRT;
            end
            ACOS_SQRT: if (ENABLE_ATAN_ACOS) begin
                start_divsqrt(1,temp_float,64'd0,3);
            end
            ATAN_SETUP: if (ENABLE_ATAN_ACOS) begin
                a=atan_y; b=atan_x;
                a_sign=(a&SIGN_MASK)!=0; b_sign=(b&SIGN_MASK)!=0;
                a_zero=((a&~SIGN_MASK)==0); b_zero=((b&~SIGN_MASK)==0);
                a_inf=((a&~SIGN_MASK)==INF_BITS); b_inf=((b&~SIGN_MASK)==INF_BITS);
                atan_negative_y<=a_sign; atan_negative_x<=b_sign;
                if (a_zero || b_zero || a_inf || b_inf) begin
                    work_q=0;
                    if (a_inf&&b_inf) work_q=b_sign?(FX_PI-(FX_PI>>>2)):(FX_PI>>>2);
                    else if (a_inf || (b_zero&&!a_zero)) work_q=FX_HALF_PI;
                    else if (b_sign) work_q=FX_PI;
                    if (a_sign) work_q=-work_q;
                    if(work_q!=0) start_pack(work_q,0,1,0);
                    else finish({5'd0,(a_sign?SIGN_MASK:64'd0)});
                end else begin
                    ma=a&FRAC_MASK; mb=b&FRAC_MASK;
                    ey=(a>>F)&EXP_MASK; ex=(b>>F)&EXP_MASK;
                    if (ey!=0) begin ma[F]=1; ey=ey-BIAS; end else ey=1-BIAS;
                    if (ex!=0) begin mb[F]=1; ex=ex-BIAS; end else ex=1-BIAS;
                    shift=(ma==0)?52:(F-leading_mantissa(ma));
                    ma=ma<<shift; ey=ey-shift;
                    shift=(mb==0)?52:(F-leading_mantissa(mb));
                    mb=mb<<shift; ex=ex-shift;
                    if (!b_sign && (ey-ex < -32)) begin
                        start_divsqrt(0,a,b,4);
                    end else begin
                        largest=(ex>ey)?ex:ey;
                        work_q=$signed({64'd0,mb})<<(128-F);
                        work_next=$signed({64'd0,ma})<<(128-F);
                        fx_x<=work_q;fx_y<=work_next;
                        atan_shift_x<=((largest-ex)>=144)?8'd144:(largest-ex);
                        atan_shift_y<=((largest-ey)>=144)?8'd144:(largest-ey);
                        fx_z<=0; iteration<=0; state<=ATAN_ALIGN;
                    end
                end
            end
            ATAN_ALIGN: if (ENABLE_ATAN_ACOS) begin
                // Both normalized inputs are positive Q128, below 2. A shift
                // >=144 is exactly zero. Bounded fixed shifts replace barrels.
                if(atan_shift_x>=8)begin fx_x<=fx_x>>>8;atan_shift_x<=atan_shift_x-8;end
                else if(atan_shift_x!=0)begin fx_x<=fx_x>>>1;atan_shift_x<=atan_shift_x-1'b1;end
                if(atan_shift_y>=8)begin fx_y<=fx_y>>>8;atan_shift_y<=atan_shift_y-8;end
                else if(atan_shift_y!=0)begin fx_y<=fx_y>>>1;atan_shift_y<=atan_shift_y-1'b1;end
                if(atan_shift_x==0 && atan_shift_y==0)state<=ATAN_ITER;
            end
            ATAN_ITER: if (ENABLE_ATAN_ACOS) begin
                cordic_vector<=1;cordic_sub<=!(fx_y>0);
                cordic_dx<=fx_y;cordic_dy<=fx_x;cordic_shift_count<=iteration;
                cordic_da<=angle_rom[iteration];state<=CORDIC_SHIFT;
            end
            CORDIC_SHIFT: begin
                // Constant arithmetic shifts retain every Q128 bit. At most
                // 15 eight-bit shifts + 7 one-bit shifts; no wide barrel.
                if(cordic_shift_count>=8)begin
                    cordic_dx<=cordic_dx>>>8;cordic_dy<=cordic_dy>>>8;
                    cordic_shift_count<=cordic_shift_count-8;
                end else if(cordic_shift_count!=0)begin
                    cordic_dx<=cordic_dx>>>1;cordic_dy<=cordic_dy>>>1;
                    cordic_shift_count<=cordic_shift_count-1'b1;
                end else state<=CORDIC_STEP;
            end
            CORDIC_STEP: if (ENABLE_SINCOS || ENABLE_ATAN_ACOS) begin
                fx_x<=cordic_x_next;fx_y<=cordic_y_next;fx_z<=cordic_z_next;
                if(iteration==127)begin
                    if(cordic_vector)begin
                        work_q=cordic_z_next;
                        if(atan_negative_x)work_q=FX_PI-work_q;
                        if(atan_negative_y)work_q=-work_q;
                    end else begin
                        if(operation==`PAR_FP_SIN)begin
                            case(quadrant)
                            0:work_q=cordic_y_next;
                            1:work_q=cordic_x_next;
                            2:work_q=-cordic_y_next;
                            3:work_q=-cordic_x_next;
                            endcase
                            if(sin_negative)work_q=-work_q;
                        end else case(quadrant)
                            0:work_q=cordic_x_next;
                            1:work_q=-cordic_y_next;
                            2:work_q=-cordic_x_next;
                            3:work_q=cordic_y_next;
                        endcase
                    end
                    start_pack(work_q,0,1,0);
                end else begin iteration<=iteration+1'b1;state<=cordic_vector?ATAN_ITER:TRIG_ITER;end
            end
            // Serial exact Q128 -> IEEE packing. No 192/256-bit variable
            // barrel shifter or combinational leading-bit encoder is needed.
            PACK_FLOAT: begin
                pack_mag<=pack_value[191] ? -pack_value : pack_value;
                pack_sign<=pack_value[191];
                pack_exp<=pack_scale-128+F+3;
                state<=PACK_NORMALIZE;
            end
            PACK_NORMALIZE: begin
                if(pack_mag==0)state<=PACK_ROUND;
                else if(pack_exp<1-BIAS-192)begin
                    pack_mag<=1;pack_exp<=1-BIAS;state<=PACK_ROUND;
                end else if(pack_exp<1-BIAS || (|pack_mag[191:F+4]))begin
                    pack_mag<={1'b0,pack_mag[191:2],pack_mag[1]|pack_mag[0]};
                    pack_exp<=pack_exp+1'b1;
                end else if(!pack_mag[F+3] && pack_exp>1-BIAS)begin
                    pack_mag<=pack_mag<<1;pack_exp<=pack_exp-1'b1;
                end else state<=PACK_ROUND;
            end
            PACK_ROUND: begin
                computed=0;
                if(pack_overflow) computed={5'b10100, (pack_sign?SIGN_MASK:64'd0)|INF_BITS};
                else begin
                    computed[63:0]=(pack_sign?SIGN_MASK:64'd0) |
                        (pack_normal ? ({{48{1'b0}},pack_biased}<<F) : 64'd0) |
                        (pack_mantissa & FRAC_MASK);
                    computed[68]=|pack_mag[2:0];
                    computed[67]=(|pack_mag[2:0]) && !pack_normal;
                end
                if(pack_inexact)computed[68]=1;
                if(pack_tiny && ((computed[63:0]&INF_BITS)==0))computed[67]=1;
                finish(computed);
            end
            BASIC_REQ: if(basic_ready)state<=BASIC_WAIT;
            BASIC_WAIT: if(eval_valid)finish(eval_result);
            SEQ_REQ: if(seq_req_ready)state<=SEQ_WAIT;
            INT_DIV: if (ENABLE_EXP || ENABLE_LOG) begin
                if(int_small)begin
                    int_remainder<={136'd0,int_four[11:4]};
                    int_quotient<={int_quotient[251:0],int_four[3:0]};
                end else begin
                    int_remainder<=int_take ? int_sub[143:0] : int_trial[143:0];
                    int_quotient<={int_quotient[254:0],int_take};
                end
                if(int_count==0)state<=INT_DONE;
                else int_count<=int_count-1'b1;
            end
            INT_DONE: if (ENABLE_EXP || ENABLE_LOG) state<=int_return;
            FIX_MUL: if (ENABLE_EXP || ENABLE_LOG || ENABLE_SINCOS) begin
                fix_acc<={fix_word_sum[31:0],fix_acc[319:32]};
                fix_carry<=fix_word_sum[63:32];
                fix_shift<={fix_shift[31:0],fix_shift[159:32]};
                if(fix_column==4)state<=FIX_CARRY;
                else fix_column<=fix_column+1'b1;
            end
            FIX_CARRY: if (ENABLE_EXP || ENABLE_LOG || ENABLE_SINCOS) begin
                // Five products advanced the word pointer by five. Store the
                // carry and rotate another six, leaving next row at +1 mod 10.
                fix_acc<={fix_with_carry[191:0],fix_with_carry[319:192]};
                fix_carry<=0;fix_column<=0;fix_bits<=fix_bits>>32;
                if(fix_row==4)state<=FIX_DONE;
                else begin fix_row<=fix_row+1'b1;state<=FIX_MUL;end
            end
            FIX_DONE: if (ENABLE_EXP || ENABLE_LOG || ENABLE_SINCOS) begin
                // Five rows leave pointer five; restore natural word order
                // before the caller consumes the signed product next cycle.
                fix_acc<={fix_acc[159:0],fix_acc[319:160]};state<=fix_return;
            end
            SEQ_WAIT: if(seq_rsp_valid)begin
                computed=0;computed[FP_W-1:0]=seq_result;computed[68:64]=seq_flags;
                case(seq_destination)
                0:finish(computed);
                1:begin atan_x<=computed[63:0];state<=LOG_SQUARE;end
                2:begin temp_float<=computed[63:0];state<=LOG_SUM;end
                3:begin atan_y<=computed[63:0];state<=ATAN_SETUP;end
                4:begin
                    computed[68]=1;
                    if((computed[63:0]&INF_BITS)==0)computed[67]=1;
                    finish(computed);
                end
                default:finish({5'b00001,NAN_BITS});
                endcase
            end
            default: state<=IDLE;
            endcase
        end
    end
endmodule
