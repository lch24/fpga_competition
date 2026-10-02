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
加减乘/转换及超越函数其他路径暂保留现有结构；不可假设固定延迟。
定点级数中的整数除法尚未调整，需根据后续综合时序报告再优化。
*/
module fp_operator #(parameter FP_W=64) (
    input wire clk,
    input wire rst_n,
    input wire req_valid,
    output wire req_ready,
    input wire [4:0] req_op,
    input wire [FP_W-1:0] req_a,
    input wire [FP_W-1:0] req_b,
    output reg rsp_valid,
    input wire rsp_ready,
    output reg [FP_W-1:0] rsp_result,
    output reg [4:0] rsp_flags,
    output reg rsp_less,
    output reg rsp_equal,
    output reg rsp_unordered
);
    `include "fp_bits.vh"
    `include "fp_constants.vh"
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
               SEQ_REQ=19, SEQ_WAIT=20;
    reg [4:0] state;
    reg [6:0] iteration;
    reg [4:0] operation;
    reg [63:0] saved_a, atan_y, atan_x, temp_float;
    reg signed [191:0] fx_x,fx_y,fx_z,term,sum,power2;
    reg signed [191:0] work_q,work_next;
    reg signed [383:0] product;
    reg [1343:0] phase_product;
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

    fp_divsqrt #(.FP_W(FP_W)) divsqrt (
        .clk(clk),.rst_n(rst_n),.req_valid(rst_n && state==SEQ_REQ),.req_ready(seq_req_ready),
        .req_sqrt(seq_sqrt),.req_a(seq_a),.req_b(seq_b),.rsp_valid(seq_rsp_valid),
        .rsp_ready(rst_n && state==SEQ_WAIT),.rsp_result(seq_result),.rsp_flags(seq_flags)
    );

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
            state<=IDLE; rsp_valid<=0; rsp_result<=0; rsp_flags<=0;
            rsp_less<=0; rsp_equal<=0; rsp_unordered<=0;
            iteration<=0; operation<=0; saved_a<=0; atan_y<=0; atan_x<=0;
            temp_float<=0; fx_x<=0; fx_y<=0; fx_z<=0; term<=0; sum<=0; power2<=0;
            quadrant<=0; sin_negative<=0; atan_negative_x<=0; atan_negative_y<=0; scale<=0;
            seq_sqrt<=0;seq_a<=0;seq_b<=0;seq_destination<=0;
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
                operation<=req_op; saved_a<=a;
                if (req_op==`PAR_FP_DIV || req_op==`PAR_FP_SQRT)
                    start_divsqrt(req_op==`PAR_FP_SQRT,a,b,0);
                else if (req_op<=`PAR_FP_MUL || req_op>=`PAR_FP_F32_TO_F64) begin
                    computed=fp_eval(req_op,a,b,FP_W);
                    if (req_op==`PAR_FP_COMPARE) begin
                        rsp_unordered<=a_nan||b_nan;
                        rsp_equal<=!(a_nan||b_nan) && ((a==b)||(a_zero&&b_zero));
                        rsp_less<=!(a_nan||b_nan) && !(a_zero&&b_zero) &&
                                  ((a_sign!=b_sign)?a_sign:(a_sign?(a>b):(a<b)));
                    end
                    finish(computed);
                end else if (a_nan || ((req_op==`PAR_FP_ATAN2)&&b_nan)) begin
                    finish({4'd0,(a_snan||((req_op==`PAR_FP_ATAN2)&&b_snan)),NAN_BITS});
                end else case (req_op)
                `PAR_FP_EXP: begin
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
                            if (work_q>=0) ex=(work_q+(FX_LN2>>>1))/FX_LN2;
                            else ex=(work_q-(FX_LN2>>>1))/FX_LN2;
                            scale<=ex; fx_z<=work_q-ex*FX_LN2;
                            term<=FX_ONE; sum<=FX_ONE; iteration<=1; state<=EXP_ITER;
                        end
                    end
                end
                `PAR_FP_LOG: begin
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
                        for (j=0;j<52;j=j+1) if (!ma[F]) begin ma=ma<<1; ex=ex-1; end
                        work_q=$signed({64'd0,ma})<<(128-F);
                        product=(work_q-FX_ONE); product=product<<<128;
                        work_next=product/(work_q+FX_ONE);
                        fx_z<=work_next; scale<=ex; state<=LOG_INIT;
                    end
                end
                `PAR_FP_SIN, `PAR_FP_COS: begin
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
                            phase_product=ma*TWO_OVER_PI;
                            shift=1280-(ex-F)-128;
                            phase=phase_product>>shift;
                            work_q=$signed({32'd0,phase[127:0]});
                            if (phase[127]) begin
                                work_q=work_q-FX_ONE; quadrant<=phase[129:128]+2'd1;
                            end else quadrant<=phase[129:128];
                            product=work_q*FX_HALF_PI; fx_z<=product>>>128;
                            fx_x<=FX_GAIN; fx_y<=0; iteration<=0;
                            sin_negative<=a_sign; state<=TRIG_ITER;
                        end
                    end
                end
                `PAR_FP_ATAN2: begin atan_y<=a; atan_x<=b; state<=ATAN_SETUP; end
                `PAR_FP_ACOS: begin
                    if (a_inf || ((a&~SIGN_MASK)>ONE_BITS)) finish({5'b00001,NAN_BITS});
                    else if (a==ONE_BITS) finish(69'd0);
                    else if (a==(ONE_BITS|SIGN_MASK)) begin
                        computed=fixed_pack(FX_PI,0,FP_W); computed[68]=1; finish(computed);
                    end else begin
                        atan_x<=a; state<=ACOS_SUB;
                    end
                end
                default: finish({5'b00001,NAN_BITS});
                endcase
            end
            EXP_ITER: begin
                product=term*fx_z; work_q=product>>>128;
                work_next=work_q/$signed({121'd0,iteration});
                term<=work_next; sum<=sum+work_next;
                if (iteration==32) begin
                    computed=fixed_pack(sum+work_next,scale,FP_W); computed[68]=1;
                    if ((computed[63:0]&INF_BITS)==0) computed[67]=1;
                    finish(computed);
                end else iteration<=iteration+1'b1;
            end
            LOG_SUB: begin
                computed=fp_eval(`PAR_FP_SUB,saved_a,ONE_BITS,FP_W);
                temp_float<=computed[63:0]; state<=LOG_ADD;
            end
            LOG_ADD: begin
                computed=fp_eval(`PAR_FP_ADD,saved_a,ONE_BITS,FP_W);
                atan_y<=computed[63:0]; state<=LOG_DIV;
            end
            LOG_DIV: begin
                start_divsqrt(0,temp_float,atan_y,1);
            end
            LOG_SQUARE: begin
                computed=fp_eval(`PAR_FP_MUL,atan_x,atan_x,FP_W);
                temp_float<=computed[63:0]; state<=LOG_CUBE;
            end
            LOG_CUBE: begin
                computed=fp_eval(`PAR_FP_MUL,temp_float,atan_x,FP_W);
                temp_float<=computed[63:0]; state<=LOG_THIRD;
            end
            LOG_THIRD: begin
                start_divsqrt(0,temp_float,(((BIAS+1)*64'd1)<<F)|(64'd1<<(F-1)),2);
            end
            LOG_SUM: begin
                computed=fp_eval(`PAR_FP_ADD,atan_x,temp_float,FP_W);
                temp_float<=computed[63:0]; state<=LOG_DOUBLE;
            end
            LOG_DOUBLE: begin
                computed=fp_eval(`PAR_FP_MUL,temp_float,((BIAS+1)*64'd1)<<F,FP_W);
                computed[68]=1; finish(computed);
            end
            LOG_INIT: begin
                product=fx_z*fx_z; power2<=product>>>128;
                term<=fx_z; sum<=fx_z; iteration<=3; state<=LOG_ITER;
            end
            LOG_ITER: begin
                product=term*power2; work_q=product>>>128;
                work_next=sum+work_q/$signed({121'd0,iteration});
                term<=work_q; sum<=work_next;
                if (iteration==79) begin
                    computed=fixed_pack((work_next<<<1)+scale*FX_LN2,0,FP_W);
                    computed[68]=1; finish(computed);
                end else iteration<=iteration+2;
            end
            TRIG_ITER: begin
                if (fx_z>=0) begin
                    fx_x<=fx_x-(fx_y>>>iteration);
                    fx_y<=fx_y+(fx_x>>>iteration);
                    fx_z<=fx_z-cordic_angle(iteration);
                    work_q=fx_x-(fx_y>>>iteration); work_next=fx_y+(fx_x>>>iteration);
                end else begin
                    fx_x<=fx_x+(fx_y>>>iteration);
                    fx_y<=fx_y-(fx_x>>>iteration);
                    fx_z<=fx_z+cordic_angle(iteration);
                    work_q=fx_x+(fx_y>>>iteration); work_next=fx_y-(fx_x>>>iteration);
                end
                if (iteration==127) begin
                    if (operation==`PAR_FP_SIN) begin
                        case (quadrant)
                        0: work_q=work_next;
                        1: work_q=work_q;
                        2: work_q=-work_next;
                        3: work_q=-work_q;
                        endcase
                        if (sin_negative) work_q=-work_q;
                    end else case (quadrant)
                        0: work_q=work_q;
                        1: work_q=-work_next;
                        2: work_q=-work_q;
                        3: work_q=work_next;
                    endcase
                    computed=fixed_pack(work_q,0,FP_W); computed[68]=1; finish(computed);
                end else iteration<=iteration+1'b1;
            end
            ACOS_SUB: begin
                computed=fp_eval(`PAR_FP_SUB,ONE_BITS,saved_a,FP_W);
                temp_float<=computed[63:0]; state<=ACOS_ADD;
            end
            ACOS_ADD: begin
                computed=fp_eval(`PAR_FP_ADD,ONE_BITS,saved_a,FP_W);
                atan_y<=computed[63:0]; state<=ACOS_MUL;
            end
            ACOS_MUL: begin
                computed=fp_eval(`PAR_FP_MUL,temp_float,atan_y,FP_W);
                temp_float<=computed[63:0]; state<=ACOS_SQRT;
            end
            ACOS_SQRT: begin
                start_divsqrt(1,temp_float,64'd0,3);
            end
            ATAN_SETUP: begin
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
                    computed=fixed_pack(work_q,0,FP_W);
                    if (work_q!=0) computed[68]=1;
                    else computed[63:0]=a_sign?SIGN_MASK:64'd0;
                    finish(computed);
                end else begin
                    ma=a&FRAC_MASK; mb=b&FRAC_MASK;
                    ey=(a>>F)&EXP_MASK; ex=(b>>F)&EXP_MASK;
                    if (ey!=0) begin ma[F]=1; ey=ey-BIAS; end else ey=1-BIAS;
                    if (ex!=0) begin mb[F]=1; ex=ex-BIAS; end else ex=1-BIAS;
                    for (j=0;j<52;j=j+1) begin
                        if (!ma[F]) begin ma=ma<<1; ey=ey-1; end
                        if (!mb[F]) begin mb=mb<<1; ex=ex-1; end
                    end
                    if (!b_sign && (ey-ex < -32)) begin
                        start_divsqrt(0,a,b,4);
                    end else begin
                        largest=(ex>ey)?ex:ey;
                        work_q=$signed({64'd0,mb})<<(128-F);
                        work_next=$signed({64'd0,ma})<<(128-F);
                        fx_x<=work_q>>>(largest-ex); fx_y<=work_next>>>(largest-ey);
                        fx_z<=0; iteration<=0; state<=ATAN_ITER;
                    end
                end
            end
            ATAN_ITER: begin
                if (fx_y>0) begin
                    fx_x<=fx_x+(fx_y>>>iteration); fx_y<=fx_y-(fx_x>>>iteration);
                    work_q=fx_z+cordic_angle(iteration);
                end else begin
                    fx_x<=fx_x-(fx_y>>>iteration); fx_y<=fx_y+(fx_x>>>iteration);
                    work_q=fx_z-cordic_angle(iteration);
                end
                fx_z<=work_q;
                if (iteration==127) begin
                    if (atan_negative_x) work_q=FX_PI-work_q;
                    if (atan_negative_y) work_q=-work_q;
                    computed=fixed_pack(work_q,0,FP_W); computed[68]=1; finish(computed);
                end else iteration<=iteration+1'b1;
            end
            SEQ_REQ: if(seq_req_ready)state<=SEQ_WAIT;
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
