`include "calib_defs.vh"
/*
FP32/FP64除法和平方根的逐拍算术核，供fp_operator独占调用。
NORMALIZE每拍左移一位处理非规格化数；DIV_STEP每拍产生一位商，
SQRT_STEP每拍带入两位被开方数并产生一位根，均只有一次比较/减法。
最后PACK逐拍规格化、ROUND最近偶数舍入，余数非零形成sticky。
尾数通路仅F+5位；正常数收尾2或3拍，非规格化结果逐拍右移。
正常除法F+5轮，开方F+4轮；非规格化输入最多增加F轮归一化。
不含组合整数除法、求余或展开的开方迭代；特殊值可提前返回。
valid/ready一次一事务；返回背压保持，复位取消在途请求。
*/
module fp_divsqrt #(parameter FP_W=64, USE_CE=0) (
    input wire ce, // Optional synchronous stall, ignored when USE_CE=0.
    input wire clk,
    input wire rst_n,
    input wire req_valid,
    output wire req_ready,
    input wire req_sqrt,
    input wire [FP_W-1:0] req_a,
    input wire [FP_W-1:0] req_b,
    output wire rsp_valid,
    input wire rsp_ready,
    output reg [FP_W-1:0] rsp_result,
    output reg [4:0] rsp_flags
);
    localparam F=(FP_W==32)?23:52;
    localparam BIAS=(FP_W==32)?127:1023;
    localparam E=FP_W-F-1;
    localparam RW=2*(F+4);
    localparam [FP_W-1:0] INF_BITS=({FP_W{1'b1}} >> (F+1)) << F;
    localparam [FP_W-1:0] NAN_BITS=INF_BITS | ({{(FP_W-1){1'b0}},1'b1} << (F-1));
    localparam IDLE=0,NORMALIZE=1,PREPARE=2,DIV_STEP=3,SQRT_STEP=4,PACK=5,RESPONSE=6,ROUND=7;
    reg [2:0] state;
    reg sqrt_mode,result_sign;
    reg [F:0] sig_a,sig_b;
    reg signed [12:0] exp_a,exp_b;
    reg signed [12:0] pack_exp;
    reg [F+5:0] remainder_q,denominator;
    reg [F+4:0] digits;
    reg [RW-1:0] radicand;
    reg [6:0] steps_left;
    reg sticky;
    wire [F+5:0] sqrt_bring=(remainder_q<<2) | {{(F+4){1'b0}},radicand[RW-1:RW-2]};
    wire [F+5:0] sqrt_trial=({1'b0,digits}<<2) | {{(F+5){1'b0}},1'b1};
    // DIV and SQRT are mutually exclusive for an entire transaction. Share
    // one subtractor; its extra bit provides unsigned borrow, replacing both
    // separate comparisons and both mode-specific subtraction datapaths.
    wire [F+5:0] step_lhs=sqrt_mode?sqrt_bring:remainder_q;
    wire [F+5:0] step_rhs=sqrt_mode?sqrt_trial:denominator;
    wire [F+6:0] step_difference={1'b0,step_lhs}-{1'b0,step_rhs};
    wire step_take=!step_difference[F+6];
    wire [F+5:0] step_rest=step_take?step_difference[F+5:0]:step_lhs;
    wire [E-1:0] field_a=req_a[FP_W-2:F],field_b=req_b[FP_W-2:F];
    wire nan_a=(&field_a) && (|req_a[F-1:0]);
    wire nan_b=(&field_b) && (|req_b[F-1:0]);
    wire snan_a=nan_a && !req_a[F-1],snan_b=nan_b && !req_b[F-1];
    wire inf_a=(&field_a) && !(|req_a[F-1:0]);
    wire inf_b=(&field_b) && !(|req_b[F-1:0]);
    wire zero_a=!(|req_a[FP_W-2:0]),zero_b=!(|req_b[FP_W-2:0]);
    wire sign_a=req_a[FP_W-1],sign_b=req_b[FP_W-1];
    wire [FP_W-1:0] div_sign_bits={(sign_a^sign_b),{(FP_W-1){1'b0}}};
    // Hidden bit at F+3, followed by guard/round/sticky bits. The digit
    // generator already guarantees this position or one bit above it.
    // Gradual underflow reuses the digit register and shifts one bit/cycle.
    wire lost=(|digits[2:0]) || sticky;
    wire [F+1:0] rounded={1'b0,digits[F+3:3]} +
        {{(F+1){1'b0}},(digits[2] && (digits[1] || digits[0] || sticky || digits[3]))};
    wire [F:0] rounded_sig=rounded[F+1]?rounded[F+1:1]:rounded[F:0];
    wire signed [12:0] final_exp=pack_exp+(rounded[F+1]?13'sd1:13'sd0);
    wire [12:0] biased_exp=final_exp+BIAS;

    assign req_ready=rst_n && state==IDLE;
    assign rsp_valid=rst_n && state==RESPONSE;
    task special_result;
        input [FP_W-1:0] value;
        input [4:0] flags;
        begin rsp_result<=value;rsp_flags<=flags;state<=RESPONSE;end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin
            state<=IDLE;sqrt_mode<=0;result_sign<=0;sig_a<=0;sig_b<=0;
            exp_a<=0;exp_b<=0;pack_exp<=0;remainder_q<=0;denominator<=0;
            digits<=0;radicand<=0;steps_left<=0;sticky<=0;rsp_result<=0;rsp_flags<=0;
        end else if(!USE_CE || ce) case(state)
        IDLE:if(req_valid && req_ready)begin
            sqrt_mode<=req_sqrt;
            result_sign<=req_sqrt?1'b0:(sign_a^sign_b);
            if(req_sqrt && nan_a)special_result(NAN_BITS,{4'd0,snan_a});
            else if(!req_sqrt && (nan_a||nan_b))special_result(NAN_BITS,{4'd0,(snan_a||snan_b)});
            else if(req_sqrt && sign_a && !zero_a)special_result(NAN_BITS,5'b00001);
            else if(req_sqrt && (inf_a||zero_a))special_result(req_a,0);
            else if(!req_sqrt && ((inf_a&&inf_b)||(zero_a&&zero_b)))special_result(NAN_BITS,5'b00001);
            else if(!req_sqrt && inf_a)special_result(div_sign_bits|INF_BITS,0);
            else if(!req_sqrt && (inf_b||zero_a))special_result(div_sign_bits,0);
            else if(!req_sqrt && zero_b)special_result(div_sign_bits|INF_BITS,5'b00010);
            else begin
                sig_a<={(|field_a),req_a[F-1:0]};
                sig_b<={(|field_b),req_b[F-1:0]};
                exp_a<=(field_a==0)?(1-BIAS):($signed({1'b0,field_a})-BIAS);
                exp_b<=(field_b==0)?(1-BIAS):($signed({1'b0,field_b})-BIAS);
                state<=NORMALIZE;
            end
        end
        NORMALIZE:begin
            if(!sig_a[F])begin sig_a<=sig_a<<1;exp_a<=exp_a-1'b1;end
            if(!sqrt_mode && !sig_b[F])begin sig_b<=sig_b<<1;exp_b<=exp_b-1'b1;end
            if(sig_a[F] && (sqrt_mode||sig_b[F]))state<=PREPARE;
        end
        PREPARE:begin
            digits<=0;sticky<=0;
            if(sqrt_mode)begin
                remainder_q<=0;
                // e调整到偶数；移位及除2只作用于小位宽整数指数。
                if(exp_a[0])begin
                    radicand<=({{(RW-F-1){1'b0}},sig_a}<<(F+7));
                    pack_exp<=(($signed(exp_a)-1)>>>1);
                end else begin
                    radicand<=({{(RW-F-1){1'b0}},sig_a}<<(F+6));
                    pack_exp<=($signed(exp_a)>>>1);
                end
                steps_left<=F+4;state<=SQRT_STEP;
            end else begin
                remainder_q<=sig_a;denominator<=sig_b;
                pack_exp<=$signed(exp_a)-$signed(exp_b)-13'sd1;
                steps_left<=F+5;state<=DIV_STEP;
            end
        end
        DIV_STEP:begin
            digits<=(digits<<1)|step_take;
            if(steps_left==1)begin
                remainder_q<=step_rest;sticky<=|step_rest;state<=PACK;
            end else begin
                remainder_q<=step_rest<<1;steps_left<=steps_left-1'b1;
            end
        end
        SQRT_STEP:begin
            digits<=(digits<<1)|step_take;
            remainder_q<=step_rest;radicand<=radicand<<2;
            if(steps_left==1)begin sticky<=|step_rest;state<=PACK;end
            else steps_left<=steps_left-1'b1;
        end
        PACK:begin
            if(pack_exp < (1-BIAS)-(F+5))begin
                // Everything is below half an ulp of the smallest subnormal.
                sticky<=sticky || (|digits);digits<=0;pack_exp<=1-BIAS;state<=ROUND;
            end else if(digits[F+4] || pack_exp < 1-BIAS)begin
                sticky<=sticky || digits[0];digits<=digits>>1;pack_exp<=pack_exp+13'sd1;
            end else state<=ROUND;
        end
        ROUND:begin
            if(final_exp>BIAS)begin
                rsp_result<={result_sign,INF_BITS[FP_W-2:0]};rsp_flags<=5'b10100;
            end else begin
                rsp_result<={result_sign,(rounded_sig[F]?biased_exp[E-1:0]:{E{1'b0}}),rounded_sig[F-1:0]};
                rsp_flags<={lost,(lost && !rounded_sig[F]),3'b000};
            end
            state<=RESPONSE;
        end
        RESPONSE:if(rsp_ready)state<=IDLE;
        default:state<=IDLE;
        endcase
    end
endmodule
