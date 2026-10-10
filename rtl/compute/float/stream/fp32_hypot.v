`timescale 1ns/1ps
// Coordinate distance: exact 24-bit squares -> rounded FP64 sum -> FP64 sqrt
// -> float. Same double intermediate rounding, without general FP64 multiplier
// and adder pipelines. One transaction in flight; CE freezes ALL state.
module fp32_hypot #(parameter USE_CE=0)(
    input wire ce,clk,rst_n,in_valid, output wire in_ready,
    input wire [31:0] in_a,in_b,
    output wire out_valid,input wire out_ready,output wire [31:0] out_r
);
    localparam IDLE=0,NORMALIZE=1,SQUARE_A=2,SQUARE_B=3,ORDER=4,
        ALIGN=5,ADD=6,NORM_SUM=7,ROUND=8,SQRT_REQ=9,SQRT_WAIT=10,
        CONVERT_WAIT=11,SPECIAL=12;
    reg [3:0] state;
    reg [23:0] ma,mb;
    reg signed [10:0] ea,eb,exponent;
    reg [47:0] square_a,square_b;
    wire [23:0] mul_input=state==SQUARE_A ? ma : mb;
    wire [47:0] square=mul_input*mul_input;
    reg [55:0] larger_q,smaller_q;
    reg [56:0] sum;
    reg [9:0] distance;
    reg [63:0] sqrt_input;
    reg [31:0] special_result;
    wire [53:0] rounded={1'b0,sum[55:3]}+
        {53'd0,(sum[2] && (sum[1] || sum[0] || sum[3]))};
    wire [52:0] mantissa=rounded[53] ? rounded[53:1] : rounded[52:0];
    wire signed [11:0] biased=$signed(exponent)+12'sd1023+(rounded[53]?12'sd1:12'sd0);
    wire sr,sv,cr,cv; wire [63:0] root;wire [31:0] converted;
    fp64_sqrt #(.USE_CE(USE_CE)) u_sqrt(.ce(ce),.clk(clk),.rst_n(rst_n),
        .in_valid(state==SQRT_REQ),.in_ready(sr),.in_x(sqrt_input),
        .out_valid(sv),.out_ready(cr && state==SQRT_WAIT),.out_r(root));
    f64_to_f32 #(.USE_CE(USE_CE)) u_cvt(.ce(ce),.clk(clk),.rst_n(rst_n),
        .in_valid(sv && state==SQRT_WAIT),.in_ready(cr),.in_x(root),
        .out_valid(cv),.out_ready(out_ready && state==CONVERT_WAIT),.out_r(converted));
    assign in_ready=rst_n && state==IDLE;
    assign out_valid=rst_n && (state==SPECIAL || (state==CONVERT_WAIT && cv));
    // The legacy converter retains fraction bits on overflow. These inputs
    // are finite and the root is nonnegative, so overflow must be +Inf.
    assign out_r=state==SPECIAL ? special_result :
        (converted[30:23]==8'hff ? 32'h7f800000 : converted);
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;ma<=0;mb<=0;ea<=0;eb<=0;exponent<=0;
            square_a<=0;square_b<=0;larger_q<=0;smaller_q<=0;sum<=0;
            distance<=0;sqrt_input<=0;special_result<=0;
        end else if(!USE_CE || ce)case(state)
            IDLE:if(in_valid)begin
                ma<={|in_a[30:23],in_a[22:0]};mb<={|in_b[30:23],in_b[22:0]};
                ea<=in_a[30:23]==0 ? -11'sd126 : $signed({3'b0,in_a[30:23]})-11'sd127;
                eb<=in_b[30:23]==0 ? -11'sd126 : $signed({3'b0,in_b[30:23]})-11'sd127;
                state<=NORMALIZE;
                if(in_a[30:0]==31'h7f800000 || in_b[30:0]==31'h7f800000)begin
                    special_result<=32'h7f800000;state<=SPECIAL;
                end else if(in_a[30:23]==255 || in_b[30:23]==255)begin
                    special_result<=32'h7fc00000;state<=SPECIAL;
                end else if(in_a[30:0]==0 && in_b[30:0]==0)begin
                    special_result<=0;state<=SPECIAL;
                end
            end
            NORMALIZE:begin
                if(ma!=0 && !ma[23])begin ma<=ma<<1;ea<=ea-1'b1;end
                if(mb!=0 && !mb[23])begin mb<=mb<<1;eb<=eb-1'b1;end
                if((ma==0 || ma[23]) && (mb==0 || mb[23]))state<=SQUARE_A;
            end
            SQUARE_A:begin square_a<=square;state<=SQUARE_B;end
            SQUARE_B:begin square_b<=square;state<=ORDER;end
            ORDER:begin
                // Eight trailing bits place the leading square bit at 54/55.
                // Exponent is relative to bit55. Squares are exact (48 bits).
                if((ea>=eb && ma!=0) || mb==0)begin
                    larger_q<={square_a,8'd0};smaller_q<={square_b,8'd0};
                    exponent<=2*ea+1;distance<=mb==0?10'd1023:2*(ea-eb);
                end else begin
                    larger_q<={square_b,8'd0};smaller_q<={square_a,8'd0};
                    exponent<=2*eb+1;distance<=ma==0?10'd1023:2*(eb-ea);
                end
                state<=ALIGN;
            end
            ALIGN:begin
                // Fixed shifts and sticky bits, no variable barrel shifter.
                if(distance>=56)begin smaller_q<={55'd0,|smaller_q};distance<=0;end
                else if(distance>=8)begin
                    smaller_q<={8'd0,smaller_q[55:9],|smaller_q[8:0]};distance<=distance-8;
                end else if(distance!=0)begin
                    smaller_q<={1'b0,smaller_q[55:2],smaller_q[1]|smaller_q[0]};distance<=distance-1'b1;
                end else state<=ADD;
            end
            ADD:begin sum<={1'b0,larger_q}+{1'b0,smaller_q};state<=NORM_SUM;end
            NORM_SUM:begin
                if(sum[56])begin sum<={1'b0,sum[56:2],sum[1]|sum[0]};exponent<=exponent+1'b1;end
                else if(!sum[55])begin sum<=sum<<1;exponent<=exponent-1'b1;end
                else state<=ROUND;
            end
            ROUND:begin sqrt_input<={1'b0,biased[10:0],mantissa[51:0]};state<=SQRT_REQ;end
            SQRT_REQ:if(sr)state<=SQRT_WAIT;
            SQRT_WAIT:if(sv && cr)state<=CONVERT_WAIT;
            CONVERT_WAIT:if(cv && out_ready)state<=IDLE;
            SPECIAL:if(out_ready)state<=IDLE;
            default:state<=IDLE;
        endcase
    end
endmodule
