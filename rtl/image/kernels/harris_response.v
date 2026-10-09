`timescale 1ns/1ps
// Integer Harris: R = 25*(A*C-B*B)-(A+C)^2, for a 3x3 Sobel tensor.
// A,C <= 9*1020^2, |B| <= 9*1020^2. Products fit 50 bits;
// signed difference and constant scaling fit 56 bits. No FP arithmetic.
// One 25x25 unsigned multiplier is time-shared across AC, BB and trace^2.
// Register boundaries separate multiplication, subtraction and shift/add.
// Positive R is rounded to FP32 only at the existing response-stream boundary;
// nonpositive R becomes zero (NMS rejects it). No response frame buffer.
// ce freezes the entire transaction, including stalled output; reset wins.
module harris_response #(parameter USE_CE=0)(
    input wire clk, rst_n, ce,
    input wire in_valid, output wire in_ready,
    input wire [31:0] in_a, in_b, in_c,
    output wire out_valid, input wire out_ready,
    output reg [31:0] out_resp
);
    localparam IDLE=0, AC=1, BB=2, TT=3, DET=4, SCALE=5,
               SCORE=6, NORMALIZE=7, ROUND=8, OUTPUT=9;
    reg [3:0] state;
    reg [24:0] ma,mb,bmag,trace;
    reg [49:0] ac,bb,tt;
    reg signed [55:0] det,scaled,score;
    reg [55:0] normalized;
    reg [7:0] exponent;
    integer k;
    reg [5:0] leading;
    wire [49:0] product=ma*mb;
    wire [24:0] rounded={1'b0,normalized[55:32]}+
        (normalized[31] && ((|normalized[30:0]) || normalized[32]));
    assign in_ready=rst_n && state==IDLE;
    assign out_valid=rst_n && state==OUTPUT;
    always @* begin
        leading=0;
        for(k=0;k<56;k=k+1) if(score[k]) leading=k;
    end
    always @(posedge clk) begin
        if(!rst_n) begin state<=IDLE;out_resp<=0;end
        else if(!USE_CE || ce) case(state)
            IDLE: if(in_valid) begin
                ma<={1'b0,in_a[23:0]}; mb<={1'b0,in_c[23:0]};
                bmag<=in_b[31] ? -$signed(in_b) : in_b;
                trace<={1'b0,in_a[23:0]}+{1'b0,in_c[23:0]};state<=AC;
            end
            AC: begin ac<=product;ma<=bmag;mb<=bmag;state<=BB;end
            BB: begin bb<=product;ma<=trace;mb<=trace;state<=TT;end
            TT: begin tt<=product;state<=DET;end
            DET: begin det<=$signed({6'b0,ac})-$signed({6'b0,bb});state<=SCALE;end
            SCALE: begin scaled<=(det<<<4)+(det<<<3)+det;state<=SCORE;end
            SCORE: begin score<=scaled-$signed({6'b0,tt});state<=NORMALIZE;end
            NORMALIZE: begin
                normalized<=score<<(55-leading);exponent<=127+leading;
                if(score<=0) begin out_resp<=0;state<=OUTPUT;end
                else state<=ROUND;
            end
            ROUND: begin
                out_resp<=rounded[24] ? {1'b0,(exponent+8'd1),23'b0} :
                    {1'b0,exponent,rounded[22:0]};state<=OUTPUT;
            end
            OUTPUT: if(out_ready) state<=IDLE;
            default: state<=IDLE;
        endcase
    end
endmodule
