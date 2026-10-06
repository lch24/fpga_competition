`timescale 1ns/1ps
// Portable serial-use binary32 arithmetic. Round-to-nearest-even, gradual
// underflow, no FMA. One operation/result slot; result holds under backpressure.
// Normalization uses a leading-bit encoder and bounded barrel shifts.
// No repeated exponent adjustment chain is unrolled into hardware.
module fp32_service (
    input wire clk, rst_n,
    input wire req_valid, output wire req_ready,
    input wire [2:0] req_op, input wire [31:0] req_a, req_b,
    output reg rsp_valid, input wire rsp_ready,
    output reg [31:0] rsp_result, output reg rsp_error
);
    assign req_ready = !rsp_valid || rsp_ready;

    // Value = mag * 2^(exp-26). mag contains a 24-bit significand plus GRS.
    function [31:0] pack;
        input sign;
        input integer exp;
        input [63:0] magnitude;
        reg [63:0] m;
        reg [24:0] rounded;
        reg sticky;
        reg [7:0] encoded_exp;
        integer e, j, leading, distance;
        begin
            m=magnitude; e=exp;
            leading=0;
            for(j=0;j<64;j=j+1) if(m[j]) leading=j;
            if(m!=0 && leading>26) begin
                distance=leading-26;
                sticky=|(m & (64'hffffffffffffffff >> (64-distance)));
                m=m>>distance; m[0]=m[0]|sticky; e=e+distance;
            end else if(m!=0 && leading<26 && e> -126) begin
                distance=26-leading;
                if(distance>e+126) distance=e+126;
                m=m<<distance; e=e-distance;
            end
            if(e< -126) begin
                distance=-126-e;
                // Preserve the former bounded loop even for out-of-domain e.
                if(distance>256) distance=256;
                if(distance>=64) m={63'd0,|m};
                else begin
                    sticky=|(m & (64'hffffffffffffffff >> (64-distance)));
                    m=m>>distance; m[0]=m[0]|sticky;
                end
                e=e+distance;
            end
            rounded={1'b0,m[26:3]};
            if(m[2] && (m[1] || m[0] || rounded[0])) rounded=rounded+25'd1;
            if(rounded[24]) begin rounded=rounded>>1; e=e+1; end
            if(e>127) pack={sign,8'hff,23'd0};
            else if(rounded==0) pack={sign,31'd0};
            else if(e== -126 && !rounded[23]) pack={sign,8'd0,rounded[22:0]};
            else begin encoded_exp=e+127; pack={sign,encoded_exp,rounded[22:0]}; end
        end
    endfunction

    function [63:0] shr_jam;
        input [63:0] v;
        input integer distance;
        reg [63:0] t;
        reg sticky;
        integer j;
        begin
            if(distance<=0) shr_jam=v;
            else if(distance>=64) shr_jam={63'd0,|v};
            else begin
                t=v>>distance;
                sticky=|(v & (64'hffffffffffffffff >> (64-distance)));
                t[0]=t[0]|sticky; shr_jam=t;
            end
        end
    endfunction

    function integer leading24;
        input [23:0] value;
        integer k;
        begin
            leading24=0;
            for(k=0;k<24;k=k+1) if(value[k]) leading24=k;
        end
    endfunction

    function [31:0] calculate;
        input [2:0] op;
        input [31:0] a,b;
        reg sa,sb,s;
        reg [23:0] ma,mb;
        reg [63:0] va,vb,m,n,q;
        integer ea,eb,e,j;
        begin
            sa=a[31]; sb=b[31]^(op==3'd1);
            ea={24'd0,a[30:23]}; eb={24'd0,b[30:23]};
            ea=(ea==0)?-126:ea-127; eb=(eb==0)?-126:eb-127;
            ma={a[30:23]!=0,a[22:0]}; mb={b[30:23]!=0,b[22:0]};
            // Normalize subnormal operands before reducing a product/quotient
            // to guard/round/sticky bits. Otherwise a tiny significand loses
            // significant low bits before the result is normalized.
            if(op==2 || op==3) begin
                if(ma!=0) begin j=23-leading24(ma);ma=ma<<j;ea=ea-j;end
                if(mb!=0) begin j=23-leading24(mb);mb=mb<<j;eb=eb-j;end
            end
            calculate=32'h7fc00000;
            if(op==3'd4) begin
                calculate=pack(1'b0,26,{32'd0,a});
            end else if(a[30:23]==255 || b[30:23]==255) begin
                // Non-finite parameters/results are rejected by callers.
                calculate=32'h7fc00000;
            end else if(op==0 || op==1) begin
                va={37'd0,ma,3'd0}; vb={37'd0,mb,3'd0};
                if(ea>=eb) begin vb=shr_jam(vb,ea-eb); e=ea; end
                else begin va=shr_jam(va,eb-ea); e=eb; end
                if(sa==sb) begin m=va+vb; s=sa; end
                else if(va>=vb) begin m=va-vb; s=sa; end
                else begin m=vb-va; s=sb; end
                if(m==0) s=(ma==0 && mb==0 && sa && sb);
                calculate=pack(s,e,m);
            end else if(op==2) begin
                m={40'd0,ma}*{40'd0,mb};
                calculate=pack(sa^sb,ea+eb,shr_jam(m,20));
            end else if(op==3) begin
                if(mb==0) calculate=(ma==0)?32'h7fc00000:{sa^sb,8'hff,23'd0};
                else if(ma==0) calculate={sa^sb,31'd0};
                else begin
                    n={40'd0,ma}<<27; q=n/mb;
                    if(n%mb!=0) q[0]=1'b1;
                    calculate=pack(sa^sb,ea-eb-1,q);
                end
            end
        end
    endfunction
    wire [31:0] result=calculate(req_op,req_a,req_b);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin rsp_valid<=0; rsp_result<=0; rsp_error<=0; end
        else if(req_ready) begin
            rsp_valid<=req_valid;
            if(req_valid) begin rsp_result<=result; rsp_error<=result[30:23]==8'hff || req_op>4; end
        end
    end
endmodule
