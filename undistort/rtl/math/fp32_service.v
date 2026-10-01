`timescale 1ns/1ps
// Portable serial-use binary32 arithmetic. Round-to-nearest-even, gradual
// underflow, no FMA. One operation/result slot; result holds under backpressure.
// Wide integer div and normalization loops are synthesizable, but their area
// and timing require synthesis measurement. Replace this wrapper with pipelined
// vendor IP for production without changing its ready/valid contract.
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
        integer e, j;
        begin
            m=magnitude; e=exp;
            for(j=0;j<64;j=j+1) begin
                if(m>=64'h08000000) begin
                    sticky=m[0]; m=(m>>1); m[0]=m[0]|sticky; e=e+1;
                end
            end
            for(j=0;j<64;j=j+1) begin
                if(m!=0 && m<64'h04000000 && e> -126) begin m=m<<1; e=e-1; end
            end
            for(j=0;j<256;j=j+1) begin
                if(e< -126) begin sticky=m[0]; m=m>>1; m[0]=m[0]|sticky; e=e+1; end
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
            t=v; sticky=0;
            for(j=0;j<256;j=j+1) if(j<distance) begin sticky=sticky|t[0]; t=t>>1; end
            t[0]=t[0]|sticky; shr_jam=t;
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
                if(m==0) s=0;
                calculate=pack(s,e,m);
            end else if(op==2) begin
                m={40'd0,ma}*{40'd0,mb};
                calculate=pack(sa^sb,ea+eb,shr_jam(m,20));
            end else if(op==3) begin
                if(mb==0) calculate=(ma==0)?32'h7fc00000:{sa^sb,8'hff,23'd0};
                else if(ma==0) calculate={sa^sb,31'd0};
                else begin
                    // Normalize both operands before division (also subnormals).
                    for(j=0;j<23;j=j+1) begin
                        if(!ma[23]) begin ma=ma<<1; ea=ea-1; end
                        if(!mb[23]) begin mb=mb<<1; eb=eb-1; end
                    end
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
