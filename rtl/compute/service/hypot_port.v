// Optional distance service connection. The external branch has a one-request
// skid register: existing pulse-driven clients need not assume arbitration
// grants in the request cycle. In-flight results still use ready/valid.
module hypot_port #(parameter SHARED=0,USE_CE=0)(
    input wire clk,rst_n,ce,in_valid,output wire in_ready,
    input wire [31:0] in_a,in_b,
    output wire out_valid,input wire out_ready,output wire [31:0] out_r,
    output wire math_hyp_valid,input wire math_hyp_ready,
    output wire [31:0] math_hyp_a,math_hyp_b,
    input wire math_hyp_rsp_valid,output wire math_hyp_rsp_ready,
    input wire [31:0] math_hyp_result
);
    generate if(SHARED)begin:g_external
        reg pending;reg [31:0] a,b;
        assign in_ready=rst_n && !pending;
        assign math_hyp_valid=rst_n && pending;
        assign math_hyp_a=a;assign math_hyp_b=b;
        assign out_valid=rst_n && math_hyp_rsp_valid;
        assign out_r=math_hyp_result;
        assign math_hyp_rsp_ready=rst_n && out_ready;
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin pending<=0;a<=0;b<=0;end
            else if(!USE_CE || ce)begin
                if(math_hyp_valid && math_hyp_ready)pending<=0;
                if(in_valid && in_ready)begin pending<=1;a<=in_a;b<=in_b;end
            end
        end
    end else begin:g_local
        assign math_hyp_valid=0;assign math_hyp_a=0;assign math_hyp_b=0;
        assign math_hyp_rsp_ready=0;
        fp32_hypot #(.USE_CE(USE_CE)) core(.clk(clk),.rst_n(rst_n),.ce(ce),
            .in_valid(in_valid),.in_ready(in_ready),.in_a(in_a),.in_b(in_b),
            .out_valid(out_valid),.out_ready(out_ready),.out_r(out_r));
    end endgenerate
endmodule
