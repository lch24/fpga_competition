`include "calib_defs.vh"
// Calibration controllers issue one arithmetic instruction at a time. Share
// the physical FP64 datapath across all stages without changing instruction
// order, precision, or rounding. A result belongs to its accepted requester
// until consumed. Per-client reset cancels delivery, but drains the core.
module fp_calibration_pool #(parameter CLIENTS=22)(
    input wire clk,rst_n,
    input wire [CLIENTS-1:0] c_req_valid,c_active,
    output reg [CLIENTS-1:0] c_req_ready,
    input wire [CLIENTS*5-1:0] c_req_op,
    input wire [CLIENTS*64-1:0] c_req_a,c_req_b,
    output reg [CLIENTS-1:0] c_rsp_valid,
    input wire [CLIENTS-1:0] c_rsp_ready,
    output wire [63:0] result,
    output wire [4:0] flags
);
    localparam OW=(CLIENTS>1)?$clog2(CLIENTS):1;
    reg [OW-1:0] owner;
    reg busy,cancelled;
    integer selected,i;
    wire core_ready,core_valid;
    wire core_response_ready=busy && (cancelled || !c_active[owner] || c_rsp_ready[owner]);
    wire issue=rst_n && !busy && selected>=0 && core_ready;
    always @* begin
        selected=-1;
        for(i=CLIENTS-1;i>=0;i=i-1)
            if(c_active[i] && c_req_valid[i]) selected=i;
        c_req_ready=0;c_rsp_valid=0;
        if(issue) c_req_ready[selected]=1;
        if(rst_n && busy && !cancelled && c_active[owner]) c_rsp_valid[owner]=core_valid;
    end
    fp_operator #(.FP_W(64)) core(
        .clk(clk),.rst_n(rst_n),.req_valid(issue),.req_ready(core_ready),
        .req_op(selected>=0?c_req_op[selected*5+:5]:5'd0),
        .req_a(selected>=0?c_req_a[selected*64+:64]:64'd0),
        .req_b(selected>=0?c_req_b[selected*64+:64]:64'd0),
        .rsp_valid(core_valid),.rsp_ready(core_response_ready),
        .rsp_result(result),.rsp_flags(flags),
        .rsp_less(),.rsp_equal(),.rsp_unordered());
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin owner<=0;busy<=0;cancelled<=0;end
        else begin
            if(issue) begin owner<=selected;busy<=1;cancelled<=0;end
            if(busy && !c_active[owner]) cancelled<=1;
            if(core_valid && core_response_ready) begin busy<=0;cancelled<=0;end
        end
    end
endmodule
