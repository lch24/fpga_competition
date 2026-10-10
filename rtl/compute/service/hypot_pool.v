// One physical distance core. Round-robin arbitration tolerates overlapping
// clients. Owner is retained through response backpressure; all clients and
// this pool must share reset/CE, so cancellation cannot misroute a result.
module hypot_pool #(parameter CLIENTS=4,USE_CE=0)(
    input wire clk,rst_n,ce,
    input wire [CLIENTS-1:0] req_valid,output reg [CLIENTS-1:0] req_ready,
    input wire [CLIENTS*32-1:0] req_a,req_b,
    output reg [CLIENTS-1:0] rsp_valid,input wire [CLIENTS-1:0] rsp_ready,
    output wire [31:0] result
);
    localparam OW=CLIENTS>1?$clog2(CLIENTS):1;
    reg [OW-1:0] owner,next_owner;
    reg busy;
    integer i,index,selected;
    wire core_ready,core_valid;
    wire issue=rst_n && !busy && selected>=0;
    wire consume=rst_n && busy && rsp_ready[owner];
    always @*begin
        selected=-1;
        for(i=0;i<CLIENTS;i=i+1)begin
            index=next_owner+i;if(index>=CLIENTS)index=index-CLIENTS;
            if(selected<0 && req_valid[index])selected=index;
        end
        req_ready=0;rsp_valid=0;
        if(issue)req_ready[selected]=core_ready;
        if(rst_n && busy)rsp_valid[owner]=core_valid;
    end
    fp32_hypot #(.USE_CE(USE_CE)) core(.clk(clk),.rst_n(rst_n),.ce(ce),
        .in_valid(issue),.in_ready(core_ready),
        .in_a(selected>=0?req_a[selected*32+:32]:32'd0),
        .in_b(selected>=0?req_b[selected*32+:32]:32'd0),
        .out_valid(core_valid),.out_ready(consume),.out_r(result));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin busy<=0;owner<=0;next_owner<=0;end
        else if(!USE_CE || ce)begin
            if(issue && core_ready)begin
                busy<=1;owner<=selected;
                next_owner<=selected==CLIENTS-1?0:selected+1;
            end
            if(core_valid && consume)busy<=0;
        end
    end
endmodule
