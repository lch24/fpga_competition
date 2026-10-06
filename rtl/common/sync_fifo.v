`timescale 1ns / 1ps
//==============================================================================
// sync_fifo.v — 同步 FIFO（同一时钟域，valid/ready 流式接口）
//------------------------------------------------------------------------------
// 用途：像素流、命令流、DDR 读返回流等所有需要缓冲的握手通路。
// 读写同一时钟。跨时钟域请勿使用本模块（那是 async_fifo 的职责）。
//
// 参数：
//   DATA_WIDTH : 数据位宽（bit）
//   ADDR_WIDTH : 地址位宽；容量固定为 2**ADDR_WIDTH（仅支持 2 的幂深度）
//
// 接口（全部为同一时钟域）：
//   写侧：in_valid / in_ready / in_data      —— 上游驱动
//   读侧：out_valid / out_ready / out_data   —— 下游驱动
//   状态：count（当前存量）/ empty / full
//
// 握手语义（团队契约：valid&&!ready 时载荷保持不变）：
//   in_ready  = !full  ：满时拒绝写入，不覆盖
//   out_valid = !empty ：空时不出数据
//   只有 valid && ready 同拍为 1 才推进指针
//
// 时序语义：
//   out_data 为组合读输出（零延迟）：empty=0 时立即有效。
//   存储按 distributed/LUT RAM 推断，适合中小深度（建议 ADDR_WIDTH<=6）；
//   更深 FIFO 需求出现时再提供 BRAM 版本（带 1 拍读延迟），届时单独
//   验证替换，不得静默换用。
//
// 复位行为：rst_n（低有效，同步释放，建议来自 reset_sync）同步复位，
//   复位后 FIFO 为空、指针清零。复位期间 in_ready=0。
//   注意：复位不保证存储内容被清除，但 empty 状态保证它们不会被读出。
//
// 同拍读写：允许。空 FIFO 同拍写+读时，因 out_valid=0 而读握手不成立，
//   刚写入的数据下一拍才可读，语义安全。
//==============================================================================
module sync_fifo #(parameter USE_CE=0,
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 4
) (
    // Global synchronous stall for variable-latency backing memory.
    input wire ce,

    input  wire                  clk,
    input  wire                  rst_n,
    // 写侧
    input  wire                  in_valid,
    output wire                  in_ready,
    input  wire [DATA_WIDTH-1:0] in_data,
    // 读侧
    output wire                  out_valid,
    input  wire                  out_ready,
    output reg  [DATA_WIDTH-1:0] out_data,
    // 状态
    output reg  [ADDR_WIDTH:0]   count,
    output wire                  empty,
    output wire                  full
);

    // 队列存储（组合读）
    // Small elastic queues must not consume a full DRM block per instance.
    (* syn_ramstyle="lut_ram" *) reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];

    // 扩展一位的读写指针
    reg  [ADDR_WIDTH:0] wr_ptr;
    reg  [ADDR_WIDTH:0] rd_ptr;

    wire do_wr = in_valid  && in_ready;
    wire do_rd = out_valid && out_ready;

    assign in_ready  = ~full;
    assign out_valid = ~empty;
    assign empty     = (wr_ptr == rd_ptr);
    assign full      = (wr_ptr[ADDR_WIDTH] != rd_ptr[ADDR_WIDTH]) &&
                       (wr_ptr[ADDR_WIDTH-1:0] == rd_ptr[ADDR_WIDTH-1:0]);

    // 指针推进
    always @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= {(ADDR_WIDTH+1){1'b0}};
            rd_ptr <= {(ADDR_WIDTH+1){1'b0}};
        end else if(!USE_CE || ce) begin begin
            if (do_wr) wr_ptr <= wr_ptr + 1'b1;
            if (do_rd) rd_ptr <= rd_ptr + 1'b1;
        end
    end // synchronous clock enable
    end

    // 计数
    always @(posedge clk) begin
        if (!rst_n)
            count <= {(ADDR_WIDTH+1){1'b0}};
        else if(!USE_CE || ce) begin if (do_wr && !do_rd)
            count <= count + 1'b1;
        else if (!do_wr && do_rd)
            count <= count - 1'b1;
    end // synchronous clock enable
    end

    // 组合读（always @(*) 驱动，故 out_data 声明为 reg）
    always @(*) begin
        out_data = mem[rd_ptr[ADDR_WIDTH-1:0]];
    end

    // 写
    always @(posedge clk) begin
        if(!USE_CE || ce) begin
        if (do_wr)
            mem[wr_ptr[ADDR_WIDTH-1:0]] <= in_data;
    end // synchronous clock enable
    end

endmodule

// Buffered arithmetic sharing. Each lane may own one queued/in-flight/result
// transaction. Requests arriving together are retained independently; results
// remain valid until that lane acknowledges. Reset cancels all transactions.
// WIDTH=32/64; KIND: 0 add (subtract by flipping b sign), 1 multiply,
// 2 divide, 3 hypot (WIDTH=32 only). LANES must be positive.
// Arbitration operates on registered requests; no combinational arithmetic
// chain crosses this boundary. All participants must use the same CE:
// valid/ready transfers occur only on enabled cycles. Payload is unspecified
// when rsp_valid=0; operand/result arrays intentionally have no reset.
module fp_arith_pool #(parameter USE_CE=0, LANES=2, KIND=0, WIDTH=32)(
    input wire clk,rst_n,ce,
    input wire [LANES-1:0] req_valid,
    output wire [LANES-1:0] req_ready,
    input wire [LANES*WIDTH-1:0] req_a,req_b,
    output reg [LANES-1:0] rsp_valid,
    input wire [LANES-1:0] rsp_ready,
    output reg [LANES*WIDTH-1:0] rsp_data
);
    localparam AW=(LANES<2)?1:$clog2(LANES);
    reg [LANES-1:0] pending,occupied;
    reg [WIDTH-1:0] operand_a[0:LANES-1],operand_b[0:LANES-1];
    reg active;
    reg [AW-1:0] owner,head,selected;
    reg found;
    integer scan,index;
    // Round-robin prevents starvation if clients are later reused independently.
    always @* begin
        found=0;selected=0;index=0;
        for(scan=0;scan<LANES;scan=scan+1) begin
            index=head+scan;
            if(index>=LANES) index=index-LANES;
            if(!found && pending[index]) begin found=1;selected=index;end
        end
    end
    assign req_ready=~occupied;
    wire core_ready,core_valid;
    wire [WIDTH-1:0] core_result;
    wire launch=found && !active;
    wire [WIDTH-1:0] a=operand_a[selected],b=operand_b[selected];
    generate
        if(WIDTH==32) begin : g_fp32
            if(KIND==0) begin : g_add
                fp32_add #(.USE_CE(USE_CE)) core(.clk(clk),.rst_n(rst_n),.ce(ce),
                    .in_valid(launch),.in_ready(core_ready),.in_a(a),.in_b(b),
                    .out_valid(core_valid),.out_ready(active),.out_r(core_result));
            end
            if(KIND==1) begin : g_mul
                fp32_mul #(.USE_CE(USE_CE)) core(.clk(clk),.rst_n(rst_n),.ce(ce),
                    .in_valid(launch),.in_ready(core_ready),.in_a(a),.in_b(b),
                    .out_valid(core_valid),.out_ready(active),.out_r(core_result));
            end
            if(KIND==2) begin : g_div
                fp32_div #(.USE_CE(USE_CE)) core(.clk(clk),.rst_n(rst_n),.ce(ce),
                    .in_valid(launch),.in_ready(core_ready),.in_a(a),.in_b(b),
                    .out_valid(core_valid),.out_ready(active),.out_r(core_result));
            end
            if(KIND==3) begin : g_hypot
                fp32_hypot #(.USE_CE(USE_CE)) core(.clk(clk),.rst_n(rst_n),.ce(ce),
                    .in_valid(launch),.in_ready(core_ready),.in_a(a),.in_b(b),
                    .out_valid(core_valid),.out_ready(active),.out_r(core_result));
            end
        end
        if(WIDTH==64) begin : g_fp64
            if(KIND==0) begin : g_add
                fp64_add #(.USE_CE(USE_CE)) core(.clk(clk),.rst_n(rst_n),.ce(ce),
                    .in_valid(launch),.in_ready(core_ready),.in_a(a),.in_b(b),
                    .out_valid(core_valid),.out_ready(active),.out_r(core_result));
            end
            if(KIND==1) begin : g_mul
                fp64_mul #(.USE_CE(USE_CE)) core(.clk(clk),.rst_n(rst_n),.ce(ce),
                    .in_valid(launch),.in_ready(core_ready),.in_a(a),.in_b(b),
                    .out_valid(core_valid),.out_ready(active),.out_r(core_result));
            end
            if(KIND==2) begin : g_div
                fp64_div #(.USE_CE(USE_CE)) core(.clk(clk),.rst_n(rst_n),.ce(ce),
                    .in_valid(launch),.in_ready(core_ready),.in_a(a),.in_b(b),
                    .out_valid(core_valid),.out_ready(active),.out_r(core_result));
            end
        end
    endgenerate
    integer lane;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pending<=0;occupied<=0;rsp_valid<=0;active<=0;owner<=0;head<=0;
        end else if(!USE_CE || ce) begin
            for(lane=0;lane<LANES;lane=lane+1) begin
                if(req_valid[lane] && req_ready[lane]) begin
                    operand_a[lane]<=req_a[lane*WIDTH+:WIDTH];
                    operand_b[lane]<=req_b[lane*WIDTH+:WIDTH];
                    pending[lane]<=1;occupied[lane]<=1;
                end
                if(rsp_valid[lane] && rsp_ready[lane]) begin
                    rsp_valid[lane]<=0;occupied[lane]<=0;
                end
            end
            if(launch && core_ready) begin
                pending[selected]<=0;active<=1;owner<=selected;
                head<=(selected==LANES-1)?0:selected+1'b1;
            end
            if(active && core_valid) begin
                rsp_data[owner*WIDTH+:WIDTH]<=core_result;
                rsp_valid[owner]<=1;active<=0;
            end
        end
    end
endmodule
