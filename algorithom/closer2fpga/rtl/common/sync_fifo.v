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
module sync_fifo #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 4
) (
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
    reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];

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
        end else begin
            if (do_wr) wr_ptr <= wr_ptr + 1'b1;
            if (do_rd) rd_ptr <= rd_ptr + 1'b1;
        end
    end

    // 计数
    always @(posedge clk) begin
        if (!rst_n)
            count <= {(ADDR_WIDTH+1){1'b0}};
        else if (do_wr && !do_rd)
            count <= count + 1'b1;
        else if (!do_wr && do_rd)
            count <= count - 1'b1;
    end

    // 组合读（always @(*) 驱动，故 out_data 声明为 reg）
    always @(*) begin
        out_data = mem[rd_ptr[ADDR_WIDTH-1:0]];
    end

    // 写
    always @(posedge clk) begin
        if (do_wr)
            mem[wr_ptr[ADDR_WIDTH-1:0]] <= in_data;
    end

endmodule
