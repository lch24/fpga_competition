`timescale 1ns / 1ps
//==============================================================================
// gray_fetch.v — M7.2 层灰度 DDR→片上 RAM 流式加载
//   （raster_dma 读方向 + 片上 gray RAM 写捕获）
//------------------------------------------------------------------------------
// 功能：把 DDR 中一片行光栅灰度区（cfg_ddr_base 起、每行 cfg_stride 字节跨度、
//   每行 cfg_w 字节、共 cfg_h 行）按光栅序加载到片上 gray RAM：
//     cfg_ram_base 起、每行 cfg_w 字节连续排布，即 ram 地址 =
//       cfg_ram_base + y*cfg_w + x（x 行内像素号，y 行号）。
//   像素 = 字节（8bit 灰度），cfg_w 同时是每行像素数与字节数。
//
// 结构：内部例化 raster_dma（cfg_dir=0 读方向，cfg_base=cfg_ddr_base，
//   cfg_stride=cfg_stride，cfg_row_bytes=cfg_w，cfg_rows=cfg_h，
//   cfg_offset=0），捕获其 out 字节流（光栅序：行内 x 递增、行间 y 递增）
//   逐像素写 gray RAM 写口。
//
// 加载时序：
//   start（busy=0 时单拍启动）→ busy=1；当拍向 raster_dma 转发单拍 start
//   （cfg_* 输入须在 start 沿保持稳定，raster_dma 在 start 沿锁存）。
//   raster_dma 完成（done 电平 0→1 沿）后一拍 gray_fetch 锁存其 status
//   （01=成功 / 10=读错误），置 done=1（电平保持，下次 start 清零）、busy=0。
//   gray RAM 写口：gray_wr_en 恒跟随 raster_dma out 接受拍（无背压，
//   out_ready 恒 1），组合输出 en/addr/data，同步写（posedge 拍写入，
//   1 拍完成）。写游标 cnt = y*cfg_w + x（out 流每字节即一像素、光栅序
//   连续），gray_wr_addr = cfg_ram_base + cnt。
//
// 错误语义：
//   raster_dma 读错误（status=10，读返回 error 置位）→ gray_fetch
//   status=10、done=1；成功 status=01。busy/done/status/游标/配置随
//   start 全复位（连续帧复用；start 只在 busy=0 时被接受）。
//
// 端口说明（与 ddr_memory_model 读通道直连，契约见 ddr_memory_model.sv）：
//   rd_req_addr 字节地址、rd_req_len 字节长度；tag=行号 y（raster_dma
//   内部产生）；读返回按 keep 解包（首字节在 data[7:0]，低位先），
//   非尾拍 keep=1111，尾拍 keep 标记剩余有效字节。
//   GRAY_ADDR_W 需 ≥ $clog2(Σ层像素 + cfg_ram_base 最大值)，保证
//   cfg_ram_base + y*cfg_w + x 写地址不截断。
//==============================================================================
module gray_fetch #(
    parameter ADDR_W = 32, LEN_W = 32, TAG_W = 16,
    parameter MAX_PIX = 1280*720,       // 单层最大像素（预留）
    parameter GRAY_ADDR_W = 26          // ≥ $clog2(Σ层像素 + ram_base)
) (
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   start,          // busy=0 时单拍启动
    input  wire [ADDR_W-1:0]      cfg_ddr_base,   // DDR 灰度区首字节地址
    input  wire [ADDR_W-1:0]      cfg_stride,     // DDR 行跨度（字节）
    input  wire [15:0]            cfg_w,          // 每行像素/字节
    input  wire [15:0]            cfg_h,          // 行数
    input  wire [GRAY_ADDR_W-1:0] cfg_ram_base,   // 片上 gray RAM 首地址（字节/像素）
    output reg                    busy,
    output reg                    done,           // 电平保持（下次 start 清零）
    output reg  [1:0]             status,         // 01=成功 10=读错误
    // 片上 gray RAM 写口（同步：wr_en 拍写入，1 拍完成，无背压）
    output reg                    gray_wr_en,
    output reg  [GRAY_ADDR_W-1:0] gray_wr_addr,
    output reg  [7:0]             gray_wr_data,
    // DDR 读事务（直连 ddr_memory_model 读通道）
    output reg                    rd_req_valid,
    input  wire                   rd_req_ready,
    output reg  [ADDR_W-1:0]      rd_req_addr,
    output reg  [LEN_W-1:0]       rd_req_len,
    output reg  [TAG_W-1:0]       rd_req_tag,
    input  wire                   rd_ret_valid,
    output wire                   rd_ret_ready,
    input  wire [31:0]            rd_ret_data,
    input  wire [3:0]             rd_ret_keep,
    input  wire [TAG_W-1:0]       rd_ret_tag,
    input  wire                   rd_ret_last,
    input  wire                   rd_ret_error
);

    //--------------------------------------------------------------------
    // 内部 raster_dma（读方向）
    //--------------------------------------------------------------------
    wire        dma_out_valid, dma_out_ready;
    wire [7:0]  dma_out_byte;
    wire        dma_busy, dma_done;
    wire [1:0]  dma_status;

    wire        dma_rd_req_valid, dma_rd_req_ready;
    wire [ADDR_W-1:0] dma_rd_req_addr;
    wire [LEN_W-1:0]  dma_rd_req_len;
    wire [TAG_W-1:0]  dma_rd_req_tag;
    wire        dma_rd_ret_valid, dma_rd_ret_ready;
    wire [31:0] dma_rd_ret_data;
    wire [3:0]  dma_rd_ret_keep;
    wire [TAG_W-1:0] dma_rd_ret_tag;
    wire        dma_rd_ret_last, dma_rd_ret_error;

    // start 脉冲：busy=0 时本模块 start 当拍转发（单拍）
    wire dma_start = start && !busy;

    raster_dma #(
        .ADDR_W (ADDR_W), .LEN_W (LEN_W), .TAG_W (TAG_W)
    ) u_dma (
        .clk (clk), .rst_n (rst_n),
        .start (dma_start), .busy (dma_busy), .done (dma_done), .status (dma_status),
        .cfg_base (cfg_ddr_base), .cfg_stride (cfg_stride),
        .cfg_row_bytes (cfg_w), .cfg_rows (cfg_h),
        .cfg_offset ({ADDR_W{1'b0}}), .cfg_dir (1'b0),
        .out_valid (dma_out_valid), .out_ready (dma_out_ready), .out_byte (dma_out_byte),
        .in_valid (1'b0), .in_ready (), .in_byte (8'd0),
        .rd_req_valid (dma_rd_req_valid), .rd_req_ready (dma_rd_req_ready),
        .rd_req_addr (dma_rd_req_addr), .rd_req_len (dma_rd_req_len), .rd_req_tag (dma_rd_req_tag),
        .rd_ret_valid (dma_rd_ret_valid), .rd_ret_ready (dma_rd_ret_ready),
        .rd_ret_data (dma_rd_ret_data), .rd_ret_keep (dma_rd_ret_keep),
        .rd_ret_tag (dma_rd_ret_tag), .rd_ret_last (dma_rd_ret_last), .rd_ret_error (dma_rd_ret_error),
        .wr_req_valid (), .wr_req_ready (1'b0), .wr_req_addr (), .wr_req_len (), .wr_req_tag (),
        .wr_dat_valid (), .wr_dat_ready (1'b0), .wr_dat_data (), .wr_dat_keep (), .wr_dat_last (),
        .wr_done_valid (1'b0), .wr_done_ready (), .wr_done_tag (16'd0), .wr_done_error (1'b0)
    );

    //--------------------------------------------------------------------
    // DDR 读事务透传（直连 ddr_memory_model 读通道）
    //   输出侧：内部 DMA 请求 → 本模块 rd_req_*；返回侧 ready → rd_ret_ready
    //   输入侧：本模块 rd_req_ready / rd_ret_* → 内部 DMA
    //--------------------------------------------------------------------
    always @(*) begin
        rd_req_valid = dma_rd_req_valid;
        rd_req_addr  = dma_rd_req_addr;
        rd_req_len   = dma_rd_req_len;
        rd_req_tag   = dma_rd_req_tag;
    end
    assign rd_ret_ready     = dma_rd_ret_ready;
    assign dma_rd_req_ready = rd_req_ready;
    assign dma_rd_ret_valid = rd_ret_valid;
    assign dma_rd_ret_data  = rd_ret_data;
    assign dma_rd_ret_keep  = rd_ret_keep;
    assign dma_rd_ret_tag   = rd_ret_tag;
    assign dma_rd_ret_last  = rd_ret_last;
    assign dma_rd_ret_error = rd_ret_error;

    //--------------------------------------------------------------------
    // 配置锁存 / 游标 / 沿检测寄存器（先声明后使用）
    //--------------------------------------------------------------------
    reg [ADDR_W-1:0]      cfg_ddr_base_r, cfg_stride_r;
    reg [15:0]            cfg_w_r, cfg_h_r;
    reg [GRAY_ADDR_W-1:0] cfg_ram_base_r;
    reg [31:0]            cnt_r;               // 游标 = y*cfg_w + x
    reg                   dma_done_d;          // 上一拍 dma_done（沿检测）
    wire dma_done_rise = dma_done && !dma_done_d;

    //--------------------------------------------------------------------
    // 游标计数 / gray RAM 写捕获
    //   out 流每字节即一像素（光栅序连续），cnt = y*cfg_w + x，
    //   gray_wr_addr = cfg_ram_base + cnt
    //--------------------------------------------------------------------
    assign dma_out_ready = 1'b1;               // 无背压

    always @(posedge clk) begin
        if (!rst_n) begin
            cnt_r <= 32'd0;
        end else if (start && !busy) begin
            cnt_r <= 32'd0;
        end else if (dma_out_valid) begin
            cnt_r <= cnt_r + 32'd1;
        end
    end

    // gray RAM 写口：组合跟随 out 接受拍（同步写，1 拍完成）
    always @(*) begin
        gray_wr_en   = dma_out_valid;
        gray_wr_addr = cfg_ram_base_r + cnt_r;   // 32 位中间量截断到 GRAY_ADDR_W
        gray_wr_data = dma_out_byte;
    end

    //--------------------------------------------------------------------
    // 帧状态机（start 全复位，连续帧复用）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            cfg_ddr_base_r <= {ADDR_W{1'b0}};
            cfg_stride_r   <= {ADDR_W{1'b0}};
            cfg_w_r        <= 16'd0;
            cfg_h_r        <= 16'd0;
            cfg_ram_base_r <= {GRAY_ADDR_W{1'b0}};
            busy           <= 1'b0;
            done           <= 1'b0;
            status         <= 2'b00;
            dma_done_d     <= 1'b0;
        end else if (start && !busy) begin
            cfg_ddr_base_r <= cfg_ddr_base;
            cfg_stride_r   <= cfg_stride;
            cfg_w_r        <= cfg_w;
            cfg_h_r        <= cfg_h;
            cfg_ram_base_r <= cfg_ram_base;
            busy           <= 1'b1;
            done           <= 1'b0;
            status         <= 2'b00;
            dma_done_d     <= 1'b0;
        end else begin
            dma_done_d <= dma_done;
            if (dma_done_rise) begin
                done   <= 1'b1;
                status <= dma_status;
                busy   <= 1'b0;
            end
        end
    end

endmodule
