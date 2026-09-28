`timescale 1ns / 1ps
//==============================================================================
// raster_dma.v — M7.1 行光栅 DMA（按行调度 DDR 读/写，字节流进出）
//------------------------------------------------------------------------------
// 功能：把一片行光栅区域（cfg_base 起、每行 cfg_stride 字节跨度、每行有效
//   cfg_row_bytes 字节、共 cfg_rows 行、payload 起点偏移 cfg_offset）作为
//   连续字节流搬入 / 搬出 DDR：
//     读方向（cfg_dir=0）：DDR → out_byte 字节流（行内 x 递增，行间 y 递增）
//     写方向（cfg_dir=1）：in_byte 字节流 → DDR
//
// 行调度时序：
//   逐行 y=0..cfg_rows-1 发一笔 DDR 事务（读或写），tag=y，addr =
//     cfg_base + y*cfg_stride + cfg_offset，len=cfg_row_bytes。
//   一笔在途：读方向收到该行 rd_ret_last 后才发下一行请求；
//             写方向收到该行 wr_done（完成）后才发下一行请求。
//   行间不发额外字节（每行只出/只入 cfg_row_bytes 个有效字节）。
//
// keep/解包与打包规则（与 ddr_memory_model 契约一致）：
//   读方向：模型把返回数据按"连续字节地址"打包成 32 位字（首字节在
//     data[7:0]，随后低位到高位排列），非尾拍 keep=1111，尾拍 keep 标记
//     剩余有效字节；本模块按 keep[k] 对应 data[8k+:8] 低位先逐字节
//     输出 out_byte（行内光栅序）。模型自行处理任意合法字节起点，
//     无需本模块做字对齐移位。
//   写方向：本模块把 payload 字节流按"连续字节流"打包成 32 位字：
//     事务第 i 个字节放 data[8*(i%4)+:8]（data[7:0] 为先收字节），
//     keep 对应位置 1；非尾拍整字 keep=1111，尾字 keep=剩余字节数
//     （0001/0011/0111/1111），整行写完 wr_dat_last=1。
//     注意：服务层按字节地址连续写入（mem[addr+off+k]），因此打包与
//     (addr%4) 字对齐无关——cfg_offset 非对齐起点（任意合法字节起点）
//     无需移位，addr 直接用 cfg_base+y*stride+cfg_offset。
//
// 协议纪律（ddr_memory_model PROTOCOL_CHECKS 零违规前提）：
//   写数据 keep 必须恒等于按长度期望的 keep（满字 1111 / 尾字剩余字节），
//   last 只在尾字；违反会被模型记协议违规并使该事务完成 error=1。
//
// 错误语义：
//   读方向 rd_ret_error 或写方向 wr_done_error → status=10/11，
//   中止后续行；当前在途事务排空（读收完 rd_ret_last，写收完 wr_done）
//   后置 done=1（错误也置 done，status 区分），busy=0，无事务残留。
//
// 接口时序：
//   start ：busy=0 时单拍启动，锁存 cfg_*，全状态复位（连续帧复用，
//           busy/done/status/行列计数器/tag 一并复位）。
//   done  ：电平保持（下次 start 清零）。
//   status：01=成功 10=读错误 11=写错误。
//   背压  ：读方向 out_ready=0 时 rd_ret_ready=0（模型停返回，不丢）；
//           写方向 in_valid=0 时停打（已凑齐的字 wr_dat_valid 保持）。
//==============================================================================
module raster_dma #(
    parameter ADDR_W = 32,      // 字节地址
    parameter LEN_W  = 32,
    parameter TAG_W  = 16
) (
    input  wire                  clk,
    input  wire                  rst_n,
    // 启动（busy=0 时单拍；锁存配置）
    input  wire                  start,
    input  wire [ADDR_W-1:0]     cfg_base,        // 区域首地址（字节）
    input  wire [ADDR_W-1:0]     cfg_stride,      // 行跨度（字节）
    input  wire [15:0]           cfg_row_bytes,   // 每行有效字节数（>0）
    input  wire [15:0]           cfg_rows,        // 行数（>0）
    input  wire [ADDR_W-1:0]     cfg_offset,      // 每行 payload 起点偏移（相对 base+y*stride）
    input  wire                  cfg_dir,         // 0=读（DDR→字节流出） 1=写（字节流入→DDR）
    output reg                   busy,
    output reg                   done,            // 电平保持（下次 start 清零）
    output reg  [1:0]            status,          // 01=成功 10=读错误 11=写错误
    // 读方向字节流输出（DDR 返回 → 客户端；光栅序：y 行内 x 递增，行间 y 递增）
    output reg                   out_valid,
    input  wire                  out_ready,
    output reg  [7:0]            out_byte,
    // 写方向字节流输入（客户端 → DDR；同光栅序）
    input  wire                  in_valid,
    output wire                  in_ready,
    input  wire [7:0]            in_byte,
    // ---- DDR 读事务 ----
    output reg                   rd_req_valid,
    input  wire                  rd_req_ready,
    output reg  [ADDR_W-1:0]     rd_req_addr,
    output reg  [LEN_W-1:0]      rd_req_len,
    output reg  [TAG_W-1:0]      rd_req_tag,
    input  wire                  rd_ret_valid,
    output wire                  rd_ret_ready,
    input  wire [31:0]           rd_ret_data,
    input  wire [3:0]            rd_ret_keep,
    input  wire [TAG_W-1:0]      rd_ret_tag,
    input  wire                  rd_ret_last,
    input  wire                  rd_ret_error,
    // ---- DDR 写事务 ----
    output reg                   wr_req_valid,
    input  wire                  wr_req_ready,
    output reg  [ADDR_W-1:0]     wr_req_addr,
    output reg  [LEN_W-1:0]      wr_req_len,
    output reg  [TAG_W-1:0]      wr_req_tag,
    output reg                   wr_dat_valid,
    input  wire                  wr_dat_ready,
    output reg  [31:0]           wr_dat_data,
    output reg  [3:0]            wr_dat_keep,
    output reg                   wr_dat_last,
    input  wire                  wr_done_valid,
    output wire                  wr_done_ready,
    input  wire [TAG_W-1:0]      wr_done_tag,
    input  wire                  wr_done_error
);

    //--------------------------------------------------------------------
    // 配置锁存（start 时采样）
    //--------------------------------------------------------------------
    reg [ADDR_W-1:0] cfg_base_r, cfg_stride_r, cfg_offset_r;
    reg [15:0]       cfg_row_bytes_r, cfg_rows_r;
    reg              cfg_dir_r;

    always @(posedge clk) begin
        if (!rst_n) begin
            cfg_base_r      <= {ADDR_W{1'b0}};
            cfg_stride_r    <= {ADDR_W{1'b0}};
            cfg_offset_r    <= {ADDR_W{1'b0}};
            cfg_row_bytes_r <= 16'd0;
            cfg_rows_r      <= 16'd0;
            cfg_dir_r       <= 1'b0;
        end else if (start && !busy) begin
            cfg_base_r      <= cfg_base;
            cfg_stride_r    <= cfg_stride;
            cfg_offset_r    <= cfg_offset;
            cfg_row_bytes_r <= cfg_row_bytes;
            cfg_rows_r      <= cfg_rows;
            cfg_dir_r       <= cfg_dir;
        end
    end

    //--------------------------------------------------------------------
    // 行调度计数 / 行起始地址（48 位中间量，避免 y*stride 截断）
    //--------------------------------------------------------------------
    reg [15:0] y_r;   // 当前行号（读、写共用，start 清零）

    wire [ADDR_W+15:0] row_addr_full = {16'd0, cfg_base_r} +
                                       ({16'd0, y_r} * cfg_stride_r) +
                                       {16'd0, cfg_offset_r};
    wire [ADDR_W-1:0] row_addr = row_addr_full[ADDR_W-1:0];

    //--------------------------------------------------------------------
    // 读方向状态机（cfg_dir=0 时运行）
    //   RD_IDLE  ：无在途读，可发起下一行请求
    //   RD_STREAM：等待/接收一个 rd_ret 返回拍
    //   RD_UNPACK：按 keep 逐字节输出（out_ready 反压，保持不丢）
    //--------------------------------------------------------------------
    localparam [1:0] RD_IDLE = 2'd0, RD_STREAM = 2'd1, RD_UNPACK = 2'd2;
    reg [1:0]  rd_state;
    reg [31:0] rd_buf;       // 当前返回拍锁存
    reg [3:0]  rd_buf_keep;
    reg [1:0]  rd_ptr;       // 解包指针 0..3
    reg        rd_last_r;    // 当前返回拍是否行尾
    reg        rd_err_latch; // 本帧内收到过 rd_ret_error

    // 请求/返回握手（rd_ret_ready 为 wire 输出，可连续赋值）
    assign rd_ret_ready = (rd_state == RD_STREAM);

    // rd_req_* 为 output reg，由组合块驱动
    always @(*) begin
        rd_req_valid = (rd_state == RD_IDLE) && busy && !cfg_dir_r &&
                       (y_r < cfg_rows_r) && !rd_err_latch;
        rd_req_addr  = row_addr;
        rd_req_len   = {16'd0, cfg_row_bytes_r};
        rd_req_tag   = y_r;
    end

    // 当前返回字最后一个字节吐完且为行尾 → 行完成（组合，供行调度 always）
    wire rd_last_beat_done = (rd_state == RD_UNPACK) && out_valid && out_ready &&
                             ((rd_ptr == 2'd3) || !rd_buf_keep[rd_ptr + 2'd1]) &&
                             rd_last_r;

    always @(posedge clk) begin
        if (!rst_n) begin
            rd_state     <= RD_IDLE;
            rd_buf       <= 32'd0;
            rd_buf_keep  <= 4'b0;
            rd_ptr       <= 2'd0;
            rd_last_r    <= 1'b0;
            rd_err_latch <= 1'b0;
        end else if (start && !busy) begin
            rd_state     <= RD_IDLE;
            rd_buf       <= 32'd0;
            rd_buf_keep  <= 4'b0;
            rd_ptr       <= 2'd0;
            rd_last_r    <= 1'b0;
            rd_err_latch <= 1'b0;
        end else begin
            case (rd_state)
                RD_IDLE: begin
                    if (rd_req_valid && rd_req_ready)
                        rd_state <= RD_STREAM;
                end
                RD_STREAM: begin
                    if (rd_ret_valid && rd_ret_ready) begin
                        if (rd_ret_error)
                            rd_err_latch <= 1'b1;
                        rd_buf       <= rd_ret_data;
                        rd_buf_keep  <= rd_ret_keep;
                        rd_last_r    <= rd_ret_last;
                        rd_ptr       <= 2'd0;
                        rd_state     <= RD_UNPACK;
                    end
                end
                RD_UNPACK: begin
                    if (out_valid && out_ready) begin
                        if ((rd_ptr == 2'd3) || !rd_buf_keep[rd_ptr + 2'd1]) begin
                            // 本返回字吐完
                            rd_ptr <= 2'd0;
                            if (rd_last_r) begin
                                rd_state  <= RD_IDLE;
                                rd_last_r <= 1'b0;
                            end else begin
                                rd_state <= RD_STREAM;
                            end
                        end else begin
                            rd_ptr <= rd_ptr + 2'd1;
                        end
                    end
                end
                default: rd_state <= RD_IDLE;
            endcase
        end
    end

    // out 组合输出（reg 由组合块驱动）
    always @(*) begin
        if (rd_state == RD_UNPACK) begin
            out_valid = 1'b1;
            out_byte  = rd_buf[8*rd_ptr +: 8];
        end else begin
            out_valid = 1'b0;
            out_byte  = 8'd0;
        end
    end

    //--------------------------------------------------------------------
    // 写方向状态机（cfg_dir=1 时运行）
    //   WR_IDLE     ：无在途写，可发起下一行请求
    //   WR_COLLECT  ：消费 in 字节流，凑满 32 位字（或行尾字节）
    //   WR_EMIT     ：已凑齐一个字，等待 wr_dat_ready 送出
    //   WR_DONE_WAIT：行数据发完，等待该行 wr_done
    //--------------------------------------------------------------------
    localparam [1:0] WR_IDLE = 2'd0, WR_COLLECT = 2'd1,
                     WR_EMIT = 2'd2, WR_DONE_WAIT = 2'd3;
    reg [1:0]  wr_state;
    reg [15:0] wcnt_r;   // 本行已接收字节数
    reg [31:0] wbuf_r;   // 当前字字节缓冲
    reg [3:0]  wmask_r;  // 当前字 keep

    // wr 侧握手（in_ready / wr_done_ready 为 wire 输出，可连续赋值）
    assign in_ready      = (wr_state == WR_COLLECT);
    assign wr_done_ready = (wr_state == WR_DONE_WAIT);

    wire [1:0] wpos       = wcnt_r[1:0];                 // 下一字节在字内位置
    wire       wr_dat_last_w = (wcnt_r == cfg_row_bytes_r); // 本行已收满

    // wr_req_* / wr_dat_* 为 output reg，由组合块驱动
    always @(*) begin
        wr_req_valid = (wr_state == WR_IDLE) && busy && cfg_dir_r &&
                       (y_r < cfg_rows_r);
        wr_req_addr  = row_addr;
        wr_req_len   = {16'd0, cfg_row_bytes_r};
        wr_req_tag   = y_r;
        wr_dat_valid = (wr_state == WR_EMIT);
        wr_dat_data  = wbuf_r;
        wr_dat_keep  = wmask_r;
        wr_dat_last  = wr_dat_last_w;
    end

    // 行写完成/中止（组合，供行调度 always）：cplt 握手拍
    wire wr_row_done = (wr_state == WR_DONE_WAIT) && wr_done_valid && wr_done_ready;

    always @(posedge clk) begin
        if (!rst_n) begin
            wr_state <= WR_IDLE;
            wcnt_r   <= 16'd0;
            wbuf_r   <= 32'd0;
            wmask_r  <= 4'b0;
        end else if (start && !busy) begin
            wr_state <= WR_IDLE;
            wcnt_r   <= 16'd0;
            wbuf_r   <= 32'd0;
            wmask_r  <= 4'b0;
        end else begin
            case (wr_state)
                WR_IDLE: begin
                    if (wr_req_valid && wr_req_ready) begin
                        wcnt_r  <= 16'd0;
                        wbuf_r  <= 32'd0;
                        wmask_r <= 4'b0;
                        wr_state <= WR_COLLECT;
                    end
                end
                WR_COLLECT: begin
                    if (in_valid && in_ready) begin
                        // 收满 4 字节（wpos==3）或收满整行（wcnt==row_bytes-1）→ 可发字
                        wbuf_r[8*wpos +: 8] <= in_byte;
                        wmask_r[wpos]        <= 1'b1;
                        wcnt_r               <= wcnt_r + 16'd1;
                        if ((wpos == 2'd3) || (wcnt_r == (cfg_row_bytes_r - 16'd1)))
                            wr_state <= WR_EMIT;
                    end
                end
                WR_EMIT: begin
                    if (wr_dat_valid && wr_dat_ready) begin
                        wmask_r <= 4'b0;
                        if (wr_dat_last_w)
                            wr_state <= WR_DONE_WAIT;
                        else
                            wr_state <= WR_COLLECT;
                    end
                end
                WR_DONE_WAIT: begin
                    if (wr_done_valid && wr_done_ready)
                        wr_state <= WR_IDLE;
                end
                default: wr_state <= WR_IDLE;
            endcase
        end
    end

    //--------------------------------------------------------------------
    // 行调度 / 帧完成（读、写共用行计数器 y_r；错误中止也走这里）
    //   行完成（读：last 返回字吐完 / 写：wr_done 握手）当拍推进 y 或置完成。
    //   错误时：读侧 rd_err_latch 早已置位（流中途），写侧直接用握手拍的
    //   wr_done_error 组合值，因此完成拍即能正确区分 status。
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            y_r    <= 16'd0;
            busy   <= 1'b0;
            done   <= 1'b0;
            status <= 2'b00;
        end else if (start && !busy) begin
            y_r    <= 16'd0;
            busy   <= 1'b1;
            done   <= 1'b0;
            status <= 2'b00;
        end else if (rd_last_beat_done || wr_row_done) begin
            if (rd_err_latch || (wr_row_done && wr_done_error)) begin
                // 读错误 / 写错误：中止，置 done（status 区分）
                done   <= 1'b1;
                status <= rd_err_latch ? 2'b10 : 2'b11;
                busy   <= 1'b0;
            end else if (y_r == (cfg_rows_r - 16'd1)) begin
                done   <= 1'b1;
                status <= 2'b01;
                busy   <= 1'b0;
            end else begin
                y_r <= y_r + 16'd1;
            end
        end
    end

endmodule
