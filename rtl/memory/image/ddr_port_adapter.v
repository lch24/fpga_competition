`timescale 1ns / 1ps
//==============================================================================
// ddr_port_adapter.v — §3.2 抽象事务 ↔ ddr_memory_model 端口薄封装（M7.1 基础件）
//------------------------------------------------------------------------------
// 用途：把算法侧的"§3.2 抽象事务接口"（读请求/读返回、写请求/写数据/写完成）
//       一对一映射到 ddr_memory_model.sv 的端口上，是未来真实 DDR 服务层
//       控制器（强文韬侧）的替换对接点：届时仅需将本模块的 m_* 端口改接到真实
//       控制器端口，客户端侧接口不变。
//
// 与 §3.2 表 / 模型的对应关系（客户端侧名 → 模型侧名，全部直连）：
//   读请求   rd_req_valid/addr/len/tag      → m_rd_req_valid/addr/len_bytes/tag
//   读返回   rd_ret_valid/data/keep/tag/    ← m_rd_ret_valid/data/keep/tag/
//             last/error                      last/error
//   写请求   wr_req_valid/addr/len/tag      → m_wr_req_valid/addr/len_bytes/tag
//   写数据   wr_dat_valid/data/keep/last    → m_wr_dat_valid/data/keep/last
//   写完成   wr_done_valid/tag/error        ← m_wr_cplt_valid/tag/error
//   （模型端口名 rd_req_len_bytes / wr_cplt_* 与 §3.2 命名不同，由本模块吸收）
//
// 在途语义（契约第3条：每客户端一笔读、一笔写在途，读写通道独立并行）：
//   - 客户端在读/写"在途"期间再次拉高对应请求 valid 时，本模块拉低 ready 阻塞
//     该请求（不吞、不产生协议违规），待当前事务完成后再接受。
//   - 客户端在读/写在途期间送写数据（wr_dat_valid），数据 valid 被门控屏蔽、
//     ready 拉低，模型不会看到"无在途写请求的写数据"（零协议违规）。
//
// tag 策略：
//   - 请求握手时在适配器内锁存 tag（rd_tag_r / wr_tag_r），返回/完成通道回带
//     锁存值。模型直通语义下等价于透传，但保留寄存器便于未来乱序路由扩展
//     （届时改为按 tag 查表回带，客户端侧不变）。
//
// 错误与零长处理：
//   - 错误透明传递：m_rd_ret_error / m_wr_cplt_error 直通客户端（不做重试，
//     是否重试由上层决定）。
//   - len==0（契约要求 len>0）：请求在客户端侧正常握手（回 ready=1），但**不**
//     发给模型（m_*_req_valid 拉低），直接回一拍 error=1 的伪返回/伪完成
//     （读：data=0、keep=0、last=1；写：仅完成通道 error=1），使上层可感知
//     自身长度违规，同时保证模型零协议违规。
//
// 复位：rst_n 低有效（同步释放，建议来自 reset_sync）；复位后两侧在途清空。
//==============================================================================
module ddr_port_adapter #(
    parameter ADDR_W = 32,
    parameter LEN_W  = 32,
    parameter TAG_W  = 16
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // ---- 客户端侧：读（与 §3.2 表同构，语义更严：一笔在读强制） ----
    input  wire                  rd_req_valid,
    output wire                  rd_req_ready,
    input  wire [ADDR_W-1:0]     rd_req_addr,
    input  wire [LEN_W-1:0]      rd_req_len,
    input  wire [TAG_W-1:0]      rd_req_tag,
    output wire                  rd_ret_valid,
    input  wire                  rd_ret_ready,
    output wire [31:0]           rd_ret_data,
    output wire [3:0]            rd_ret_keep,
    output wire [TAG_W-1:0]      rd_ret_tag,
    output wire                  rd_ret_last,
    output wire                  rd_ret_error,

    // ---- 客户端侧：写 ----
    input  wire                  wr_req_valid,
    output wire                  wr_req_ready,
    input  wire [ADDR_W-1:0]     wr_req_addr,
    input  wire [LEN_W-1:0]      wr_req_len,
    input  wire [TAG_W-1:0]      wr_req_tag,
    input  wire                  wr_dat_valid,
    output wire                  wr_dat_ready,
    input  wire [31:0]           wr_dat_data,
    input  wire [3:0]            wr_dat_keep,
    input  wire                  wr_dat_last,
    output wire                  wr_done_valid,
    input  wire                  wr_done_ready,
    output wire [TAG_W-1:0]      wr_done_tag,
    output wire                  wr_done_error,

    // ---- 模型侧：读（m_ 前缀，直连 ddr_memory_model 端口） ----
    output wire                  m_rd_req_valid,
    input  wire                  m_rd_req_ready,
    output wire [ADDR_W-1:0]     m_rd_req_addr,
    output wire [LEN_W-1:0]      m_rd_req_len_bytes,
    output wire [TAG_W-1:0]      m_rd_req_tag,
    input  wire                  m_rd_ret_valid,
    output wire                  m_rd_ret_ready,
    input  wire [31:0]           m_rd_ret_data,
    input  wire [3:0]            m_rd_ret_keep,
    input  wire [TAG_W-1:0]      m_rd_ret_tag,
    input  wire                  m_rd_ret_last,
    input  wire                  m_rd_ret_error,

    // ---- 模型侧：写 ----
    output wire                  m_wr_req_valid,
    input  wire                  m_wr_req_ready,
    output wire [ADDR_W-1:0]     m_wr_req_addr,
    output wire [LEN_W-1:0]      m_wr_req_len_bytes,
    output wire [TAG_W-1:0]      m_wr_req_tag,
    output wire                  m_wr_dat_valid,
    input  wire                  m_wr_dat_ready,
    output wire [31:0]           m_wr_dat_data,
    output wire [3:0]            m_wr_dat_keep,
    output wire                  m_wr_dat_last,
    input  wire                  m_wr_cplt_valid,
    output wire                  m_wr_cplt_ready,
    input  wire [TAG_W-1:0]      m_wr_cplt_tag,
    input  wire                  m_wr_cplt_error
);

    //--------------------------------------------------------------------
    // 读通道状态：IDLE（无在读）/ ACTIVE（模型在读，返回流直通）/
    //            ZERO（len==0 伪返回，等待客户端接收）
    //--------------------------------------------------------------------
    localparam [1:0] RD_IDLE   = 2'd0,
                     RD_ACTIVE = 2'd1,
                     RD_ZERO   = 2'd2;

    reg [1:0]   rd_state;
    reg [TAG_W-1:0] rd_tag_r;

    // 请求握手：仅空闲且模型可收时接受（len==0 也在空闲接受，但不发模型）
    assign rd_req_ready   = (rd_state == RD_IDLE) && ((rd_req_len == 0) || m_rd_req_ready);
    assign m_rd_req_valid = (rd_state == RD_IDLE) && rd_req_valid && (rd_req_len != {LEN_W{1'b0}});
    assign m_rd_req_addr  = rd_req_addr;
    assign m_rd_req_len_bytes = rd_req_len;
    assign m_rd_req_tag   = rd_req_tag;

    // 读返回：模型流直通（ACTIVE）；len==0 伪返回（ZERO）
    assign rd_ret_valid = (rd_state == RD_ACTIVE) ? m_rd_ret_valid : (rd_state == RD_ZERO);
    assign rd_ret_data  = (rd_state == RD_ZERO) ? 32'd0 : m_rd_ret_data;
    assign rd_ret_keep  = (rd_state == RD_ZERO) ? 4'b0000 : m_rd_ret_keep;
    assign rd_ret_last  = (rd_state == RD_ZERO) ? 1'b1 : m_rd_ret_last;
    assign rd_ret_error = (rd_state == RD_ZERO) ? 1'b1 : m_rd_ret_error;
    assign rd_ret_tag   = rd_tag_r;
    assign m_rd_ret_ready = rd_ret_ready;

    always @(posedge clk) begin
        if (!rst_n) begin
            rd_state <= RD_IDLE;
            rd_tag_r <= {TAG_W{1'b0}};
        end else begin
            case (rd_state)
                RD_IDLE: begin
                    if (rd_req_valid && rd_req_ready) begin
                        rd_tag_r <= rd_req_tag;
                        if (rd_req_len == {LEN_W{1'b0}})
                            rd_state <= RD_ZERO;   // 零长：不发给模型，伪返回
                        else
                            rd_state <= RD_ACTIVE;
                    end
                end

                RD_ACTIVE: begin
                    if (m_rd_ret_valid && m_rd_ret_ready && m_rd_ret_last)
                        rd_state <= RD_IDLE;
                end

                RD_ZERO: begin
                    if (rd_ret_valid && rd_ret_ready)
                        rd_state <= RD_IDLE;
                end

                default: rd_state <= RD_IDLE;
            endcase
        end
    end

    //--------------------------------------------------------------------
    // 写通道状态：IDLE（无在写）/ ACTIVE（模型在写，数据/完成直通）/
    //            ZERO（len==0 伪完成）
    //--------------------------------------------------------------------
    localparam [1:0] WR_IDLE   = 2'd0,
                     WR_ACTIVE = 2'd1,
                     WR_ZERO   = 2'd2;

    reg [1:0]   wr_state;
    reg [TAG_W-1:0] wr_tag_r;

    assign wr_req_ready   = (wr_state == WR_IDLE) && ((wr_req_len == 0) || m_wr_req_ready);
    assign m_wr_req_valid = (wr_state == WR_IDLE) && wr_req_valid && (wr_req_len != {LEN_W{1'b0}});
    assign m_wr_req_addr  = wr_req_addr;
    assign m_wr_req_len_bytes = wr_req_len;
    assign m_wr_req_tag   = wr_req_tag;

    // 写数据：仅在模型已接受写请求（ACTIVE）时透传；无在途写时 valid 屏蔽 +
    // ready 拉低（客户端违反不产生协议违规）
    assign wr_dat_ready   = m_wr_dat_ready && (wr_state == WR_ACTIVE);
    assign m_wr_dat_valid = wr_dat_valid && (wr_state == WR_ACTIVE);
    assign m_wr_dat_data  = wr_dat_data;
    assign m_wr_dat_keep  = wr_dat_keep;
    assign m_wr_dat_last  = wr_dat_last;

    // 写完成：模型流直通（ACTIVE）；len==0 伪完成（ZERO）
    assign wr_done_valid = (wr_state == WR_ACTIVE) ? m_wr_cplt_valid : (wr_state == WR_ZERO);
    assign wr_done_tag   = wr_tag_r;
    assign wr_done_error = (wr_state == WR_ZERO) ? 1'b1 : m_wr_cplt_error;
    assign m_wr_cplt_ready = wr_done_ready;

    always @(posedge clk) begin
        if (!rst_n) begin
            wr_state <= WR_IDLE;
            wr_tag_r <= {TAG_W{1'b0}};
        end else begin
            case (wr_state)
                WR_IDLE: begin
                    if (wr_req_valid && wr_req_ready) begin
                        wr_tag_r <= wr_req_tag;
                        if (wr_req_len == {LEN_W{1'b0}})
                            wr_state <= WR_ZERO;   // 零长：不发给模型，伪完成
                        else
                            wr_state <= WR_ACTIVE;
                    end
                end

                WR_ACTIVE: begin
                    if (m_wr_cplt_valid && m_wr_cplt_ready)
                        wr_state <= WR_IDLE;
                end

                WR_ZERO: begin
                    if (wr_done_valid && wr_done_ready)
                        wr_state <= WR_IDLE;
                end

                default: wr_state <= WR_IDLE;
            endcase
        end
    end

endmodule
