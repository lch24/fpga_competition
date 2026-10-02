`timescale 1ns / 1ps
//==============================================================================
// ddr_port_arbiter.v — DDR 多客户端读写仲裁（M8 基础件）
//------------------------------------------------------------------------------
// 用途：把多个 DMA 客户端（raster_dma / gray_fetch / resp_ddr_writer 等）的
//       §3.2 抽象事务接口收敛为单一 DDR 端口，复用 ddr_port_adapter /
//       ddr_memory_model 的一组模型侧端口：同一时刻仅一笔读、一笔写在途，
//       读写两通道相互独立并行。
//
// 仲裁策略（round-robin）：
//   - 读/写通道各自独立仲裁（独立状态机、独立轮转指针），互不干扰；
//     同一客户端可同时占读槽与写槽。
//   - 空闲时从"有请求 valid 的客户端"中自轮转指针起循环扫描，选中第一个
//     有请求的客户端；指针在每次释放（事务完成）后轮转到下一客户端，
//     保证公平、无饿死。
//   - 被选客户端独占模型侧通道直至释放：读返回/写数据/写完成只回/只收
//     被选客户端，其余客户端 ready 恒 0（数据 valid 屏蔽，模型零协议违规）。
//   - 读释放 = 该客户端 rd_ret_last 被接受；写释放 = 该客户端写完成被接受。
//
// 在途单笔语义（镜像 ddr_port_adapter）：
//   - 客户端在途期间再次拉高请求 valid：ready 拉低阻塞（不吞请求、不产生
//     协议违规），当前事务完成后按轮转重新接受。
//   - 客户端未获准/不在途时送写数据：m_wr_dat_valid 屏蔽 + ready 拉低。
//
// 零长（len==0）：
//   - 请求在客户端侧正常握手（ready 与正常事务同源），但不发给模型
//     （m_*_req_valid 拉低），本地回一拍伪返回/伪完成
//     （读：data=0、keep=0、last=1、error=1；写：仅完成通道 error=1），
//     模型零协议违规，上层可感知自身长度违规。
//==============================================================================
module ddr_port_arbiter #(
    parameter ADDR_W = 32,
    parameter LEN_W  = 32,
    parameter TAG_W  = 16,
    parameter N_RD   = 2,      // 读客户端数
    parameter N_WR   = 2       // 写客户端数
) (
    input  wire clk, rst_n,
    // ---- 客户端侧（端口数组，方向同 ddr_port_adapter 客户端侧）----
    input  wire rd_req_valid  [0:N_RD-1],
    output wire rd_req_ready  [0:N_RD-1],
    input  wire [ADDR_W-1:0] rd_req_addr [0:N_RD-1],
    input  wire [LEN_W-1:0]  rd_req_len  [0:N_RD-1],
    input  wire [TAG_W-1:0]  rd_req_tag  [0:N_RD-1],
    output wire rd_ret_valid [0:N_RD-1],
    input  wire rd_ret_ready [0:N_RD-1],
    output wire [31:0]       rd_ret_data [0:N_RD-1],
    output wire [3:0]        rd_ret_keep [0:N_RD-1],
    output wire [TAG_W-1:0]  rd_ret_tag  [0:N_RD-1],
    output wire rd_ret_last  [0:N_RD-1],
    output wire rd_ret_error [0:N_RD-1],
    input  wire wr_req_valid  [0:N_WR-1],
    output wire wr_req_ready  [0:N_WR-1],
    input  wire [ADDR_W-1:0] wr_req_addr [0:N_WR-1],
    input  wire [LEN_W-1:0]  wr_req_len  [0:N_WR-1],
    input  wire [TAG_W-1:0]  wr_req_tag  [0:N_WR-1],
    input  wire wr_dat_valid [0:N_WR-1],
    output wire wr_dat_ready [0:N_WR-1],
    input  wire [31:0]       wr_dat_data [0:N_WR-1],
    input  wire [3:0]        wr_dat_keep [0:N_WR-1],
    input  wire wr_dat_last  [0:N_WR-1],
    output wire wr_done_valid [0:N_WR-1],
    input  wire wr_done_ready [0:N_WR-1],
    output wire [TAG_W-1:0]   wr_done_tag  [0:N_WR-1],
    output wire wr_done_error [0:N_WR-1],
    // ---- 模型侧（单组，方向同 ddr_memory_model / ddr_port_adapter m_*）----
    output wire m_rd_req_valid,
    input  wire m_rd_req_ready,
    output wire [ADDR_W-1:0] m_rd_req_addr,
    output wire [LEN_W-1:0]  m_rd_req_len_bytes,
    output wire [TAG_W-1:0]  m_rd_req_tag,
    input  wire m_rd_ret_valid,
    output wire m_rd_ret_ready,
    input  wire [31:0]       m_rd_ret_data,
    input  wire [3:0]        m_rd_ret_keep,
    input  wire [TAG_W-1:0]  m_rd_ret_tag,
    input  wire m_rd_ret_last,
    input  wire m_rd_ret_error,
    output wire m_wr_req_valid,
    input  wire m_wr_req_ready,
    output wire [ADDR_W-1:0] m_wr_req_addr,
    output wire [LEN_W-1:0]  m_wr_req_len_bytes,
    output wire [TAG_W-1:0]  m_wr_req_tag,
    output wire m_wr_dat_valid,
    input  wire m_wr_dat_ready,
    output wire [31:0]       m_wr_dat_data,
    output wire [3:0]        m_wr_dat_keep,
    output wire m_wr_dat_last,
    input  wire m_wr_cplt_valid,
    output wire m_wr_cplt_ready,
    input  wire [TAG_W-1:0]  m_wr_cplt_tag,
    input  wire m_wr_cplt_error
);

    localparam RD_IW = (N_RD > 1) ? $clog2(N_RD) : 1;
    localparam WR_IW = (N_WR > 1) ? $clog2(N_WR) : 1;

    //====================================================================
    // 读通道仲裁
    //   RD_IDLE   : 无在读，round-robin 选客户端并发请求
    //   RD_ACTIVE : 模型在读，返回流直通被选客户端
    //   RD_ZERO   : len==0 伪返回，等待被选客户端接收
    //====================================================================
    localparam [1:0] RD_IDLE   = 2'd0,
                     RD_ACTIVE = 2'd1,
                     RD_ZERO   = 2'd2;

    reg [1:0]       rd_state;
    reg [RD_IW-1:0] rd_ptr, rd_cur;
    reg [TAG_W-1:0] rd_tag_r;

    // round-robin 选通：自 rd_ptr 起循环扫描第一个有请求的客户端
    reg [N_RD-1:0] rd_req_rot, rd_grant_rot, rd_grant;
    integer rk;
    always @(*) begin
        for (rk = 0; rk < N_RD; rk = rk + 1)
            rd_req_rot[rk] = rd_req_valid[(rk + rd_ptr) % N_RD];
        rd_grant_rot = {N_RD{1'b0}};
        for (rk = 0; rk < N_RD; rk = rk + 1)
            if (rd_req_rot[rk] && (rd_grant_rot == {N_RD{1'b0}}))
                rd_grant_rot[rk] = 1'b1;
        rd_grant = {N_RD{1'b0}};
        for (rk = 0; rk < N_RD; rk = rk + 1)
            if (rd_grant_rot[rk])
                rd_grant[(rk + rd_ptr) % N_RD] = 1'b1;
    end

    wire rd_sel_valid = |rd_grant;
    reg [RD_IW-1:0] rd_sel;
    integer rsi;
    always @(*) begin
        rd_sel = {RD_IW{1'b0}};
        for (rsi = 0; rsi < N_RD; rsi = rsi + 1)
            if (rd_grant[rsi])
                rd_sel = rsi[RD_IW-1:0];
    end

    // 请求握手：仅空闲；被选客户端 ready 直通模型，其余 0；
    // m_rd_req_valid 仅在被选客户端请求且模型就绪时拉高（len==0 不发模型）
    genvar rg;
    generate
        for (rg = 0; rg < N_RD; rg = rg + 1) begin : rd_req_ready_gen
            assign rd_req_ready[rg] = (rd_state == RD_IDLE) && rd_grant[rg] && m_rd_req_ready;
        end
    endgenerate

    assign m_rd_req_valid     = (rd_state == RD_IDLE) && rd_sel_valid && m_rd_req_ready &&
                                (rd_req_len[rd_sel] != {LEN_W{1'b0}});
    assign m_rd_req_addr      = rd_req_addr[rd_sel];
    assign m_rd_req_len_bytes = rd_req_len[rd_sel];
    assign m_rd_req_tag       = rd_req_tag[rd_sel];

    // 返回通道：只回被选客户端（ACTIVE：模型流直通；ZERO：len==0 伪返回）
    assign m_rd_ret_ready = (rd_state == RD_ACTIVE) && rd_ret_ready[rd_cur];

    generate
        for (rg = 0; rg < N_RD; rg = rg + 1) begin : rd_ret_gen
            wire sel = (rd_state != RD_IDLE) && (rg == rd_cur);
            assign rd_ret_valid[rg] = sel && ((rd_state == RD_ACTIVE) ? m_rd_ret_valid : 1'b1);
            assign rd_ret_data[rg]  = sel ? ((rd_state == RD_ZERO) ? 32'd0 : m_rd_ret_data) : 32'd0;
            assign rd_ret_keep[rg]  = sel ? ((rd_state == RD_ZERO) ? 4'b0000 : m_rd_ret_keep) : 4'b0000;
            assign rd_ret_last[rg]  = sel && ((rd_state == RD_ZERO) ? 1'b1 : m_rd_ret_last);
            assign rd_ret_error[rg] = sel && ((rd_state == RD_ZERO) ? 1'b1 : m_rd_ret_error);
            assign rd_ret_tag[rg]   = sel ? rd_tag_r : {TAG_W{1'b0}};
        end
    endgenerate

    always @(posedge clk) begin
        if (!rst_n) begin
            rd_state <= RD_IDLE;
            rd_ptr   <= {RD_IW{1'b0}};
            rd_cur   <= {RD_IW{1'b0}};
            rd_tag_r <= {TAG_W{1'b0}};
        end else begin
            case (rd_state)
                RD_IDLE: begin
                    if (rd_sel_valid && m_rd_req_ready) begin
                        rd_cur   <= rd_sel;
                        rd_tag_r <= rd_req_tag[rd_sel];
                        if (rd_req_len[rd_sel] == {LEN_W{1'b0}})
                            rd_state <= RD_ZERO;   // 零长：不发给模型，伪返回
                        else
                            rd_state <= RD_ACTIVE;
                    end
                end

                RD_ACTIVE: begin
                    if (m_rd_ret_valid && m_rd_ret_ready && m_rd_ret_last) begin
                        rd_state <= RD_IDLE;   // 尾拍返回被接受 → 释放
                        rd_ptr   <= rd_ptr + 1'b1;   // 轮转
                    end
                end

                RD_ZERO: begin
                    if (rd_ret_ready[rd_cur]) begin
                        rd_state <= RD_IDLE;   // 伪返回被接受 → 释放
                        rd_ptr   <= rd_ptr + 1'b1;
                    end
                end

                default: rd_state <= RD_IDLE;
            endcase
        end
    end

    //====================================================================
    // 写通道仲裁
    //   WR_IDLE   : 无在写，round-robin 选客户端发写请求
    //   WR_ACTIVE : 模型在写，数据/完成直通被选客户端
    //   WR_ZERO   : len==0 伪完成，等待被选客户端接收
    //====================================================================
    localparam [1:0] WR_IDLE   = 2'd0,
                     WR_ACTIVE = 2'd1,
                     WR_ZERO   = 2'd2;

    reg [1:0]       wr_state;
    reg [WR_IW-1:0] wr_ptr, wr_cur;
    reg [TAG_W-1:0] wr_tag_r;

    reg [N_WR-1:0] wr_req_rot, wr_grant_rot, wr_grant;
    integer wk;
    always @(*) begin
        for (wk = 0; wk < N_WR; wk = wk + 1)
            wr_req_rot[wk] = wr_req_valid[(wk + wr_ptr) % N_WR];
        wr_grant_rot = {N_WR{1'b0}};
        for (wk = 0; wk < N_WR; wk = wk + 1)
            if (wr_req_rot[wk] && (wr_grant_rot == {N_WR{1'b0}}))
                wr_grant_rot[wk] = 1'b1;
        wr_grant = {N_WR{1'b0}};
        for (wk = 0; wk < N_WR; wk = wk + 1)
            if (wr_grant_rot[wk])
                wr_grant[(wk + wr_ptr) % N_WR] = 1'b1;
    end

    wire wr_sel_valid = |wr_grant;
    reg [WR_IW-1:0] wr_sel;
    integer wsi;
    always @(*) begin
        wr_sel = {WR_IW{1'b0}};
        for (wsi = 0; wsi < N_WR; wsi = wsi + 1)
            if (wr_grant[wsi])
                wr_sel = wsi[WR_IW-1:0];
    end

    genvar wg;
    generate
        for (wg = 0; wg < N_WR; wg = wg + 1) begin : wr_req_ready_gen
            assign wr_req_ready[wg] = (wr_state == WR_IDLE) && wr_grant[wg] && m_wr_req_ready;
        end
    endgenerate

    assign m_wr_req_valid     = (wr_state == WR_IDLE) && wr_sel_valid && m_wr_req_ready &&
                                (wr_req_len[wr_sel] != {LEN_W{1'b0}});
    assign m_wr_req_addr      = wr_req_addr[wr_sel];
    assign m_wr_req_len_bytes = wr_req_len[wr_sel];
    assign m_wr_req_tag       = wr_req_tag[wr_sel];

    // 写数据：仅 ACTIVE（在途写）双向透传被选客户端；其余客户端 ready 拉低
    // （数据 valid 被屏蔽，无在途请求的写数据不产生协议违规）
    assign m_wr_dat_valid = (wr_state == WR_ACTIVE) && wr_dat_valid[wr_cur];
    assign m_wr_dat_data  = wr_dat_data[wr_cur];
    assign m_wr_dat_keep  = wr_dat_keep[wr_cur];
    assign m_wr_dat_last  = wr_dat_last[wr_cur];

    generate
        for (wg = 0; wg < N_WR; wg = wg + 1) begin : wr_dat_ready_gen
            assign wr_dat_ready[wg] = (wr_state == WR_ACTIVE) && (wg == wr_cur) && m_wr_dat_ready;
        end
    endgenerate

    // 完成：只回被选客户端（ACTIVE：模型完成直通；ZERO：len==0 伪完成 error=1）
    assign m_wr_cplt_ready = (wr_state == WR_ACTIVE) && wr_done_ready[wr_cur];

    generate
        for (wg = 0; wg < N_WR; wg = wg + 1) begin : wr_done_gen
            wire sel = (wr_state != WR_IDLE) && (wg == wr_cur);
            assign wr_done_valid[wg] = sel && ((wr_state == WR_ACTIVE) ? m_wr_cplt_valid : 1'b1);
            assign wr_done_tag[wg]   = sel ? wr_tag_r : {TAG_W{1'b0}};
            assign wr_done_error[wg] = sel && ((wr_state == WR_ACTIVE) ? m_wr_cplt_error : 1'b1);
        end
    endgenerate

    always @(posedge clk) begin
        if (!rst_n) begin
            wr_state <= WR_IDLE;
            wr_ptr   <= {WR_IW{1'b0}};
            wr_cur   <= {WR_IW{1'b0}};
            wr_tag_r <= {TAG_W{1'b0}};
        end else begin
            case (wr_state)
                WR_IDLE: begin
                    if (wr_sel_valid && m_wr_req_ready) begin
                        wr_cur   <= wr_sel;
                        wr_tag_r <= wr_req_tag[wr_sel];
                        if (wr_req_len[wr_sel] == {LEN_W{1'b0}})
                            wr_state <= WR_ZERO;   // 零长：不发给模型，伪完成
                        else
                            wr_state <= WR_ACTIVE;
                    end
                end

                WR_ACTIVE: begin
                    if (m_wr_cplt_valid && m_wr_cplt_ready) begin
                        wr_state <= WR_IDLE;   // 写完成被接受 → 释放
                        wr_ptr   <= wr_ptr + 1'b1;   // 轮转
                    end
                end

                WR_ZERO: begin
                    if (wr_done_ready[wr_cur]) begin
                        wr_state <= WR_IDLE;   // 伪完成被接受 → 释放
                        wr_ptr   <= wr_ptr + 1'b1;
                    end
                end

                default: wr_state <= WR_IDLE;
            endcase
        end
    end

endmodule
