`timescale 1ns / 1ps
//==============================================================================
// tb_frame_top.sv — M8 集成对拍：corner_detect_ddr_top（帧级一键处理）
//------------------------------------------------------------------------------
// 结构：ddr_memory_model（随机延迟/背压/proto 全开）+ 两实例 top
//   （big 1280x720 DEPTH=2 / board5x8 272x96 DEPTH=1）共享模型端口（TB active mux）。
// 外部客户端（arbiter 下标 1）：
//   - ext_wr：raster_dma 写方向预载灰度（u_pre_big / u_pre_b5，各连各 top）
//   - ext_rd：raster_dma 读方向读回响应图（u_rb_big / u_rb_b5，各连各 top）
// 内部客户端（下标 0）：gray_fetch 读 + resp_ddr_writer 写，由 frame_task_ctrl 调度。
//
// 用例：
//   1) big 帧 A：预载 m6_big_gray.bin @0x1000 → process（pyr_en=1, dump_en=1,
//      resp_base=0x300000）→ 40 点 vs m6_chain_big.bin + ext_rd 读回 921600B
//      vs m7_resp_big.bin（0 错）
//   2) board5x8 帧 D：预载 m5_board5x8_gray.bin @0x200000 → process（pyr_en=0,
//      dump_en=1, resp_base=0x400000）→ 40 点 vs m6_chain_board5x8.bin +
//      读回 104448B vs m7_resp_board5x8.bin
//   3) big 帧 B（连续帧复用）：同实例再 process（resp_base=0x310000 验证 cfg
//      重新锁存、done 清零）→ 40 点 + 读回 @0x310000
//   4) big 帧 C（cfg_resp_dump_en=0）：正常完成且只验证 done/40 点（resp 区不写）
//
// 判据：err=0 && proto_violations=0 && 每帧 40/40 → ALL FRAME-TOP TESTS PASSED
//==============================================================================
module tb_frame_top;

    localparam integer CLK_PERIOD = 10;
    logic clk = 1'b0;
    logic rst_n;
    always #(CLK_PERIOD/2) clk = ~clk;

    integer fd, code;
    reg [7:0]  fbuf [0:4194303];           // 4MB 向量缓冲

    //--------------------------------------------------------------------
    // DDR 模型
    //--------------------------------------------------------------------
    logic        m_rd_req_valid, m_rd_req_ready;
    logic [31:0] m_rd_req_addr,  m_rd_req_len;
    logic [15:0] m_rd_req_tag;
    logic        m_rd_ret_valid, m_rd_ret_ready;
    logic [31:0] m_rd_ret_data;
    logic [3:0]  m_rd_ret_keep;
    logic [15:0] m_rd_ret_tag;
    logic        m_rd_ret_last,  m_rd_ret_error;
    logic        m_wr_req_valid, m_wr_req_ready;
    logic [31:0] m_wr_req_addr,  m_wr_req_len;
    logic [15:0] m_wr_req_tag;
    logic        m_wr_dat_valid, m_wr_dat_ready;
    logic [31:0] m_wr_dat_data;
    logic [3:0]  m_wr_dat_keep;
    logic        m_wr_dat_last;
    logic        m_wr_done_valid, m_wr_done_ready;
    logic [15:0] m_wr_done_tag;
    logic        m_wr_done_error;
    logic [15:0] m_proto_viol;

    ddr_memory_model #(
        .LATENCY_MIN (1), .LATENCY_MAX (6),
        .JITTER_EN (1'b1), .BACKPRESSURE_EN (1'b1),
        .PROTOCOL_CHECKS (1'b1), .SEED (32'h2468_ACE0)
    ) u_ddr (
        .clk (clk), .rst_n (rst_n),
        .rd_req_valid (m_rd_req_valid), .rd_req_ready (m_rd_req_ready),
        .rd_req_addr (m_rd_req_addr), .rd_req_len_bytes (m_rd_req_len), .rd_req_tag (m_rd_req_tag),
        .rd_ret_valid (m_rd_ret_valid), .rd_ret_ready (m_rd_ret_ready),
        .rd_ret_data (m_rd_ret_data), .rd_ret_keep (m_rd_ret_keep),
        .rd_ret_tag (m_rd_ret_tag), .rd_ret_last (m_rd_ret_last), .rd_ret_error (m_rd_ret_error),
        .wr_req_valid (m_wr_req_valid), .wr_req_ready (m_wr_req_ready),
        .wr_req_addr (m_wr_req_addr), .wr_req_len_bytes (m_wr_req_len), .wr_req_tag (m_wr_req_tag),
        .wr_dat_valid (m_wr_dat_valid), .wr_dat_ready (m_wr_dat_ready),
        .wr_dat_data (m_wr_dat_data), .wr_dat_keep (m_wr_dat_keep), .wr_dat_last (m_wr_dat_last),
        .wr_cplt_valid (m_wr_done_valid), .wr_cplt_ready (m_wr_done_ready),
        .wr_cplt_tag (m_wr_done_tag), .wr_cplt_error (m_wr_done_error),
        .proto_violations (m_proto_viol)
    );

    //--------------------------------------------------------------------
    // u_top_big：corner_detect_ddr_top（1280x720 DEPTH=2 金字塔路径）
    //--------------------------------------------------------------------
    logic        big_process;
    logic [31:0] big_cfg_gray_base, big_cfg_gray_stride, big_cfg_resp_base;
    logic [15:0] big_cfg_gray_w, big_cfg_gray_h;
    logic [20:0] big_cfg_ram_base;
    logic        big_cfg_resp_dump_en, big_cfg_pyr_en;
    logic        big_busy, big_done;
    logic [1:0]  big_status;
    logic        big_out_valid, big_out_ready, big_out_grid_ok;
    logic [31:0] big_out_x, big_out_y;
    logic [15:0] big_out_total;

    logic        big_m_rd_req_valid, big_m_rd_req_ready;
    logic [31:0] big_m_rd_req_addr,  big_m_rd_req_len;
    logic [15:0] big_m_rd_req_tag;
    logic        big_m_rd_ret_valid, big_m_rd_ret_ready;
    logic [31:0] big_m_rd_ret_data;
    logic [3:0]  big_m_rd_ret_keep;
    logic [15:0] big_m_rd_ret_tag;
    logic        big_m_rd_ret_last,  big_m_rd_ret_error;
    logic        big_m_wr_req_valid, big_m_wr_req_ready;
    logic [31:0] big_m_wr_req_addr,  big_m_wr_req_len;
    logic [15:0] big_m_wr_req_tag;
    logic        big_m_wr_dat_valid, big_m_wr_dat_ready;
    logic [31:0] big_m_wr_dat_data;
    logic [3:0]  big_m_wr_dat_keep;
    logic        big_m_wr_dat_last;
    logic        big_m_wr_done_valid, big_m_wr_done_ready;
    logic [15:0] big_m_wr_done_tag;
    logic        big_m_wr_done_error;

    logic        big_ext_rd_req_valid, big_ext_rd_req_ready;
    logic [31:0] big_ext_rd_req_addr,  big_ext_rd_req_len;
    logic [15:0] big_ext_rd_req_tag;
    logic        big_ext_rd_ret_valid, big_ext_rd_ret_ready;
    logic [31:0] big_ext_rd_ret_data;
    logic [3:0]  big_ext_rd_ret_keep;
    logic [15:0] big_ext_rd_ret_tag;
    logic        big_ext_rd_ret_last,  big_ext_rd_ret_error;
    logic        big_ext_wr_req_valid, big_ext_wr_req_ready;
    logic [31:0] big_ext_wr_req_addr,  big_ext_wr_req_len;
    logic [15:0] big_ext_wr_req_tag;
    logic        big_ext_wr_dat_valid, big_ext_wr_dat_ready;
    logic [31:0] big_ext_wr_dat_data;
    logic [3:0]  big_ext_wr_dat_keep;
    logic        big_ext_wr_dat_last;
    logic        big_ext_wr_done_valid, big_ext_wr_done_ready;
    logic [15:0] big_ext_wr_done_tag;
    logic        big_ext_wr_done_error;

    corner_detect_ddr_top #(
        .W0 (1280), .H0 (720), .DEPTH (2), .GRAY_ADDR_W (21),
        .MAX_W (2560), .MAX_H (1440), .MAX_DEPTH (4)
    ) u_top_big (
        .clk (clk), .rst_n (rst_n),
        .process_frame (big_process), .busy (big_busy), .done (big_done), .status (big_status),
        .cfg_gray_base (big_cfg_gray_base), .cfg_gray_stride (big_cfg_gray_stride),
        .cfg_gray_w (big_cfg_gray_w), .cfg_gray_h (big_cfg_gray_h),
        .cfg_ram_base (big_cfg_ram_base), .cfg_resp_base (big_cfg_resp_base),
        .cfg_resp_dump_en (big_cfg_resp_dump_en), .cfg_pyr_en (big_cfg_pyr_en),
        .out_valid (big_out_valid), .out_ready (big_out_ready),
        .out_x (big_out_x), .out_y (big_out_y),
        .out_total (big_out_total), .out_grid_ok (big_out_grid_ok),
        .m_rd_req_valid (big_m_rd_req_valid), .m_rd_req_ready (big_m_rd_req_ready),
        .m_rd_req_addr (big_m_rd_req_addr), .m_rd_req_len_bytes (big_m_rd_req_len), .m_rd_req_tag (big_m_rd_req_tag),
        .m_rd_ret_valid (big_m_rd_ret_valid), .m_rd_ret_ready (big_m_rd_ret_ready),
        .m_rd_ret_data (big_m_rd_ret_data), .m_rd_ret_keep (big_m_rd_ret_keep),
        .m_rd_ret_tag (big_m_rd_ret_tag), .m_rd_ret_last (big_m_rd_ret_last), .m_rd_ret_error (big_m_rd_ret_error),
        .m_wr_req_valid (big_m_wr_req_valid), .m_wr_req_ready (big_m_wr_req_ready),
        .m_wr_req_addr (big_m_wr_req_addr), .m_wr_req_len_bytes (big_m_wr_req_len), .m_wr_req_tag (big_m_wr_req_tag),
        .m_wr_dat_valid (big_m_wr_dat_valid), .m_wr_dat_ready (big_m_wr_dat_ready),
        .m_wr_dat_data (big_m_wr_dat_data), .m_wr_dat_keep (big_m_wr_dat_keep), .m_wr_dat_last (big_m_wr_dat_last),
        .m_wr_cplt_valid (big_m_wr_done_valid), .m_wr_cplt_ready (big_m_wr_done_ready),
        .m_wr_cplt_tag (big_m_wr_done_tag), .m_wr_cplt_error (big_m_wr_done_error),
        .ext_rd_req_valid (big_ext_rd_req_valid), .ext_rd_req_ready (big_ext_rd_req_ready),
        .ext_rd_req_addr (big_ext_rd_req_addr), .ext_rd_req_len (big_ext_rd_req_len), .ext_rd_req_tag (big_ext_rd_req_tag),
        .ext_rd_ret_valid (big_ext_rd_ret_valid), .ext_rd_ret_ready (big_ext_rd_ret_ready),
        .ext_rd_ret_data (big_ext_rd_ret_data), .ext_rd_ret_keep (big_ext_rd_ret_keep),
        .ext_rd_ret_tag (big_ext_rd_ret_tag), .ext_rd_ret_last (big_ext_rd_ret_last), .ext_rd_ret_error (big_ext_rd_ret_error),
        .ext_wr_req_valid (big_ext_wr_req_valid), .ext_wr_req_ready (big_ext_wr_req_ready),
        .ext_wr_req_addr (big_ext_wr_req_addr), .ext_wr_req_len (big_ext_wr_req_len), .ext_wr_req_tag (big_ext_wr_req_tag),
        .ext_wr_dat_valid (big_ext_wr_dat_valid), .ext_wr_dat_ready (big_ext_wr_dat_ready),
        .ext_wr_dat_data (big_ext_wr_dat_data), .ext_wr_dat_keep (big_ext_wr_dat_keep), .ext_wr_dat_last (big_ext_wr_dat_last),
        .ext_wr_done_valid (big_ext_wr_done_valid), .ext_wr_done_ready (big_ext_wr_done_ready),
        .ext_wr_done_tag (big_ext_wr_done_tag), .ext_wr_done_error (big_ext_wr_done_error)
    );

    //--------------------------------------------------------------------
    // u_top_b5：corner_detect_ddr_top（272x96 DEPTH=1 native 路径）
    //--------------------------------------------------------------------
    logic        b5_process;
    logic [31:0] b5_cfg_gray_base, b5_cfg_gray_stride, b5_cfg_resp_base;
    logic [15:0] b5_cfg_gray_w, b5_cfg_gray_h;
    logic [20:0] b5_cfg_ram_base;
    logic        b5_cfg_resp_dump_en, b5_cfg_pyr_en;
    logic        b5_busy, b5_done;
    logic [1:0]  b5_status;
    logic        b5_out_valid, b5_out_ready, b5_out_grid_ok;
    logic [31:0] b5_out_x, b5_out_y;
    logic [15:0] b5_out_total;

    logic        b5_m_rd_req_valid, b5_m_rd_req_ready;
    logic [31:0] b5_m_rd_req_addr,  b5_m_rd_req_len;
    logic [15:0] b5_m_rd_req_tag;
    logic        b5_m_rd_ret_valid, b5_m_rd_ret_ready;
    logic [31:0] b5_m_rd_ret_data;
    logic [3:0]  b5_m_rd_ret_keep;
    logic [15:0] b5_m_rd_ret_tag;
    logic        b5_m_rd_ret_last,  b5_m_rd_ret_error;
    logic        b5_m_wr_req_valid, b5_m_wr_req_ready;
    logic [31:0] b5_m_wr_req_addr,  b5_m_wr_req_len;
    logic [15:0] b5_m_wr_req_tag;
    logic        b5_m_wr_dat_valid, b5_m_wr_dat_ready;
    logic [31:0] b5_m_wr_dat_data;
    logic [3:0]  b5_m_wr_dat_keep;
    logic        b5_m_wr_dat_last;
    logic        b5_m_wr_done_valid, b5_m_wr_done_ready;
    logic [15:0] b5_m_wr_done_tag;
    logic        b5_m_wr_done_error;

    logic        b5_ext_rd_req_valid, b5_ext_rd_req_ready;
    logic [31:0] b5_ext_rd_req_addr,  b5_ext_rd_req_len;
    logic [15:0] b5_ext_rd_req_tag;
    logic        b5_ext_rd_ret_valid, b5_ext_rd_ret_ready;
    logic [31:0] b5_ext_rd_ret_data;
    logic [3:0]  b5_ext_rd_ret_keep;
    logic [15:0] b5_ext_rd_ret_tag;
    logic        b5_ext_rd_ret_last,  b5_ext_rd_ret_error;
    logic        b5_ext_wr_req_valid, b5_ext_wr_req_ready;
    logic [31:0] b5_ext_wr_req_addr,  b5_ext_wr_req_len;
    logic [15:0] b5_ext_wr_req_tag;
    logic        b5_ext_wr_dat_valid, b5_ext_wr_dat_ready;
    logic [31:0] b5_ext_wr_dat_data;
    logic [3:0]  b5_ext_wr_dat_keep;
    logic        b5_ext_wr_dat_last;
    logic        b5_ext_wr_done_valid, b5_ext_wr_done_ready;
    logic [15:0] b5_ext_wr_done_tag;
    logic        b5_ext_wr_done_error;

    corner_detect_ddr_top #(
        .W0 (272), .H0 (96), .DEPTH (1), .GRAY_ADDR_W (21),
        .MAX_W (2560), .MAX_H (1440), .MAX_DEPTH (4)
    ) u_top_b5 (
        .clk (clk), .rst_n (rst_n),
        .process_frame (b5_process), .busy (b5_busy), .done (b5_done), .status (b5_status),
        .cfg_gray_base (b5_cfg_gray_base), .cfg_gray_stride (b5_cfg_gray_stride),
        .cfg_gray_w (b5_cfg_gray_w), .cfg_gray_h (b5_cfg_gray_h),
        .cfg_ram_base (b5_cfg_ram_base), .cfg_resp_base (b5_cfg_resp_base),
        .cfg_resp_dump_en (b5_cfg_resp_dump_en), .cfg_pyr_en (b5_cfg_pyr_en),
        .out_valid (b5_out_valid), .out_ready (b5_out_ready),
        .out_x (b5_out_x), .out_y (b5_out_y),
        .out_total (b5_out_total), .out_grid_ok (b5_out_grid_ok),
        .m_rd_req_valid (b5_m_rd_req_valid), .m_rd_req_ready (b5_m_rd_req_ready),
        .m_rd_req_addr (b5_m_rd_req_addr), .m_rd_req_len_bytes (b5_m_rd_req_len), .m_rd_req_tag (b5_m_rd_req_tag),
        .m_rd_ret_valid (b5_m_rd_ret_valid), .m_rd_ret_ready (b5_m_rd_ret_ready),
        .m_rd_ret_data (b5_m_rd_ret_data), .m_rd_ret_keep (b5_m_rd_ret_keep),
        .m_rd_ret_tag (b5_m_rd_ret_tag), .m_rd_ret_last (b5_m_rd_ret_last), .m_rd_ret_error (b5_m_rd_ret_error),
        .m_wr_req_valid (b5_m_wr_req_valid), .m_wr_req_ready (b5_m_wr_req_ready),
        .m_wr_req_addr (b5_m_wr_req_addr), .m_wr_req_len_bytes (b5_m_wr_req_len), .m_wr_req_tag (b5_m_wr_req_tag),
        .m_wr_dat_valid (b5_m_wr_dat_valid), .m_wr_dat_ready (b5_m_wr_dat_ready),
        .m_wr_dat_data (b5_m_wr_dat_data), .m_wr_dat_keep (b5_m_wr_dat_keep), .m_wr_dat_last (b5_m_wr_dat_last),
        .m_wr_cplt_valid (b5_m_wr_done_valid), .m_wr_cplt_ready (b5_m_wr_done_ready),
        .m_wr_cplt_tag (b5_m_wr_done_tag), .m_wr_cplt_error (b5_m_wr_done_error),
        .ext_rd_req_valid (b5_ext_rd_req_valid), .ext_rd_req_ready (b5_ext_rd_req_ready),
        .ext_rd_req_addr (b5_ext_rd_req_addr), .ext_rd_req_len (b5_ext_rd_req_len), .ext_rd_req_tag (b5_ext_rd_req_tag),
        .ext_rd_ret_valid (b5_ext_rd_ret_valid), .ext_rd_ret_ready (b5_ext_rd_ret_ready),
        .ext_rd_ret_data (b5_ext_rd_ret_data), .ext_rd_ret_keep (b5_ext_rd_ret_keep),
        .ext_rd_ret_tag (b5_ext_rd_ret_tag), .ext_rd_ret_last (b5_ext_rd_ret_last), .ext_rd_ret_error (b5_ext_rd_ret_error),
        .ext_wr_req_valid (b5_ext_wr_req_valid), .ext_wr_req_ready (b5_ext_wr_req_ready),
        .ext_wr_req_addr (b5_ext_wr_req_addr), .ext_wr_req_len (b5_ext_wr_req_len), .ext_wr_req_tag (b5_ext_wr_req_tag),
        .ext_wr_dat_valid (b5_ext_wr_dat_valid), .ext_wr_dat_ready (b5_ext_wr_dat_ready),
        .ext_wr_dat_data (b5_ext_wr_dat_data), .ext_wr_dat_keep (b5_ext_wr_dat_keep), .ext_wr_dat_last (b5_ext_wr_dat_last),
        .ext_wr_done_valid (b5_ext_wr_done_valid), .ext_wr_done_ready (b5_ext_wr_done_ready),
        .ext_wr_done_tag (b5_ext_wr_done_tag), .ext_wr_done_error (b5_ext_wr_done_error)
    );

    //--------------------------------------------------------------------
    // 模型端口 active mux（big / b5 串行共享 u_ddr）
    //--------------------------------------------------------------------
    reg big_active, b5_active;

    assign m_rd_req_valid = big_active ? big_m_rd_req_valid : b5_active ? b5_m_rd_req_valid : 1'b0;
    assign m_rd_req_addr  = big_active ? big_m_rd_req_addr  : b5_active ? b5_m_rd_req_addr  : 32'd0;
    assign m_rd_req_len   = big_active ? big_m_rd_req_len   : b5_active ? b5_m_rd_req_len   : 32'd0;
    assign m_rd_req_tag   = big_active ? big_m_rd_req_tag   : b5_active ? b5_m_rd_req_tag   : 16'd0;
    assign big_m_rd_req_ready = big_active ? m_rd_req_ready : 1'b0;
    assign b5_m_rd_req_ready = b5_active ? m_rd_req_ready : 1'b0;

    assign big_m_rd_ret_valid = big_active ? m_rd_ret_valid : 1'b0;
    assign big_m_rd_ret_data  = big_active ? m_rd_ret_data  : 32'd0;
    assign big_m_rd_ret_keep  = big_active ? m_rd_ret_keep  : 4'd0;
    assign big_m_rd_ret_tag   = big_active ? m_rd_ret_tag   : 16'd0;
    assign big_m_rd_ret_last  = big_active ? m_rd_ret_last  : 1'b0;
    assign big_m_rd_ret_error = big_active ? m_rd_ret_error : 1'b0;
    assign b5_m_rd_ret_valid = b5_active ? m_rd_ret_valid : 1'b0;
    assign b5_m_rd_ret_data  = b5_active ? m_rd_ret_data  : 32'd0;
    assign b5_m_rd_ret_keep  = b5_active ? m_rd_ret_keep  : 4'd0;
    assign b5_m_rd_ret_tag   = b5_active ? m_rd_ret_tag   : 16'd0;
    assign b5_m_rd_ret_last  = b5_active ? m_rd_ret_last  : 1'b0;
    assign b5_m_rd_ret_error = b5_active ? m_rd_ret_error : 1'b0;
    assign m_rd_ret_ready = big_active ? big_m_rd_ret_ready : b5_active ? b5_m_rd_ret_ready : 1'b0;

    assign m_wr_req_valid = big_active ? big_m_wr_req_valid : b5_active ? b5_m_wr_req_valid : 1'b0;
    assign m_wr_req_addr  = big_active ? big_m_wr_req_addr  : b5_active ? b5_m_wr_req_addr  : 32'd0;
    assign m_wr_req_len   = big_active ? big_m_wr_req_len   : b5_active ? b5_m_wr_req_len   : 32'd0;
    assign m_wr_req_tag   = big_active ? big_m_wr_req_tag   : b5_active ? b5_m_wr_req_tag   : 16'd0;
    assign big_m_wr_req_ready = big_active ? m_wr_req_ready : 1'b0;
    assign b5_m_wr_req_ready = b5_active ? m_wr_req_ready : 1'b0;

    assign m_wr_dat_valid = big_active ? big_m_wr_dat_valid : b5_active ? b5_m_wr_dat_valid : 1'b0;
    assign m_wr_dat_data  = big_active ? big_m_wr_dat_data  : b5_active ? b5_m_wr_dat_data  : 32'd0;
    assign m_wr_dat_keep  = big_active ? big_m_wr_dat_keep  : b5_active ? b5_m_wr_dat_keep  : 4'd0;
    assign m_wr_dat_last  = big_active ? big_m_wr_dat_last  : b5_active ? b5_m_wr_dat_last  : 1'b0;
    assign big_m_wr_dat_ready = big_active ? m_wr_dat_ready : 1'b0;
    assign b5_m_wr_dat_ready = b5_active ? m_wr_dat_ready : 1'b0;

    assign big_m_wr_done_valid = big_active ? m_wr_done_valid : 1'b0;
    assign big_m_wr_done_tag   = big_active ? m_wr_done_tag   : 16'd0;
    assign big_m_wr_done_error = big_active ? m_wr_done_error : 1'b0;
    assign b5_m_wr_done_valid = b5_active ? m_wr_done_valid : 1'b0;
    assign b5_m_wr_done_tag   = b5_active ? m_wr_done_tag   : 16'd0;
    assign b5_m_wr_done_error = b5_active ? m_wr_done_error : 1'b0;
    assign m_wr_done_ready = big_active ? big_m_wr_done_ready : b5_active ? b5_m_wr_done_ready : 1'b0;

    //--------------------------------------------------------------------
    // u_pre_big：DDR 预载（raster_dma 写方向，连 u_top_big ext_wr）
    //--------------------------------------------------------------------
    logic        pre_big_start, pre_big_busy, pre_big_done;
    logic [1:0]  pre_big_status;
    logic        pre_big_in_valid, pre_big_in_ready;
    logic [7:0]  pre_big_in_byte;
    reg  [31:0]  pre_big_cfg_base, pre_big_cfg_stride, pre_big_cfg_offset;
    reg  [15:0]  pre_big_cfg_row_bytes, pre_big_cfg_rows;
    logic        pre_big_req_valid, pre_big_req_ready;
    logic [31:0] pre_big_req_addr, pre_big_req_len;
    logic [15:0] pre_big_req_tag;
    logic        pre_big_dat_valid, pre_big_dat_ready, pre_big_dat_last;
    logic [31:0] pre_big_dat_data;
    logic [3:0]  pre_big_dat_keep;
    logic        pre_big_done_valid, pre_big_done_ready, pre_big_done_error;
    logic [15:0] pre_big_done_tag;

    raster_dma u_pre_big (
        .clk (clk), .rst_n (rst_n),
        .start (pre_big_start), .busy (pre_big_busy), .done (pre_big_done), .status (pre_big_status),
        .cfg_base (pre_big_cfg_base), .cfg_stride (pre_big_cfg_stride),
        .cfg_row_bytes (pre_big_cfg_row_bytes), .cfg_rows (pre_big_cfg_rows),
        .cfg_offset (pre_big_cfg_offset), .cfg_dir (1'b1),
        .out_valid (), .out_ready (1'b1), .out_byte (),
        .in_valid (pre_big_in_valid), .in_ready (pre_big_in_ready), .in_byte (pre_big_in_byte),
        .rd_req_valid (), .rd_req_ready (1'b0), .rd_req_addr (), .rd_req_len (), .rd_req_tag (),
        .rd_ret_valid (1'b0), .rd_ret_ready (), .rd_ret_data (32'd0), .rd_ret_keep (4'd0),
        .rd_ret_tag (16'd0), .rd_ret_last (1'b0), .rd_ret_error (1'b0),
        .wr_req_valid (pre_big_req_valid), .wr_req_ready (pre_big_req_ready),
        .wr_req_addr (pre_big_req_addr), .wr_req_len (pre_big_req_len), .wr_req_tag (pre_big_req_tag),
        .wr_dat_valid (pre_big_dat_valid), .wr_dat_ready (pre_big_dat_ready),
        .wr_dat_data (pre_big_dat_data), .wr_dat_keep (pre_big_dat_keep), .wr_dat_last (pre_big_dat_last),
        .wr_done_valid (pre_big_done_valid), .wr_done_ready (pre_big_done_ready),
        .wr_done_tag (pre_big_done_tag), .wr_done_error (pre_big_done_error)
    );

    // u_pre_big → u_top_big.ext_wr（写客户端 1）
    assign big_ext_wr_req_valid = pre_big_req_valid;
    assign pre_big_req_ready    = big_ext_wr_req_ready;
    assign big_ext_wr_req_addr  = pre_big_req_addr;
    assign big_ext_wr_req_len   = pre_big_req_len;
    assign big_ext_wr_req_tag   = pre_big_req_tag;
    assign big_ext_wr_dat_valid = pre_big_dat_valid;
    assign pre_big_dat_ready    = big_ext_wr_dat_ready;
    assign big_ext_wr_dat_data  = pre_big_dat_data;
    assign big_ext_wr_dat_keep  = pre_big_dat_keep;
    assign big_ext_wr_dat_last  = pre_big_dat_last;
    assign pre_big_done_valid   = big_ext_wr_done_valid;
    assign big_ext_wr_done_ready= pre_big_done_ready;
    assign pre_big_done_tag     = big_ext_wr_done_tag;
    assign pre_big_done_error   = big_ext_wr_done_error;

    //--------------------------------------------------------------------
    // u_pre_b5：DDR 预载（连 u_top_b5 ext_wr）
    //--------------------------------------------------------------------
    logic        pre_b5_start, pre_b5_busy, pre_b5_done;
    logic [1:0]  pre_b5_status;
    logic        pre_b5_in_valid, pre_b5_in_ready;
    logic [7:0]  pre_b5_in_byte;
    reg  [31:0]  pre_b5_cfg_base, pre_b5_cfg_stride, pre_b5_cfg_offset;
    reg  [15:0]  pre_b5_cfg_row_bytes, pre_b5_cfg_rows;
    logic        pre_b5_req_valid, pre_b5_req_ready;
    logic [31:0] pre_b5_req_addr, pre_b5_req_len;
    logic [15:0] pre_b5_req_tag;
    logic        pre_b5_dat_valid, pre_b5_dat_ready, pre_b5_dat_last;
    logic [31:0] pre_b5_dat_data;
    logic [3:0]  pre_b5_dat_keep;
    logic        pre_b5_done_valid, pre_b5_done_ready, pre_b5_done_error;
    logic [15:0] pre_b5_done_tag;

    raster_dma u_pre_b5 (
        .clk (clk), .rst_n (rst_n),
        .start (pre_b5_start), .busy (pre_b5_busy), .done (pre_b5_done), .status (pre_b5_status),
        .cfg_base (pre_b5_cfg_base), .cfg_stride (pre_b5_cfg_stride),
        .cfg_row_bytes (pre_b5_cfg_row_bytes), .cfg_rows (pre_b5_cfg_rows),
        .cfg_offset (pre_b5_cfg_offset), .cfg_dir (1'b1),
        .out_valid (), .out_ready (1'b1), .out_byte (),
        .in_valid (pre_b5_in_valid), .in_ready (pre_b5_in_ready), .in_byte (pre_b5_in_byte),
        .rd_req_valid (), .rd_req_ready (1'b0), .rd_req_addr (), .rd_req_len (), .rd_req_tag (),
        .rd_ret_valid (1'b0), .rd_ret_ready (), .rd_ret_data (32'd0), .rd_ret_keep (4'd0),
        .rd_ret_tag (16'd0), .rd_ret_last (1'b0), .rd_ret_error (1'b0),
        .wr_req_valid (pre_b5_req_valid), .wr_req_ready (pre_b5_req_ready),
        .wr_req_addr (pre_b5_req_addr), .wr_req_len (pre_b5_req_len), .wr_req_tag (pre_b5_req_tag),
        .wr_dat_valid (pre_b5_dat_valid), .wr_dat_ready (pre_b5_dat_ready),
        .wr_dat_data (pre_b5_dat_data), .wr_dat_keep (pre_b5_dat_keep), .wr_dat_last (pre_b5_dat_last),
        .wr_done_valid (pre_b5_done_valid), .wr_done_ready (pre_b5_done_ready),
        .wr_done_tag (pre_b5_done_tag), .wr_done_error (pre_b5_done_error)
    );

    assign b5_ext_wr_req_valid = pre_b5_req_valid;
    assign pre_b5_req_ready    = b5_ext_wr_req_ready;
    assign b5_ext_wr_req_addr  = pre_b5_req_addr;
    assign b5_ext_wr_req_len   = pre_b5_req_len;
    assign b5_ext_wr_req_tag   = pre_b5_req_tag;
    assign b5_ext_wr_dat_valid = pre_b5_dat_valid;
    assign pre_b5_dat_ready    = b5_ext_wr_dat_ready;
    assign b5_ext_wr_dat_data  = pre_b5_dat_data;
    assign b5_ext_wr_dat_keep  = pre_b5_dat_keep;
    assign b5_ext_wr_dat_last  = pre_b5_dat_last;
    assign pre_b5_done_valid   = b5_ext_wr_done_valid;
    assign b5_ext_wr_done_ready= pre_b5_done_ready;
    assign pre_b5_done_tag     = b5_ext_wr_done_tag;
    assign pre_b5_done_error   = b5_ext_wr_done_error;

    //--------------------------------------------------------------------
    // u_rb_big：读回 resp DDR 区（raster_dma 读方向，连 u_top_big ext_rd）
    //--------------------------------------------------------------------
    logic        rb_big_start, rb_big_busy, rb_big_done;
    logic [1:0]  rb_big_status;
    logic        rb_big_out_valid, rb_big_out_ready;
    logic [7:0]  rb_big_out_byte;
    reg  [31:0]  rb_big_cfg_base, rb_big_cfg_stride, rb_big_cfg_offset;
    reg  [15:0]  rb_big_cfg_row_bytes, rb_big_cfg_rows;
    logic        rb_big_req_valid, rb_big_req_ready;
    logic [31:0] rb_big_req_addr, rb_big_req_len;
    logic [15:0] rb_big_req_tag;
    logic        rb_big_ret_valid, rb_big_ret_ready, rb_big_ret_last, rb_big_ret_error;
    logic [31:0] rb_big_ret_data;
    logic [3:0]  rb_big_ret_keep;
    logic [15:0] rb_big_ret_tag;

    raster_dma u_rb_big (
        .clk (clk), .rst_n (rst_n),
        .start (rb_big_start), .busy (rb_big_busy), .done (rb_big_done), .status (rb_big_status),
        .cfg_base (rb_big_cfg_base), .cfg_stride (rb_big_cfg_stride),
        .cfg_row_bytes (rb_big_cfg_row_bytes), .cfg_rows (rb_big_cfg_rows),
        .cfg_offset (rb_big_cfg_offset), .cfg_dir (1'b0),
        .out_valid (rb_big_out_valid), .out_ready (rb_big_out_ready), .out_byte (rb_big_out_byte),
        .in_valid (1'b0), .in_ready (), .in_byte (8'd0),
        .rd_req_valid (rb_big_req_valid), .rd_req_ready (rb_big_req_ready),
        .rd_req_addr (rb_big_req_addr), .rd_req_len (rb_big_req_len), .rd_req_tag (rb_big_req_tag),
        .rd_ret_valid (rb_big_ret_valid), .rd_ret_ready (rb_big_ret_ready),
        .rd_ret_data (rb_big_ret_data), .rd_ret_keep (rb_big_ret_keep),
        .rd_ret_tag (rb_big_ret_tag), .rd_ret_last (rb_big_ret_last), .rd_ret_error (rb_big_ret_error),
        .wr_req_valid (), .wr_req_ready (1'b0), .wr_req_addr (), .wr_req_len (), .wr_req_tag (),
        .wr_dat_valid (), .wr_dat_ready (1'b0), .wr_dat_data (), .wr_dat_keep (), .wr_dat_last (),
        .wr_done_valid (1'b0), .wr_done_ready (), .wr_done_tag (16'd0), .wr_done_error (1'b0)
    );

    assign big_ext_rd_req_valid = rb_big_req_valid;
    assign rb_big_req_ready     = big_ext_rd_req_ready;
    assign big_ext_rd_req_addr  = rb_big_req_addr;
    assign big_ext_rd_req_len   = rb_big_req_len;
    assign big_ext_rd_req_tag   = rb_big_req_tag;
    assign rb_big_ret_valid     = big_ext_rd_ret_valid;
    assign big_ext_rd_ret_ready = rb_big_ret_ready;
    assign rb_big_ret_data      = big_ext_rd_ret_data;
    assign rb_big_ret_keep      = big_ext_rd_ret_keep;
    assign rb_big_ret_tag       = big_ext_rd_ret_tag;
    assign rb_big_ret_last      = big_ext_rd_ret_last;
    assign rb_big_ret_error     = big_ext_rd_ret_error;
    assign rb_big_out_ready     = 1'b1;

    //--------------------------------------------------------------------
    // u_rb_b5：读回 resp DDR 区（连 u_top_b5 ext_rd）
    //--------------------------------------------------------------------
    logic        rb_b5_start, rb_b5_busy, rb_b5_done;
    logic [1:0]  rb_b5_status;
    logic        rb_b5_out_valid, rb_b5_out_ready;
    logic [7:0]  rb_b5_out_byte;
    reg  [31:0]  rb_b5_cfg_base, rb_b5_cfg_stride, rb_b5_cfg_offset;
    reg  [15:0]  rb_b5_cfg_row_bytes, rb_b5_cfg_rows;
    logic        rb_b5_req_valid, rb_b5_req_ready;
    logic [31:0] rb_b5_req_addr, rb_b5_req_len;
    logic [15:0] rb_b5_req_tag;
    logic        rb_b5_ret_valid, rb_b5_ret_ready, rb_b5_ret_last, rb_b5_ret_error;
    logic [31:0] rb_b5_ret_data;
    logic [3:0]  rb_b5_ret_keep;
    logic [15:0] rb_b5_ret_tag;

    raster_dma u_rb_b5 (
        .clk (clk), .rst_n (rst_n),
        .start (rb_b5_start), .busy (rb_b5_busy), .done (rb_b5_done), .status (rb_b5_status),
        .cfg_base (rb_b5_cfg_base), .cfg_stride (rb_b5_cfg_stride),
        .cfg_row_bytes (rb_b5_cfg_row_bytes), .cfg_rows (rb_b5_cfg_rows),
        .cfg_offset (rb_b5_cfg_offset), .cfg_dir (1'b0),
        .out_valid (rb_b5_out_valid), .out_ready (rb_b5_out_ready), .out_byte (rb_b5_out_byte),
        .in_valid (1'b0), .in_ready (), .in_byte (8'd0),
        .rd_req_valid (rb_b5_req_valid), .rd_req_ready (rb_b5_req_ready),
        .rd_req_addr (rb_b5_req_addr), .rd_req_len (rb_b5_req_len), .rd_req_tag (rb_b5_req_tag),
        .rd_ret_valid (rb_b5_ret_valid), .rd_ret_ready (rb_b5_ret_ready),
        .rd_ret_data (rb_b5_ret_data), .rd_ret_keep (rb_b5_ret_keep),
        .rd_ret_tag (rb_b5_ret_tag), .rd_ret_last (rb_b5_ret_last), .rd_ret_error (rb_b5_ret_error),
        .wr_req_valid (), .wr_req_ready (1'b0), .wr_req_addr (), .wr_req_len (), .wr_req_tag (),
        .wr_dat_valid (), .wr_dat_ready (1'b0), .wr_dat_data (), .wr_dat_keep (), .wr_dat_last (),
        .wr_done_valid (1'b0), .wr_done_ready (), .wr_done_tag (16'd0), .wr_done_error (1'b0)
    );

    assign b5_ext_rd_req_valid = rb_b5_req_valid;
    assign rb_b5_req_ready     = b5_ext_rd_req_ready;
    assign b5_ext_rd_req_addr  = rb_b5_req_addr;
    assign b5_ext_rd_req_len   = rb_b5_req_len;
    assign b5_ext_rd_req_tag   = rb_b5_req_tag;
    assign rb_b5_ret_valid     = b5_ext_rd_ret_valid;
    assign b5_ext_rd_ret_ready = rb_b5_ret_ready;
    assign rb_b5_ret_data      = b5_ext_rd_ret_data;
    assign rb_b5_ret_keep      = b5_ext_rd_ret_keep;
    assign rb_b5_ret_tag       = b5_ext_rd_ret_tag;
    assign rb_b5_ret_last      = b5_ext_rd_ret_last;
    assign rb_b5_ret_error     = b5_ext_rd_ret_error;
    assign rb_b5_out_ready     = 1'b1;

    //--------------------------------------------------------------------
    // 期望向量与输出采集
    //--------------------------------------------------------------------
    reg [31:0] exp_chain [0:1023];
    integer    exp_N, exp_valid, err_cnt = 0, oi_big = 0, oi_b5 = 0;

    always @(posedge clk) begin
        if (big_out_valid && big_out_ready) begin
            if (oi_big >= exp_N || big_out_x !== exp_chain[2 + oi_big*2] ||
                big_out_y !== exp_chain[3 + oi_big*2]) begin
                err_cnt = err_cnt + 1;
                if (err_cnt <= 10)
                    $display("[FAIL][big] pt%0d got=(%08x,%08x) exp=(%08x,%08x)",
                             oi_big, big_out_x, big_out_y, exp_chain[2+oi_big*2], exp_chain[3+oi_big*2]);
            end
            oi_big = oi_big + 1;
        end
        if (b5_out_valid && b5_out_ready) begin
            if (oi_b5 >= exp_N || b5_out_x !== exp_chain[2 + oi_b5*2] ||
                b5_out_y !== exp_chain[3 + oi_b5*2]) begin
                err_cnt = err_cnt + 1;
                if (err_cnt <= 10)
                    $display("[FAIL][board5x8] pt%0d got=(%08x,%08x) exp=(%08x,%08x)",
                             oi_b5, b5_out_x, b5_out_y, exp_chain[2+oi_b5*2], exp_chain[3+oi_b5*2]);
            end
            oi_b5 = oi_b5 + 1;
        end
    end
    assign big_out_ready = 1'b1;
    assign b5_out_ready  = 1'b1;

    //--------------------------------------------------------------------
    // 文件读取 / 端序工具
    //--------------------------------------------------------------------
    task automatic load_vec(input string path);
        begin
            fd = $fopen(path, "rb");
            if (fd == 0) begin $display("[FATAL] no %s", path); $finish; end
            code = $fread(fbuf, fd);
            $fclose(fd);
        end
    endtask

    task automatic load_chain(input string path);
        integer i;
        begin
            load_vec(path);
            exp_valid = (le32(0) == 1) ? 1 : 0;
            exp_N     = le32(4);
            for (i = 0; i < 2 + exp_N*2; ++i)
                exp_chain[i] = le32(i*4);
        end
    endtask

    function automatic [31:0] le32(input integer i);
        le32 = {fbuf[i+3], fbuf[i+2], fbuf[i+1], fbuf[i]};
    endfunction

    //--------------------------------------------------------------------
    // DDR 预载（u_pre 写方向 + TB 喂 .bin 字节；which=0→big 1→board5x8）
    //--------------------------------------------------------------------
    task automatic preload_ext(input integer which, input [31:0] base,
                               input [31:0] stride, input [15:0] row_bytes,
                               input [15:0] rows, input integer soff);
        integer i;
        begin
            if (which == 0) begin
                pre_big_cfg_base = base; pre_big_cfg_stride = stride;
                pre_big_cfg_row_bytes = row_bytes; pre_big_cfg_rows = rows;
                pre_big_cfg_offset = 0;
                pre_big_in_valid = 1'b0;
                @(negedge clk);
                pre_big_start = 1'b1;
                @(negedge clk);
                pre_big_start = 1'b0;
                i = 0;
                while (i < rows * row_bytes) begin
                    if (pre_big_in_ready) begin
                        pre_big_in_valid <= 1'b1;
                        pre_big_in_byte  <= fbuf[soff + i];
                        @(negedge clk);
                        i = i + 1;
                        pre_big_in_valid <= 1'b0;
                    end else begin
                        @(negedge clk);
                    end
                end
                begin
                    reg d_prev;
                    d_prev = 1'b0;
                    forever begin
                        @(negedge clk);
                        if (pre_big_done && !d_prev) break;
                        d_prev = pre_big_done;
                    end
                end
                repeat (3) @(negedge clk);
            end else begin
                pre_b5_cfg_base = base; pre_b5_cfg_stride = stride;
                pre_b5_cfg_row_bytes = row_bytes; pre_b5_cfg_rows = rows;
                pre_b5_cfg_offset = 0;
                pre_b5_in_valid = 1'b0;
                @(negedge clk);
                pre_b5_start = 1'b1;
                @(negedge clk);
                pre_b5_start = 1'b0;
                i = 0;
                while (i < rows * row_bytes) begin
                    if (pre_b5_in_ready) begin
                        pre_b5_in_valid <= 1'b1;
                        pre_b5_in_byte  <= fbuf[soff + i];
                        @(negedge clk);
                        i = i + 1;
                        pre_b5_in_valid <= 1'b0;
                    end else begin
                        @(negedge clk);
                    end
                end
                begin
                    reg d_prev;
                    d_prev = 1'b0;
                    forever begin
                        @(negedge clk);
                        if (pre_b5_done && !d_prev) break;
                        d_prev = pre_b5_done;
                    end
                end
                repeat (3) @(negedge clk);
            end
        end
    endtask

    // 等电平 0→1 沿（先同步当前值，兼容连续帧 done 残留）
    task automatic wait_done_big();
        reg d_prev;
        begin
            @(negedge clk);
            d_prev = big_done;
            forever begin
                @(negedge clk);
                if (big_done && !d_prev) break;
                d_prev = big_done;
            end
        end
    endtask

    task automatic wait_done_b5();
        reg d_prev;
        begin
            @(negedge clk);
            d_prev = b5_done;
            forever begin
                @(negedge clk);
                if (b5_done && !d_prev) break;
                d_prev = b5_done;
            end
        end
    endtask

    //--------------------------------------------------------------------
    // 读回 resp DDR 区并逐字节比对（fbuf[8+k]，跳过 u32 W/H 头）
    //--------------------------------------------------------------------
    task automatic verify_resp_ext(input integer which, input [31:0] base,
                                   input integer nbytes, input string tag);
        integer k, bad, rcnt;
        reg d_prev;
        begin
            bad = 0;
            if (which == 0) begin
                rb_big_cfg_base = base; rb_big_cfg_stride = 2048;
                rb_big_cfg_row_bytes = 2048; rb_big_cfg_rows = nbytes / 2048;
                rb_big_cfg_offset = 0;
                @(negedge clk);
                rb_big_start = 1'b1;
                @(negedge clk);
                rb_big_start = 1'b0;
                k = 0; rcnt = 0;
                d_prev = 1'b0;
                forever begin
                    @(negedge clk);
                    if (rb_big_out_valid && rb_big_out_ready) begin
                        if (fbuf[8 + k] !== rb_big_out_byte) begin
                            bad = bad + 1;
                            if (bad <= 5)
                                $display("[FAIL][%s] byte%0d got=%02x exp=%02x", tag, k,
                                         rb_big_out_byte, fbuf[8 + k]);
                        end
                        k = k + 1;
                        rcnt = rcnt + 1;
                    end
                    if (rb_big_done && !d_prev) break;
                    d_prev = rb_big_done;
                end
                repeat (3) @(negedge clk);
                $display("[%s] resp DDR readback err=%0d / %0d (rb_status=%02b)",
                         tag, bad, nbytes, rb_big_status);
                if (bad != 0 || rcnt != nbytes) err_cnt = err_cnt + 1;
            end else begin
                rb_b5_cfg_base = base; rb_b5_cfg_stride = 2048;
                rb_b5_cfg_row_bytes = 2048; rb_b5_cfg_rows = nbytes / 2048;
                rb_b5_cfg_offset = 0;
                @(negedge clk);
                rb_b5_start = 1'b1;
                @(negedge clk);
                rb_b5_start = 1'b0;
                k = 0; rcnt = 0;
                d_prev = 1'b0;
                forever begin
                    @(negedge clk);
                    if (rb_b5_out_valid && rb_b5_out_ready) begin
                        if (fbuf[8 + k] !== rb_b5_out_byte) begin
                            bad = bad + 1;
                            if (bad <= 5)
                                $display("[FAIL][%s] byte%0d got=%02x exp=%02x", tag, k,
                                         rb_b5_out_byte, fbuf[8 + k]);
                        end
                        k = k + 1;
                        rcnt = rcnt + 1;
                    end
                    if (rb_b5_done && !d_prev) break;
                    d_prev = rb_b5_done;
                end
                repeat (3) @(negedge clk);
                $display("[%s] resp DDR readback err=%0d / %0d (rb_status=%02b)",
                         tag, bad, nbytes, rb_b5_status);
                if (bad != 0 || rcnt != nbytes) err_cnt = err_cnt + 1;
            end
        end
    endtask

    //--------------------------------------------------------------------
    // 执行
    //--------------------------------------------------------------------
    integer t;
    initial begin
        string vec_dir = "../tests/build/vectors/";
        rst_n = 1'b0;
        big_process = 1'b0; b5_process = 1'b0;
        big_cfg_gray_base = 32'd0; big_cfg_gray_stride = 32'd0;
        big_cfg_gray_w = 16'd0; big_cfg_gray_h = 16'd0;
        big_cfg_ram_base = 21'd0; big_cfg_resp_base = 32'd0;
        big_cfg_resp_dump_en = 1'b0; big_cfg_pyr_en = 1'b0;
        b5_cfg_gray_base = 32'd0; b5_cfg_gray_stride = 32'd0;
        b5_cfg_gray_w = 16'd0; b5_cfg_gray_h = 16'd0;
        b5_cfg_ram_base = 21'd0; b5_cfg_resp_base = 32'd0;
        b5_cfg_resp_dump_en = 1'b0; b5_cfg_pyr_en = 1'b0;
        pre_big_start = 1'b0; pre_b5_start = 1'b0;
        rb_big_start = 1'b0; rb_b5_start = 1'b0;
        pre_big_in_valid = 1'b0; pre_big_in_byte = 8'd0;
        pre_b5_in_valid = 1'b0; pre_b5_in_byte = 8'd0;
        big_active = 1'b0; b5_active = 1'b0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // ================ 场景 1：big 帧 A（金字塔路径 + 响应写 DDR） ================
        $display("=== scene: big frame A (pyramid + resp dump) @ %0t ===", $time);
        big_active = 1'b1;
        load_vec({vec_dir, "m6_big_gray.bin"});
        preload_ext(0, 32'h0000_1000, 1280, 1280, 720, 0);
        // 帧配置
        big_cfg_gray_base = 32'h0000_1000; big_cfg_gray_stride = 1280;
        big_cfg_gray_w = 1280; big_cfg_gray_h = 720;
        big_cfg_ram_base = 21'd0; big_cfg_resp_base = 32'h0030_0000;
        big_cfg_resp_dump_en = 1'b1; big_cfg_pyr_en = 1'b1;
        load_chain({vec_dir, "m6_chain_big.bin"});
        oi_big = 0;
        @(negedge clk);
        big_process = 1'b1; @(negedge clk); big_process = 1'b0;
        wait_done_big();
        $display("[big A] frame done @ %0t status=%02b total=%0d grid_ok=%b pts=%0d",
                 $time, big_status, big_out_total, big_out_grid_ok, oi_big);
        if (big_status !== 2'b01 || big_out_grid_ok !== 1'b1 || oi_big != 40) err_cnt = err_cnt + 1;
        // 读回响应图 vs m7_resp_big.bin
        load_vec({vec_dir, "m7_resp_big.bin"});
        if (le32(0) !== 32'd640 || le32(4) !== 32'd360) begin
            $display("[FATAL][big] m7_resp_big.bin header (%0d,%0d) != (640,360)", le32(0), le32(4));
            err_cnt = err_cnt + 1;
        end
        verify_resp_ext(0, 32'h0030_0000, 230400*4, "big_A");
        repeat (5) @(negedge clk);

        // ================ 场景 3：big 帧 B（连续帧复用：cfg 重新锁存、done 清零） ================
        $display("=== scene: big frame B (frame reuse, resp_base reload) @ %0t ===", $time);
        // DDR 灰度区保留（帧 A 预载未变），gray_fetch 重新读；仅改 resp_base 验证 cfg 锁存
        big_cfg_resp_base = 32'h0031_0000;
        load_chain({vec_dir, "m6_chain_big.bin"});
        oi_big = 0;
        @(negedge clk);
        big_process = 1'b1; @(negedge clk); big_process = 1'b0;
        wait_done_big();
        $display("[big B] frame done @ %0t status=%02b total=%0d grid_ok=%b pts=%0d",
                 $time, big_status, big_out_total, big_out_grid_ok, oi_big);
        if (big_status !== 2'b01 || big_out_grid_ok !== 1'b1 || oi_big != 40) err_cnt = err_cnt + 1;
        load_vec({vec_dir, "m7_resp_big.bin"});
        verify_resp_ext(0, 32'h0031_0000, 230400*4, "big_B");
        repeat (5) @(negedge clk);

        // ================ 场景 4：big 帧 C（cfg_resp_dump_en=0：正常完成、resp 区不写） ================
        $display("=== scene: big frame C (resp dump disabled) @ %0t ===", $time);
        big_cfg_resp_dump_en = 1'b0;
        load_chain({vec_dir, "m6_chain_big.bin"});
        oi_big = 0;
        @(negedge clk);
        big_process = 1'b1; @(negedge clk); big_process = 1'b0;
        wait_done_big();
        $display("[big C] frame done @ %0t status=%02b total=%0d grid_ok=%b pts=%0d",
                 $time, big_status, big_out_total, big_out_grid_ok, oi_big);
        if (big_status !== 2'b01 || big_out_grid_ok !== 1'b1 || oi_big != 40) err_cnt = err_cnt + 1;
        big_active = 1'b0;
        repeat (5) @(negedge clk);

        // ================ 场景 2：board5x8 帧 D（native 路径 + 响应写 DDR） ================
        $display("=== scene: board5x8 frame D (native + resp dump) @ %0t ===", $time);
        b5_active = 1'b1;
        load_vec({vec_dir, "m5_board5x8_gray.bin"});
        preload_ext(1, 32'h0020_0000, 272, 272, 96, 0);
        b5_cfg_gray_base = 32'h0020_0000; b5_cfg_gray_stride = 272;
        b5_cfg_gray_w = 272; b5_cfg_gray_h = 96;
        b5_cfg_ram_base = 21'd0; b5_cfg_resp_base = 32'h0040_0000;
        b5_cfg_resp_dump_en = 1'b1; b5_cfg_pyr_en = 1'b0;
        load_chain({vec_dir, "m6_chain_board5x8.bin"});
        oi_b5 = 0;
        @(negedge clk);
        b5_process = 1'b1; @(negedge clk); b5_process = 1'b0;
        wait_done_b5();
        $display("[board5x8] frame done @ %0t status=%02b total=%0d grid_ok=%b pts=%0d",
                 $time, b5_status, b5_out_total, b5_out_grid_ok, oi_b5);
        if (b5_status !== 2'b01 || b5_out_grid_ok !== 1'b1 || oi_b5 != 40) err_cnt = err_cnt + 1;
        load_vec({vec_dir, "m7_resp_board5x8.bin"});
        if (le32(0) !== 32'd272 || le32(4) !== 32'd96) begin
            $display("[FATAL][board5x8] m7_resp_board5x8.bin header (%0d,%0d) != (272,96)", le32(0), le32(4));
            err_cnt = err_cnt + 1;
        end
        verify_resp_ext(1, 32'h0040_0000, 26112*4, "board5x8");
        b5_active = 1'b0;

        // ================= 结果 =================
        repeat (20) @(negedge clk);
        $display("==============================================");
        $display("FRAME-TOP TB: err=%0d proto_violations=%0d (big_frames=3 b5_frame=1)",
                 err_cnt, m_proto_viol);
        if (err_cnt == 0 && m_proto_viol == 0)
            $display("TB RESULT: ALL FRAME-TOP TESTS PASSED");
        else
            $display("TB RESULT: FAILED");
        $finish;
    end

    // 超时看门狗（big 3 帧 + board5x8 + 预载/读回 ≈ 300ms 仿真量级）
    initial begin
        #400_000_000;
        $display("[FATAL] global timeout err=%0d big_done=%b b5_done=%b pre_big_done=%b pre_b5_done=%b",
                 err_cnt, big_done, b5_done, pre_big_done, pre_b5_done);
        $display("[FATAL] big fsm st=%0d f_busy=%b f_done=%b pyr_busy=%b pyr_done=%b det_busy=%b det_done=%b w_busy=%b w_done=%b",
                 u_top_big.u_frame.st, u_top_big.u_fetch.busy, u_top_big.u_fetch.done,
                 u_top_big.u_pyr.busy, u_top_big.u_pyr.done,
                 u_top_big.u_det.busy, u_top_big.u_det.done,
                 u_top_big.u_wresp.busy, u_top_big.u_wresp.done);
        $display("[FATAL] big det stage=%0d d_st=%0d gray_rd_en=%b out_v=%b out_r=%b",
                 u_top_big.u_det.stage, u_top_big.u_det.d_st,
                 u_top_big.det_gray_rd_en, u_top_big.out_valid, u_top_big.out_ready);
        $display("[FATAL] big wresp state=%0d w_in_v=%b w_in_r=%b arb_wr_st=%0d arb_rd_st=%0d ddr_wr_st=%0d",
                 u_top_big.u_wresp.state, u_top_big.det_resp_dump_valid, u_top_big.det_resp_dump_ready,
                 u_top_big.u_arb.wr_state, u_top_big.u_arb.rd_state, u_ddr.wr_state);
        $finish;
    end

endmodule
