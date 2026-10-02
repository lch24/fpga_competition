`timescale 1ns / 1ps
//==============================================================================
// tb_detect_ddr.sv — M7.2 集成：灰度经 DDR（gray_fetch）+ detect 全链位级对拍
//------------------------------------------------------------------------------
// 流程（big 场景，金字塔路径）：
//   1) DDR 预载 L0 灰度 m6_big_gray.bin @ddr_base=0x1000（stride=1280,1280×720）
//      （u_pre：raster_dma 写方向，TB 喂 .bin 字节——与 tb_ddr_copy 同款）
//   2) gray_fetch：L0 DDR → 片上 gray RAM base 0（921600 字节）
//   3) pyramid_ctrl：片上 L0 → 生成片上 L1 @base 921600（640×360）
//   4) detect_ctrl(DEPTH=2)：读片上灰度，native@L1 → 2p+0.5 → refine@L0
//      → 40 点 vs m6_chain_big.bin 逐位一致
// 场景 2（board5x8，native 路径）：DDR 预载 m5_board5x8_gray.bin（272×96）→
//   gray_fetch → detect_ctrl(DEPTH=1) → 40 点 vs m6_chain_board5x8.bin。
// 权威：tests/rtl/export_m6.cpp detect_chessboard_ref。
// 响应写 DDR（resp_tap→字宽写路径）为 M7.3 遗留，本 TB 只验证灰度经 DDR 的全链。
//==============================================================================
module tb_detect_ddr;

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
    // 片上 gray RAM（同步写；读 1 拍延迟；detect/pyramid 共享读，fetch/pyramid 写）
    //--------------------------------------------------------------------
    localparam integer GRAY_AW = 21;       // ≥ $clog2(1152000)
    reg [7:0]  gray_mem [0:(1<<GRAY_AW)-1];
    logic            gray_rd_en;
    logic [GRAY_AW-1:0] gray_rd_addr;
    logic [7:0]       gray_rd_data;
    wire             gray_wr_en;
    wire [GRAY_AW-1:0] gray_wr_addr;
    wire [7:0]       gray_wr_data;

    always @(posedge clk) begin
        if (!rst_n) gray_rd_data <= 8'd0;
        else if (gray_rd_en) gray_rd_data <= gray_mem[gray_rd_addr];
    end
    always @(posedge clk) begin
        if (gray_wr_en) gray_mem[gray_wr_addr] <= gray_wr_data;
    end

    //--------------------------------------------------------------------
    // u_pre：DDR 预载（raster_dma 写方向）
    //--------------------------------------------------------------------
    logic        pre_start, pre_busy, pre_done;
    logic [1:0]  pre_status;
    logic        pre_in_valid, pre_in_ready;
    logic [7:0]  pre_in_byte;
    reg  [31:0]  pre_cfg_base, pre_cfg_stride, pre_cfg_offset;
    reg  [15:0]  pre_cfg_row_bytes, pre_cfg_rows;
    logic        pre_req_valid, pre_req_ready;
    logic [31:0] pre_req_addr, pre_req_len;
    logic [15:0] pre_req_tag;
    logic        pre_dat_valid, pre_dat_ready, pre_dat_last;
    logic [31:0] pre_dat_data;
    logic [3:0]  pre_dat_keep;
    logic        pre_done_valid, pre_done_ready, pre_done_error;
    logic [15:0] pre_done_tag;

    raster_dma u_pre (
        .clk (clk), .rst_n (rst_n),
        .start (pre_start), .busy (pre_busy), .done (pre_done), .status (pre_status),
        .cfg_base (pre_cfg_base), .cfg_stride (pre_cfg_stride),
        .cfg_row_bytes (pre_cfg_row_bytes), .cfg_rows (pre_cfg_rows),
        .cfg_offset (pre_cfg_offset), .cfg_dir (1'b1),
        .out_valid (), .out_ready (1'b1), .out_byte (),
        .in_valid (pre_in_valid), .in_ready (pre_in_ready), .in_byte (pre_in_byte),
        .rd_req_valid (), .rd_req_ready (1'b0), .rd_req_addr (), .rd_req_len (), .rd_req_tag (),
        .rd_ret_valid (1'b0), .rd_ret_ready (), .rd_ret_data (32'd0), .rd_ret_keep (4'd0),
        .rd_ret_tag (16'd0), .rd_ret_last (1'b0), .rd_ret_error (1'b0),
        .wr_req_valid (pre_req_valid), .wr_req_ready (pre_req_ready),
        .wr_req_addr (pre_req_addr), .wr_req_len (pre_req_len), .wr_req_tag (pre_req_tag),
        .wr_dat_valid (pre_dat_valid), .wr_dat_ready (pre_dat_ready),
        .wr_dat_data (pre_dat_data), .wr_dat_keep (pre_dat_keep), .wr_dat_last (pre_dat_last),
        .wr_done_valid (pre_done_valid), .wr_done_ready (pre_done_ready),
        .wr_done_tag (pre_done_tag), .wr_done_error (pre_done_error)
    );

    //--------------------------------------------------------------------
    // u_fetch：gray_fetch（DDR → 片上 gray RAM）
    //--------------------------------------------------------------------
    logic        f_start, f_busy, f_done;
    logic [1:0]  f_status;
    reg  [31:0]  f_cfg_base, f_cfg_stride;
    reg  [15:0]  f_cfg_w, f_cfg_h;
    reg  [20:0]  f_cfg_ram_base;
    logic        f_gray_wr_en;
    logic [20:0] f_gray_wr_addr;
    logic [7:0]  f_gray_wr_data;
    logic        f_rd_req_valid, f_rd_req_ready;
    logic [31:0] f_rd_req_addr, f_rd_req_len;
    logic [15:0] f_rd_req_tag;
    logic        f_rd_ret_valid, f_rd_ret_ready, f_rd_ret_last, f_rd_ret_error;
    logic [31:0] f_rd_ret_data;
    logic [3:0]  f_rd_ret_keep;
    logic [15:0] f_rd_ret_tag;

    gray_fetch #(.GRAY_ADDR_W (21)) u_fetch (
        .clk (clk), .rst_n (rst_n),
        .start (f_start), .busy (f_busy), .done (f_done), .status (f_status),
        .cfg_ddr_base (f_cfg_base), .cfg_stride (f_cfg_stride),
        .cfg_w (f_cfg_w), .cfg_h (f_cfg_h), .cfg_ram_base (f_cfg_ram_base),
        .gray_wr_en (f_gray_wr_en), .gray_wr_addr (f_gray_wr_addr), .gray_wr_data (f_gray_wr_data),
        .rd_req_valid (f_rd_req_valid), .rd_req_ready (f_rd_req_ready),
        .rd_req_addr (f_rd_req_addr), .rd_req_len (f_rd_req_len), .rd_req_tag (f_rd_req_tag),
        .rd_ret_valid (f_rd_ret_valid), .rd_ret_ready (f_rd_ret_ready),
        .rd_ret_data (f_rd_ret_data), .rd_ret_keep (f_rd_ret_keep),
        .rd_ret_tag (f_rd_ret_tag), .rd_ret_last (f_rd_ret_last), .rd_ret_error (f_rd_ret_error)
    );

    //--------------------------------------------------------------------
    // u_pyr + u_det（big DEPTH=2）
    //--------------------------------------------------------------------
    logic        pyr_start, pyr_busy, pyr_done;
    logic [2:0]  pyr_level_count;
    logic [20:0] pyr_lvl_base [0:3];
    logic [10:0] pyr_lvl_w    [0:3];
    logic [10:0] pyr_lvl_h    [0:3];
    logic        pyr_gray_en;
    logic [20:0] pyr_gray_addr;
    logic [7:0]  pyr_gray_q;
    wire         pyr_wr_en;
    wire [20:0]  pyr_wr_addr;
    wire [7:0]   pyr_wr_data;

    pyramid_ctrl #(
        .MAX_W (2560), .MAX_H (1440), .MAX_DEPTH (4), .GRAY_ADDR_W (21)
    ) u_pyr (
        .clk (clk), .rst_n (rst_n),
        .start (pyr_start), .busy (pyr_busy), .done (pyr_done),
        .cfg_w0 (11'd1280), .cfg_h0 (11'd720), .cfg_base0 (21'd0),
        .gray_rd_en (pyr_gray_en), .gray_rd_addr (pyr_gray_addr), .gray_rd_data (gray_rd_data),
        .gray_wr_en (pyr_wr_en), .gray_wr_addr (pyr_wr_addr), .gray_wr_data (pyr_wr_data),
        .level_count (pyr_level_count),
        .level_base (pyr_lvl_base), .level_w (pyr_lvl_w), .level_h (pyr_lvl_h)
    );

    logic        det_start, det_busy, det_done;
    logic [1:0]  det_status;
    logic        det_gray_en;
    logic [20:0] det_gray_addr;
    logic [7:0]  det_gray_q;
    logic        det_out_valid, det_out_ready, det_out_grid_ok;
    logic [31:0] det_out_x, det_out_y;
    logic [15:0] det_out_total;
    logic        resp_tap_valid;
    logic [31:0] resp_tap_data;

    detect_ctrl #(
        .W0 (1280), .H0 (720), .DEPTH (2), .GRAY_ADDR_W (21)
    ) u_det (
        .clk (clk), .rst_n (rst_n),
        .start (det_start), .busy (det_busy), .done (det_done), .status (det_status),
        .cfg_base0 (21'd0),
        .gray_rd_en (det_gray_en), .gray_rd_addr (det_gray_addr), .gray_rd_data (gray_rd_data),
        .resp_tap_valid (resp_tap_valid), .resp_tap_data (resp_tap_data),
        .cfg_resp_dump_en (1'b0), .resp_dump_valid (), .resp_dump_ready (1'b0),
        .resp_dump_data (), .resp_dump_done (),
        .out_valid (det_out_valid), .out_ready (det_out_ready),
        .out_x (det_out_x), .out_y (det_out_y),
        .out_total (det_out_total), .out_grid_ok (det_out_grid_ok)
    );

    //--------------------------------------------------------------------
    // u_det2（board5x8 DEPTH=1）
    //--------------------------------------------------------------------
    logic        det2_start, det2_busy, det2_done;
    logic [1:0]  det2_status;
    logic        det2_gray_en;
    logic [20:0] det2_gray_addr;
    logic [7:0]  det2_gray_q;
    logic        det2_out_valid, det2_out_ready, det2_out_grid_ok;
    logic [31:0] det2_out_x, det2_out_y;
    logic [15:0] det2_out_total;

    detect_ctrl #(
        .W0 (272), .H0 (96), .DEPTH (1), .GRAY_ADDR_W (21)
    ) u_det2 (
        .clk (clk), .rst_n (rst_n),
        .start (det2_start), .busy (det2_busy), .done (det2_done), .status (det2_status),
        .cfg_base0 (21'd0),
        .gray_rd_en (det2_gray_en), .gray_rd_addr (det2_gray_addr), .gray_rd_data (gray_rd_data),
        .resp_tap_valid (), .resp_tap_data (),
        .cfg_resp_dump_en (1'b0), .resp_dump_valid (), .resp_dump_ready (1'b0),
        .resp_dump_data (), .resp_dump_done (),
        .out_valid (det2_out_valid), .out_ready (det2_out_ready),
        .out_x (det2_out_x), .out_y (det2_out_y),
        .out_total (det2_out_total), .out_grid_ok (det2_out_grid_ok)
    );

    //--------------------------------------------------------------------
    // 灰度 RAM 读/写选通（阶段互斥）
    //--------------------------------------------------------------------
    reg fetch_active, pyr_active, det_active, det2_active;
    assign gray_rd_en   = pyr_active ? pyr_gray_en : det_active ? det_gray_en : det2_active ? det2_gray_en : 1'b0;
    assign gray_rd_addr = pyr_active ? pyr_gray_addr : det_active ? det_gray_addr : det2_gray_addr;
    assign gray_wr_en   = fetch_active ? f_gray_wr_en : pyr_active ? pyr_wr_en : 1'b0;
    assign gray_wr_addr = fetch_active ? f_gray_wr_addr : pyr_active ? pyr_wr_addr : 21'd0;
    assign gray_wr_data = fetch_active ? f_gray_wr_data : pyr_active ? pyr_wr_data : 8'd0;

    // 模型读通道归 u_fetch
    assign m_rd_req_valid = f_rd_req_valid;  assign f_rd_req_ready = m_rd_req_ready;
    assign m_rd_req_addr  = f_rd_req_addr;   assign m_rd_req_len = f_rd_req_len;
    assign m_rd_req_tag   = f_rd_req_tag;
    assign f_rd_ret_valid = m_rd_ret_valid;  assign m_rd_ret_ready = f_rd_ret_ready;
    assign f_rd_ret_data  = m_rd_ret_data;   assign f_rd_ret_keep = m_rd_ret_keep;
    assign f_rd_ret_tag   = m_rd_ret_tag;    assign f_rd_ret_last = m_rd_ret_last;
    assign f_rd_ret_error = m_rd_ret_error;
    // 模型写通道归 u_pre（预载）
    reg pre_active;
    assign m_wr_req_valid = pre_active ? pre_req_valid : 1'b0;
    assign m_wr_req_addr  = pre_active ? pre_req_addr  : 32'd0;
    assign m_wr_req_len   = pre_active ? pre_req_len   : 32'd0;
    assign m_wr_req_tag   = pre_active ? pre_req_tag   : 16'd0;
    assign m_wr_dat_valid = pre_active ? pre_dat_valid : 1'b0;
    assign m_wr_dat_data  = pre_active ? pre_dat_data  : 32'd0;
    assign m_wr_dat_keep  = pre_active ? pre_dat_keep  : 4'd0;
    assign m_wr_dat_last  = pre_active ? pre_dat_last  : 1'b0;
    assign m_wr_done_ready = pre_active ? pre_done_ready : 1'b0;
    assign pre_req_ready  = pre_active ? m_wr_req_ready : 1'b0;
    assign pre_dat_ready  = pre_active ? m_wr_dat_ready : 1'b0;
    assign pre_done_valid = pre_active ? m_wr_done_valid : 1'b0;
    assign pre_done_tag   = pre_active ? m_wr_done_tag : 16'd0;
    assign pre_done_error = pre_active ? m_wr_done_error : 1'b0;

    //--------------------------------------------------------------------
    // 期望向量与输出采集
    //--------------------------------------------------------------------
    reg [31:0] exp_chain [0:1023];
    integer    exp_N, exp_valid, err_cnt = 0, oi = 0, oi2 = 0;

    always @(posedge clk) begin
        if (det_out_valid && det_out_ready && det_active) begin
            if (oi >= exp_N || det_out_x !== exp_chain[2 + oi*2] ||
                det_out_y !== exp_chain[3 + oi*2]) begin
                err_cnt = err_cnt + 1;
                if (err_cnt <= 10)
                    $display("[FAIL][big] pt%0d got=(%08x,%08x) exp=(%08x,%08x)",
                             oi, det_out_x, det_out_y, exp_chain[2+oi*2], exp_chain[3+oi*2]);
            end
            oi = oi + 1;
        end
        if (det2_out_valid && det2_out_ready && det2_active) begin
            if (oi2 >= exp_N || det2_out_x !== exp_chain[2 + oi2*2] ||
                det2_out_y !== exp_chain[3 + oi2*2]) begin
                err_cnt = err_cnt + 1;
                if (err_cnt <= 10)
                    $display("[FAIL][board5x8] pt%0d got=(%08x,%08x) exp=(%08x,%08x)",
                             oi2, det2_out_x, det2_out_y, exp_chain[2+oi2*2], exp_chain[3+oi2*2]);
            end
            oi2 = oi2 + 1;
        end
    end
    assign det_out_ready  = 1'b1;
    assign det2_out_ready = 1'b1;

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
    // DDR 预载（u_pre 写方向 + TB 喂 .bin 字节）
    //--------------------------------------------------------------------
    task automatic preload_ddr(input [31:0] base, input [31:0] stride,
                               input [15:0] row_bytes, input [15:0] rows,
                               input integer soff);   // fbuf 源偏移
        integer i;
        begin
            pre_cfg_base = base; pre_cfg_stride = stride;
            pre_cfg_row_bytes = row_bytes; pre_cfg_rows = rows;
            pre_cfg_offset = 0;
            pre_active = 1'b1;
            pre_in_valid = 1'b0;
            @(negedge clk);
            pre_start = 1'b1;
            @(negedge clk);
            pre_start = 1'b0;
            i = 0;
            while (i < rows * row_bytes) begin
                if (pre_in_ready) begin
                    pre_in_valid <= 1'b1;
                    pre_in_byte  <= fbuf[soff + i];
                    @(negedge clk);
                    i = i + 1;
                    pre_in_valid <= 1'b0;
                end else begin
                    @(negedge clk);
                end
            end
            begin
                reg d_prev;
                d_prev = 1'b0;
                forever begin
                    @(negedge clk);
                    if (pre_done && !d_prev) break;
                    d_prev = pre_done;
                end
            end
            pre_active = 1'b0;
            repeat (3) @(negedge clk);
        end
    endtask

    // 等电平 0→1 沿
    task automatic wait_rise(input string who);
        reg d_prev;
        begin
            d_prev = 1'b0;
            forever begin
                @(negedge clk);
                if (who == "f" ? (f_done && !d_prev) :
                    who == "pyr" ? (pyr_done && !d_prev) :
                    who == "det" ? (det_done && !d_prev) :
                    who == "det2" ? (det2_done && !d_prev) : 1'b0) break;
                d_prev = (who == "f") ? f_done : (who == "pyr") ? pyr_done :
                         (who == "det") ? det_done : det2_done;
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
        pre_start = 1'b0; f_start = 1'b0; pyr_start = 1'b0;
        det_start = 1'b0; det2_start = 1'b0;
        pre_active = 1'b0; fetch_active = 1'b0;
        pyr_active = 1'b0; det_active = 1'b0; det2_active = 1'b0;
        pre_in_valid = 1'b0; pre_in_byte = 8'd0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // ================= 场景 1：big（灰度经 DDR，金字塔路径） =================
        $display("=== scene: big via DDR (pyramid path) ===");
        // DDR 预载 L0 灰度
        load_vec({vec_dir, "m6_big_gray.bin"});
        preload_ddr(32'h0000_1000, 1280, 1280, 720, 0);
        // gray_fetch L0 → 片上 gray RAM base 0
        f_cfg_base = 32'h0000_1000; f_cfg_stride = 1280; f_cfg_w = 1280; f_cfg_h = 720;
        f_cfg_ram_base = 21'd0;
        fetch_active = 1'b1;
        @(negedge clk);
        f_start = 1'b1; @(negedge clk); f_start = 1'b0;
        wait_rise("f");
        $display("[big] gray_fetch L0 done status=%02b", f_status);
        if (f_status !== 2'b01) err_cnt = err_cnt + 1;
        // 校验片上 L0 == m6_big_gray.bin
        begin
            automatic integer bad = 0, k;
            for (k = 0; k < 1280*720; ++k)
                if (gray_mem[k] !== fbuf[k]) bad = bad + 1;
            $display("[big] onchip L0 err=%0d / %0d", bad, 1280*720);
            if (bad != 0) err_cnt = err_cnt + 1;
        end
        // pyramid L1（片上）
        fetch_active = 1'b0;
        pyr_active = 1'b1;
        @(negedge clk);
        pyr_start = 1'b1; @(negedge clk); pyr_start = 1'b0;
        wait_rise("pyr");
        $display("[big] pyramid done count=%0d", pyr_level_count);
        if (pyr_level_count != 2) err_cnt = err_cnt + 1;
        pyr_active = 1'b0;
        // detect DEPTH=2
        load_chain({vec_dir, "m6_chain_big.bin"});
        det_active = 1'b1;
        @(negedge clk);
        det_start = 1'b1; @(negedge clk); det_start = 1'b0;
        wait_rise("det");
        $display("[big] detect done status=%02b total=%0d grid_ok=%b got=%0d",
                 det_status, det_out_total, det_out_grid_ok, oi);
        if (det_status !== 2'b01 || det_out_grid_ok !== 1'b1 || oi != 40) err_cnt = err_cnt + 1;
        det_active = 1'b0;
        repeat (5) @(negedge clk);

        // ================= 场景 2：board5x8（灰度经 DDR，native 路径） =================
        $display("=== scene: board5x8 via DDR (native path) ===");
        load_vec({vec_dir, "m5_board5x8_gray.bin"});
        preload_ddr(32'h0020_0000, 272, 272, 96, 0);
        f_cfg_base = 32'h0020_0000; f_cfg_stride = 272; f_cfg_w = 272; f_cfg_h = 96;
        f_cfg_ram_base = 21'd0;
        fetch_active = 1'b1;
        @(negedge clk);
        f_start = 1'b1; @(negedge clk); f_start = 1'b0;
        wait_rise("f");
        $display("[board5x8] gray_fetch done status=%02b", f_status);
        if (f_status !== 2'b01) err_cnt = err_cnt + 1;
        fetch_active = 1'b0;
        load_chain({vec_dir, "m6_chain_board5x8.bin"});
        det2_active = 1'b1;
        @(negedge clk);
        det2_start = 1'b1; @(negedge clk); det2_start = 1'b0;
        wait_rise("det2");
        $display("[board5x8] detect done status=%02b total=%0d grid_ok=%b got=%0d",
                 det2_status, det2_out_total, det2_out_grid_ok, oi2);
        if (det2_status !== 2'b01 || det2_out_grid_ok !== 1'b1 || oi2 != 40) err_cnt = err_cnt + 1;
        det2_active = 1'b0;

        // ================= 结果 =================
        repeat (20) @(negedge clk);
        $display("==============================================");
        $display("DETECT-DDR TB: err=%0d proto_violations=%0d (big_pts=%0d board_pts=%0d)",
                 err_cnt, m_proto_viol, oi, oi2);
        if (err_cnt == 0 && m_proto_viol == 0 && oi == 40 && oi2 == 40)
            $display("TB RESULT: ALL DETECT-DDR TESTS PASSED");
        else
            $display("TB RESULT: FAILED");
        $finish;
    end

    // 超时看门狗（big 全链 ~63ms 仿真 + 预载/加载/金字塔）
    initial begin
        #800_000_000;
        $display("[FATAL] global timeout err=%0d f_done=%b pyr_done=%b det_done=%b det2_done=%b",
                 err_cnt, f_done, pyr_done, det_done, det2_done);
        $finish;
    end

endmodule
