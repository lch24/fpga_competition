`timescale 1ns / 1ps
//==============================================================================
// tb_grayfetch.sv — M7.2 gray_fetch 位级对拍（层灰度 DDR→片上 RAM 流式加载）
//------------------------------------------------------------------------------
// 权威：tests/build/vectors/m6_big_gray.bin（1280×720 L0 灰度）、
//       m6_down_big.bin（u32 W,H + 源 1280×720 + 640×360 期望缩图段，
//       段起点 = 文件偏移 8 + 921600 = 921608，长度 230400）。
//
// 流程（每用例）：u_pre（raster_dma 写方向，TB 握手喂字节）把向量字节预载到
//   DDR → gray_fetch 加载到片上 gray RAM → 片上 RAM 逐字节 == 向量。
//
// 用例：
//   1) big L0   ：m6_big_gray.bin @ddr 0x1000，stride=1280，1280×720，ram_base=0
//   2) big L1   ：m6_down_big.bin 期望段（640×360）@ddr 0xE2000，stride=640，
//                 ram_base=0
//   3) 非对齐/尾字节：小图 127×63（m6_down_big 前 8001 字节）stride=131，
//                 ddr 0x200000，ram_base=0x40000（覆盖 base 偏移）
//   4) 读错误注入：inject_error 命中 → status=10
// DDR 模型：LATENCY_MIN=1..6，JITTER/BACKPRESSURE 开，PROTOCOL_CHECKS 开。
// 最终：err==0 && proto_violations==0 → ALL GRAYFETCH TESTS PASSED；超时保护。
//==============================================================================
module tb_grayfetch;

    localparam integer CLK_PERIOD = 10;
    logic clk = 1'b0;
    logic rst_n;
    always #(CLK_PERIOD/2) clk = ~clk;

    //--------------------------------------------------------------------
    // DDR 模型（随机延迟+背压开）
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
        .PROTOCOL_CHECKS (1'b1), .SEED (32'h1357_9BDF)
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
    // DUT：gray_fetch（DDR 读通道直连模型）
    //--------------------------------------------------------------------
    logic        start, busy, done;
    logic [1:0]  status;
    logic [31:0] cfg_ddr_base, cfg_stride;
    logic [15:0] cfg_w, cfg_h;
    logic [25:0] cfg_ram_base;
    logic        gray_wr_en;
    logic [25:0] gray_wr_addr;
    logic [7:0]  gray_wr_data;

    gray_fetch #(
        .ADDR_W (32), .LEN_W (32), .TAG_W (16),
        .MAX_PIX (1280*720), .GRAY_ADDR_W (26)
    ) u_dut (
        .clk (clk), .rst_n (rst_n),
        .start (start), .busy (busy), .done (done), .status (status),
        .cfg_ddr_base (cfg_ddr_base), .cfg_stride (cfg_stride),
        .cfg_w (cfg_w), .cfg_h (cfg_h), .cfg_ram_base (cfg_ram_base),
        .gray_wr_en (gray_wr_en), .gray_wr_addr (gray_wr_addr), .gray_wr_data (gray_wr_data),
        .rd_req_valid (m_rd_req_valid), .rd_req_ready (m_rd_req_ready),
        .rd_req_addr (m_rd_req_addr), .rd_req_len (m_rd_req_len), .rd_req_tag (m_rd_req_tag),
        .rd_ret_valid (m_rd_ret_valid), .rd_ret_ready (m_rd_ret_ready),
        .rd_ret_data (m_rd_ret_data), .rd_ret_keep (m_rd_ret_keep),
        .rd_ret_tag (m_rd_ret_tag), .rd_ret_last (m_rd_ret_last), .rd_ret_error (m_rd_ret_error)
    );

    //--------------------------------------------------------------------
    // u_pre：raster_dma 写方向，预载用（写通道归其所有，pre_active 互斥）
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

    // 模型写通道选通：pre_active=1 时归 u_pre（读通道恒归 DUT）
    reg pre_active;
    assign m_wr_req_valid  = pre_active ? pre_req_valid  : 1'b0;
    assign m_wr_req_addr   = pre_active ? pre_req_addr   : 32'd0;
    assign m_wr_req_len    = pre_active ? pre_req_len    : 32'd0;
    assign m_wr_req_tag    = pre_active ? pre_req_tag    : 16'd0;
    assign m_wr_dat_valid  = pre_active ? pre_dat_valid  : 1'b0;
    assign m_wr_dat_data   = pre_active ? pre_dat_data   : 32'd0;
    assign m_wr_dat_keep   = pre_active ? pre_dat_keep   : 4'd0;
    assign m_wr_dat_last   = pre_active ? pre_dat_last   : 1'b0;
    assign m_wr_done_ready = pre_active ? pre_done_ready : 1'b0;
    assign pre_req_ready   = pre_active ? m_wr_req_ready : 1'b0;
    assign pre_dat_ready   = pre_active ? m_wr_dat_ready : 1'b0;
    assign pre_done_valid  = pre_active ? m_wr_done_valid : 1'b0;
    assign pre_done_tag    = pre_active ? m_wr_done_tag   : 16'd0;
    assign pre_done_error  = pre_active ? m_wr_done_error : 1'b0;

    //--------------------------------------------------------------------
    // 片上 gray RAM 模型（同步写捕获：posedge 拍写入）
    //--------------------------------------------------------------------
    reg [7:0] gray_mem [0:1048575];
    always @(posedge clk) begin
        if (gray_wr_en) gray_mem[gray_wr_addr] <= gray_wr_data;
    end

    //--------------------------------------------------------------------
    // 向量缓冲
    //   big_gray = m6_big_gray.bin（921600 字节）
    //   down_buf = m6_down_big.bin（1152008 字节；L1 期望段 @偏移 921608）
    //   src      = 当前用例预载/比对源副本（prep_src 填充）
    //--------------------------------------------------------------------
    reg [7:0] big_gray [0:921599];
    reg [7:0] down_buf [0:1152007];
    reg [7:0] src [0:1048575];
    integer   fd, code;

    task automatic prep_src(input integer which, input integer off, input integer len);
        begin
            for (integer k = 0; k < len; ++k)
                src[k] = (which == 0) ? big_gray[off + k] : down_buf[off + k];
        end
    endtask

    integer err_cnt = 0;

    //--------------------------------------------------------------------
    // 预载：u_pre 写方向 + TB 握手喂 src 字节（tb_bilinear 握手模式）
    //--------------------------------------------------------------------
    task automatic preload(input [31:0] base, input [31:0] stride, input [15:0] row_bytes,
                           input [15:0] rows, input string name);
        integer i;
        begin
            pre_cfg_base = base; pre_cfg_stride = stride;
            pre_cfg_row_bytes = row_bytes; pre_cfg_rows = rows;
            pre_cfg_offset = 32'd0;
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
                    pre_in_byte  <= src[i];
                    @(negedge clk);
                    i = i + 1;
                    pre_in_valid <= 1'b0;
                end else begin
                    @(negedge clk);
                end
            end
            // 等 u_pre done（电平 0→1 沿）
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
            $display("[preload %s] done bytes=%0d", name, i);
        end
    endtask

    //--------------------------------------------------------------------
    // gray_fetch 加载：start 单拍 → 等 done 电平 0→1 沿
    //--------------------------------------------------------------------
    task automatic fetch(input [31:0] ddr_base, input [31:0] stride, input [15:0] w,
                         input [15:0] h, input [25:0] ram_base, input string name);
        reg d_prev;
        integer pix;
        begin
            cfg_ddr_base = ddr_base; cfg_stride = stride;
            cfg_w = w; cfg_h = h; cfg_ram_base = ram_base;
            pix = w * h;                     // 32 位上下文，避免 $display 截断
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            wait (busy === 1'b1);
            d_prev = 1'b0;
            forever begin
                @(negedge clk);
                if (done && !d_prev) break;
                d_prev = done;
            end
            $display("[fetch %s] done status=%02b pixels=%0d", name, status, pix);
        end
    endtask

    //--------------------------------------------------------------------
    // 比对：片上 gray RAM[ram_base..ram_base+len) == src
    //--------------------------------------------------------------------
    task automatic check_gray(input integer len, input integer ram_base, input string name);
        integer i, bad;
        begin
            bad = 0;
            for (i = 0; i < len; ++i)
                if (gray_mem[ram_base + i] !== src[i])
                    bad = bad + 1;
            $display("[check %s] gray RAM err=%0d / %0d", name, bad, len);
            if (bad != 0) err_cnt = err_cnt + 1;
        end
    endtask

    //--------------------------------------------------------------------
    // 主流程
    //--------------------------------------------------------------------
    initial begin
        rst_n = 1'b0;
        start = 1'b0; pre_start = 1'b0; pre_active = 1'b0;
        pre_in_valid = 1'b0; pre_in_byte = 8'd0;
        cfg_ddr_base = 32'd0; cfg_stride = 32'd0;
        cfg_w = 16'd0; cfg_h = 16'd0; cfg_ram_base = 26'd0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // ---- 向量加载 ----
        fd = $fopen("../../data/image/m6_big_gray.bin", "rb");
        if (fd == 0) begin $display("[FATAL] cannot open m6_big_gray.bin"); $finish; end
        code = $fread(big_gray, fd); $fclose(fd);
        $display("[TB] m6_big_gray.bin read %0d bytes (expect 921600)", code);
        fd = $fopen("../../data/image/m6_down_big.bin", "rb");
        if (fd == 0) begin $display("[FATAL] cannot open m6_down_big.bin"); $finish; end
        code = $fread(down_buf, fd); $fclose(fd);
        $display("[TB] m6_down_big.bin read %0d bytes (expect 1152008)", code);

        // ---- 用例 1：big L0（1280×720，stride=1280）----
        $display("=== case1 big-L0 1280x720 ===");
        prep_src(0, 0, 921600);                       // 源 = m6_big_gray.bin
        preload(32'h0000_1000, 1280, 1280, 720, "case1-pre");
        fetch(32'h0000_1000, 1280, 1280, 720, 26'd0, "case1");
        check_gray(921600, 0, "case1-bigL0");
        $display("[case1] status=%02b (expect 01)", status);
        if (status !== 2'b01) err_cnt = err_cnt + 1;
        repeat (5) @(negedge clk);

        // ---- 用例 2：big L1（m6_down_big 期望段 640×360 @文件偏移 8+921600）----
        $display("=== case2 big-L1 640x360 ===");
        prep_src(1, 921608, 230400);                  // 源 = m6_down_big.bin L1 期望段
        preload(32'h000E_2000, 640, 640, 360, "case2-pre");
        fetch(32'h000E_2000, 640, 640, 360, 26'd0, "case2");
        check_gray(230400, 0, "case2-bigL1");
        $display("[case2] status=%02b (expect 01)", status);
        if (status !== 2'b01) err_cnt = err_cnt + 1;
        repeat (5) @(negedge clk);

        // ---- 用例 3：非对齐/尾字节（127×63，stride=131，ram_base 非 0）----
        $display("=== case3 unalign/tail 127x63 stride=131 ===");
        prep_src(1, 0, 8001);                         // 源 = m6_down_big.bin 前 8001 字节
        preload(32'h0020_0000, 131, 127, 63, "case3-pre");
        fetch(32'h0020_0000, 131, 127, 63, 26'd262144, "case3");
        check_gray(8001, 262144, "case3-unalign");
        $display("[case3] status=%02b (expect 01)", status);
        if (status !== 2'b01) err_cnt = err_cnt + 1;
        repeat (5) @(negedge clk);

        // ---- 用例 4：读错误注入（命中加载区 → status=10）----
        $display("=== case4 read-error injection 64x16 ===");
        prep_src(1, 0, 1024);                         // 填充字节无所谓，仅占位
        preload(32'h0040_0000, 64, 64, 16, "case4-pre");
        u_ddr.inject_error(32'h0040_0000, 32'h0040_1FFF);
        fetch(32'h0040_0000, 64, 64, 16, 26'd0, "case4");
        u_ddr.clear_error_injection();
        $display("[case4] status=%02b (expect 10)", status);
        if (status !== 2'b10) err_cnt = err_cnt + 1;
        repeat (5) @(negedge clk);

        // ---- 结果 ----
        repeat (20) @(negedge clk);
        $display("==============================================");
        $display("GRAYFETCH TB: err=%0d proto_violations=%0d", err_cnt, m_proto_viol);
        if (err_cnt == 0 && m_proto_viol == 0)
            $display("TB RESULT: ALL GRAYFETCH TESTS PASSED");
        else
            $display("TB RESULT: FAILED");
        $finish;
    end

    // 超时看门狗（big 720 行 × 1280 像素预载+加载，给足时间）
    initial begin
        #50_000_000;
        $display("[FATAL] global timeout err=%0d busy=%b done=%b pre_active=%b status=%02b",
                 err_cnt, busy, done, pre_active, status);
        $display("  [dbg] u_dut.u_dma.rd_state=%0d u_dut.u_dma.y_r=%0d u_dut.cnt_r=%0d",
                 u_dut.u_dma.rd_state, u_dut.u_dma.y_r, u_dut.cnt_r);
        $display("  [dbg] u_ddr.rd_state=%0d u_ddr.rd_off_r=%0d u_pre.wr_state=%0d u_pre.y_r=%0d",
                 u_ddr.rd_state, u_ddr.rd_off_r, u_pre.wr_state, u_pre.y_r);
        $finish;
    end

endmodule
