`timescale 1ns / 1ps
//==============================================================================
// tb_ddr_copy.sv — M7.1 集成：raster_dma 读 A 区 → 字节流 → raster_dma 写 B 区
//------------------------------------------------------------------------------
// 验证 DDR 行拷贝闭环（连续字节语义，§3.2 + ddr_memory_model 行为权威）：
//   1) 预载：u_pre（raster_dma 写方向）把确定性 pattern 写入 A 区
//      （行布局 base/stride/row_bytes/offset 任意；u_pre 的打包已由 tb_dma 位级验证）
//   2) 拷贝：u_rd(cfg_dir=0, A 区) 读 → 字节流 → u_wr(cfg_dir=1, B 区) 写
//   3) 比对：B 区模型内存逐字节 == pattern（A 布局）
// 用例：对齐/非对齐/尾字节/双路背压/读错误注入。
// 模型写通道在 u_pre（预载）与 u_wr（拷贝）之间选通（pre_active 互斥）。
//==============================================================================
module tb_ddr_copy;

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
    // 配置寄存器（用例任务写；先声明供例化端口连接）
    //--------------------------------------------------------------------
    reg [31:0] rd_cfg_base,  rd_cfg_stride,  rd_cfg_offset;
    reg [15:0] rd_cfg_row_bytes, rd_cfg_rows;
    reg [31:0] wr_cfg_base,  wr_cfg_stride,  wr_cfg_offset;
    reg [15:0] wr_cfg_row_bytes, wr_cfg_rows;

    // 读/写/预载 3 个 raster_dma 实例
    logic        rd_start, rd_busy, rd_done;
    logic [1:0]  rd_status;
    logic        rd_out_valid, rd_out_ready;
    logic [7:0]  rd_out_byte;

    logic        wr_start, wr_busy, wr_done;
    logic [1:0]  wr_status;
    logic        wr_in_valid, wr_in_ready;
    logic [7:0]  wr_in_byte;

    logic        pre_start, pre_busy, pre_done;
    logic [1:0]  pre_status;
    logic        pre_in_valid, pre_in_ready;
    logic [7:0]  pre_in_byte;
    reg  [31:0]  pre_cfg_base, pre_cfg_stride, pre_cfg_offset;
    reg  [15:0]  pre_cfg_row_bytes, pre_cfg_rows;

    // DDR 事务信号（先声明供例化端口连接）
    logic        rd_req_valid, rd_req_ready;
    logic [31:0] rd_req_addr, rd_req_len;
    logic [15:0] rd_req_tag;
    logic        rd_ret_valid, rd_ret_ready, rd_ret_last, rd_ret_error;
    logic [31:0] rd_ret_data;
    logic [3:0]  rd_ret_keep;
    logic [15:0] rd_ret_tag;

    logic        wr_req_valid, wr_req_ready;
    logic [31:0] wr_req_addr, wr_req_len;
    logic [15:0] wr_req_tag;
    logic        wr_dat_valid, wr_dat_ready, wr_dat_last;
    logic [31:0] wr_dat_data;
    logic [3:0]  wr_dat_keep;
    logic        wr_done_valid, wr_done_ready, wr_done_error;
    logic [15:0] wr_done_tag;

    logic        pre_req_valid, pre_req_ready;
    logic [31:0] pre_req_addr, pre_req_len;
    logic [15:0] pre_req_tag;
    logic        pre_dat_valid, pre_dat_ready, pre_dat_last;
    logic [31:0] pre_dat_data;
    logic [3:0]  pre_dat_keep;
    logic        pre_done_valid, pre_done_ready, pre_done_error;
    logic [15:0] pre_done_tag;

    raster_dma u_rd (
        .clk (clk), .rst_n (rst_n),
        .start (rd_start), .busy (rd_busy), .done (rd_done), .status (rd_status),
        .cfg_base (rd_cfg_base), .cfg_stride (rd_cfg_stride),
        .cfg_row_bytes (rd_cfg_row_bytes), .cfg_rows (rd_cfg_rows),
        .cfg_offset (rd_cfg_offset), .cfg_dir (1'b0),
        .out_valid (rd_out_valid), .out_ready (rd_out_ready), .out_byte (rd_out_byte),
        .in_valid (1'b0), .in_ready (), .in_byte (8'd0),
        .rd_req_valid (rd_req_valid), .rd_req_ready (rd_req_ready),
        .rd_req_addr (rd_req_addr), .rd_req_len (rd_req_len), .rd_req_tag (rd_req_tag),
        .rd_ret_valid (rd_ret_valid), .rd_ret_ready (rd_ret_ready),
        .rd_ret_data (rd_ret_data), .rd_ret_keep (rd_ret_keep),
        .rd_ret_tag (rd_ret_tag), .rd_ret_last (rd_ret_last), .rd_ret_error (rd_ret_error),
        .wr_req_valid (), .wr_req_ready (1'b0), .wr_req_addr (), .wr_req_len (), .wr_req_tag (),
        .wr_dat_valid (), .wr_dat_ready (1'b0), .wr_dat_data (), .wr_dat_keep (), .wr_dat_last (),
        .wr_done_valid (1'b0), .wr_done_ready (), .wr_done_tag (16'd0), .wr_done_error (1'b0)
    );

    raster_dma u_wr (
        .clk (clk), .rst_n (rst_n),
        .start (wr_start), .busy (wr_busy), .done (wr_done), .status (wr_status),
        .cfg_base (wr_cfg_base), .cfg_stride (wr_cfg_stride),
        .cfg_row_bytes (wr_cfg_row_bytes), .cfg_rows (wr_cfg_rows),
        .cfg_offset (wr_cfg_offset), .cfg_dir (1'b1),
        .out_valid (), .out_ready (1'b1), .out_byte (),
        .in_valid (wr_in_valid), .in_ready (wr_in_ready), .in_byte (wr_in_byte),
        .rd_req_valid (), .rd_req_ready (1'b0), .rd_req_addr (), .rd_req_len (), .rd_req_tag (),
        .rd_ret_valid (1'b0), .rd_ret_ready (), .rd_ret_data (32'd0), .rd_ret_keep (4'd0),
        .rd_ret_tag (16'd0), .rd_ret_last (1'b0), .rd_ret_error (1'b0),
        .wr_req_valid (wr_req_valid), .wr_req_ready (wr_req_ready),
        .wr_req_addr (wr_req_addr), .wr_req_len (wr_req_len), .wr_req_tag (wr_req_tag),
        .wr_dat_valid (wr_dat_valid), .wr_dat_ready (wr_dat_ready),
        .wr_dat_data (wr_dat_data), .wr_dat_keep (wr_dat_keep), .wr_dat_last (wr_dat_last),
        .wr_done_valid (wr_done_valid), .wr_done_ready (wr_done_ready),
        .wr_done_tag (wr_done_tag), .wr_done_error (wr_done_error)
    );

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
    // 模型侧选通：读通道归 u_rd；写通道在 u_pre（预载）与 u_wr（拷贝）间互斥
    //--------------------------------------------------------------------
    reg pre_active;

    assign m_rd_req_valid = rd_req_valid;  assign rd_req_ready = m_rd_req_ready;
    assign m_rd_req_addr  = rd_req_addr;   assign m_rd_req_len = rd_req_len;
    assign m_rd_req_tag   = rd_req_tag;
    assign rd_ret_valid   = m_rd_ret_valid;  assign m_rd_ret_ready = rd_ret_ready;
    assign rd_ret_data    = m_rd_ret_data;   assign rd_ret_keep = m_rd_ret_keep;
    assign rd_ret_tag     = m_rd_ret_tag;    assign rd_ret_last = m_rd_ret_last;
    assign rd_ret_error   = m_rd_ret_error;

    assign m_wr_req_valid = pre_active ? pre_req_valid  : wr_req_valid;
    assign m_wr_req_addr  = pre_active ? pre_req_addr   : wr_req_addr;
    assign m_wr_req_len   = pre_active ? pre_req_len    : wr_req_len;
    assign m_wr_req_tag   = pre_active ? pre_req_tag    : wr_req_tag;
    assign m_wr_dat_valid = pre_active ? pre_dat_valid  : wr_dat_valid;
    assign m_wr_dat_data  = pre_active ? pre_dat_data   : wr_dat_data;
    assign m_wr_dat_keep  = pre_active ? pre_dat_keep   : wr_dat_keep;
    assign m_wr_dat_last  = pre_active ? pre_dat_last   : wr_dat_last;
    assign m_wr_done_ready = pre_active ? pre_done_ready : wr_done_ready;
    assign wr_req_ready   = pre_active ? 1'b0 : m_wr_req_ready;
    assign wr_dat_ready   = pre_active ? 1'b0 : m_wr_dat_ready;
    assign wr_done_valid  = pre_active ? 1'b0 : m_wr_done_valid;
    assign wr_done_tag    = pre_active ? 16'd0 : m_wr_done_tag;
    assign wr_done_error  = pre_active ? 1'b0 : m_wr_done_error;
    // u_pre 侧模型返回
    assign pre_req_ready  = pre_active ? m_wr_req_ready : 1'b0;
    assign pre_dat_ready  = pre_active ? m_wr_dat_ready : 1'b0;
    assign pre_done_valid = pre_active ? m_wr_done_valid : 1'b0;
    assign pre_done_tag   = pre_active ? m_wr_done_tag : 16'd0;
    assign pre_done_error = pre_active ? m_wr_done_error : 1'b0;

    //--------------------------------------------------------------------
    // 读→写字节流互连（插背压：每 8 拍停 1 拍）
    //--------------------------------------------------------------------
    reg [2:0] bp_cnt;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) bp_cnt <= 3'd0;
        else        bp_cnt <= bp_cnt + 3'd1;
    assign rd_out_ready = (bp_cnt == 3'd0) ? 1'b0 : wr_in_ready;
    assign wr_in_valid  = rd_out_valid && (bp_cnt != 3'd0);
    assign wr_in_byte   = rd_out_byte;

    //--------------------------------------------------------------------
    // 期望图案与校验
    //--------------------------------------------------------------------
    function automatic [7:0] pat(input integer y, input integer x, input integer seed);
        pat = (y * 131 + x * 7 + seed) & 8'hff;
    endfunction

    integer err_cnt = 0;

    // 预载：u_pre 写方向 + TB 喂 pattern 字节（tb_bilinear 握手模式）
    task automatic preload_a(input [31:0] base, input [31:0] stride, input [15:0] row_bytes,
                             input [15:0] rows, input [31:0] offset, input integer seed);
        integer i, y, x;
        begin
            pre_cfg_base = base; pre_cfg_stride = stride;
            pre_cfg_row_bytes = row_bytes; pre_cfg_rows = rows;
            pre_cfg_offset = offset;
            pre_active = 1'b1;
            pre_in_valid = 1'b0;
            @(negedge clk);
            pre_start = 1'b1;
            @(negedge clk);
            pre_start = 1'b0;
            i = 0;
            while (i < rows * row_bytes) begin
                if (pre_in_ready) begin
                    y = i / row_bytes; x = i % row_bytes;
                    pre_in_valid <= 1'b1;
                    pre_in_byte  <= pat(y, x, seed);
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
        end
    endtask

    // 比对：B 区模型内存逐字节 == pat
    task automatic check_b(input [31:0] base, input [31:0] stride, input [15:0] row_bytes,
                           input [15:0] rows, input [31:0] offset, input integer seed,
                           input string name);
        integer y, x, bad;
        begin
            bad = 0;
            for (y = 0; y < rows; y = y + 1)
                for (x = 0; x < row_bytes; x = x + 1)
                    if (u_ddr.mem[base + y*stride + offset + x] !== pat(y, x, seed))
                        bad = bad + 1;
            $display("[%0s] copy check err=%0d / %0d", name, bad, rows*row_bytes);
            if (bad != 0) err_cnt = err_cnt + 1;
        end
    endtask

    // 等电平 0→1 沿（内联负沿采样判沿）
    task automatic wait_rise(input string who);
        reg d_prev;
        begin
            d_prev = 1'b0;
            forever begin
                @(negedge clk);
                if (who == "rd" ? (rd_done && !d_prev) :
                    who == "wr" ? (wr_done && !d_prev) : 1'b0) break;
                d_prev = (who == "rd") ? rd_done : wr_done;
            end
        end
    endtask

    //--------------------------------------------------------------------
    // 用例执行
    //--------------------------------------------------------------------
    initial begin
        integer seed;
        rst_n = 1'b0;
        rd_start = 1'b0; wr_start = 1'b0; pre_start = 1'b0;
        pre_active = 1'b0; pre_in_valid = 1'b0; pre_in_byte = 8'd0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // ---- 用例 1：对齐拷贝（stride 含行尾 padding）----
        seed = 11;
        rd_cfg_base = 32'h0000_0100; rd_cfg_stride = 132; rd_cfg_row_bytes = 128;
        rd_cfg_rows = 64; rd_cfg_offset = 0;
        wr_cfg_base = 32'h0010_0000; wr_cfg_stride = 132; wr_cfg_row_bytes = 128;
        wr_cfg_rows = 64; wr_cfg_offset = 0;
        preload_a(rd_cfg_base, rd_cfg_stride, rd_cfg_row_bytes, rd_cfg_rows, rd_cfg_offset, seed);
        wr_start = 1'b1; @(negedge clk); wr_start = 1'b0;
        rd_start = 1'b1; @(negedge clk); rd_start = 1'b0;
        wait_rise("rd");
        wait_rise("wr");
        check_b(wr_cfg_base, wr_cfg_stride, wr_cfg_row_bytes, wr_cfg_rows, wr_cfg_offset, seed, "case1-align");
        $display("[case1] rd_status=%02b wr_status=%02b", rd_status, wr_status);
        if (rd_status !== 2'b01 || wr_status !== 2'b01) err_cnt = err_cnt + 1;
        repeat (5) @(negedge clk);

        // ---- 用例 2：非对齐（offset=2，row_bytes=127，stride=131）----
        seed = 23;
        rd_cfg_base = 32'h0002_0000; rd_cfg_stride = 131; rd_cfg_row_bytes = 127;
        rd_cfg_rows = 33; rd_cfg_offset = 2;
        wr_cfg_base = 32'h0011_0000; wr_cfg_stride = 131; wr_cfg_row_bytes = 127;
        wr_cfg_rows = 33; wr_cfg_offset = 2;
        preload_a(rd_cfg_base, rd_cfg_stride, rd_cfg_row_bytes, rd_cfg_rows, rd_cfg_offset, seed);
        wr_start = 1'b1; @(negedge clk); wr_start = 1'b0;
        rd_start = 1'b1; @(negedge clk); rd_start = 1'b0;
        wait_rise("rd");
        wait_rise("wr");
        check_b(wr_cfg_base, wr_cfg_stride, wr_cfg_row_bytes, wr_cfg_rows, wr_cfg_offset, seed, "case2-unalign");
        if (rd_status !== 2'b01 || wr_status !== 2'b01) err_cnt = err_cnt + 1;
        repeat (5) @(negedge clk);

        // ---- 用例 3：尾字节（row_bytes=130，stride=130 无 padding）----
        seed = 37;
        rd_cfg_base = 32'h0003_0000; rd_cfg_stride = 130; rd_cfg_row_bytes = 130;
        rd_cfg_rows = 17; rd_cfg_offset = 0;
        wr_cfg_base = 32'h0012_0000; wr_cfg_stride = 130; wr_cfg_row_bytes = 130;
        wr_cfg_rows = 17; wr_cfg_offset = 0;
        preload_a(rd_cfg_base, rd_cfg_stride, rd_cfg_row_bytes, rd_cfg_rows, rd_cfg_offset, seed);
        wr_start = 1'b1; @(negedge clk); wr_start = 1'b0;
        rd_start = 1'b1; @(negedge clk); rd_start = 1'b0;
        wait_rise("rd");
        wait_rise("wr");
        check_b(wr_cfg_base, wr_cfg_stride, wr_cfg_row_bytes, wr_cfg_rows, wr_cfg_offset, seed, "case3-tail");
        if (rd_status !== 2'b01 || wr_status !== 2'b01) err_cnt = err_cnt + 1;
        repeat (5) @(negedge clk);

        // ---- 用例 4：读错误注入（命中 A 区 → rd status=10）----
        seed = 41;
        rd_cfg_base = 32'h0004_0000; rd_cfg_stride = 64; rd_cfg_row_bytes = 64;
        rd_cfg_rows = 16; rd_cfg_offset = 0;
        wr_cfg_base = 32'h0013_0000; wr_cfg_stride = 64; wr_cfg_row_bytes = 64;
        wr_cfg_rows = 16; wr_cfg_offset = 0;
        preload_a(rd_cfg_base, rd_cfg_stride, rd_cfg_row_bytes, rd_cfg_rows, rd_cfg_offset, seed);
        u_ddr.inject_error(32'h0004_0000, 32'h0004_FFFF);
        wr_start = 1'b1; @(negedge clk); wr_start = 1'b0;
        rd_start = 1'b1; @(negedge clk); rd_start = 1'b0;
        // 只等读侧完成（读错误后字节流中断，写侧无源错误感知会等数据挂起——预期语义）
        wait_rise("rd");
        u_ddr.clear_error_injection();
        $display("[case4] rd_status=%02b (expect 10)", rd_status);
        if (rd_status !== 2'b10) err_cnt = err_cnt + 1;
        repeat (5) @(negedge clk);

        // ---- 结果 ----
        repeat (20) @(negedge clk);
        $display("==============================================");
        $display("DDR-COPY TB: err=%0d proto_violations=%0d", err_cnt, m_proto_viol);
        if (err_cnt == 0 && m_proto_viol == 0)
            $display("TB RESULT: ALL DDR-COPY TESTS PASSED");
        else
            $display("TB RESULT: FAILED");
        $finish;
    end

    // 超时看门狗
    initial begin
        #20_000_000;
        $display("[FATAL] global timeout err=%0d rd_done=%b wr_done=%b pre_active=%b",
                 err_cnt, rd_done, wr_done, pre_active);
        $display("  [dbg] u_rd.rd_state=%0d u_rd.y=%0d u_wr.wr_state=%0d u_wr.y=%0d u_pre.wr_state=%0d u_pre.y=%0d",
                 u_rd.rd_state, u_rd.y_r, u_wr.wr_state, u_wr.y_r, u_pre.wr_state, u_pre.y_r);
        $display("  [dbg] u_ddr.wr_state=%0d u_ddr.wr_off=%0d u_ddr.rd_state=%0d", u_ddr.wr_state, u_ddr.wr_off_r, u_ddr.rd_state);
        $display("  [dbg] pre_start=%b pre_in_valid=%b pre_in_ready=%b pre_done=%b pre_busy=%b",
                 pre_start, pre_in_valid, pre_in_ready, pre_done, pre_busy);
        $finish;
    end

endmodule
