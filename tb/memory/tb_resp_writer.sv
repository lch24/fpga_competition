`timescale 1ns / 1ps
//==============================================================================
// tb_resp_writer.sv — M7.3 单元对拍：resp_ddr_writer（响应字流 → DDR 写）
//------------------------------------------------------------------------------
// 权威：ddr_memory_model.sv 连续字节流语义（写按 mem[addr+off+k] 逐字节，
//   keep 标记有效字节，逐拍校验 keep==期望，last 只在尾拍；违规→error=1
//   且 proto_violations 计数；写通道随机背压 gate_* 约 1/8 拍拉低）。
//
// 用例：
//   1) 小字数 257（=1028 字节 = 257 整字，keep 全 1111，尾字 last=1）：
//      逐字喂确定性 pattern(w+i) → 完成后读模型内存 mem[base+4k..+3]
//      （小端 4 字节 == 字 k）逐字比对。
//   2) 背压/随机延迟下多帧连续复用同一实例（start→done→再 start，
//      257/100/33 字，不同 base），验证复位干净（done 清零、状态机回 IDLE）。
//   3) cfg_words=0 → status=10 立即 done，模型零协议违规。
//   4) 写错误注入：模型 inject_error 命中写请求地址 → 写完成 error=1
//      → status=10、done（错误注入的写不生效，不比对内存）。
//
// 判据：所有字比对 0 错、proto_violations=0 → PASS。
//==============================================================================
module tb_resp_writer;

    localparam integer CLK_PERIOD = 10;
    logic clk = 1'b0;
    logic rst_n;
    always #(CLK_PERIOD/2) clk = ~clk;

    //--------------------------------------------------------------------
    // DDR 模型（随机延迟+背压+协议检查开）
    //--------------------------------------------------------------------
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

    // 读通道：本 TB 不用
    logic        m_rd_req_valid, m_rd_req_ready;
    logic [31:0] m_rd_req_addr,  m_rd_req_len;
    logic [15:0] m_rd_req_tag;
    logic        m_rd_ret_valid, m_rd_ret_ready;
    logic [31:0] m_rd_ret_data;
    logic [3:0]  m_rd_ret_keep;
    logic [15:0] m_rd_ret_tag;
    logic        m_rd_ret_last,  m_rd_ret_error;

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

    assign m_rd_req_valid = 1'b0;
    assign m_rd_req_addr  = 32'd0;
    assign m_rd_req_len   = 32'd0;
    assign m_rd_req_tag   = 16'd0;
    assign m_rd_ret_ready = 1'b0;

    //--------------------------------------------------------------------
    // DUT：resp_ddr_writer（写通道直连模型）
    //--------------------------------------------------------------------
    logic        start, busy, done;
    logic [1:0]  status;
    logic [31:0] cfg_base;
    logic [31:0] cfg_words;
    logic        in_valid, in_ready;
    logic [31:0] in_data;

    resp_ddr_writer #(
        .ADDR_W (32), .LEN_W (32), .TAG_W (16)
    ) u_dut (
        .clk (clk), .rst_n (rst_n),
        .start (start), .cfg_base (cfg_base), .cfg_words (cfg_words),
        .busy (busy), .done (done), .status (status),
        .in_valid (in_valid), .in_ready (in_ready), .in_data (in_data),
        .wr_req_valid (m_wr_req_valid), .wr_req_ready (m_wr_req_ready),
        .wr_req_addr (m_wr_req_addr), .wr_req_len (m_wr_req_len), .wr_req_tag (m_wr_req_tag),
        .wr_dat_valid (m_wr_dat_valid), .wr_dat_ready (m_wr_dat_ready),
        .wr_dat_data (m_wr_dat_data), .wr_dat_keep (m_wr_dat_keep), .wr_dat_last (m_wr_dat_last),
        .wr_done_valid (m_wr_done_valid), .wr_done_ready (m_wr_done_ready),
        .wr_done_tag (m_wr_done_tag), .wr_done_error (m_wr_done_error)
    );

    //--------------------------------------------------------------------
    // 期望 pattern：word k = w + k
    //--------------------------------------------------------------------
    function automatic [31:0] pat(input integer k, input [31:0] w);
        pat = w + k;
    endfunction

    integer err_cnt = 0;
    integer tot_words = 0;

    //--------------------------------------------------------------------
    // 启动：start 单拍。expect_busy=1 → 等 busy=1（正常用例，并确认 done 清零）；
    //        expect_busy=0 → 零字用例（busy 恒 0，不等）
    //--------------------------------------------------------------------
    task automatic kick(input [31:0] base, input [31:0] words, input integer expect_busy,
                        input string name);
        begin
            cfg_base  = base;
            cfg_words = words;
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            if (expect_busy) begin
                // 等 busy 拉高（start 被接受、cfg 锁存）
                while (busy !== 1'b1) @(negedge clk);
                if (done !== 1'b0) begin
                    $display("[%0s] ERROR: done not cleared by start (stale level)", name);
                    err_cnt = err_cnt + 1;
                end
            end
        end
    endtask

    //--------------------------------------------------------------------
    // 喂字流：逐字（in_valid && in_ready 接受），背压时等待
    //--------------------------------------------------------------------
    task automatic feed(input integer words, input [31:0] w, input string name);
        integer i;
        begin
            i = 0;
            while (i < words) begin
                if (in_ready) begin
                    in_valid <= 1'b1;
                    in_data  <= pat(i, w);
                    @(negedge clk);
                    i = i + 1;
                    in_valid <= 1'b0;
                end else begin
                    @(negedge clk);
                end
            end
            $display("[%0s] fed words=%0d", name, i);
        end
    endtask

    //--------------------------------------------------------------------
    // 等 done 完成（轮询 done && !busy，上限 max_clk 拍；不依赖 0→1 沿，
    //   兼容 cfg_words=0 时 done 可能无沿的情况）
    //--------------------------------------------------------------------
    task automatic wait_finish(input integer max_clk, input string name);
        integer c;
        begin
            c = 0;
            while (!(done === 1'b1 && busy === 1'b0) && c < max_clk) begin
                @(negedge clk);
                c = c + 1;
            end
            if (c >= max_clk) begin
                $display("[%0s] FATAL: done timeout (busy=%b done=%b status=%02b)", name, busy, done, status);
                err_cnt = err_cnt + 1;
            end else begin
                $display("[%0s] finished status=%02b (cycles after start ~%0d)", name, status, c);
            end
        end
    endtask

    //--------------------------------------------------------------------
    // 内存比对：mem[base+4k .. +3] 小端 4 字节 == 字 k
    //--------------------------------------------------------------------
    task automatic check_mem(input [31:0] base, input integer words, input [31:0] w,
                             input string name);
        integer k, bad;
        logic [31:0] got;
        begin
            bad = 0;
            for (k = 0; k < words; k = k + 1) begin
                got = {u_ddr.mem[base + 32'(4*k) + 32'd3],
                       u_ddr.mem[base + 32'(4*k) + 32'd2],
                       u_ddr.mem[base + 32'(4*k) + 32'd1],
                       u_ddr.mem[base + 32'(4*k) + 32'd0]};
                if (got !== pat(k, w)) begin
                    if (bad < 4)
                        $display("[%0s] word %0d mismatch got=%08h exp=%08h", name, k, got, pat(k, w));
                    bad = bad + 1;
                end
            end
            $display("[%0s] mem check err=%0d / %0d words (%0d bytes)", name, bad, words, words*4);
            tot_words = tot_words + words;
            if (bad != 0) err_cnt = err_cnt + 1;
        end
    endtask

    //--------------------------------------------------------------------
    // 主流程
    //--------------------------------------------------------------------
    initial begin
        rst_n = 1'b0;
        start = 1'b0; in_valid = 1'b0; in_data = 32'd0;
        cfg_base = 32'd0; cfg_words = 32'd0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // ---- 用例 1：257 字（1028 字节），确定性 pattern ----
        $display("=== case1 257 words @0x10000 ===");
        kick(32'h0001_0000, 32'd257, 1, "case1");
        feed(257, 32'hA5B6_C7D8, "case1");
        wait_finish(50000, "case1");
        $display("[case1] status=%02b (expect 01)", status);
        if (status !== 2'b01) err_cnt = err_cnt + 1;
        check_mem(32'h0001_0000, 257, 32'hA5B6_C7D8, "case1");
        repeat (3) @(negedge clk);

        // ---- 用例 2：多帧连续复用（257/100/33 字，不同 base）----
        $display("=== case2 multi-frame reuse ===");
        kick(32'h0002_0000, 32'd257, 1, "case2a");
        feed(257, 32'h1122_3344, "case2a");
        wait_finish(50000, "case2a");
        if (status !== 2'b01) err_cnt = err_cnt + 1;
        check_mem(32'h0002_0000, 257, 32'h1122_3344, "case2a");

        kick(32'h0003_0000, 32'd100, 1, "case2b");
        feed(100, 32'h0FFF_0000, "case2b");
        wait_finish(50000, "case2b");
        if (status !== 2'b01) err_cnt = err_cnt + 1;
        check_mem(32'h0003_0000, 100, 32'h0FFF_0000, "case2b");

        kick(32'h0004_0000, 32'd33, 1, "case2c");
        feed(33, 32'hDEAD_BEEF, "case2c");
        wait_finish(50000, "case2c");
        if (status !== 2'b01) err_cnt = err_cnt + 1;
        check_mem(32'h0004_0000, 33, 32'hDEAD_BEEF, "case2c");
        repeat (3) @(negedge clk);

        // ---- 用例 3：cfg_words=0 → status=10 立即 done ----
        $display("=== case3 zero words ===");
        kick(32'h0005_0000, 32'd0, 0, "case3");
        // 零字不发事务：不喂字，直接等完成
        wait_finish(1000, "case3");
        $display("[case3] status=%02b (expect 10)", status);
        if (status !== 2'b10) err_cnt = err_cnt + 1;
        repeat (3) @(negedge clk);

        // ---- 用例 4：写错误注入 → status=10 ----
        $display("=== case4 write-error injection 129 words ===");
        u_ddr.inject_error(32'h0006_0000, 32'h0006_3FFF);   // 命中写请求地址
        kick(32'h0006_0000, 32'd129, 1, "case4");
        feed(129, 32'hCAFE_0000, "case4");
        wait_finish(50000, "case4");
        u_ddr.clear_error_injection();
        $display("[case4] status=%02b (expect 10)", status);
        if (status !== 2'b10) err_cnt = err_cnt + 1;
        // 错误注入的写不生效（模拟地址不可达），不比对内存
        repeat (3) @(negedge clk);

        // ---- 结果 ----
        repeat (20) @(negedge clk);
        $display("==============================================");
        $display("RESP-WRITER TB: err=%0d words=%0d bytes=%0d proto_violations=%0d",
                 err_cnt, tot_words, tot_words*4, m_proto_viol);
        if (err_cnt == 0 && m_proto_viol == 0)
            $display("TB RESULT: ALL RESP-WRITER TESTS PASSED");
        else
            $display("TB RESULT: FAILED");
        $finish;
    end

    // 超时看门狗
    initial begin
        #10_000_000;
        $display("[FATAL] global timeout err=%0d busy=%b done=%b status=%02b",
                 err_cnt, busy, done, status);
        $display("  [dbg] u_dut.state=%0d u_dut.n_r=%0d u_adapter.wr_state=%0d",
                 u_dut.state, u_dut.n_r, u_dut.u_adapter.wr_state);
        $display("  [dbg] u_ddr.wr_state=%0d u_ddr.wr_off_r=%0d m_wr_done_valid=%b",
                 u_ddr.wr_state, u_ddr.wr_off_r, m_wr_done_valid);
        $finish;
    end

endmodule
