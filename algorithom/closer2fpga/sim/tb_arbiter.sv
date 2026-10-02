`timescale 1ns / 1ps
//==============================================================================
// tb_arbiter.sv — ddr_port_arbiter 单元对拍（M8 交付）
//------------------------------------------------------------------------------
// 验证内容（冻结契约）：
//   1) 两读客户端交错（随机间隔请求 + 模型随机背压）→ 各读回数据逐字节正确、
//      无串扰；
//   2) 两写客户端交错（随机背压）→ 模型内存各写区内容逐字节正确；
//   3) 读写并发（1 读 + 1 写同时持续，正/反两相）→ 读写通道独立并行、
//      数据互不污染；
//   4) round-robin 公平：两客户端持续有请求 → 两者都被服务（无饿死）；
//   5) 在途单笔：客户端请求被接受后立刻再拉 valid → ready 拉低阻塞
//      （不吞请求，事务完成后接受）；
//   6) 零长：len==0 请求 → 客户端侧正常握手、伪返回/伪完成 error=1、
//      模型零协议违规。
// 模型参数：LATENCY_MIN=1、LATENCY_MAX=6、JITTER/BACKPRESSURE/
//           PROTOCOL_CHECKS 全开，SEED 任意固定。
// 握手检测：posedge 采样 fire 标志（与 tb_adapter 一致，避免就绪瞬间误判）。
// 通过标准：errs==0 且模型 proto_violations==0 →
//   "TB RESULT: ALL ARBITER TESTS PASSED"。
//==============================================================================
module tb_arbiter;

    localparam CLK_PERIOD = 10;

    logic clk = 1'b0;
    logic rst_n;
    always #(CLK_PERIOD/2) clk = ~clk;

    //--------------------------------------------------------------------
    // DUT：ddr_port_arbiter（N_RD=2, N_WR=2）+ ddr_memory_model
    //--------------------------------------------------------------------
    // 模型侧信号
    logic        m_rd_req_valid, m_rd_req_ready;
    logic [31:0] m_rd_req_addr,  m_rd_req_len_bytes;
    logic [15:0] m_rd_req_tag;
    logic        m_rd_ret_valid, m_rd_ret_ready;
    logic [31:0] m_rd_ret_data;
    logic [3:0]  m_rd_ret_keep;
    logic [15:0] m_rd_ret_tag;
    logic        m_rd_ret_last, m_rd_ret_error;
    logic        m_wr_req_valid, m_wr_req_ready;
    logic [31:0] m_wr_req_addr,  m_wr_req_len_bytes;
    logic [15:0] m_wr_req_tag;
    logic        m_wr_dat_valid, m_wr_dat_ready;
    logic [31:0] m_wr_dat_data;
    logic [3:0]  m_wr_dat_keep;
    logic        m_wr_dat_last;
    logic        m_wr_cplt_valid, m_wr_cplt_ready;
    logic [15:0] m_wr_cplt_tag;
    logic        m_wr_cplt_error;
    logic [15:0] proto_violations;

    // 客户端侧信号（数组）
    logic        rd_req_valid [0:1], rd_req_ready [0:1];
    logic [31:0] rd_req_addr [0:1], rd_req_len [0:1];
    logic [15:0] rd_req_tag [0:1];
    logic        rd_ret_valid [0:1], rd_ret_ready [0:1];
    logic [31:0] rd_ret_data [0:1];
    logic [3:0]  rd_ret_keep [0:1];
    logic [15:0] rd_ret_tag [0:1];
    logic        rd_ret_last [0:1], rd_ret_error [0:1];
    logic        wr_req_valid [0:1], wr_req_ready [0:1];
    logic [31:0] wr_req_addr [0:1], wr_req_len [0:1];
    logic [15:0] wr_req_tag [0:1];
    logic        wr_dat_valid [0:1], wr_dat_ready [0:1];
    logic [31:0] wr_dat_data [0:1];
    logic [3:0]  wr_dat_keep [0:1];
    logic        wr_dat_last [0:1];
    logic        wr_done_valid [0:1], wr_done_ready [0:1];
    logic [15:0] wr_done_tag [0:1];
    logic        wr_done_error [0:1];

    ddr_port_arbiter #(
        .ADDR_W (32), .LEN_W (32), .TAG_W (16),
        .N_RD (2), .N_WR (2)
    ) u_arb (
        .clk (clk), .rst_n (rst_n),
        .rd_req_valid  (rd_req_valid),  .rd_req_ready  (rd_req_ready),
        .rd_req_addr   (rd_req_addr),   .rd_req_len    (rd_req_len),
        .rd_req_tag    (rd_req_tag),
        .rd_ret_valid  (rd_ret_valid),  .rd_ret_ready  (rd_ret_ready),
        .rd_ret_data   (rd_ret_data),   .rd_ret_keep   (rd_ret_keep),
        .rd_ret_tag    (rd_ret_tag),    .rd_ret_last   (rd_ret_last),
        .rd_ret_error  (rd_ret_error),
        .wr_req_valid  (wr_req_valid),  .wr_req_ready  (wr_req_ready),
        .wr_req_addr   (wr_req_addr),   .wr_req_len    (wr_req_len),
        .wr_req_tag    (wr_req_tag),
        .wr_dat_valid  (wr_dat_valid),  .wr_dat_ready  (wr_dat_ready),
        .wr_dat_data   (wr_dat_data),   .wr_dat_keep   (wr_dat_keep),
        .wr_dat_last   (wr_dat_last),
        .wr_done_valid (wr_done_valid), .wr_done_ready (wr_done_ready),
        .wr_done_tag   (wr_done_tag),   .wr_done_error (wr_done_error),
        .m_rd_req_valid     (m_rd_req_valid),     .m_rd_req_ready     (m_rd_req_ready),
        .m_rd_req_addr      (m_rd_req_addr),      .m_rd_req_len_bytes (m_rd_req_len_bytes),
        .m_rd_req_tag       (m_rd_req_tag),
        .m_rd_ret_valid     (m_rd_ret_valid),     .m_rd_ret_ready     (m_rd_ret_ready),
        .m_rd_ret_data      (m_rd_ret_data),      .m_rd_ret_keep      (m_rd_ret_keep),
        .m_rd_ret_tag       (m_rd_ret_tag),       .m_rd_ret_last      (m_rd_ret_last),
        .m_rd_ret_error     (m_rd_ret_error),
        .m_wr_req_valid     (m_wr_req_valid),     .m_wr_req_ready     (m_wr_req_ready),
        .m_wr_req_addr      (m_wr_req_addr),      .m_wr_req_len_bytes (m_wr_req_len_bytes),
        .m_wr_req_tag       (m_wr_req_tag),
        .m_wr_dat_valid     (m_wr_dat_valid),     .m_wr_dat_ready     (m_wr_dat_ready),
        .m_wr_dat_data      (m_wr_dat_data),      .m_wr_dat_keep      (m_wr_dat_keep),
        .m_wr_dat_last      (m_wr_dat_last),
        .m_wr_cplt_valid    (m_wr_cplt_valid),    .m_wr_cplt_ready    (m_wr_cplt_ready),
        .m_wr_cplt_tag      (m_wr_cplt_tag),      .m_wr_cplt_error    (m_wr_cplt_error)
    );

    ddr_memory_model #(
        .LATENCY_MIN     (1),
        .LATENCY_MAX     (6),
        .JITTER_EN       (1'b1),
        .BACKPRESSURE_EN (1'b1),
        .PROTOCOL_CHECKS (1'b1),
        .SEED            (32'h2468_ACE0)
    ) u_ddr (
        .clk (clk), .rst_n (rst_n),
        .rd_req_valid     (m_rd_req_valid),     .rd_req_ready     (m_rd_req_ready),
        .rd_req_addr      (m_rd_req_addr),      .rd_req_len_bytes (m_rd_req_len_bytes),
        .rd_req_tag       (m_rd_req_tag),
        .rd_ret_valid     (m_rd_ret_valid),     .rd_ret_ready     (m_rd_ret_ready),
        .rd_ret_data      (m_rd_ret_data),      .rd_ret_keep      (m_rd_ret_keep),
        .rd_ret_tag       (m_rd_ret_tag),       .rd_ret_last      (m_rd_ret_last),
        .rd_ret_error     (m_rd_ret_error),
        .wr_req_valid     (m_wr_req_valid),     .wr_req_ready     (m_wr_req_ready),
        .wr_req_addr      (m_wr_req_addr),      .wr_req_len_bytes (m_wr_req_len_bytes),
        .wr_req_tag       (m_wr_req_tag),
        .wr_dat_valid     (m_wr_dat_valid),     .wr_dat_ready     (m_wr_dat_ready),
        .wr_dat_data      (m_wr_dat_data),      .wr_dat_keep      (m_wr_dat_keep),
        .wr_dat_last      (m_wr_dat_last),
        .wr_cplt_valid    (m_wr_cplt_valid),    .wr_cplt_ready    (m_wr_cplt_ready),
        .wr_cplt_tag      (m_wr_cplt_tag),      .wr_cplt_error    (m_wr_cplt_error),
        .proto_violations (proto_violations)
    );

    //--------------------------------------------------------------------
    // posedge 采样监视器：fire 标志与载荷（与 tb_adapter 同风格）
    //--------------------------------------------------------------------
    logic f_rd_req [0:1], f_rd_ret [0:1];
    logic f_wr_req [0:1], f_wr_dat [0:1], f_wr_done [0:1];
    logic [31:0] s_rd_data [0:1];
    logic [3:0]  s_rd_keep [0:1];
    logic [15:0] s_rd_tag [0:1];
    logic        s_rd_last [0:1], s_rd_err [0:1];
    logic [15:0] s_wr_done_tag [0:1];
    logic        s_wr_done_err [0:1];

    always @(posedge clk) begin
        for (int c = 0; c < 2; c++) begin
            f_rd_req[c]  <= rd_req_valid[c]  && rd_req_ready[c];
            f_rd_ret[c]  <= rd_ret_valid[c]  && rd_ret_ready[c];
            f_wr_req[c]  <= wr_req_valid[c]  && wr_req_ready[c];
            f_wr_dat[c]  <= wr_dat_valid[c]  && wr_dat_ready[c];
            f_wr_done[c] <= wr_done_valid[c] && wr_done_ready[c];
            s_rd_data[c]    <= rd_ret_data[c];
            s_rd_keep[c]    <= rd_ret_keep[c];
            s_rd_tag[c]     <= rd_ret_tag[c];
            s_rd_last[c]    <= rd_ret_last[c];
            s_rd_err[c]     <= rd_ret_error[c];
            s_wr_done_tag[c] <= wr_done_tag[c];
            s_wr_done_err[c] <= wr_done_error[c];
        end
    end

    //--------------------------------------------------------------------
    // 参考图案与自检
    //--------------------------------------------------------------------
    int errs = 0;

    function automatic bit [7:0] pat(input int unsigned a, input int unsigned seed);
        pat = (a * 7 + seed * 13) & 8'hff;
    endfunction

    task automatic preload(input logic [31:0] addr, input int unsigned len, input int unsigned seed);
        for (int unsigned i = 0; i < len; i++)
            u_ddr.mem[addr + i] = pat(addr + i, seed);
    endtask

    task automatic check_mem(input logic [31:0] addr, input int unsigned len,
                            input int unsigned seed, input string name);
        int bad;
        bad = 0;
        for (int unsigned i = 0; i < len; i++)
            if (u_ddr.mem[addr + i] !== pat(addr + i, seed)) bad++;
        if (bad != 0) begin
            errs++;
            $display("[TB][FAIL] %s @0x%08x len%0d mem err=%0d/%0d", name, addr, len, bad, len);
            if (errs > 40) $finish;
        end
    endtask

    //--------------------------------------------------------------------
    // 读返回流接收与逐字节比对（addr 任意，keep/last 按 len 期望）
    //--------------------------------------------------------------------
    task automatic recv_rd_stream(input int c, input logic [31:0] addr, input int unsigned len,
                                  input logic [15:0] tag, input int unsigned seed);
        int unsigned off;
        rd_ret_ready[c] = 1'b1;
        off = 0;
        forever begin
            @(negedge clk);
            if (f_rd_ret[c]) begin
                if (s_rd_tag[c] !== tag) begin
                    errs++;
                    $display("[TB][FAIL] @%0t rd cli%0d tag %0d != %0d", $time, c, s_rd_tag[c], tag);
                end
                begin
                    logic [3:0] ekeep;
                    logic elast;
                    ekeep = 4'b0000;
                    for (int k = 0; k < 4; k++) if (off + k < len) ekeep[k] = 1'b1;
                    elast = (off + 4 >= len);
                    if (s_rd_keep[c] !== ekeep || s_rd_last[c] !== elast) begin
                        errs++;
                        $display("[TB][FAIL] @%0t rd cli%0d beat%0d keep/last got %04b/%0b exp %04b/%0b",
                                 $time, c, off/4, s_rd_keep[c], s_rd_last[c], ekeep, elast);
                    end
                    for (int k = 0; k < 4; k++) begin
                        if (s_rd_keep[c][k]) begin
                            bit [7:0] eb = pat(addr + off + k, seed);
                            if (s_rd_data[c][8*k +: 8] !== eb) begin
                                errs++;
                                $display("[TB][FAIL] @%0t rd cli%0d byte@0x%08x got %02x exp %02x",
                                         $time, c, addr + off + k, s_rd_data[c][8*k +: 8], eb);
                                if (errs > 40) $finish;
                            end
                        end
                    end
                end
                if (s_rd_err[c] !== 1'b0) begin
                    errs++;
                    $display("[TB][FAIL] @%0t rd cli%0d unexpected error=%0b", $time, c, s_rd_err[c]);
                end
                off += 4;
                if (s_rd_last[c]) break;
            end
        end
        rd_ret_ready[c] = 1'b0;
    endtask

    // 读事务：请求握手 → 收流比对
    task automatic cli_read(input int c, input logic [31:0] addr, input int unsigned len,
                            input logic [15:0] tag, input int unsigned seed);
        @(negedge clk);
        rd_req_valid[c] = 1'b1;
        rd_req_addr[c]  = addr;
        rd_req_len[c]   = len;
        rd_req_tag[c]   = tag;
        forever begin
            @(negedge clk);
            if (f_rd_req[c]) break;
        end
        rd_req_valid[c] = 1'b0;
        recv_rd_stream(c, addr, len, tag, seed);
    endtask

    // 写数据拍发送（keep/last 按 len 期望，背压握手）
    task automatic send_wr_data(input int c, input logic [31:0] addr, input int unsigned len,
                                input int unsigned seed);
        int unsigned off, b, k;
        logic [31:0] d;
        logic [3:0]  kp;
        logic        lst;
        @(negedge clk);
        wr_dat_valid[c] = 1'b1;
        for (b = 0; b < (len + 3) / 4; b++) begin
            off = b * 4;
            d = 32'h0; kp = 4'b0000;
            for (k = 0; k < 4; k++) begin
                if (off + k < len) begin
                    d[8*k +: 8] = pat(addr + off + k, seed);
                    kp[k] = 1'b1;
                end
            end
            lst = (b == (len + 3) / 4 - 1);
            wr_dat_data[c] = d;
            wr_dat_keep[c] = kp;
            wr_dat_last[c] = lst;
            forever begin
                @(negedge clk);
                if (f_wr_dat[c]) break;
            end
        end
        wr_dat_valid[c] = 1'b0;
    endtask

    // 写完成接收（tag/error 校验）
    task automatic wait_wr_done(input int c, input logic [15:0] tag);
        wr_done_ready[c] = 1'b1;
        forever begin
            @(negedge clk);
            if (f_wr_done[c]) begin
                if (s_wr_done_tag[c] !== tag) begin
                    errs++;
                    $display("[TB][FAIL] @%0t wr cli%0d done tag %0d != %0d", $time, c, s_wr_done_tag[c], tag);
                end
                if (s_wr_done_err[c] !== 1'b0) begin
                    errs++;
                    $display("[TB][FAIL] @%0t wr cli%0d done error=%0b", $time, c, s_wr_done_err[c]);
                end
                break;
            end
        end
        wr_done_ready[c] = 1'b0;
    endtask

    // 写事务：请求握手 → 数据 → 完成 → 内存比对
    task automatic cli_write(input int c, input logic [31:0] addr, input int unsigned len,
                             input logic [15:0] tag, input int unsigned seed);
        @(negedge clk);
        wr_req_valid[c] = 1'b1;
        wr_req_addr[c]  = addr;
        wr_req_len[c]   = len;
        wr_req_tag[c]   = tag;
        forever begin
            @(negedge clk);
            if (f_wr_req[c]) break;
        end
        wr_req_valid[c] = 1'b0;
        send_wr_data(c, addr, len, seed);
        wait_wr_done(c, tag);
        check_mem(addr, len, seed, $sformatf("wr-cli%0d", c));
    endtask

    //--------------------------------------------------------------------
    // 主测试序列
    //--------------------------------------------------------------------
    initial begin
        for (int c = 0; c < 2; c++) begin
            rd_req_valid[c] = 1'b0; rd_req_addr[c] = '0; rd_req_len[c] = '0; rd_req_tag[c] = '0;
            rd_ret_ready[c] = 1'b0;
            wr_req_valid[c] = 1'b0; wr_req_addr[c] = '0; wr_req_len[c] = '0; wr_req_tag[c] = '0;
            wr_dat_valid[c] = 1'b0; wr_dat_data[c] = '0; wr_dat_keep[c] = '0; wr_dat_last[c] = 1'b0;
            wr_done_ready[c] = 1'b0;
        end
        rst_n = 1'b0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5)  @(negedge clk);

        // 预载读区（两客户端各自专属区，图案随地址/种子区分）
        preload(32'h0000_1000, 4096, 11);
        preload(32'h0000_8000, 4096, 23);

        //------------------------------------------------------------
        // 用例 1：两读客户端交错（随机间隔 + 模型随机背压）
        //------------------------------------------------------------
        begin
            fork
                begin
                    logic [15:0] t;
                    t = 0;
                    for (int it = 0; it < 8; it++) begin
                        repeat ($urandom_range(0, 15)) @(negedge clk);
                        cli_read(0, 32'h0000_1000 + $urandom_range(0, 3800),
                                 1 + $urandom_range(0, 30), t, 11);
                        t += 1;
                    end
                end
                begin
                    logic [15:0] t;
                    t = 0;
                    for (int it = 0; it < 8; it++) begin
                        repeat ($urandom_range(0, 15)) @(negedge clk);
                        cli_read(1, 32'h0000_8000 + $urandom_range(0, 3800),
                                 1 + $urandom_range(0, 30), t, 23);
                        t += 1;
                    end
                end
            join
            $display("[TB] case1 read interleave (cli0+cli1) : done errs=%0d", errs);
        end

        //------------------------------------------------------------
        // 用例 2：两写客户端交错（随机背压，各写专属区）
        //------------------------------------------------------------
        begin
            fork
                begin
                    logic [15:0] t;
                    t = 0;
                    for (int it = 0; it < 8; it++) begin
                        repeat ($urandom_range(0, 12)) @(negedge clk);
                        cli_write(0, 32'h0001_0000 + $urandom_range(0, 3800),
                                  1 + $urandom_range(0, 40), t, 31);
                        t += 1;
                    end
                end
                begin
                    logic [15:0] t;
                    t = 0;
                    for (int it = 0; it < 8; it++) begin
                        repeat ($urandom_range(0, 12)) @(negedge clk);
                        cli_write(1, 32'h0002_0000 + $urandom_range(0, 3800),
                                  1 + $urandom_range(0, 40), t, 37);
                        t += 1;
                    end
                end
            join
            $display("[TB] case2 write interleave (cli0+cli1) : done errs=%0d", errs);
        end

        //------------------------------------------------------------
        // 用例 3：读写并发（1 读 + 1 写同时持续，两相）
        //------------------------------------------------------------
        preload(32'h0003_0000, 2048, 43);   // 读区（cli0）
        begin
            fork
                begin
                    logic [15:0] t;
                    t = 0;
                    for (int it = 0; it < 12; it++) begin
                        repeat ($urandom_range(0, 6)) @(negedge clk);
                        cli_read(0, 32'h0003_0000 + $urandom_range(0, 1900),
                                 1 + $urandom_range(0, 24), t, 43);
                        t += 1;
                    end
                end
                begin
                    logic [15:0] t;
                    t = 0;
                    for (int it = 0; it < 12; it++) begin
                        repeat ($urandom_range(0, 6)) @(negedge clk);
                        cli_write(1, 32'h0004_0000 + $urandom_range(0, 1900),
                                  1 + $urandom_range(0, 24), t, 47);
                        t += 1;
                    end
                end
            join
            $display("[TB] case3a rd0+wr1 concurrent         : done errs=%0d", errs);
        end
        preload(32'h0007_0000, 2048, 53);   // 读区（cli1）
        begin
            fork
                begin
                    logic [15:0] t;
                    t = 0;
                    for (int it = 0; it < 12; it++) begin
                        repeat ($urandom_range(0, 6)) @(negedge clk);
                        cli_read(1, 32'h0007_0000 + $urandom_range(0, 1900),
                                 1 + $urandom_range(0, 24), t, 53);
                        t += 1;
                    end
                end
                begin
                    logic [15:0] t;
                    t = 0;
                    for (int it = 0; it < 12; it++) begin
                        repeat ($urandom_range(0, 6)) @(negedge clk);
                        cli_write(0, 32'h0008_0000 + $urandom_range(0, 1900),
                                  1 + $urandom_range(0, 24), t, 59);
                        t += 1;
                    end
                end
            join
            $display("[TB] case3b rd1+wr0 concurrent         : done errs=%0d", errs);
        end

        //------------------------------------------------------------
        // 用例 4：round-robin 公平（两客户端持续有请求，读/写各测）
        //------------------------------------------------------------
        begin
            int rc0, rc1, wc0, wc1;
            rc0 = 0; rc1 = 0; wc0 = 0; wc1 = 0;
            preload(32'h0000_1000, 2048, 11);
            preload(32'h0000_8000, 2048, 23);
            fork
                begin
                    logic [15:0] t;
                    t = 0;
                    for (int it = 0; it < 10; it++) begin
                        cli_read(0, 32'h0000_1000, 16, t, 11); rc0++;
                        t += 1;
                    end
                end
                begin
                    logic [15:0] t;
                    t = 0;
                    for (int it = 0; it < 10; it++) begin
                        cli_read(1, 32'h0000_8000, 16, t, 23); rc1++;
                        t += 1;
                    end
                end
                begin
                    logic [15:0] t;
                    t = 0;
                    for (int it = 0; it < 10; it++) begin
                        cli_write(0, 32'h0001_0000, 12, t, 31); wc0++;
                        t += 1;
                    end
                end
                begin
                    logic [15:0] t;
                    t = 0;
                    for (int it = 0; it < 10; it++) begin
                        cli_write(1, 32'h0002_0000, 12, t, 37); wc1++;
                        t += 1;
                    end
                end
            join
            if (rc0 != 10 || rc1 != 10 || wc0 != 10 || wc1 != 10) begin
                errs++;
                $display("[TB][FAIL] case4 fairness: rd c0=%0d c1=%0d wr c0=%0d c1=%0d",
                         rc0, rc1, wc0, wc1);
            end
            $display("[TB] case4 round-robin fairness        : done (rd %0d/%0d wr %0d/%0d)",
                     rc0, rc1, wc0, wc1);
        end

        //------------------------------------------------------------
        // 用例 5：在途单笔（请求被接受后立刻再拉 valid → ready 拉低阻塞，
        //         事务完成后接受，不吞请求）
        //------------------------------------------------------------
        begin
            logic [15:0] t;
            t = 0;
            preload(32'h0000_1000, 512, 11);

            // ---- 5a：读在途 ----
            @(negedge clk);
            rd_req_valid[0] = 1'b1;
            rd_req_addr[0]  = 32'h0000_1000;
            rd_req_len[0]   = 32'd16;
            rd_req_tag[0]   = t;
            forever begin
                @(negedge clk);
                if (f_rd_req[0]) break;              // A1 接受 → 在途
            end
            // 立刻换 A2 参数并保持 valid；在途期间 ready 必须恒 0
            rd_req_addr[0] = 32'h0000_1200;
            rd_req_len[0]  = 32'd8;
            rd_req_tag[0]  = t + 1;
            begin
                int bad;
                bad = 0;
                repeat (6) begin
                    @(negedge clk);
                    if (rd_req_ready[0]) bad++;
                end
                if (bad != 0) begin
                    errs++;
                    $display("[TB][FAIL] case5a: in-flight rd_req_ready pulsed %0d times", bad);
                end
            end
            recv_rd_stream(0, 32'h0000_1000, 16, t, 11);   // A1 返回
            forever begin
                @(negedge clk);
                if (f_rd_req[0]) break;              // A2 释放后被接受
            end
            recv_rd_stream(0, 32'h0000_1200, 8, t + 1, 11); // A2 返回
            rd_req_valid[0] = 1'b0;
            $display("[TB] case5a read in-flight block      : done errs=%0d", errs);

            // ---- 5b：写在途 ----
            @(negedge clk);
            wr_req_valid[0] = 1'b1;
            wr_req_addr[0]  = 32'h0001_0000;
            wr_req_len[0]   = 32'd12;
            wr_req_tag[0]   = t;
            forever begin
                @(negedge clk);
                if (f_wr_req[0]) break;              // W1 接受 → 在途
            end
            // 立刻换 W2 参数并保持 valid；在途期间 ready 必须恒 0
            wr_req_addr[0] = 32'h0001_2000;
            wr_req_len[0]  = 32'd8;
            wr_req_tag[0]  = t + 1;
            begin
                int bad;
                bad = 0;
                repeat (6) begin
                    @(negedge clk);
                    if (wr_req_ready[0]) bad++;
                end
                if (bad != 0) begin
                    errs++;
                    $display("[TB][FAIL] case5b: in-flight wr_req_ready pulsed %0d times", bad);
                end
            end
            send_wr_data(0, 32'h0001_0000, 12, 31);        // W1 数据
            wait_wr_done(0, t);                            // W1 完成
            forever begin
                @(negedge clk);
                if (f_wr_req[0]) break;              // W2 释放后被接受
            end
            send_wr_data(0, 32'h0001_2000, 8, 31);         // W2 数据
            wait_wr_done(0, t + 1);                        // W2 完成
            wr_req_valid[0] = 1'b0;
            $display("[TB] case5b write in-flight block     : done errs=%0d", errs);
        end

        //------------------------------------------------------------
        // 用例 6：零长（len==0）→ 客户端侧正常握手、伪返回/伪完成 error=1
        //------------------------------------------------------------
        begin
            // 读零长（cli0 / cli1）
            for (int c = 0; c < 2; c++) begin
                logic [15:0] tt;
                tt = 16'(100 + c);
                @(negedge clk);
                rd_req_valid[c] = 1'b1;
                rd_req_addr[c]  = 32'h0000_3000;
                rd_req_len[c]   = 32'd0;
                rd_req_tag[c]   = tt;
                forever begin
                    @(negedge clk);
                    if (f_rd_req[c]) break;
                end
                rd_req_valid[c] = 1'b0;
                rd_ret_ready[c] = 1'b1;
                forever begin
                    @(negedge clk);
                    if (f_rd_ret[c]) begin
                        if (!s_rd_err[c] || !s_rd_last[c] || (s_rd_keep[c] !== 4'b0000) ||
                            (s_rd_data[c] !== 32'd0) || (s_rd_tag[c] !== tt)) begin
                            errs++;
                            $display("[TB][FAIL] case6 len0 read cli%0d: err=%0b last=%0b keep=%04b data=%08h tag=%0d",
                                     c, s_rd_err[c], s_rd_last[c], s_rd_keep[c], s_rd_data[c], s_rd_tag[c]);
                        end
                        break;
                    end
                end
                rd_ret_ready[c] = 1'b0;
            end

            // 写零长（cli0 / cli1）
            for (int c = 0; c < 2; c++) begin
                logic [15:0] tt;
                tt = 16'(110 + c);
                @(negedge clk);
                wr_req_valid[c] = 1'b1;
                wr_req_addr[c]  = 32'h0000_4000;
                wr_req_len[c]   = 32'd0;
                wr_req_tag[c]   = tt;
                forever begin
                    @(negedge clk);
                    if (f_wr_req[c]) break;
                end
                wr_req_valid[c] = 1'b0;
                wr_done_ready[c] = 1'b1;
                forever begin
                    @(negedge clk);
                    if (f_wr_done[c]) begin
                        if (!s_wr_done_err[c] || (s_wr_done_tag[c] !== tt)) begin
                            errs++;
                            $display("[TB][FAIL] case6 len0 write cli%0d: err=%0b tag=%0d",
                                     c, s_wr_done_err[c], s_wr_done_tag[c]);
                        end
                        break;
                    end
                end
                wr_done_ready[c] = 1'b0;
            end
            $display("[TB] case6 zero-length pseudo          : done errs=%0d", errs);
        end

        //------------------------------------------------------------
        // 汇总：errs==0 且模型零协议违规
        //------------------------------------------------------------
        repeat (10) @(negedge clk);
        $display("==================================================");
        $display("ARBITER TB: errs=%0d proto_violations=%0d", errs, proto_violations);
        if (proto_violations != 16'd0) begin
            errs++;
            $display("[TB][FAIL] model protocol violations = %0d (expect 0)", proto_violations);
        end
        if (errs == 0)
            $display("TB RESULT: ALL ARBITER TESTS PASSED");
        else
            $display("TB RESULT: FAILED");
        $display("==================================================");
        $finish;
    end

    //--------------------------------------------------------------------
    // 全局看门狗
    //--------------------------------------------------------------------
    initial begin
        #60_000_000;
        $display("[TB][FATAL] global timeout errs=%0d", errs);
        $finish;
    end

endmodule
