`timescale 1ns / 1ps
//==============================================================================
// tb_adapter.sv — ddr_port_adapter 验证平台（M7.1 交付）
//------------------------------------------------------------------------------
// 验证内容：
//   T1-T4 正常读写事务（不同 addr/len/tag，含对齐/非对齐/大块 1920B），
//          数据回带、keep/last 流、tag 回带、error 直通逐一比对；
//   T5    读错误注入：命中地址读返回 error=1（数据照常比对）；
//   T6    写错误注入：写完成 error=1 且内存内容不被破坏；
//   T7    len==0 读/写：客户端侧正常握手，适配器回 error=1 伪返回/伪完成，
//          模型零协议违规（PROTOCOL_CHECKS 全程必须为 0）；
//   T8    一笔在读/写强制：在读/写在途期间再次拉请求 valid，
//          适配器必须拉低 ready 阻塞（不握手、不违规、原事务正常完成）。
//
// 模型参数：JITTER/BACKPRESSURE/PROTOCOL_CHECKS 全开。
// 握手检测：与 tb_memory 一致（negedge 驱动激励、posedge 采样 fire 与载荷）。
// 通过标准：fails==0 且模型 proto_violations==0（直连零协议违规）。
//==============================================================================
module tb_adapter;

    localparam CLK_PERIOD = 10;

    logic clk = 1'b0;
    logic rst_n;

    always #(CLK_PERIOD/2) clk = ~clk;

    //--------------------------------------------------------------------
    // DUT：ddr_port_adapter（客户端侧）+ ddr_memory_model（模型侧）
    //--------------------------------------------------------------------
    // 客户端侧信号
    logic        rd_req_valid, rd_req_ready;
    logic [31:0] rd_req_addr, rd_req_len;
    logic [15:0] rd_req_tag;
    logic        rd_ret_valid, rd_ret_ready;
    logic [31:0] rd_ret_data;
    logic [3:0]  rd_ret_keep;
    logic [15:0] rd_ret_tag;
    logic        rd_ret_last, rd_ret_error;
    logic        wr_req_valid, wr_req_ready;
    logic [31:0] wr_req_addr, wr_req_len;
    logic [15:0] wr_req_tag;
    logic        wr_dat_valid, wr_dat_ready;
    logic [31:0] wr_dat_data;
    logic [3:0]  wr_dat_keep;
    logic        wr_dat_last;
    logic        wr_done_valid, wr_done_ready;
    logic [15:0] wr_done_tag;
    logic        wr_done_error;

    // 模型侧信号（适配器 m_* ↔ 模型端口）
    logic        m_rd_req_valid, m_rd_req_ready;
    logic [31:0] m_rd_req_addr, m_rd_req_len_bytes;
    logic [15:0] m_rd_req_tag;
    logic        m_rd_ret_valid, m_rd_ret_ready;
    logic [31:0] m_rd_ret_data;
    logic [3:0]  m_rd_ret_keep;
    logic [15:0] m_rd_ret_tag;
    logic        m_rd_ret_last, m_rd_ret_error;
    logic        m_wr_req_valid, m_wr_req_ready;
    logic [31:0] m_wr_req_addr, m_wr_req_len_bytes;
    logic [15:0] m_wr_req_tag;
    logic        m_wr_dat_valid, m_wr_dat_ready;
    logic [31:0] m_wr_dat_data;
    logic [3:0]  m_wr_dat_keep;
    logic        m_wr_dat_last;
    logic        m_wr_cplt_valid, m_wr_cplt_ready;
    logic [15:0] m_wr_cplt_tag;
    logic        m_wr_cplt_error;
    logic [15:0] proto_violations;

    ddr_port_adapter #(
        .ADDR_W (32),
        .LEN_W  (32),
        .TAG_W  (16)
    ) dut (
        .clk              (clk),
        .rst_n            (rst_n),
        // 客户端侧：读
        .rd_req_valid     (rd_req_valid),
        .rd_req_ready     (rd_req_ready),
        .rd_req_addr      (rd_req_addr),
        .rd_req_len       (rd_req_len),
        .rd_req_tag       (rd_req_tag),
        .rd_ret_valid     (rd_ret_valid),
        .rd_ret_ready     (rd_ret_ready),
        .rd_ret_data      (rd_ret_data),
        .rd_ret_keep      (rd_ret_keep),
        .rd_ret_tag       (rd_ret_tag),
        .rd_ret_last      (rd_ret_last),
        .rd_ret_error     (rd_ret_error),
        // 客户端侧：写
        .wr_req_valid     (wr_req_valid),
        .wr_req_ready     (wr_req_ready),
        .wr_req_addr      (wr_req_addr),
        .wr_req_len       (wr_req_len),
        .wr_req_tag       (wr_req_tag),
        .wr_dat_valid     (wr_dat_valid),
        .wr_dat_ready     (wr_dat_ready),
        .wr_dat_data      (wr_dat_data),
        .wr_dat_keep      (wr_dat_keep),
        .wr_dat_last      (wr_dat_last),
        .wr_done_valid    (wr_done_valid),
        .wr_done_ready    (wr_done_ready),
        .wr_done_tag      (wr_done_tag),
        .wr_done_error    (wr_done_error),
        // 模型侧：读
        .m_rd_req_valid       (m_rd_req_valid),
        .m_rd_req_ready       (m_rd_req_ready),
        .m_rd_req_addr        (m_rd_req_addr),
        .m_rd_req_len_bytes   (m_rd_req_len_bytes),
        .m_rd_req_tag         (m_rd_req_tag),
        .m_rd_ret_valid       (m_rd_ret_valid),
        .m_rd_ret_ready       (m_rd_ret_ready),
        .m_rd_ret_data        (m_rd_ret_data),
        .m_rd_ret_keep        (m_rd_ret_keep),
        .m_rd_ret_tag         (m_rd_ret_tag),
        .m_rd_ret_last        (m_rd_ret_last),
        .m_rd_ret_error       (m_rd_ret_error),
        // 模型侧：写
        .m_wr_req_valid       (m_wr_req_valid),
        .m_wr_req_ready       (m_wr_req_ready),
        .m_wr_req_addr        (m_wr_req_addr),
        .m_wr_req_len_bytes   (m_wr_req_len_bytes),
        .m_wr_req_tag         (m_wr_req_tag),
        .m_wr_dat_valid       (m_wr_dat_valid),
        .m_wr_dat_ready       (m_wr_dat_ready),
        .m_wr_dat_data        (m_wr_dat_data),
        .m_wr_dat_keep        (m_wr_dat_keep),
        .m_wr_dat_last        (m_wr_dat_last),
        .m_wr_cplt_valid      (m_wr_cplt_valid),
        .m_wr_cplt_ready      (m_wr_cplt_ready),
        .m_wr_cplt_tag        (m_wr_cplt_tag),
        .m_wr_cplt_error      (m_wr_cplt_error)
    );

    ddr_memory_model #(
        .LATENCY_MIN     (1),
        .LATENCY_MAX     (6),
        .JITTER_EN       (1'b1),
        .BACKPRESSURE_EN (1'b1),
        .PROTOCOL_CHECKS (1'b1),
        .SEED            (32'h1234_5679)
    ) mem (
        .clk              (clk),
        .rst_n            (rst_n),
        .rd_req_valid     (m_rd_req_valid),
        .rd_req_ready     (m_rd_req_ready),
        .rd_req_addr      (m_rd_req_addr),
        .rd_req_len_bytes (m_rd_req_len_bytes),
        .rd_req_tag       (m_rd_req_tag),
        .rd_ret_valid     (m_rd_ret_valid),
        .rd_ret_ready     (m_rd_ret_ready),
        .rd_ret_data      (m_rd_ret_data),
        .rd_ret_keep      (m_rd_ret_keep),
        .rd_ret_tag       (m_rd_ret_tag),
        .rd_ret_last      (m_rd_ret_last),
        .rd_ret_error     (m_rd_ret_error),
        .wr_req_valid     (m_wr_req_valid),
        .wr_req_ready     (m_wr_req_ready),
        .wr_req_addr      (m_wr_req_addr),
        .wr_req_len_bytes (m_wr_req_len_bytes),
        .wr_req_tag       (m_wr_req_tag),
        .wr_dat_valid     (m_wr_dat_valid),
        .wr_dat_ready     (m_wr_dat_ready),
        .wr_dat_data      (m_wr_dat_data),
        .wr_dat_keep      (m_wr_dat_keep),
        .wr_dat_last      (m_wr_dat_last),
        .wr_cplt_valid    (m_wr_cplt_valid),
        .wr_cplt_ready    (m_wr_cplt_ready),
        .wr_cplt_tag      (m_wr_cplt_tag),
        .wr_cplt_error    (m_wr_cplt_error),
        .proto_violations (proto_violations)
    );

    //--------------------------------------------------------------------
    // posedge 采样监视器：fire 标志与载荷
    //--------------------------------------------------------------------
    logic f_rd_req, f_rd_ret, f_wr_req, f_wr_dat, f_wr_done;
    logic [31:0] s_rd_data;
    logic [3:0]  s_rd_keep;
    logic [15:0] s_rd_tag;
    logic        s_rd_last, s_rd_err;
    logic [15:0] s_done_tag;
    logic        s_done_err;

    always @(posedge clk) begin
        f_rd_req  <= rd_req_valid  && rd_req_ready;
        f_rd_ret  <= rd_ret_valid  && rd_ret_ready;
        f_wr_req  <= wr_req_valid  && wr_req_ready;
        f_wr_dat  <= wr_dat_valid  && wr_dat_ready;
        f_wr_done <= wr_done_valid && wr_done_ready;
        s_rd_data  <= rd_ret_data;
        s_rd_keep  <= rd_ret_keep;
        s_rd_tag   <= rd_ret_tag;
        s_rd_last  <= rd_ret_last;
        s_rd_err   <= rd_ret_error;
        s_done_tag <= wr_done_tag;
        s_done_err <= wr_done_error;
    end

    //--------------------------------------------------------------------
    // 参考内存（TB 侧稀疏字节存储，用于读写数据比对）
    //--------------------------------------------------------------------
    bit [7:0] ref_mem [bit [31:0]];

    int fails = 0;
    logic [15:0] g_tag = 16'd0;

    function automatic bit [7:0] pat(input int unsigned i);
        pat = (i * 7 + 13) % 256;
    endfunction

    // keep/last 流检查（连续流期望：非尾拍 1111、尾拍按剩余）
    task automatic check_stream(input string name, input int unsigned len,
                                ref logic [4:0] beats_q [$]);
        int unsigned nb = (len + 3) / 4;
        logic [3:0]  ekeep;
        logic        elast;
        if (beats_q.size() != nb) begin
            fails = fails + 1;
            $display("[TB][FAIL] %s: beat count %0d != expected %0d",
                     name, beats_q.size(), nb);
            return;
        end
        for (int unsigned b = 0; b < nb; b++) begin
            int unsigned off = b * 4;
            int unsigned rem = len - off;
            case (rem)
                32'd1:    ekeep = 4'b0001;
                32'd2:    ekeep = 4'b0011;
                32'd3:    ekeep = 4'b0111;
                default:  ekeep = 4'b1111;
            endcase
            elast = (off + 4 >= len);
            if (beats_q[b] !== {elast, ekeep}) begin
                fails = fails + 1;
                $display("[TB][FAIL] %s: beat %0d {last,keep}=%05b != %05b",
                         name, b, beats_q[b], {elast, ekeep});
            end
        end
    endtask

    //--------------------------------------------------------------------
    // BFM：写事务（请求→数据→完成）。exp_err=1 时期望完成 error=1，
    // 且内存不被破坏（ref_mem 不更新）。
    //--------------------------------------------------------------------
    task automatic cli_write(input logic [31:0] addr, input int unsigned len,
                             ref bit [7:0] q [], input bit exp_err,
                             input logic [15:0] tag);
        int unsigned nbeats, b, k, off;
        logic [31:0] d;
        logic [3:0]  kp;
        logic        lst;

        @(negedge clk);
        wr_req_valid = 1'b1;
        wr_req_addr  = addr;
        wr_req_len   = len;
        wr_req_tag   = tag;
        forever begin
            @(negedge clk);
            if (f_wr_req) break;
        end
        wr_req_valid = 1'b0;

        nbeats = (len + 3) / 4;
        @(negedge clk);
        wr_dat_valid = 1'b1;
        for (b = 0; b < nbeats; b++) begin
            off = b * 4;
            d = 32'h0;
            kp = 4'b0;
            for (k = 0; k < 4; k++) begin
                if (off + k < len) begin
                    d[8*k +: 8] = q[off + k];
                    kp[k] = 1'b1;
                end
            end
            lst = (b == nbeats - 1);
            wr_dat_data = d;
            wr_dat_keep = kp;
            wr_dat_last = lst;
            forever begin
                @(negedge clk);
                if (f_wr_dat) break;
            end
        end
        wr_dat_valid = 1'b0;

        forever begin
            @(negedge clk);
            if (f_wr_done) begin
                if (s_done_tag !== tag) begin
                    fails = fails + 1;
                    $display("[TB][FAIL] @%0t write done tag %0d != %0d",
                             $time, s_done_tag, tag);
                end
                if (s_done_err !== exp_err) begin
                    fails = fails + 1;
                    $display("[TB][FAIL] @%0t write done error=%0b, expected %0b",
                             $time, s_done_err, exp_err);
                end
                break;
            end
        end

        // 参考内存更新（写失败注入时不生效）
        if (!exp_err) begin
            for (int unsigned i = 0; i < len; i++)
                ref_mem[addr + i] = q[i];
        end
    endtask

    //--------------------------------------------------------------------
    // BFM：读事务（请求→收流）。比对数据（ref_mem）、keep/last 流、tag、error。
    //--------------------------------------------------------------------
    task automatic cli_read(input logic [31:0] addr, input int unsigned len,
                            ref bit [7:0] qr [], input bit exp_err,
                            input logic [15:0] tag,
                            ref logic [4:0] beats_q [$], output bit got_err);
        int unsigned off;

        @(negedge clk);
        rd_req_valid = 1'b1;
        rd_req_addr  = addr;
        rd_req_len   = len;
        rd_req_tag   = tag;
        forever begin
            @(negedge clk);
            if (f_rd_req) break;
        end
        rd_req_valid = 1'b0;

        beats_q.delete();
        got_err = 1'b0;
        off = 0;
        rd_ret_ready = 1'b1;
        forever begin
            @(negedge clk);
            if (f_rd_ret) begin
                if (s_rd_tag !== tag) begin
                    fails = fails + 1;
                    $display("[TB][FAIL] @%0t read ret tag %0d != %0d",
                             $time, s_rd_tag, tag);
                end
                beats_q.push_back({s_rd_last, s_rd_keep});
                got_err = got_err | s_rd_err;
                for (int k = 0; k < 4; k++) begin
                    if (s_rd_keep[k]) begin
                        qr[off + k] = s_rd_data[8*k +: 8];
                        if (qr[off + k] !== ref_mem[addr + off + k]) begin
                            fails = fails + 1;
                            $display("[TB][FAIL] @%0t read byte @0x%08x got 0x%02x != 0x%02x",
                                     $time, addr + off + k, qr[off + k],
                                     ref_mem[addr + off + k]);
                            if (fails > 20) return;
                        end
                    end
                end
                off += 4;
                if (s_rd_last) break;
            end
        end
        rd_ret_ready = 1'b0;

        if (got_err !== exp_err) begin
            fails = fails + 1;
            $display("[TB][FAIL] @%0t read error=%0b, expected %0b",
                     $time, got_err, exp_err);
        end
    endtask

    //--------------------------------------------------------------------
    // BFM：len==0 读/写（适配器回 error=1 伪返回/伪完成，模型零违规）
    //--------------------------------------------------------------------
    task automatic check_len0();
        // 读零长
        @(negedge clk);
        rd_req_valid = 1'b1;
        rd_req_addr  = 32'h1000;
        rd_req_len   = 32'd0;
        rd_req_tag   = g_tag;
        forever begin
            @(negedge clk);
            if (f_rd_req) break;
        end
        rd_req_valid = 1'b0;
        rd_ret_ready = 1'b1;
        forever begin
            @(negedge clk);
            if (f_rd_ret) begin
                if (!s_rd_err || !s_rd_last || (s_rd_keep !== 4'b0000) ||
                    (s_rd_data !== 32'd0) || (s_rd_tag !== g_tag)) begin
                    fails = fails + 1;
                    $display("[TB][FAIL] len0 read: err=%0b last=%0b keep=%04b data=%08h tag=%0d",
                             s_rd_err, s_rd_last, s_rd_keep, s_rd_data, s_rd_tag);
                end
                break;
            end
        end
        rd_ret_ready = 1'b0;
        g_tag = g_tag + 16'd1;

        // 写零长
        @(negedge clk);
        wr_req_valid = 1'b1;
        wr_req_addr  = 32'h1000;
        wr_req_len   = 32'd0;
        wr_req_tag   = g_tag;
        forever begin
            @(negedge clk);
            if (f_wr_req) break;
        end
        wr_req_valid = 1'b0;
        forever begin
            @(negedge clk);
            if (f_wr_done) begin
                if (!s_done_err || (s_done_tag !== g_tag)) begin
                    fails = fails + 1;
                    $display("[TB][FAIL] len0 write: err=%0b tag=%0d",
                             s_done_err, s_done_tag);
                end
                break;
            end
        end
        g_tag = g_tag + 16'd1;
        $display("[TB] T7  len==0 pseudo return/complete      : done");
    endtask

    //--------------------------------------------------------------------
    // BFM：读在途强制（在途期间拉请求 valid 必须被 ready=0 阻塞）
    //--------------------------------------------------------------------
    task automatic check_rd_inflight(input logic [31:0] addr, input int unsigned len);
        bit [7:0] qr[];
        logic [4:0] beats[$];
        bit err;
        int bad;

        qr = new[len];

        // 发读请求并握手
        @(negedge clk);
        rd_req_valid = 1'b1;
        rd_req_addr  = addr;
        rd_req_len   = len;
        rd_req_tag   = g_tag;
        forever begin
            @(negedge clk);
            if (f_rd_req) break;
        end
        rd_req_valid = 1'b0;

        // 在途期间：卡住返回流（rd_ret_ready=0），拉请求 valid 检查 ready 必须为 0
        rd_ret_ready = 1'b0;
        repeat (2) @(negedge clk);
        bad = 0;
        rd_req_valid = 1'b1;
        repeat (8) begin
            @(negedge clk);
            if (rd_req_ready) bad++;
        end
        rd_req_valid = 1'b0;
        if (bad > 0) begin
            fails = fails + 1;
            $display("[TB][FAIL] rd in-flight: rd_req_ready pulsed %0d times",
                     bad);
        end

        // 原请求必须仍正常完成（收流并比对）
        beats.delete();
        err = 1'b0;
        begin
            int unsigned off = 0;
            rd_ret_ready = 1'b1;
            forever begin
                @(negedge clk);
                if (f_rd_ret) begin
                    beats.push_back({s_rd_last, s_rd_keep});
                    err = err | s_rd_err;
                    for (int k = 0; k < 4; k++)
                        if (s_rd_keep[k])
                            qr[off + k] = s_rd_data[8*k +: 8];
                    off += 4;
                    if (s_rd_last) break;
                end
            end
            rd_ret_ready = 1'b0;
        end
        if (err !== 1'b0) begin
            fails = fails + 1;
            $display("[TB][FAIL] rd in-flight: original read got error");
        end
        check_stream("RD-INFL", len, beats);
        for (int unsigned i = 0; i < len; i++)
            if (qr[i] !== ref_mem[addr + i]) begin
                fails = fails + 1;
                $display("[TB][FAIL] rd in-flight: byte %0d mismatch", i);
                break;
            end
        g_tag = g_tag + 16'd1;
        $display("[TB] T8a read in-flight blocked (rd_req)     : done");
    endtask

    //--------------------------------------------------------------------
    // BFM：写在途强制（在途期间拉请求 valid 必须被 ready=0 阻塞）
    //--------------------------------------------------------------------
    task automatic check_wr_inflight(input logic [31:0] addr, input int unsigned len);
        bit [7:0] q[];
        int bad;

        q = new[len];
        for (int unsigned i = 0; i < len; i++) q[i] = pat(i + 7000);

        // 发写请求并握手
        @(negedge clk);
        wr_req_valid = 1'b1;
        wr_req_addr  = addr;
        wr_req_len   = len;
        wr_req_tag   = g_tag;
        forever begin
            @(negedge clk);
            if (f_wr_req) break;
        end
        wr_req_valid = 1'b0;

        // 在途期间（模型等写数据，未发完成）：拉请求 valid 检查 ready 必须为 0
        repeat (2) @(negedge clk);
        bad = 0;
        wr_req_valid = 1'b1;
        repeat (8) begin
            @(negedge clk);
            if (wr_req_ready) bad++;
        end
        wr_req_valid = 1'b0;
        if (bad > 0) begin
            fails = fails + 1;
            $display("[TB][FAIL] wr in-flight: wr_req_ready pulsed %0d times",
                     bad);
        end

        // 原事务仍正常完成（发数据 + 等完成）
        begin
            int unsigned nbeats = (len + 3) / 4;
            int unsigned b, off, k;
            logic [31:0] d;
            logic [3:0]  kp;
            logic        lst;
            @(negedge clk);
            wr_dat_valid = 1'b1;
            for (b = 0; b < nbeats; b++) begin
                off = b * 4;
                d = 32'h0;
                kp = 4'b0;
                for (k = 0; k < 4; k++) begin
                    if (off + k < len) begin
                        d[8*k +: 8] = q[off + k];
                        kp[k] = 1'b1;
                    end
                end
                lst = (b == nbeats - 1);
                wr_dat_data = d;
                wr_dat_keep = kp;
                wr_dat_last = lst;
                forever begin
                    @(negedge clk);
                    if (f_wr_dat) break;
                end
            end
            wr_dat_valid = 1'b0;
            forever begin
                @(negedge clk);
                if (f_wr_done) break;
            end
        end
        for (int unsigned i = 0; i < len; i++)
            ref_mem[addr + i] = q[i];
        g_tag = g_tag + 16'd1;
        $display("[TB] T8b write in-flight blocked (wr_req)    : done");
    endtask

    //--------------------------------------------------------------------
    // 主测试序列
    //--------------------------------------------------------------------
    initial begin
        rd_req_valid = 1'b0; rd_req_addr = '0; rd_req_len = '0; rd_req_tag = '0;
        rd_ret_ready = 1'b0;
        wr_req_valid = 1'b0; wr_req_addr = '0; wr_req_len = '0; wr_req_tag = '0;
        wr_dat_valid = 1'b0; wr_dat_data = '0; wr_dat_keep = '0; wr_dat_last = 1'b0;
        wr_done_ready = 1'b1;
        rst_n = 1'b0;

        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5)  @(negedge clk);

        begin
            bit [7:0] q[], qr[];
            logic [4:0] beats[$];
            bit err;

            //------------------------------------------------------------
            // T1: 4 字节对齐块读写（256B）
            //------------------------------------------------------------
            q  = new[256]; qr = new[256];
            for (int unsigned i = 0; i < 256; i++) q[i] = pat(i);
            cli_write(32'd64, 256, q, 1'b0, g_tag);
            cli_read (32'd64, 256, qr, 1'b0, g_tag, beats, err);
            check_stream("T1", 256, beats);
            $display("[TB] T1  4-aligned 256B rw               : done");
            g_tag = g_tag + 16'd2;

            //------------------------------------------------------------
            // T2: 非对齐起点 addr=5, len=7
            //------------------------------------------------------------
            q  = new[7]; qr = new[7];
            for (int unsigned i = 0; i < 7; i++) q[i] = pat(i + 100);
            cli_write(32'd5, 7, q, 1'b0, g_tag);
            cli_read (32'd5, 7, qr, 1'b0, g_tag, beats, err);
            check_stream("T2", 7, beats);
            $display("[TB] T2  unaligned addr=5 len=7           : done");
            g_tag = g_tag + 16'd2;

            //------------------------------------------------------------
            // T3: len=5 尾拍 keep=0001
            //------------------------------------------------------------
            q  = new[5]; qr = new[5];
            for (int unsigned i = 0; i < 5; i++) q[i] = pat(i + 200);
            cli_write(32'd1000, 5, q, 1'b0, g_tag);
            cli_read (32'd1000, 5, qr, 1'b0, g_tag, beats, err);
            check_stream("T3", 5, beats);
            $display("[TB] T3  len=5 tail keep                  : done");
            g_tag = g_tag + 16'd2;

            //------------------------------------------------------------
            // T4: 大块 1920B + 随机小事务（不同 len/tag/addr）
            //------------------------------------------------------------
            q  = new[1920]; qr = new[1920];
            for (int unsigned i = 0; i < 1920; i++) q[i] = pat(i + 5000);
            cli_write(32'h0000_1000, 1920, q, 1'b0, g_tag);
            cli_read (32'h0000_1000, 1920, qr, 1'b0, g_tag, beats, err);
            check_stream("T4", 1920, beats);
            $display("[TB] T4  1920B row-sized rw               : done");
            g_tag = g_tag + 16'd2;

            for (int c = 0; c < 15; c++) begin
                int unsigned A, L;
                A = $urandom_range(0, 1 << 22);
                L = $urandom_range(1, 40);
                q  = new[L]; qr = new[L];
                for (int unsigned j = 0; j < L; j++) q[j] = pat(j + 9000 + c * 53);
                cli_write(A, L, q, 1'b0, g_tag);
                cli_read (A, L, qr, 1'b0, g_tag, beats, err);
                check_stream($sformatf("RND%0d", c), L, beats);
                g_tag = g_tag + 16'd2;
            end
            $display("[TB] T4b 15 random rw transactions         : done");

            //------------------------------------------------------------
            // T5: 读错误注入（命中地址返回 error=1，数据照常比对）
            //------------------------------------------------------------
            q  = new[16]; qr = new[16];
            for (int unsigned i = 0; i < 16; i++) q[i] = pat(i + 7000);
            cli_write(32'd5500, 16, q, 1'b0, g_tag);
            mem.inject_error(32'd5000, 32'd6000);
            cli_read (32'd5500, 16, qr, 1'b1, g_tag, beats, err);
            mem.clear_error_injection();
            check_stream("T5", 16, beats);
            $display("[TB] T5  read error injection             : done");
            g_tag = g_tag + 16'd2;

            //------------------------------------------------------------
            // T6: 写错误注入（完成 error=1 且内存不被破坏）
            //------------------------------------------------------------
            begin
                bit [7:0] good[], bad[];
                good = new[8]; bad = new[8];
                for (int unsigned i = 0; i < 8; i++) good[i] = pat(i + 8000);
                for (int unsigned i = 0; i < 8; i++) bad[i]  = ~pat(i + 8000);
                cli_write(32'd7500, 8, good, 1'b0, g_tag);
                mem.inject_error(32'd7000, 32'd8000);
                cli_write(32'd7500, 8, bad, 1'b1, g_tag);
                mem.clear_error_injection();
                qr = new[8];
                cli_read (32'd7500, 8, qr, 1'b0, g_tag, beats, err);
                check_stream("T6", 8, beats);
                for (int unsigned i = 0; i < 8; i++)
                    if (qr[i] !== good[i]) begin
                        fails = fails + 1;
                        $display("[TB][FAIL] T6: byte %0d corrupted by bad write", i);
                    end
                g_tag = g_tag + 16'd4;
                $display("[TB] T6  write error injection (mem intact) : done");
            end

            //------------------------------------------------------------
            // T7: len==0（伪返回/伪完成，模型零违规）
            //------------------------------------------------------------
            check_len0();

            //------------------------------------------------------------
            // T8: 一笔在读/写强制（在途期间请求被 ready 阻塞，原事务正常完成）
            //------------------------------------------------------------
            q = new[32];
            for (int unsigned i = 0; i < 32; i++) q[i] = pat(i + 300);
            cli_write(32'd9000, 32, q, 1'b0, g_tag);
            g_tag = g_tag + 16'd2;
            check_rd_inflight(32'd9000, 32);
            check_wr_inflight(32'd9100, 24);

            //------------------------------------------------------------
            // 汇总：fails==0 且模型零违规
            //------------------------------------------------------------
            $display("--------------------------------------------------");
            $display("model proto_violations = %0d (must be 0)", proto_violations);
            if (proto_violations != 16'd0) begin
                fails = fails + 1;
                $display("[TB][FAIL] model protocol violations = %0d (expect 0)",
                         proto_violations);
            end
        end

        $display("==================================================");
        $display("TB SUMMARY: fails=%0d", fails);
        if (fails == 0)
            $display("TB RESULT: ALL TESTS PASSED");
        else
            $display("TB RESULT: TESTS FAILED");
        $display("==================================================");
        $finish;
    end

    //--------------------------------------------------------------------
    // 全局看门狗
    //--------------------------------------------------------------------
    initial begin
        #60_000_000;
        $display("[TB][FATAL] global timeout");
        $finish;
    end

endmodule
