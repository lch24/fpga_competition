`timescale 1ns / 1ps
//==============================================================================
// tb_memory.sv — ddr_memory_model 自测平台
//------------------------------------------------------------------------------
// M1 验收项：仿真 DDR 模型通过读改写一致性自测。
//
// 握手检测方法（重要）：
//   TB 在 negedge 用阻塞赋值驱动激励；在 posedge 用采样寄存器（NBA）
//   捕获 valid&&ready 的"fire"标志与载荷。采样寄存器的 RHS 在 posedge
//   活动区求值，与模型判定逻辑看到的是同一组（更新前）信号值，
//   因此 fire ⇔ 模型已接受。避免了"negedge 读组合 ready"与
//   "模型 posedge 采样"之间的幻象握手竞争。
//
// 覆盖的检查（对应 VERILOG_DESIGN_PLAN 第11节"DDR 服务"行）：
//   T1  4 字节对齐块写入→读回逐字节比对（读改写一致性）
//   T2  非对齐起点 addr=5、len=7 的读写（字节打包/keep 语义）
//   T3  len=5 的尾拍 keep=0001、last 位置检查
//   T4  部分覆盖写：小写事务不得破坏周围数据（padding/覆盖语义）
//   T5  1920 字节大块（模拟一行 BGR 数据，480 拍背靠背）
//   T6  读错误注入：命中范围返回 error=1
//   T7  写错误注入：完成 error=1 且内存内容不被破坏
//   T8  tag 匹配（贯穿所有事务的自动检查）
//   T9  valid&&!ready 时载荷保持（全局监视器自动检查）
//   T10 故意发 early-LAST 的违规写：模型计数违规、返回 error、不死锁
//   T11 违规后模型恢复正常服务
//
// 全程开启：随机延迟 + 随机背压（服务端 ready 与客户端 ready 均随机，
// LFSR 种子固定，运行完全可复现）。
//
// 通过标准：fails==0 且 dut 的违规计数==1（仅 T10 的故意违规）。
//==============================================================================
module tb_memory;

    localparam CLK_PERIOD = 10;

    logic clk = 1'b0;
    logic rst_n;

    always #(CLK_PERIOD/2) clk = ~clk;

    //--------------------------------------------------------------------
    // DUT 连线
    //--------------------------------------------------------------------
    logic        rd_req_valid, rd_req_ready;
    logic [31:0] rd_req_addr, rd_req_len_bytes;
    logic [15:0] rd_req_tag;
    logic        rd_ret_valid, rd_ret_ready;
    logic [31:0] rd_ret_data;
    logic [3:0]  rd_ret_keep;
    logic [15:0] rd_ret_tag;
    logic        rd_ret_last, rd_ret_error;
    logic        wr_req_valid, wr_req_ready;
    logic [31:0] wr_req_addr, wr_req_len_bytes;
    logic [15:0] wr_req_tag;
    logic        wr_dat_valid, wr_dat_ready;
    logic [31:0] wr_dat_data;
    logic [3:0]  wr_dat_keep;
    logic        wr_dat_last;
    logic        wr_cplt_valid, wr_cplt_ready;
    logic [15:0] wr_cplt_tag;
    logic        wr_cplt_error;

    ddr_memory_model #(
        .LATENCY_MIN     (1),
        .LATENCY_MAX     (6),
        .JITTER_EN       (1'b1),
        .BACKPRESSURE_EN (1'b1),
        .PROTOCOL_CHECKS (1'b1),
        .SEED            (32'h1234_5679)
    ) dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .rd_req_valid    (rd_req_valid),
        .rd_req_ready    (rd_req_ready),
        .rd_req_addr     (rd_req_addr),
        .rd_req_len_bytes(rd_req_len_bytes),
        .rd_req_tag      (rd_req_tag),
        .rd_ret_valid    (rd_ret_valid),
        .rd_ret_ready    (rd_ret_ready),
        .rd_ret_data     (rd_ret_data),
        .rd_ret_keep     (rd_ret_keep),
        .rd_ret_tag      (rd_ret_tag),
        .rd_ret_last     (rd_ret_last),
        .rd_ret_error    (rd_ret_error),
        .wr_req_valid    (wr_req_valid),
        .wr_req_ready    (wr_req_ready),
        .wr_req_addr     (wr_req_addr),
        .wr_req_len_bytes(wr_req_len_bytes),
        .wr_req_tag      (wr_req_tag),
        .wr_dat_valid    (wr_dat_valid),
        .wr_dat_ready    (wr_dat_ready),
        .wr_dat_data     (wr_dat_data),
        .wr_dat_keep     (wr_dat_keep),
        .wr_dat_last     (wr_dat_last),
        .wr_cplt_valid   (wr_cplt_valid),
        .wr_cplt_ready   (wr_cplt_ready),
        .wr_cplt_tag     (wr_cplt_tag),
        .wr_cplt_error   (wr_cplt_error),
        .proto_violations(                            )
    );

    //--------------------------------------------------------------------
    // 客户端侧随机 ready（读返回/写完成通道），LFSR 可复现。
    // 注意：ready 由 posedge 更新的 LFSR 组合驱动；模型在 posedge 活动
    // 区采样到的是"上一拍"的 LFSR 值——这是有意保留的真实背压场景，
    // fire 采样器与模型看到的是同一组值，不会产生幻象握手。
    //--------------------------------------------------------------------
    logic [15:0] tb_lfsr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tb_lfsr <= 16'hACE1;
        else
            tb_lfsr <= {tb_lfsr[14:0], tb_lfsr[15] ^ tb_lfsr[13] ^ tb_lfsr[12] ^ tb_lfsr[2]};
    end
    assign rd_ret_ready  = !(tb_lfsr[1:0] == 2'b00);   // ~25% 拉低
    assign wr_cplt_ready = !(tb_lfsr[4:3] == 2'b00);

    //--------------------------------------------------------------------
    // posedge 采样监视器：fire 标志与载荷
    // RHS 在 posedge 活动区求值（寄存器 NBA 更新前），与模型判定一致。
    //--------------------------------------------------------------------
    logic f_rd_req, f_wr_req, f_wr_dat, f_rd_ret, f_wr_cplt;
    logic [31:0] s_rd_data;
    logic [3:0]  s_rd_keep;
    logic [15:0] s_rd_tag;
    logic        s_rd_last, s_rd_err;
    logic [15:0] s_cplt_tag;
    logic        s_cplt_err;

    always @(posedge clk) begin
        f_rd_req  <= rd_req_valid && rd_req_ready;
        f_wr_req  <= wr_req_valid && wr_req_ready;
        f_wr_dat  <= wr_dat_valid && wr_dat_ready;
        f_rd_ret  <= rd_ret_valid && rd_ret_ready;
        f_wr_cplt <= wr_cplt_valid && wr_cplt_ready;
        s_rd_data <= rd_ret_data;
        s_rd_keep <= rd_ret_keep;
        s_rd_tag  <= rd_ret_tag;
        s_rd_last <= rd_ret_last;
        s_rd_err  <= rd_ret_error;
        s_cplt_tag <= wr_cplt_tag;
        s_cplt_err <= wr_cplt_error;
    end

    //--------------------------------------------------------------------
    // 全局 T9 检查：读返回流 valid&&!ready 停顿时，载荷必须逐拍保持
    //--------------------------------------------------------------------
    int fails = 0;

    logic        p_vld, p_fire;
    logic [54:0] p_pl;   // {data, keep, tag, last, error}
    logic [54:0] now_pl;

    assign now_pl = {rd_ret_data, rd_ret_keep, rd_ret_tag, rd_ret_last, rd_ret_error};

    always @(posedge clk) begin
        if (rst_n && p_vld && !p_fire) begin
            if (!rd_ret_valid || (now_pl !== p_pl)) begin
                fails = fails + 1;
                $display("[TB][FAIL] @%0t read stream payload not held during stall (T9)",
                         $time);
            end
        end
        p_vld  <= rd_ret_valid;
        p_fire <= rd_ret_valid && rd_ret_ready;
        p_pl   <= now_pl;
    end

    //--------------------------------------------------------------------
    // 测试变量与统计
    //--------------------------------------------------------------------
    logic [15:0] g_tag = 16'd0;

    bit [7:0]   q256[], qr256[];
    bit [7:0]   qbig[], qrbig[];
    bit [7:0]   qsmall[], qrsmall[];
    logic [4:0] beats[$];

    function automatic bit [7:0] pat(input int unsigned i);
        pat = (i * 7 + 13) % 256;
    endfunction

    //--------------------------------------------------------------------
    // BFM：写事务（请求→数据→完成）。exp_err=1 时期望完成通道 error=1。
    // 激励：negedge 阻塞赋值；握手：negedge 检查 fire 标志。
    //--------------------------------------------------------------------
    task automatic cli_write(input logic [31:0] addr, input int unsigned len,
                             ref bit [7:0] q [], input bit exp_err);
        int unsigned nbeats, b, k, off;
        logic [31:0] d;
        logic [3:0]  kp;
        logic        lst;

        // 1) 请求
        @(negedge clk);
        wr_req_valid     = 1'b1;
        wr_req_addr      = addr;
        wr_req_len_bytes = len;
        wr_req_tag       = g_tag;
        forever begin
            @(negedge clk);
            if (f_wr_req) break;
        end
        wr_req_valid = 1'b0;

        // 2) 数据拍
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

        // 3) 完成
        forever begin
            @(negedge clk);
            if (f_wr_cplt) begin
                if (s_cplt_tag !== g_tag) begin
                    fails = fails + 1;
                    $display("[TB][FAIL] @%0t write complete tag %0d != %0d",
                             $time, s_cplt_tag, g_tag);
                end
                if (s_cplt_err !== exp_err) begin
                    fails = fails + 1;
                    $display("[TB][FAIL] @%0t write complete error=%0b, expected %0b",
                             $time, s_cplt_err, exp_err);
                end
                break;
            end
        end
        g_tag = g_tag + 16'd1;
    endtask

    //--------------------------------------------------------------------
    // BFM：读事务（请求→收流）。收集字节流与 (last,keep) 序列。
    //--------------------------------------------------------------------
    task automatic cli_read(input logic [31:0] addr, input int unsigned len,
                            ref bit [7:0] q [], input bit exp_err,
                            ref logic [4:0] beats_q [$], output bit got_err);
        int unsigned off;

        // 1) 请求
        @(negedge clk);
        rd_req_valid     = 1'b1;
        rd_req_addr      = addr;
        rd_req_len_bytes = len;
        rd_req_tag       = g_tag;
        forever begin
            @(negedge clk);
            if (f_rd_req) break;
        end
        rd_req_valid = 1'b0;

        // 2) 收流（fire 由全局监视器捕获，载荷取同拍采样值）
        off      = 0;
        got_err  = 1'b0;
        beats_q.delete();
        forever begin
            @(negedge clk);
            if (f_rd_ret) begin
                if (s_rd_tag !== g_tag) begin
                    fails = fails + 1;
                    $display("[TB][FAIL] @%0t read return tag %0d != %0d",
                             $time, s_rd_tag, g_tag);
                end
                for (int k = 0; k < 4; k++)
                    if (s_rd_keep[k])
                        q[off + k] = s_rd_data[8*k +: 8];
                beats_q.push_back({s_rd_last, s_rd_keep});
                got_err = got_err | s_rd_err;
                off += 4;
                if (s_rd_last) break;
            end
        end
        if (got_err !== exp_err) begin
            fails = fails + 1;
            $display("[TB][FAIL] @%0t read error=%0b, expected %0b",
                     $time, got_err, exp_err);
        end
        g_tag = g_tag + 16'd1;
    endtask

    //--------------------------------------------------------------------
    // keep/last 流序列检查
    //--------------------------------------------------------------------
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
    // BFM：故意违规写（early LAST）—— T10 专用
    //--------------------------------------------------------------------
    task automatic cli_write_bad_early_last(input logic [31:0] addr,
                                            input int unsigned len,
                                            input bit exp_err);
        @(negedge clk);
        wr_req_valid     = 1'b1;
        wr_req_addr      = addr;
        wr_req_len_bytes = len;
        wr_req_tag       = g_tag;
        forever begin
            @(negedge clk);
            if (f_wr_req) break;
        end
        wr_req_valid = 1'b0;

        @(negedge clk);
        wr_dat_valid = 1'b1;
        wr_dat_data  = 32'hAABB_CCDD;
        wr_dat_keep  = 4'b1111;
        wr_dat_last  = 1'b1;            // len=8 却在第 1 拍就 last：违规
        forever begin
            @(negedge clk);
            if (f_wr_dat) break;
        end
        wr_dat_valid = 1'b0;

        forever begin
            @(negedge clk);
            if (f_wr_cplt) begin
                if (s_cplt_tag !== g_tag) begin
                    fails = fails + 1;
                    $display("[TB][FAIL] bad-write tag mismatch");
                end
                if (s_cplt_err !== exp_err) begin
                    fails = fails + 1;
                    $display("[TB][FAIL] bad-write error=%0b, expected %0b",
                             s_cplt_err, exp_err);
                end
                break;
            end
        end
        g_tag = g_tag + 16'd1;
    endtask

    //--------------------------------------------------------------------
    // 数据比对辅助
    //--------------------------------------------------------------------
    task automatic check_data(input string name, input int unsigned len,
                              ref bit [7:0] got [], ref bit [7:0] exp []);
        for (int unsigned i = 0; i < len; i++) begin
            if (got[i] !== exp[i]) begin
                fails = fails + 1;
                $display("[TB][FAIL] %s: byte %0d got 0x%02x != 0x%02x",
                         name, i, got[i], exp[i]);
                if (fails > 20) return;
            end
        end
    endtask

    bit err;
    logic [15:0] viol_before, viol_after;

    //--------------------------------------------------------------------
    // 主测试序列
    //--------------------------------------------------------------------
    initial begin
        // 激励初始化（阻塞赋值，避免 x 传播）
        rd_req_valid = 1'b0; rd_req_addr = '0; rd_req_len_bytes = '0; rd_req_tag = '0;
        wr_req_valid = 1'b0; wr_req_addr = '0; wr_req_len_bytes = '0; wr_req_tag = '0;
        wr_dat_valid = 1'b0; wr_dat_data = '0; wr_dat_keep = '0; wr_dat_last = 1'b0;
        rst_n = 1'b0;

        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5)  @(negedge clk);

        //--------------------------------------------------------------
        // T1: 4 字节对齐块读写（256B，64 拍）
        //--------------------------------------------------------------
        begin
            q256  = new[256];
            qr256 = new[256];
            for (int unsigned i = 0; i < 256; i++) q256[i] = pat(i);
            cli_write(32'd64, 256, q256, 1'b0);
            cli_read (32'd64, 256, qr256, 1'b0, beats, err);
            check_stream("T1", 256, beats);
            check_data  ("T1", 256, qr256, q256);
            $display("[TB] T1  4-aligned 256B rw               : done");
        end

        //--------------------------------------------------------------
        // T2: 非对齐起点 addr=5, len=7（2 拍：1111 / 0111）
        //--------------------------------------------------------------
        begin
            qsmall  = new[7];
            qrsmall = new[7];
            for (int unsigned i = 0; i < 7; i++) qsmall[i] = pat(i + 100);
            cli_write(32'd5, 7, qsmall, 1'b0);
            cli_read (32'd5, 7, qrsmall, 1'b0, beats, err);
            check_stream("T2", 7, beats);
            check_data  ("T2", 7, qrsmall, qsmall);
            $display("[TB] T2  unaligned addr=5 len=7           : done");
        end

        //--------------------------------------------------------------
        // T3: len=5 尾拍 keep=0001（2 拍：1111 / 0001）
        //--------------------------------------------------------------
        begin
            qsmall  = new[5];
            qrsmall = new[5];
            for (int unsigned i = 0; i < 5; i++) qsmall[i] = pat(i + 200);
            cli_write(32'd1000, 5, qsmall, 1'b0);
            cli_read (32'd1000, 5, qrsmall, 1'b0, beats, err);
            check_stream("T3", 5, beats);
            check_data  ("T3", 5, qrsmall, qsmall);
            $display("[TB] T3  len=5 tail keep                  : done");
        end

        //--------------------------------------------------------------
        // T4: 部分覆盖写不破坏周围数据
        //--------------------------------------------------------------
        begin
            bit [7:0] patch[];
            patch = new[4];
            patch[0] = 8'hAB; patch[1] = 8'hAB; patch[2] = 8'hAB; patch[3] = 8'hAB;
            cli_write(32'd70, 4, patch, 1'b0);
            cli_read(32'd64, 256, qr256, 1'b0, beats, err);
            for (int unsigned i = 0; i < 256; i++) begin
                if (i >= 6 && i <= 9)
                    q256[i] = 8'hAB;
                else
                    q256[i] = pat(i);
            end
            check_data  ("T4", 256, qr256, q256);
            $display("[TB] T4  partial overwrite protection     : done");
        end

        //--------------------------------------------------------------
        // T5: 大块读写 1920B（模拟一行 BGR，480 拍背靠背）
        //--------------------------------------------------------------
        begin
            qbig  = new[1920];
            qrbig = new[1920];
            for (int unsigned i = 0; i < 1920; i++) qbig[i] = pat(i + 5000);
            cli_write(32'h0000_1000, 1920, qbig, 1'b0);
            cli_read (32'h0000_1000, 1920, qrbig, 1'b0, beats, err);
            check_stream("T5", 1920, beats);
            check_data  ("T5", 1920, qrbig, qbig);
            $display("[TB] T5  1920B row-sized rw               : done");
        end

        //--------------------------------------------------------------
        // T6: 读错误注入
        //--------------------------------------------------------------
        begin
            qrsmall = new[16];
            dut.inject_error(32'd2000, 32'd3000);
            cli_read(32'd2500, 16, qrsmall, 1'b1, beats, err);
            dut.clear_error_injection();
            $display("[TB] T6  read error injection             : done");
        end

        //--------------------------------------------------------------
        // T7: 写错误注入：完成 error=1 且内存不被破坏
        //--------------------------------------------------------------
        begin
            bit [7:0] good[];
            bit [7:0] bad[];
            good = new[8];
            bad  = new[8];
            for (int unsigned i = 0; i < 8; i++) good[i] = pat(i + 7000);
            for (int unsigned i = 0; i < 8; i++) bad[i]  = ~pat(i + 7000);
            cli_write(32'd5500, 8, good, 1'b0);
            dut.inject_error(32'd5000, 32'd6000);
            cli_write(32'd5500, 8, bad, 1'b1);
            dut.clear_error_injection();
            qrsmall = new[8];
            cli_read(32'd5500, 8, qrsmall, 1'b0, beats, err);
            check_data("T7", 8, qrsmall, good);
            $display("[TB] T7  write error injection            : done");
        end

        //--------------------------------------------------------------
        // T8: tag 匹配检查已内嵌 cli_write/cli_read
        // T9: 载荷保持检查由全局监视器执行
        //--------------------------------------------------------------
        $display("[TB] T8  tag match (embedded)              : done");
        $display("[TB] T9  payload hold (monitor)            : done");

        //--------------------------------------------------------------
        // T10: 故意 early-LAST 违规写：违规计数 +1、error=1、不死锁
        //--------------------------------------------------------------
        begin
            viol_before = dut.viol;
            cli_write_bad_early_last(32'd3000, 8, 1'b1);
            viol_after = dut.viol;
            if (viol_after !== viol_before + 16'd1) begin
                fails = fails + 1;
                $display("[TB][FAIL] T10: violations %0d -> %0d, expected +1",
                         viol_before, viol_after);
            end
            $display("[TB] T10 early-LAST violation detected    : done");
        end

        //--------------------------------------------------------------
        // T11: 违规后模型恢复正常
        //--------------------------------------------------------------
        begin
            qsmall  = new[12];
            qrsmall = new[12];
            for (int unsigned i = 0; i < 12; i++) qsmall[i] = pat(i + 9000);
            cli_write(32'd4000, 12, qsmall, 1'b0);
            cli_read (32'd4000, 12, qrsmall, 1'b0, beats, err);
            check_stream("T11", 12, beats);
            check_data  ("T11", 12, qrsmall, qsmall);
            $display("[TB] T11 recovery after violation         : done");
        end

        //--------------------------------------------------------------
        // 汇总
        //--------------------------------------------------------------
        $display("==================================================");
        $display("TB SUMMARY: fails=%0d  model violations=%0d (expected 1)",
                 fails, dut.viol);
        if (fails == 0 && dut.viol == 16'd1)
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
        #50_000_000;
        $display("[TB][FATAL] global timeout");
        $finish;
    end

endmodule
