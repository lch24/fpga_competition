`timescale 1ns / 1ps
//==============================================================================
// tb_dma.sv — M7.1 raster_dma 位级对拍 TB（ModelSim 10.6e）
//------------------------------------------------------------------------------
// DUT  ：rtl/mem/raster_dma.v（行光栅 DDR 读/写调度器）
// 模型 ：ddr_memory_model.sv（M1 交付，只读；参数按 M7 要求全开：
//        LATENCY_MIN=1..LATENCY_MAX=6、JITTER_EN=1、BACKPRESSURE_EN=1、
//        PROTOCOL_CHECKS=1；固定 SEED 保证可复现）。
//
// 用例矩阵（每用例独立 start，连续帧复用）：
//   T1  读对齐      ：128×64，stride=132（行尾 padding），row_bytes=128，offset=0
//   T2  读非对齐    ：offset=2、row_bytes=127、stride=131（非对齐起点 + keep 解包）
//   T3a 写对齐      ：T1 布局反向（模型内存逐字节 == 源字节流）
//   T3b 写非对齐    ：T2 布局反向
//   T4a 读行尾尾字节：row_bytes=130（%4==2，尾字 keep=0011）
//   T4b 写行尾尾字节：同布局反向
//   T5a 读随机背压  ：模型背压全开 + 输出侧每 8 拍停 1 拍（out_ready）
//   T5b 写随机背压  ：模型背压全开 + 输入侧周期间隙（每 4 字节停 2 拍）
//   T6a 读错误注入  ：inject_error() 命中 → status=10
//   T6b 写错误注入  ：inject_error() 命中 → status=11
//   T7  随机布局回归：固定种子 LFSR 循环 20 例（base/stride/row_bytes/rows/offset
//                     随机，含 4 字节对齐边界与行尾 padding）
//
// 校验点：模型内存/字节流逐字节一致、keep/last 协议零违规（proto_violations
//   不增长）、错误路径 status 正确、超时保护（每用例 + 全局 watchdog）。
// 最终打印 "TB RESULT: ALL DMA TESTS PASSED" / FAILED。
//==============================================================================
module tb_dma;

    //--------------------------------------------------------------------
    // 时钟 / 复位（rst_n 同步释放）
    //--------------------------------------------------------------------
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst_n = 1'b0;
    initial begin
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
    end

    // 全局看门狗（防死锁；30ms，远大于用例总时长）
    initial begin
        #30_000_000;   // 30ms
        $display("[FATAL] GLOBAL TIMEOUT");
        $display("TB RESULT: SOME DMA TESTS FAILED");
        $finish;
    end

    //--------------------------------------------------------------------
    // DUT 接口
    //--------------------------------------------------------------------
    reg                  start;
    reg [31:0]           cfg_base, cfg_stride, cfg_offset;
    reg [15:0]           cfg_row_bytes, cfg_rows;
    reg                  cfg_dir;
    wire                 busy, done;
    wire [1:0]           status;
    wire                 out_valid;
    reg                  out_ready;
    wire [7:0]           out_byte;
    reg                  in_valid;
    wire                 in_ready;
    reg [7:0]            in_byte;

    // 读返回 / 写完成 tag 观测（信息打印用）
    wire [15:0]          rd_ret_tag, wr_done_tag;

    //--------------------------------------------------------------------
    // DDR 模型例化（参数按 M7 契约全开）
    //--------------------------------------------------------------------
    wire rd_req_valid, rd_req_ready, rd_ret_valid, rd_ret_ready, rd_ret_last, rd_ret_error;
    wire [31:0] rd_req_addr, rd_ret_data;
    wire [31:0] rd_req_len;
    wire [15:0] rd_req_tag;
    wire [3:0]  rd_ret_keep;
    wire wr_req_valid, wr_req_ready, wr_dat_valid, wr_dat_ready, wr_dat_last;
    wire wr_done_valid, wr_done_ready, wr_done_error;
    wire [31:0] wr_req_addr, wr_req_len, wr_dat_data;
    wire [15:0] wr_req_tag;
    wire [3:0]  wr_dat_keep;
    wire [15:0] proto_violations;

    ddr_memory_model #(
        .LATENCY_MIN     (1),
        .LATENCY_MAX     (6),
        .JITTER_EN       (1'b1),
        .BACKPRESSURE_EN (1'b1),
        .PROTOCOL_CHECKS (1'b1),
        .SEED            (32'h1234_5679)
    ) u_ddr (
        .clk            (clk),
        .rst_n          (rst_n),
        .rd_req_valid   (rd_req_valid),
        .rd_req_ready   (rd_req_ready),
        .rd_req_addr    (rd_req_addr),
        .rd_req_len_bytes(rd_req_len),
        .rd_req_tag     (rd_req_tag),
        .rd_ret_valid   (rd_ret_valid),
        .rd_ret_ready   (rd_ret_ready),
        .rd_ret_data    (rd_ret_data),
        .rd_ret_keep    (rd_ret_keep),
        .rd_ret_tag     (rd_ret_tag),
        .rd_ret_last    (rd_ret_last),
        .rd_ret_error   (rd_ret_error),
        .wr_req_valid   (wr_req_valid),
        .wr_req_ready   (wr_req_ready),
        .wr_req_addr    (wr_req_addr),
        .wr_req_len_bytes(wr_req_len),
        .wr_req_tag     (wr_req_tag),
        .wr_dat_valid   (wr_dat_valid),
        .wr_dat_ready   (wr_dat_ready),
        .wr_dat_data    (wr_dat_data),
        .wr_dat_keep    (wr_dat_keep),
        .wr_dat_last    (wr_dat_last),
        .wr_cplt_valid  (wr_done_valid),
        .wr_cplt_ready  (wr_done_ready),
        .wr_cplt_tag    (wr_done_tag),
        .wr_cplt_error  (wr_done_error),
        .proto_violations(proto_violations)
    );

    //--------------------------------------------------------------------
    // DUT 例化（冻结接口）
    //--------------------------------------------------------------------
    raster_dma u_dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (start),
        .cfg_base      (cfg_base),
        .cfg_stride    (cfg_stride),
        .cfg_row_bytes (cfg_row_bytes),
        .cfg_rows      (cfg_rows),
        .cfg_offset    (cfg_offset),
        .cfg_dir       (cfg_dir),
        .busy          (busy),
        .done          (done),
        .status        (status),
        .out_valid     (out_valid),
        .out_ready     (out_ready),
        .out_byte      (out_byte),
        .in_valid      (in_valid),
        .in_ready      (in_ready),
        .in_byte       (in_byte),
        .rd_req_valid  (rd_req_valid),
        .rd_req_ready  (rd_req_ready),
        .rd_req_addr   (rd_req_addr),
        .rd_req_len    (rd_req_len),
        .rd_req_tag    (rd_req_tag),
        .rd_ret_valid  (rd_ret_valid),
        .rd_ret_ready  (rd_ret_ready),
        .rd_ret_data   (rd_ret_data),
        .rd_ret_keep   (rd_ret_keep),
        .rd_ret_tag    (rd_ret_tag),
        .rd_ret_last   (rd_ret_last),
        .rd_ret_error  (rd_ret_error),
        .wr_req_valid  (wr_req_valid),
        .wr_req_ready  (wr_req_ready),
        .wr_req_addr   (wr_req_addr),
        .wr_req_len    (wr_req_len),
        .wr_req_tag    (wr_req_tag),
        .wr_dat_valid  (wr_dat_valid),
        .wr_dat_ready  (wr_dat_ready),
        .wr_dat_data   (wr_dat_data),
        .wr_dat_keep   (wr_dat_keep),
        .wr_dat_last   (wr_dat_last),
        .wr_done_valid (wr_done_valid),
        .wr_done_ready (wr_done_ready),
        .wr_done_tag   (wr_done_tag),
        .wr_done_error (wr_done_error)
    );

    //--------------------------------------------------------------------
    // 输出侧背压模式（0=恒 1；1=每 8 拍停 1 拍）
    //--------------------------------------------------------------------
    reg        out_bp_mode = 1'b0;
    reg [3:0]  obp_cnt = 4'd0;
    always @(posedge clk) begin
        if (!rst_n) begin
            obp_cnt <= 4'd0;
            out_ready <= 1'b1;
        end else if (out_bp_mode) begin
            obp_cnt  <= obp_cnt + 4'd1;
            out_ready <= (obp_cnt[2:0] != 3'b111) ? 1'b1 : 1'b0;
        end else begin
            obp_cnt  <= 4'd0;
            out_ready <= 1'b1;
        end
    end

    //--------------------------------------------------------------------
    // 统计 / 判定
    //--------------------------------------------------------------------
    integer fail_cnt = 0;
    integer viol_last = 0;

    // 字节流缓冲
    logic [7:0] got_arr[];
    logic [7:0] src_arr[];

    // 数据模式：mode 0=地址低 8 位（连续递增，错位立现）
    //            1=地址位混叠（行内/行间强相关变化）
    //            2=低位异或移位（伪随机感）
    function automatic logic [7:0] pat_val(input longint a, input int mode);
        case (mode)
            0:      pat_val = a[7:0];
            1:      pat_val = (a[7:0] + a[11:8]) & 8'hFF;
            default: pat_val = (a[11:0] ^ (a[11:0] >> 1)) & 8'hFF;
        endcase
    endfunction

    // 预载模型内存 payload 区
    task automatic preload_mem(input longint base, input longint stride,
                               input int row_bytes, input int rows,
                               input longint offset, input int mode);
        for (int y = 0; y < rows; y++)
            for (int i = 0; i < row_bytes; i++) begin
                longint a = base + y*stride + offset + i;
                u_ddr.mem[a[31:0]] = pat_val(a, mode);
            end
    endtask

    // 启动一帧（配置提前稳定，start 单拍；返回后 DUT 已完成锁存）
    task automatic dma_start(input longint base, input longint stride,
                             input int row_bytes, input int rows,
                             input longint offset, input int dir);
        @(posedge clk);
        cfg_base      <= base[31:0];
        cfg_stride    <= stride[31:0];
        cfg_row_bytes <= row_bytes[15:0];
        cfg_rows      <= rows[15:0];
        cfg_offset    <= offset[31:0];
        cfg_dir       <= dir[0];
        @(posedge clk);
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;
        @(posedge clk);   // 等 DUT 锁存完成（busy/done/status 更新生效）
    endtask

    // 等待 done（带超时保护）
    task automatic wait_done(input string name);
        integer guard = 0;
        while (!done) begin
            if (guard > 500000) begin
                $display("[%s] TIMEOUT waiting done", name);
                fail_cnt = fail_cnt + 1;
                return;
            end
            guard = guard + 1;
            @(posedge clk);
        end
        repeat (2) @(posedge clk);
    endtask

    //--------------------------------------------------------------------
    // 读方向用例
    //--------------------------------------------------------------------
    task automatic run_read(input string name, input longint base, input longint stride,
                            input int row_bytes, input int rows, input longint offset,
                            input int mode, input int bp);
        integer total, cnt, err, i;
        longint t0;
        total = rows * row_bytes;
        t0 = $time;
        preload_mem(base, stride, row_bytes, rows, offset, mode);
        out_bp_mode = bp[0];
        dma_start(base, stride, row_bytes, rows, offset, 0);
        // 接收 out 字节流
        got_arr = new[total];
        cnt = 0;
        while (!done) begin
            if (cnt > total + 16) begin
                $display("[%s] byte count overflow %0d", name, cnt);
                fail_cnt = fail_cnt + 1;
                return;
            end
            @(posedge clk);
            if (out_valid && out_ready) begin
                if (cnt < total) got_arr[cnt] = out_byte;
                cnt = cnt + 1;
            end
        end
        // 对比
        err = 0;
        for (i = 0; i < total; i++) begin
            longint a = base + (i / row_bytes)*stride + offset + (i % row_bytes);
            if (got_arr[i] !== pat_val(a, mode)) err = err + 1;
        end
        if (cnt != total) begin
            $display("[%s] byte count mismatch: got %0d expect %0d", name, cnt, total);
            fail_cnt = fail_cnt + 1;
            return;
        end
        if (err != 0) begin
            $display("[%s] data mismatch: %0d bytes", name, err);
            fail_cnt = fail_cnt + 1;
        end
        if (status !== 2'b01 || busy !== 1'b0) begin
            $display("[%s] status/busy wrong: status=%b busy=%b", name, status, busy);
            fail_cnt = fail_cnt + 1;
        end
        check_viol(name);
        $display("[%s] bytes=%0d time=%0tns -> %s", name, total, $time - t0,
                 (err == 0 && cnt == total && status === 2'b01) ? "PASS" : "FAIL");
        out_bp_mode = 1'b0;
    endtask

    // 读方向错误注入用例
    task automatic run_read_err(input string name, input longint base, input longint stride,
                                input int row_bytes, input int rows, input longint offset);
        integer total, cnt, guard;
        logic [31:0] e_min, e_max;
        total = rows * row_bytes;
        preload_mem(base, stride, row_bytes, rows, offset, 1);
        out_bp_mode = 1'b0;
        // 错误注入覆盖整个区域 → 首行请求即命中
        e_min = base;                              // longint → 低 32 位
        e_max = base + rows*stride + offset + row_bytes;
        u_ddr.inject_error(e_min, e_max);
        dma_start(base, stride, row_bytes, rows, offset, 0);
        cnt = 0;
        guard = 0;
        while (!done) begin
            if (guard > 500000) begin
                $display("[%s] TIMEOUT waiting done", name);
                fail_cnt = fail_cnt + 1;
                return;
            end
            guard = guard + 1;
            @(posedge clk);
            if (out_valid && out_ready) cnt = cnt + 1;   // 只接收，不对比
        end
        u_ddr.clear_error_injection();
        if (status !== 2'b10) begin
            $display("[%s] status wrong: expect 10 got %b", name, status);
            fail_cnt = fail_cnt + 1;
        end
        if (busy !== 1'b0) begin
            $display("[%s] busy not clear", name);
            fail_cnt = fail_cnt + 1;
        end
        check_viol(name);
        $display("[%s] drained_bytes=%0d -> %s", name, cnt,
                 (status === 2'b10 && busy === 1'b0) ? "PASS" : "FAIL");
    endtask

    //--------------------------------------------------------------------
    // 写方向用例（in 字节流 → DDR；gap_every/gap_len=输入侧周期间隙）
    //--------------------------------------------------------------------
    task automatic feed_in(input int total, input int gap_every, input int gap_len);
        integer i;
        i = 0;
        in_valid = 1'b0;
        while (i < total) begin
            if (done) begin
                // DUT 提前中止（错误路径只消费了部分行）→ 停止喂流
                in_valid = 1'b0;
                break;
            end
            if (in_valid && in_ready) begin
                i = i + 1;                       // 本拍 DUT 已收上一字节
                if (i >= total) begin
                    in_valid = 1'b0;
                end else if (gap_every > 0 && (i % gap_every) == 0) begin
                    in_valid = 1'b0;             // 周期间隙
                    repeat (gap_len) @(posedge clk);
                    in_valid = 1'b1;
                    in_byte  = src_arr[i];
                end else begin
                    in_byte = src_arr[i];
                end
            end else if (!in_valid) begin
                in_valid = 1'b1;
                in_byte  = src_arr[i];
            end
            @(posedge clk);
        end
        in_valid = 1'b0;
    endtask

    task automatic run_write(input string name, input longint base, input longint stride,
                             input int row_bytes, input int rows, input longint offset,
                             input int mode, input int gap_every, input int gap_len);
        integer total, err, i;
        longint t0;
        total = rows * row_bytes;
        t0 = $time;
        // 生成源字节流
        src_arr = new[total];
        for (i = 0; i < total; i++) begin
            longint a = base + (i / row_bytes)*stride + offset + (i % row_bytes);
            src_arr[i] = pat_val(a, mode);
        end
        dma_start(base, stride, row_bytes, rows, offset, 1);
        feed_in(total, gap_every, gap_len);
        wait_done(name);
        if (done !== 1'b1) return;   // 超时已记 fail
        // 对比模型内存
        err = 0;
        for (i = 0; i < total; i++) begin
            longint a = base + (i / row_bytes)*stride + offset + (i % row_bytes);
            if (u_ddr.mem[a[31:0]] !== pat_val(a, mode)) err = err + 1;
        end
        if (err != 0) begin
            $display("[%s] mem mismatch: %0d bytes", name, err);
            fail_cnt = fail_cnt + 1;
        end
        if (status !== 2'b01 || busy !== 1'b0) begin
            $display("[%s] status/busy wrong: status=%b busy=%b", name, status, busy);
            fail_cnt = fail_cnt + 1;
        end
        check_viol(name);
        $display("[%s] bytes=%0d time=%0tns -> %s", name, total, $time - t0,
                 (err == 0 && status === 2'b01) ? "PASS" : "FAIL");
    endtask

    // 写方向错误注入用例
    task automatic run_write_err(input string name, input longint base, input longint stride,
                                 input int row_bytes, input int rows, input longint offset);
        integer total, i;
        logic [31:0] e_min, e_max;
        total = rows * row_bytes;
        src_arr = new[total];
        for (i = 0; i < total; i++) begin
            longint a = base + (i / row_bytes)*stride + offset + (i % row_bytes);
            src_arr[i] = pat_val(a, 1);
        end
        e_min = base;
        e_max = base + rows*stride + offset + row_bytes;
        u_ddr.inject_error(e_min, e_max);
        dma_start(base, stride, row_bytes, rows, offset, 1);
        feed_in(total, 0, 0);
        wait_done(name);
        u_ddr.clear_error_injection();
        if (done !== 1'b1) return;
        if (status !== 2'b11) begin
            $display("[%s] status wrong: expect 11 got %b", name, status);
            fail_cnt = fail_cnt + 1;
        end
        if (busy !== 1'b0) begin
            $display("[%s] busy not clear", name);
            fail_cnt = fail_cnt + 1;
        end
        check_viol(name);
        $display("[%s] -> %s", name,
                 (status === 2'b11 && busy === 1'b0) ? "PASS" : "FAIL");
    endtask

    // 协议违规计数检查（不增长）
    task automatic check_viol(input string name);
        if (proto_violations !== viol_last) begin
            $display("[%s] PROTOCOL VIOLATIONS: %0d (was %0d)", name,
                     proto_violations, viol_last);
            fail_cnt = fail_cnt + 1;
        end
        viol_last = proto_violations;
    endtask

    //--------------------------------------------------------------------
    // 固定种子随机源（xorshift32）
    //--------------------------------------------------------------------
    reg [31:0] rnd = 32'hDEAD_BEEF;
    task automatic next_rand();
        rnd = rnd ^ (rnd << 13);
        rnd = rnd ^ (rnd >> 17);
        rnd = rnd ^ (rnd << 5);
    endtask

    //--------------------------------------------------------------------
    // 主流程
    //--------------------------------------------------------------------
    initial begin
        start = 1'b0;
        in_valid = 1'b0;
        in_byte  = 8'd0;
        cfg_base = 32'd0; cfg_stride = 32'd0; cfg_offset = 32'd0;
        cfg_row_bytes = 16'd0; cfg_rows = 16'd0; cfg_dir = 1'b0;

        // 等复位释放
        @(posedge rst_n);
        @(posedge clk);
        @(posedge clk);
        viol_last = proto_violations;
        $display("=== tb_dma: raster_dma vs ddr_memory_model 位级对拍开始 ===");

        // ---- T1 读对齐 ----
        run_read("T1 read-aligned", 32'h0001_0000, 132, 128, 64, 0, 0, 0);
        // ---- T2 读非对齐（offset=2） ----
        run_read("T2 read-misaligned", 32'h0001_4000, 131, 127, 32, 2, 1, 0);
        // ---- T3a/T3b 写对齐/非对齐 ----
        run_write("T3a write-aligned", 32'h0001_8000, 132, 128, 32, 0, 0, 0, 0);
        run_write("T3b write-misaligned", 32'h0001_C000, 131, 127, 32, 2, 1, 0, 0);
        // ---- T4a/T4b 行尾尾字节（row_bytes=130，%4==2） ----
        run_read("T4a read-tail", 32'h0002_0000, 136, 130, 16, 0, 2, 0);
        run_write("T4b write-tail", 32'h0002_4000, 136, 130, 16, 0, 2, 0, 0);
        // ---- T5a 读随机背压（输出侧每 8 拍停 1） ----
        run_read("T5a read-backpressure", 32'h0002_8000, 132, 128, 16, 0, 0, 1);
        // ---- T5b 写随机背压（输入侧每 4 字节停 2 拍） ----
        run_write("T5b write-backpressure", 32'h0002_C000, 132, 128, 16, 0, 0, 4, 2);
        // ---- T6a 读错误注入 ----
        run_read_err("T6a read-error", 32'h0003_0000, 136, 130, 8, 1);
        // ---- T6b 写错误注入 ----
        run_write_err("T6b write-error", 32'h0003_4000, 136, 130, 8, 1);
        // ---- T7 随机布局回归（固定种子 20 例，含 4 字节对齐边界） ----
        for (int c = 0; c < 20; c++) begin
            longint bbase, bstride, boffset;
            int brow_bytes, brows, bdir, bmode;
            next_rand(); bbase = 32'h0003_8000 + (rnd % 24'h8000);
            next_rand(); brow_bytes = 1 + (rnd % 160);
            next_rand(); bstride = brow_bytes + (rnd % 9);        // 行尾 padding 0..8
            next_rand(); boffset = rnd % (bstride - brow_bytes + 1);
            next_rand(); brows = 1 + (rnd % 8);
            next_rand(); bdir = rnd[0];
            next_rand(); bmode = 0 + (rnd % 3);
            if (bdir[0] == 1'b0)
                run_read($sformatf("T7-r%0d read", c), bbase, bstride, brow_bytes, brows, boffset, bmode, 0);
            else
                run_write($sformatf("T7-r%0d write", c), bbase, bstride, brow_bytes, brows, boffset, bmode, 0, 0);
        end

        // ---- 汇总 ----
        $display("=== tb_dma 完成：总失败计数 = %0d ===", fail_cnt);
        if (fail_cnt == 0)
            $display("TB RESULT: ALL DMA TESTS PASSED");
        else
            $display("TB RESULT: SOME DMA TESTS FAILED");
        $finish;
    end

endmodule
