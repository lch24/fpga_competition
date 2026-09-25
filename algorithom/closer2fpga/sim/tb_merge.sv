`timescale 1ns / 1ps
//==============================================================================
// tb_merge.sv — M3 candidate_store + candidate_merge 位级对拍
//------------------------------------------------------------------------------
// 流程（每场景两轮）：
//   1) 读 m3_<scene>_in.bin → 灌入 store（phase=0 写 A）
//   2) phase=1（merge 读 A，输出流式写 B），start merge(radius=5.0f)
//      输出逐点与 m3_<scene>_merge5.bin 位级比对
//   3) phase=0（merge 读 B，输出写 A），start merge(radius=3.0f)
//      输出逐点与 m3_<scene>_merge3.bin 比对
//   坐标/半径均为 fp32 位模式：5.0f=32'h40A00000，3.0f=32'h40400000。
// 向量格式：u32 N + N×{x,y}（fp32 位模式，小端）。
// 全部位级一致（fp32 位模式完全相等）打印 ALL PASSED。
//==============================================================================
module tb_merge;

    reg clk = 1'b0;
    always #5 clk = ~clk;                 // 10ns 周期

    reg rst_n;

    //--------------------------------------------------------------------
    // DUT 例化
    //--------------------------------------------------------------------
    reg        clr, phase;
    reg        load_phase;                // 1=TB 灌点，0=merge 输出回灌
    reg        tb_wr_v;
    reg [31:0] tb_wr_x, tb_wr_y;
    wire       wr_ready;
    wire [15:0] store_count;
    wire       rd_en_m;
    wire [13:0] rd_addr_m;
    wire [31:0] rd_x_m, rd_y_m;

    reg        start_m;
    wire       busy_m, done_m;
    reg  [15:0] n_in_m;
    reg  [31:0] radius_m;
    wire       res_valid_m;
    wire [31:0] res_x_m, res_y_m;
    wire [14:0] res_count_m;

    assign res_ready_m = 1'b1;            // 无背压

    candidate_store #(.N_ADDR_W(14)) u_store (
        .clk      (clk),
        .rst_n    (rst_n),
        .clr      (clr),
        .phase    (phase),
        .wr_valid (load_phase ? tb_wr_v : res_valid_m),
        .wr_ready (wr_ready),
        .wr_x     (load_phase ? tb_wr_x : res_x_m),
        .wr_y     (load_phase ? tb_wr_y : res_y_m),
        .count    (store_count),
        .rd_en    (rd_en_m),
        .rd_addr  (rd_addr_m),
        .rd_x     (rd_x_m),
        .rd_y     (rd_y_m)
    );

    candidate_merge #(.N_ADDR_W(14)) u_merge (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (start_m),
        .busy     (busy_m),
        .done     (done_m),
        .n_in     (n_in_m),
        .radius   (radius_m),
        .rd_en    (rd_en_m),
        .rd_addr  (rd_addr_m),
        .rd_x     (rd_x_m),
        .rd_y     (rd_y_m),
        .res_valid(res_valid_m),
        .res_ready(res_ready_m),
        .res_x    (res_x_m),
        .res_y    (res_y_m),
        .res_count(res_count_m)
    );

    //--------------------------------------------------------------------
    // 向量数据
    //--------------------------------------------------------------------
    reg [7:0]   fbuf[0:1048575];
    reg [31:0]  in_x[0:16383], in_y[0:16383];
    reg [31:0]  exp_x[0:16383], exp_y[0:16383];
    integer     N_in, N_exp;
    integer     cmp_n, match_cnt, fail_cnt;
    integer     fd, code;

    function automatic [31:0] rd32(input integer base);
        rd32 = {fbuf[base+3], fbuf[base+2], fbuf[base+1], fbuf[base+0]};
    endfunction

    task read_pts(input string path);
        integer k;
        begin
            fd = $fopen(path, "rb");
            if (fd == 0) begin $display("[TB][FATAL] cannot open %s", path); $finish; end
            code = $fread(fbuf, fd);
            $fclose(fd);
            N_in = rd32(0);
            for (k = 0; k < N_in; k = k + 1) begin
                in_x[k] = rd32(4 + k * 8);
                in_y[k] = rd32(4 + k * 8 + 4);
            end
            $display("[TB] load %s: N=%0d", path, N_in);
        end
    endtask

    task read_exp(input string path);
        integer k;
        begin
            fd = $fopen(path, "rb");
            if (fd == 0) begin $display("[TB][FATAL] cannot open %s", path); $finish; end
            code = $fread(fbuf, fd);
            $fclose(fd);
            N_exp = rd32(0);
            for (k = 0; k < N_exp; k = k + 1) begin
                exp_x[k] = rd32(4 + k * 8);
                exp_y[k] = rd32(4 + k * 8 + 4);
            end
            $display("[TB] expect %s: N=%0d", path, N_exp);
        end
    endtask

    // 灌点（phase=0 写 A），灌完 phase=1（merge 读 A 写 B）
    task load_pts();
        integer k;
        begin
            phase = 1'b0;
            clr = 1'b1; @(negedge clk); clr = 1'b0;
            load_phase = 1'b1;
            for (k = 0; k < N_in; k = k + 1) begin
                tb_wr_v  = 1'b1;
                tb_wr_x  = in_x[k];
                tb_wr_y  = in_y[k];
                @(negedge clk);
                if (!wr_ready) begin
                    $display("[TB][FATAL] store full at k=%0d", k);
                    $finish;
                end
            end
            tb_wr_v = 1'b0;
            load_phase = 1'b0;
            phase = 1'b1;
        end
    endtask

    // 第二轮准备：phase=0（merge 读 B 写 A），clr 清 A 写指针
    task prep_round2();
        begin
            phase = 1'b0;
            clr = 1'b1; @(negedge clk); clr = 1'b0;
        end
    endtask

    // 执行一轮 merge 并对拍
    task run_merge_exp(input string exp_path, input [31:0] rad,
                       input integer n_src, output integer pass_out,
                       output integer out_n);
        begin
            read_exp(exp_path);
            n_in_m  = n_src[15:0];
            radius_m = rad;
            start_m = 1'b1;
            @(negedge clk);
            start_m = 1'b0;
            cmp_n = 0; match_cnt = 0; fail_cnt = 0;
            while (busy_m) begin
                @(negedge clk);
                if (res_valid_m && res_ready_m) begin
                if (cmp_n < N_exp &&
                    res_x_m == exp_x[cmp_n] && res_y_m == exp_y[cmp_n])
                    match_cnt = match_cnt + 1;
                else begin
                    fail_cnt = fail_cnt + 1;
                    if (cmp_n < N_exp) begin
                        if (fail_cnt <= 20)
                            $display("[TB][FAIL] idx=%0d got %08x,%08x exp %08x,%08x",
                                     cmp_n, res_x_m, res_y_m,
                                     exp_x[cmp_n], exp_y[cmp_n]);
                    end else
                        $display("[TB][FAIL] extra point got %08x,%08x", res_x_m, res_y_m);
                end
                cmp_n = cmp_n + 1;
            end
            end
            out_n   = res_count_m;
            pass_out = ((cmp_n == N_exp) && (fail_cnt == 0)) ? 1 : 0;
            $display("[TB] merge rad=%08x: out=%0d cmp=%0d match=%0d fail=%0d exp=%0d -> %s",
                     rad, res_count_m, cmp_n, match_cnt, fail_cnt, N_exp,
                     (pass_out ? "PASS" : "FAIL"));
        end
    endtask

    //--------------------------------------------------------------------
    // 主流程
    //--------------------------------------------------------------------
    integer p1, p2, p3, p4;
    integer n1, n2, n3, n4;

    initial begin
        clr = 1'b0; phase = 1'b0; load_phase = 1'b0;
        tb_wr_v = 1'b0; tb_wr_x = 32'd0; tb_wr_y = 32'd0;
        start_m = 1'b0; n_in_m = 16'd0; radius_m = 32'd0;
        rst_n = 1'b0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // ================= scene: small =================
        $display("[TB] ================= scene: small =================");
        read_pts("../tests/build/vectors/m3_small_in.bin");
        load_pts();
        run_merge_exp("../tests/build/vectors/m3_small_merge5.bin", 32'h40A00000, N_in, p1, n1);
        prep_round2();
        run_merge_exp("../tests/build/vectors/m3_small_merge3.bin", 32'h40400000, n1, p2, n2);

        // ================= scene: texture =================
        $display("[TB] ================= scene: texture =================");
        read_pts("../tests/build/vectors/m3_texture_in.bin");
        load_pts();
        run_merge_exp("../tests/build/vectors/m3_texture_merge5.bin", 32'h40A00000, N_in, p3, n3);
        prep_round2();
        run_merge_exp("../tests/build/vectors/m3_texture_merge3.bin", 32'h40400000, n3, p4, n4);

        // ================= 汇总 =================
        $display("[TB] ==============================================");
        if (p1 && p2 && p3 && p4)
            $display("TB RESULT: ALL PASSED (small merge5=%0d merge3=%0d; texture merge5=%0d merge3=%0d)",
                     n1, n2, n3, n4);
        else
            $display("TB RESULT: SOME FAILED (p=%0d,%0d,%0d,%0d)", p1, p2, p3, p4);
        $finish;
    end

    // 看门狗（200ms 仿真时间）
    initial begin
        #200_000_000;
        $display("[TB][FATAL] watchdog timeout");
        $finish;
    end

endmodule
