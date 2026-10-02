`timescale 1ns / 1ps
//==============================================================================
// tb_sort.sv — M4 index_sort 位级对拍
//------------------------------------------------------------------------------
// 向量：m4_sort_<n>_<cs>.bin，格式 u32 n + n×key(f32 位模式) + n×exp_idx(u32)。
// 流程：读文件 → 填外部 key RAM（idx=地址）→ start(n_in=n) →
//       收 out_idx 流与 exp_idx 逐项位级比对。
// 输出带周期性背压（每 8 拍停 1 拍），覆盖 out_ready 反驱。
// 默认跑全部 9 个向量；可用 +VEC=path 只跑单文件（调试）。
//==============================================================================
module tb_sort;

    reg clk = 1'b0;
    always #5 clk = ~clk;                 // 10ns 周期

    reg rst_n;

    //--------------------------------------------------------------------
    // DUT 例化（N_ADDR_W=7，DEPTH=128）
    //--------------------------------------------------------------------
    reg        start;
    wire       busy, done;
    reg  [15:0] n_in;
    wire       rd_en;
    wire [6:0] rd_addr;
    reg  [31:0] rd_key;
    reg  [6:0] rd_idx;
    wire       out_valid;
    reg        out_ready;
    wire [6:0] out_idx;

    index_sort #(.N_ADDR_W(7)) u_dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (start),
        .busy      (busy),
        .done      (done),
        .n_in      (n_in),
        .rd_en     (rd_en),
        .rd_addr   (rd_addr),
        .rd_key    (rd_key),
        .rd_idx    (rd_idx),
        .out_valid (out_valid),
        .out_ready (out_ready),
        .out_idx   (out_idx)
    );

    //--------------------------------------------------------------------
    // 外部 key/idx RAM 模型（1 拍延迟；idx = 请求地址 = 原索引）
    //--------------------------------------------------------------------
    reg [31:0] ext_key[0:127];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_key <= 32'd0;
            rd_idx <= 7'd0;
        end else if (rd_en) begin
            rd_key <= ext_key[rd_addr];
            rd_idx <= rd_addr;
        end
    end

    // 背压：每 8 拍停 1 拍
    reg [3:0] bp_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) bp_cnt <= 4'd0;
        else        bp_cnt <= bp_cnt + 4'd1;
    end
    assign out_ready = (bp_cnt[2:0] == 3'd0) ? 1'b0 : 1'b1;

    //--------------------------------------------------------------------
    // 向量读取
    //--------------------------------------------------------------------
    reg [7:0] fbuf[0:1048575];
    reg [31:0] exp_idx[0:1023];
    integer fd, code;

    function automatic [31:0] rd32(input integer base);
        rd32 = {fbuf[base+3], fbuf[base+2], fbuf[base+1], fbuf[base+0]};
    endfunction

    task run_case(input string path, output integer pass);
        integer k, n, cmp_n, match_cnt, fail_cnt;
        begin
            fd = $fopen(path, "rb");
            if (fd == 0) begin $display("[TB][FATAL] cannot open %s", path); $finish; end
            code = $fread(fbuf, fd);
            $fclose(fd);
            n = rd32(0);
            for (k = 0; k < n; k = k + 1) begin
                ext_key[k] = rd32(4 + k * 4);
                exp_idx[k] = rd32(4 + n * 4 + k * 4);
            end
            $display("[TB] load %s: n=%0d", path, n);

            n_in  = n[15:0];
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;

            cmp_n = 0; match_cnt = 0; fail_cnt = 0;
            while (busy || out_valid) begin
                @(negedge clk);
                if (out_valid && out_ready) begin
                    if (cmp_n < n && out_idx == exp_idx[cmp_n]) begin
                        match_cnt = match_cnt + 1;
                    end else begin
                        fail_cnt = fail_cnt + 1;
                        if (fail_cnt <= 20)
                            $display("[TB][FAIL] idx=%0d got %0d exp %0d",
                                     cmp_n, out_idx, exp_idx[cmp_n]);
                    end
                    cmp_n = cmp_n + 1;
                end
            end

            pass = ((cmp_n == n) && (fail_cnt == 0)) ? 1 : 0;
            $display("[TB] %s: out=%0d cmp=%0d match=%0d fail=%0d exp=%0d -> %s",
                     path, cmp_n, cmp_n, match_cnt, fail_cnt, n,
                     (pass ? "PASS" : "FAIL"));
        end
    endtask

    //--------------------------------------------------------------------
    // 主流程
    //--------------------------------------------------------------------
    string vec_list[0:8] = '{
        "../tests/build/vectors/m4_sort_8_0.bin",
        "../tests/build/vectors/m4_sort_8_1.bin",
        "../tests/build/vectors/m4_sort_8_2.bin",
        "../tests/build/vectors/m4_sort_16_0.bin",
        "../tests/build/vectors/m4_sort_16_1.bin",
        "../tests/build/vectors/m4_sort_16_2.bin",
        "../tests/build/vectors/m4_sort_40_0.bin",
        "../tests/build/vectors/m4_sort_40_1.bin",
        "../tests/build/vectors/m4_sort_40_2.bin"
    };

    integer pass_arr[0:8];
    integer npass, i;
    string  single;

    initial begin
        start = 1'b0;
        n_in  = 16'd0;
        rst_n = 1'b0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        npass = 0;
        if ($value$plusargs("VEC=%s", single)) begin
            run_case(single, pass_arr[0]);
            npass = pass_arr[0];
        end else begin
            for (i = 0; i < 9; i = i + 1) begin
                run_case(vec_list[i], pass_arr[i]);
                npass = npass + pass_arr[i];
            end
        end

        $display("[TB] ==============================================");
        if (npass == 9)
            $display("TB RESULT: ALL PASSED (9/9)");
        else if ($value$plusargs("VEC=%s", single))
            $display("TB RESULT: %s (%0d/1)", (npass ? "ALL PASSED" : "SOME FAILED"), npass);
        else
            $display("TB RESULT: SOME FAILED (%0d/9)", npass);
        $finish;
    end

    // 看门狗
    initial begin
        #200_000_000;
        $display("[TB][FATAL] watchdog timeout");
        $finish;
    end

endmodule
