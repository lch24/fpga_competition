`timescale 1ns / 1ps
//==============================================================================
// tb_acc.sv — subpixel_accum 位级对拍（M5 累加核）
//------------------------------------------------------------------------------
// 向量：../tests/build/vectors/m5_acc.bin（export_m5.cpp::export_acc 生成）
//   格式（小端）：u64 N_win + 每窗口 { r(u64, double 位模式) +
//     (2r+1)^2 组 {x,y,w,gx,gy}(各 u64) + 5 项 {a,b,c,bx,by}(u64) }
// 流程：读 r → n=(2r+1)^2 → start(n_win=n) → 依序喂 n 组样本（y 外 x 内，
//   与 C++ 累加顺序一致）→ 收 5 项输出与期望位级比对。共 8 窗口（r=2,2,3,
//   4,7,7,10,15）。输入侧 in_ready 由 DUT 处理延迟自然反驱。
//==============================================================================
module tb_acc;

    localparam integer CLK_PERIOD = 10;
    logic clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;
    logic rst_n;

    //----------------------------------------------------------------------
    // DUT
    //----------------------------------------------------------------------
    logic        start;
    logic        busy, done;
    logic [15:0] n_win;
    logic        in_valid, in_ready;
    logic [63:0] in_x, in_y, in_w, in_gx, in_gy;
    logic        out_valid, out_ready;
    logic [63:0] out_a, out_b, out_c, out_bx, out_by;

    subpixel_accum #(.N_ADDR_W(8)) u_dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (start),
        .busy      (busy),
        .done      (done),
        .n_win     (n_win),
        .in_valid  (in_valid),
        .in_ready  (in_ready),
        .in_x      (in_x),
        .in_y      (in_y),
        .in_w      (in_w),
        .in_gx     (in_gx),
        .in_gy     (in_gy),
        .out_valid (out_valid),
        .out_ready (out_ready),
        .out_a     (out_a),
        .out_b     (out_b),
        .out_c     (out_c),
        .out_bx    (out_bx),
        .out_by    (out_by)
    );

    //----------------------------------------------------------------------
    // 向量缓冲（小端）
    //----------------------------------------------------------------------
    reg [7:0] fbuf[0:2097151];
    integer  fd, code;
    integer  N_win;

    function automatic [63:0] rd64(input integer base);
        rd64 = {fbuf[base+7], fbuf[base+6], fbuf[base+5], fbuf[base+4],
                fbuf[base+3], fbuf[base+2], fbuf[base+1], fbuf[base+0]};
    endfunction

    // double 位模式 → 整数（仅精确小正整数，r=2..15）
    function automatic [63:0] dbl2int(input [63:0] bits);
        reg [10:0]  ef;
        reg signed [11:0] sh;
        begin
            ef = bits[62:52];
            if (ef == 11'h0) begin
                dbl2int = 64'd0;
            end else begin
                sh = $signed({1'b0, ef}) - 12'sd1023 - 12'sd52;
                if (sh >= 12'sd0)
                    dbl2int = ({1'b1, bits[51:0]}) << sh;
                else
                    dbl2int = ({1'b1, bits[51:0]}) >> (-sh);
            end
        end
    endfunction

    //----------------------------------------------------------------------
    // 喂一个样本（等 in_ready 再握手）
    //----------------------------------------------------------------------
    task automatic feed_sample(input [63:0] x, y, w, gx, gy);
        begin
            while (!in_ready) @(negedge clk);
            in_x = x; in_y = y; in_w = w; in_gx = gx; in_gy = gy;
            in_valid = 1'b1;
            @(negedge clk);
            in_valid = 1'b0;
            @(negedge clk);
        end
    endtask

    //----------------------------------------------------------------------
    // 跑一个窗口：start → 喂 n 样本 → 收 5 项比对
    //----------------------------------------------------------------------
    task automatic run_window(input integer base,
                              output integer next_base,
                              output integer wpass);
        integer  r, n, k, off;
        reg [63:0] rbits, ex_a, ex_b, ex_c, ex_bx, ex_by;
        reg [63:0] got_a, got_b, got_c, got_bx, got_by;
        begin
            rbits = rd64(base);
            r = dbl2int(rbits);
            n = (2 * r + 1) * (2 * r + 1);
            $display("[TB] window r=%0d n=%0d", r, n);

            n_win = n[15:0];
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;

            off = base + 8;
            for (k = 0; k < n; k = k + 1) begin
                feed_sample(rd64(off),     rd64(off + 8),
                            rd64(off + 16), rd64(off + 24), rd64(off + 32));
                off = off + 40;
            end
            ex_a  = rd64(off);
            ex_b  = rd64(off + 8);
            ex_c  = rd64(off + 16);
            ex_bx = rd64(off + 24);
            ex_by = rd64(off + 32);

            // 收输出（out_ready 恒 1；out_valid 拍沿采样）
            while (!(out_valid && out_ready)) @(negedge clk);
            got_a  = out_a;
            got_b  = out_b;
            got_c  = out_c;
            got_bx = out_bx;
            got_by = out_by;
            while (!done) @(negedge clk);

            wpass = 0;
            if (got_a !== ex_a || got_b !== ex_b || got_c !== ex_c ||
                got_bx !== ex_bx || got_by !== ex_by) begin
                $display("[FAIL] r=%0d", r);
                $display("  a  rtl=%016h exp=%016h", got_a,  ex_a);
                $display("  b  rtl=%016h exp=%016h", got_b,  ex_b);
                $display("  c  rtl=%016h exp=%016h", got_c,  ex_c);
                $display("  bx rtl=%016h exp=%016h", got_bx, ex_bx);
                $display("  by rtl=%016h exp=%016h", got_by, ex_by);
            end else begin
                wpass = 1;
                $display("[PASS] r=%0d a=%016h b=%016h c=%016h bx=%016h by=%016h",
                         r, got_a, got_b, got_c, got_bx, got_by);
            end
            next_base = off + 40;
        end
    endtask

    //----------------------------------------------------------------------
    // 主流程
    //----------------------------------------------------------------------
    integer pass_arr[0:7];
    integer npass, i, off, wpass;
    string  vec;

    initial begin
        vec = "../tests/build/vectors/m5_acc.bin";
        if (!$value$plusargs("VEC=%s", vec))
            vec = "../tests/build/vectors/m5_acc.bin";
        fd = $fopen(vec, "rb");
        if (fd == 0) begin $display("[FATAL] cannot open %s", vec); $finish; end
        code = $fread(fbuf, fd);
        $fclose(fd);
        N_win = rd64(0);
        $display("[TB] %s N_win=%0d bytes=%0d", vec, N_win, code);

        rst_n = 1'b0;
        start = 1'b0;
        in_valid = 1'b0;
        out_ready = 1'b1;
        n_win = 16'd0;
        in_x = 64'd0; in_y = 64'd0; in_w = 64'd0; in_gx = 64'd0; in_gy = 64'd0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        npass = 0;
        off = 8;
        for (i = 0; i < N_win; i = i + 1) begin
            run_window(off, off, wpass);
            pass_arr[i] = wpass;
            npass = npass + wpass;
        end

        $display("======================================");
        if (npass == N_win && off == code)
            $display("TB RESULT: ALL PASSED (%0d/%0d)", npass, N_win);
        else
            $display("TB RESULT: SOME FAILED (%0d/%0d, off=%0d bytes=%0d)", npass, N_win, off, code);
        $finish;
    end

    // 看门狗
    initial begin
        #200_000_000;
        $display("[TB][FATAL] watchdog timeout");
        $finish;
    end

endmodule
