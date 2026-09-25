`timescale 1ns / 1ps
//==============================================================================
// tb_tensor.sv — tensor_solve 位级对拍（M5）
//------------------------------------------------------------------------------
// 向量：../tests/build/vectors/m5_tensor.bin
//   格式（小端 u64）：u64 N + N×{a,b,c,bx,by,dx,dy,ok}（每例 8 个 u64 内联，
//   a..by 为 double 位模式，dx/dy 为 double 位模式，ok 为 0/1）
// 流程：逐例 start → 等 done → 采样 out_ok/out_dx/out_dy 与期望比对。
//   ok=0 只比对 out_ok（dx/dy 可任意）；ok=1 全位级比对。
//==============================================================================
module tb_tensor;

    reg clk = 1'b0;
    always #5 clk = ~clk;                 // 10ns 周期

    reg rst_n;

    //--------------------------------------------------------------------
    // DUT 例化
    //--------------------------------------------------------------------
    reg        start;
    wire       busy, done;
    reg  [63:0] in_a, in_b, in_c, in_bx, in_by;
    wire       out_ok;
    wire [63:0] out_dx, out_dy;

    tensor_solve u_dut (
        .clk   (clk),
        .rst_n (rst_n),
        .start (start),
        .busy  (busy),
        .done  (done),
        .in_a  (in_a),
        .in_b  (in_b),
        .in_c  (in_c),
        .in_bx (in_bx),
        .in_by (in_by),
        .out_ok(out_ok),
        .out_dx(out_dx),
        .out_dy(out_dy)
    );

    //--------------------------------------------------------------------
    // 向量读取（小端 u64）
    //--------------------------------------------------------------------
    reg [7:0] fbuf[0:1048575];
    integer fd, code, N;

    function automatic [63:0] rd64(input integer base);
        rd64 = {fbuf[base+7], fbuf[base+6], fbuf[base+5], fbuf[base+4],
                fbuf[base+3], fbuf[base+2], fbuf[base+1], fbuf[base+0]};
    endfunction

    reg [63:0] src_a[0:4095], src_b[0:4095], src_c[0:4095], src_bx[0:4095], src_by[0:4095];
    reg [63:0] exp_dx[0:4095], exp_dy[0:4095], exp_ok[0:4095];

    integer err = 0, done_cnt = 0;

    initial begin
        string vec = "../tests/build/vectors/m5_tensor.bin";
        if (!$value$plusargs("VEC=%s", vec))
            vec = "../tests/build/vectors/m5_tensor.bin";
        fd = $fopen(vec, "rb");
        if (fd == 0) begin $display("[FATAL] cannot open %s", vec); $finish; end
        code = $fread(fbuf, fd); $fclose(fd);
        N = rd64(0);
        for (integer i = 0; i < N; ++i) begin
            src_a[i]  = rd64(8 + i*64);
            src_b[i]  = rd64(16 + i*64);
            src_c[i]  = rd64(24 + i*64);
            src_bx[i] = rd64(32 + i*64);
            src_by[i] = rd64(40 + i*64);
            exp_dx[i] = rd64(48 + i*64);
            exp_dy[i] = rd64(56 + i*64);
            exp_ok[i] = rd64(64 + i*64);
        end
        $display("[TB] loaded %s: N=%0d", vec, N);
    end

    //--------------------------------------------------------------------
    // 主流程：逐例 start → 等 done → 采样比对
    //--------------------------------------------------------------------
    initial begin
        rst_n = 1'b0;
        start = 1'b0;
        in_a = 64'd0; in_b = 64'd0; in_c = 64'd0; in_bx = 64'd0; in_by = 64'd0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        for (integer i = 0; i < N; ++i) begin
            in_a <= src_a[i]; in_b <= src_b[i]; in_c <= src_c[i];
            in_bx <= src_bx[i]; in_by <= src_by[i];
            start <= 1'b1;
            @(negedge clk);
            start <= 1'b0;
            while (!done) begin
                @(negedge clk);
                if ($time > 50_000_000) begin
                    $display("[FATAL] timeout at case %0d/%0d", i, N);
                    $finish;
                end
            end
            // done=1（负沿采样）：out_ok/out_dx/out_dy 已稳定
            if (exp_ok[i] == 64'd0) begin
                if (out_ok !== 1'b0) begin
                    err = err + 1;
                    if (err <= 10) $display("[FAIL] tensor #%0d ok: got %0d exp 0", i, out_ok);
                end
            end else begin
                if (out_ok !== 1'b1) begin
                    err = err + 1;
                    if (err <= 10) $display("[FAIL] tensor #%0d ok: got %0d exp 1", i, out_ok);
                end else if (out_dx !== exp_dx[i] || out_dy !== exp_dy[i]) begin
                    err = err + 1;
                    if (err <= 10) $display("[FAIL] tensor #%0d dx=0x%016x(exp 0x%016x) dy=0x%016x(exp 0x%016x)",
                                             i, out_dx, exp_dx[i], out_dy, exp_dy[i]);
                end
            end
            done_cnt = done_cnt + 1;
        end

        $display("======================================");
        $display("TENSOR TB: err=%0d done=%0d/%0d", err, done_cnt, N);
        if (err == 0 && done_cnt == N)
            $display("TB RESULT: ALL TENSOR TESTS PASSED (%0d/%0d)", done_cnt, N);
        else
            $display("TB RESULT: FAILED");
        $finish;
    end

endmodule