`timescale 1ns / 1ps
//==============================================================================
// tb_arith.sv — min_eigen_core 单元对拍（与 test0_eigen.bin 逐位比对，4034 笔）
//------------------------------------------------------------------------------
// 向量文件：../tests/build/vectors/test0_eigen.bin（小端）
//   格式：u32 N，随后 N×4 个 u32 {a_bits, b_bits, c_bits, resp_bits}。
//   resp_bits 为官方 kernels::min_eigenvalue 期望输出位（fp32 位模式）。
//
// 激励：逐笔握手送入 {a,b,c}（in_valid/in_ready）；out_ready 施加 80% 可用率以
//   验证弹性流水在背压下不丢、保序；输出按到达顺序与 (idx*4+3) 逐位比对。
// 结束：fed==N 且 resp_idx==N 时报 ALL ARITHMETIC TESTS PASSED (4034/4034)。
//==============================================================================
module tb_arith;

    reg        clk = 1'b0;
    reg        rst_n = 1'b0;
    reg        in_valid = 1'b0;
    wire       in_ready;
    reg  [31:0] in_a, in_b, in_c;
    wire       out_valid;
    reg        out_ready;
    wire [31:0] out_resp;

    min_eigen_core dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (in_valid),
        .in_ready (in_ready),
        .in_a     (in_a),
        .in_b     (in_b),
        .in_c     (in_c),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_resp (out_resp)
    );

    //--------------------------------------------------------------------------------
    // 向量存储
    //--------------------------------------------------------------------------------
    reg [7:0]  memb [0:131071];            // 4034*4*4+4 = 64548 字节
    reg [31:0] qA[0:8191], qB[0:8191], qC[0:8191], qR[0:8191];
    integer    N = 0;
    integer    nbytes = 0;
    integer    f;

    // 小端读 4 字节并拼成 u32
    function automatic [31:0] rd_u32;
        input integer off;
        begin
            rd_u32 = 32'd0;
            rd_u32[31:24] = memb[off+3];
            rd_u32[23:16] = memb[off+2];
            rd_u32[15:8]  = memb[off+1];
            rd_u32[7:0]   = memb[off+0];
        end
    endfunction

    integer i;

    // 背压/周期与比对
    integer resp_idx = 0;
    integer fed      = 0;
    integer err      = 0;
    reg  [31:0] cyc = 0;
    assign out_ready = (cyc % 5) != 0;     // 80% 可用，制造周期性背压

    always #5 clk = ~clk;

    always @(posedge clk) begin
        cyc <= cyc + 1;
        if (out_valid && out_ready) begin
            if (resp_idx < N) begin
                if (out_resp !== qR[resp_idx]) begin
                    $display("MISMATCH idx=%0d got=0x%08x exp=0x%08x", resp_idx, out_resp, qR[resp_idx]);
                    err = err + 1;
                end else if (resp_idx < 4) begin
                    $display("  [ok] idx=%0d got=0x%08x", resp_idx, out_resp);
                end
                resp_idx = resp_idx + 1;
            end else begin
                $display("EXTRA OUTPUT after N=%0d", N);
                err = err + 1;
            end
        end
    end

    integer max_cyc;
    initial begin
        f = $fopen("../tests/build/vectors/test0_eigen.bin", "rb");
        if (f == 0) begin
            $display("FATAL: cannot open vector file");
            $finish;
        end
        nbytes = $fread(memb, f);
        $fclose(f);
        N = rd_u32(0);
        $display("vector file bytes=%0d N=%0d", nbytes, N);
        for (i = 0; i < N && i < 8192; i = i + 1) begin
            qA[i] = rd_u32(4 + 16*i + 0);
            qB[i] = rd_u32(4 + 16*i + 4);
            qC[i] = rd_u32(4 + 16*i + 8);
            qR[i] = rd_u32(4 + 16*i + 12);
        end

        // 复位
        rst_n = 1'b0;
        repeat (6) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // 逐笔握手送入：每个 posedge 若 in_valid&&in_ready 才视为成交并前移指针，
        // 保证一拍一 token、不遗漏（此前 wait(in_ready);@(posedge) 会在 ready 于
        // wait 返回后、边沿前下降时跳过 token）。
        in_valid = 1'b1;
        fed = 0;
        while (fed < N) begin
            in_a = qA[fed];
            in_b = qB[fed];
            in_c = qC[fed];
            @(posedge clk);
            if (in_valid && in_ready) fed = fed + 1;
        end
        in_valid = 1'b0;

        // 等待排空（带超时保护）
        max_cyc = 0;
        while (resp_idx < N) begin
            @(posedge clk);
            max_cyc = max_cyc + 1;
            if (max_cyc > 2000000) begin
                $display("TIMEOUT draining out (resp_idx=%0d/%0d)", resp_idx, N);
                err = err + 1;
                break;
            end
        end

        if (err == 0 && resp_idx == N && N == 4034)
            $display("ALL ARITHMETIC TESTS PASSED (4034/4034)");
        else begin
            $display("ARITHMETIC TEST FAILED: err=%0d resp_idx=%0d N=%0d", err, resp_idx, N);
        end
        $finish;
    end

endmodule