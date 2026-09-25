`timescale 1ns / 1ps
//==============================================================================
// tb_log.sv — M4 fp32_log 位级对拍
//------------------------------------------------------------------------------
// 向量：tests/build/vectors/m4_log.bin = u32 N + N×{x, exp}（fp32 位模式）。
// 流程：逐笔握手喂 fp32_log → 比对 out_r 与 exp。要求 N=4010 全过。
//==============================================================================
module tb_log;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg        rst_n;
    reg        in_valid;
    wire       in_ready;
    reg  [31:0] in_x;
    wire       out_valid;
    reg        out_ready;
    wire [31:0] out_r;

    fp32_log u_dut (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_ready(in_ready),
        .in_x(in_x),
        .out_valid(out_valid), .out_ready(out_ready),
        .out_r(out_r)
    );

    //--------------------------------------------------------------------
    reg [7:0] fbuf[0:1048575];
    integer   N_in, k, fail_cnt, match_cnt, fd, code;

    function automatic [31:0] rd32(input integer base);
        rd32 = {fbuf[base+3], fbuf[base+2], fbuf[base+1], fbuf[base+0]};
    endfunction

    initial begin
        in_valid  = 1'b0;
        in_x      = 32'd0;
        out_ready = 1'b1;
        fail_cnt  = 0;
        match_cnt = 0;

        rst_n = 1'b0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        fd = $fopen("../tests/build/vectors/m4_log.bin", "rb");
        if (fd == 0) begin $display("[TB][FATAL] cannot open m4_log.bin"); $finish; end
        code = $fread(fbuf, fd);
        $fclose(fd);
        N_in = rd32(0);
        $display("[TB] m4_log.bin %0d bytes, N=%0d", code, N_in);

        for (k = 0; k < N_in; k = k + 1) begin
            in_x     = rd32(4 + k * 8);
            in_valid = 1'b1;
            @(negedge clk);
            while (!(in_valid && in_ready)) @(negedge clk);   // 接受拍
            in_valid = 1'b0;
            while (!out_valid) @(negedge clk);                // 等结果
            if (out_r == rd32(4 + k * 8 + 4))
                match_cnt = match_cnt + 1;
            else begin
                fail_cnt = fail_cnt + 1;
                if (fail_cnt <= 10)
                    $display("[TB][FAIL] log[%0d] x=%08x got=%08x exp=%08x",
                             k, rd32(4 + k * 8), out_r, rd32(4 + k * 8 + 4));
            end
            @(negedge clk);
        end

        $display("==============================================");
        if (fail_cnt == 0 && match_cnt == N_in)
            $display("TB RESULT: log ALL PASSED (%0d/%0d)", match_cnt, N_in);
        else
            $display("TB RESULT: log FAIL match=%0d fail=%0d / %0d", match_cnt, fail_cnt, N_in);
        $finish;
    end

    // 看门狗
    initial begin
        #500_000_000;
        $display("[TB][FATAL] watchdog timeout");
        $finish;
    end

endmodule
