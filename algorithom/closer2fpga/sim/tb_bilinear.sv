`timescale 1ns / 1ps
//==============================================================================
// tb_bilinear.sv — bilinear_core 位级对拍（M5）
//------------------------------------------------------------------------------
// 向量：../tests/build/vectors/m5_bilinear.bin
//   格式（小端）：u32 N + N×{p00,p10,p01,p11,dx,dy,期望 out}（每例 7 个 u32 内联，fp32 位模式）
// 流程：逐例喂入（in_valid/in_ready 握手）→ 收 out_r 流与期望逐位比对。
// 输出带周期性背压（每 8 拍停 1 拍），覆盖 out_ready 反驱。
//==============================================================================
module tb_bilinear;

    reg clk = 1'b0;
    always #5 clk = ~clk;                 // 10ns 周期

    reg rst_n;

    //--------------------------------------------------------------------
    // DUT 例化
    //--------------------------------------------------------------------
    reg        in_valid;
    wire       in_ready;
    reg  [31:0] in_p00, in_p10, in_p01, in_p11, in_dx, in_dy;
    wire       out_valid;
    reg        out_ready;
    wire [31:0] out_r;

    bilinear_core u_dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (in_valid),
        .in_ready (in_ready),
        .in_p00   (in_p00),
        .in_p10   (in_p10),
        .in_p01   (in_p01),
        .in_p11   (in_p11),
        .in_dx    (in_dx),
        .in_dy    (in_dy),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_r    (out_r)
    );

    // 背压：每 8 拍停 1 拍
    reg [3:0] bp_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) bp_cnt <= 4'd0;
        else        bp_cnt <= bp_cnt + 4'd1;
    end
    assign out_ready = (bp_cnt[2:0] == 3'd0) ? 1'b0 : 1'b1;

    //--------------------------------------------------------------------
    // 向量读取（小端 u32）
    //--------------------------------------------------------------------
    reg [7:0] fbuf[0:1048575];
    integer fd, code, N;

    function automatic [31:0] rd32(input integer base);
        rd32 = {fbuf[base+3], fbuf[base+2], fbuf[base+1], fbuf[base+0]};
    endfunction

    reg [31:0] src_p00[0:4095], src_p10[0:4095], src_p01[0:4095];
    reg [31:0] src_p11[0:4095], src_dx[0:4095], src_dy[0:4095];
    reg [31:0] exp_out[0:4095];

    //--------------------------------------------------------------------
    // 输出比对
    //--------------------------------------------------------------------
    integer oi = 0, err = 0;
    always @(posedge clk) begin
        if (rst_n && out_valid && out_ready) begin
            if (oi >= N || out_r !== exp_out[oi]) begin
                err = err + 1;
                if (err <= 10)
                    $display("[FAIL] bilinear #%0d rtl=0x%08x exp=0x%08x", oi, out_r, exp_out[oi]);
            end
            oi = oi + 1;
        end
    end

    //--------------------------------------------------------------------
    // 驱动（等待向量加载完成 + 复位释放）
    //--------------------------------------------------------------------
    integer pi = 0;
    reg loaded_done = 1'b0;
    initial begin
        rst_n = 1'b0;
        in_valid = 1'b0;
        in_p00 = 32'd0; in_p10 = 32'd0; in_p01 = 32'd0; in_p11 = 32'd0;
        in_dx  = 32'd0; in_dy  = 32'd0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);
    end
    initial begin
        wait (rst_n && loaded_done);
        while (pi < N) begin
            if (in_ready) begin
                in_p00 <= src_p00[pi]; in_p10 <= src_p10[pi];
                in_p01 <= src_p01[pi]; in_p11 <= src_p11[pi];
                in_dx  <= src_dx[pi];  in_dy  <= src_dy[pi];
                in_valid <= 1'b1;
                @(negedge clk);         // DUT 在 posedge 接受（in_ready=1 拍）
                pi = pi + 1;
                in_valid <= 1'b0;
            end else begin
                @(negedge clk);         // DUT 忙：in_valid=0 等待
            end
        end
        in_valid <= 1'b0;
        $display("[TB] bilinear fed %0d", N);
    end

    //--------------------------------------------------------------------
    // 加载向量
    //--------------------------------------------------------------------
    initial begin
        string vec = "../tests/build/vectors/m5_bilinear.bin";
        if (!$value$plusargs("VEC=%s", vec))
            vec = "../tests/build/vectors/m5_bilinear.bin";
        fd = $fopen(vec, "rb");
        if (fd == 0) begin $display("[FATAL] cannot open %s", vec); $finish; end
        code = $fread(fbuf, fd); $fclose(fd);
        N = rd32(0);
        for (integer i = 0; i < N; ++i) begin
            src_p00[i] = rd32(4 + i*28);
            src_p10[i] = rd32(8 + i*28);
            src_p01[i] = rd32(12 + i*28);
            src_p11[i] = rd32(16 + i*28);
            src_dx[i]  = rd32(20 + i*28);
            src_dy[i]  = rd32(24 + i*28);
            exp_out[i] = rd32(28 + i*28);
        end
        $display("[TB] loaded %s: N=%0d", vec, N);
        loaded_done = 1'b1;
    end

    //--------------------------------------------------------------------
    // 完成判定
    //--------------------------------------------------------------------
    initial begin
        while (oi < N) begin
            @(posedge clk);
            if ($time > 50_000_000) begin
                $display("[FATAL] timeout got %0d/%0d", oi, N);
                $finish;
            end
        end
        repeat (20) @(negedge clk);
        $display("======================================");
        $display("BILINEAR TB: err=%0d got=%0d/%0d", err, oi, N);
        if (err == 0 && oi == N)
            $display("TB RESULT: ALL BILINEAR TESTS PASSED (%0d/%0d)", oi, N);
        else
            $display("TB RESULT: FAILED");
        $finish;
    end

endmodule