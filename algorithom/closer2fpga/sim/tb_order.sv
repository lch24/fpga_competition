`timescale 1ns / 1ps
//==============================================================================
// tb_order.sv — M4 grid_order_ctrl 全链位级对拍
//------------------------------------------------------------------------------
// 向量：
//   m4_board5x8_inner.bin = u32 N + N×{x,y} fp32（organize_grid 输入 80 点）
//   m4_board5x8_grid.bin  = u32 ok + u32 N + N×{x,y} fp32（期望输出 40 点）
// 流程：复位 → 喂 pts 流（80 点，pts_valid/pts_ready 握手，末拍 pts_done）→
//   等 done → 校验 status/out_grid_ok/out_total → 收 40 点 out 流逐点位级比对。
// 要求：40/40 全过（含原点规范化后的行列序）。
//==============================================================================
module tb_order;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg        rst_n;
    reg        start;
    wire       busy, done;
    wire [1:0] status;
    reg        pts_valid, pts_done;
    wire       pts_ready;
    reg  [31:0] pts_x, pts_y;
    wire       out_valid;
    reg        out_ready;
    wire [31:0] out_x, out_y;
    wire [15:0] out_total;
    wire        out_grid_ok;

    grid_order_ctrl #(.ROWS(5), .COLS(8), .N_ADDR_W(8), .GAP_ADDR_W(8)) u_dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .busy(busy), .done(done), .status(status),
        .pts_valid(pts_valid), .pts_ready(pts_ready),
        .pts_x(pts_x), .pts_y(pts_y), .pts_done(pts_done),
        .out_valid(out_valid), .out_ready(out_ready),
        .out_x(out_x), .out_y(out_y),
        .out_total(out_total), .out_grid_ok(out_grid_ok)
    );

    //--------------------------------------------------------------------
    reg [7:0] fbuf[0:1048575];
    integer  N_in, N_out, ok_exp, k, fd, code, fail_cnt;
    reg [31:0] inner_x[0:255], inner_y[0:255];
    reg [31:0] exp_x[0:63], exp_y[0:63];

    function automatic [31:0] rd32(input integer base);
        rd32 = {fbuf[base+3], fbuf[base+2], fbuf[base+1], fbuf[base+0]};
    endfunction

    task read_file(input string path);
        begin
            fd = $fopen(path, "rb");
            if (fd == 0) begin $display("[TB][FATAL] cannot open %s", path); $finish; end
            code = $fread(fbuf, fd);
            $fclose(fd);
            $display("[TB] load %s (%0d bytes)", path, code);
        end
    endtask

    initial begin
        // 初始化
        rst_n = 1'b0; start = 1'b0;
        pts_valid = 1'b0; pts_done = 1'b0;
        pts_x = 32'd0; pts_y = 32'd0;
        out_ready = 1'b1;
        fail_cnt = 0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // 读输入向量（M5 全链 inner：subpixel 后经 ring 的输出）
        read_file("../tests/build/vectors/m5_board5x8_inner.bin");
        N_in = rd32(0);
        for (k = 0; k < N_in; k = k + 1) begin
            inner_x[k] = rd32(4 + k * 8);
            inner_y[k] = rd32(4 + k * 8 + 4);
        end
        $display("[TB] inner N=%0d", N_in);

        // 读期望输出（M5 全链 grid：organize_grid 40 点）
        read_file("../tests/build/vectors/m5_board5x8_grid.bin");
        ok_exp = rd32(0);
        N_out  = rd32(4);
        for (k = 0; k < N_out; k = k + 1) begin
            exp_x[k] = rd32(8 + k * 8);
            exp_y[k] = rd32(8 + k * 8 + 4);
        end
        $display("[TB] grid ok=%0d N=%0d", ok_exp, N_out);
        if (ok_exp != 1) begin
            $display("[TB][FATAL] expected ok=1"); $finish;
        end

        // 启动
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        // 喂 pts 流（末拍 pts_done 与 pts_valid 同拍）
        for (k = 0; k < N_in; k = k + 1) begin
            pts_valid = 1'b1;
            pts_x = inner_x[k];
            pts_y = inner_y[k];
            if (k == N_in - 1) pts_done = 1'b1;
            @(negedge clk);
        end
        pts_valid = 1'b0;
        pts_done  = 1'b0;

        // 等完成
        while (!done) @(negedge clk);
        $display("[TB] done status=%02b grid_ok=%0d total=%0d", status, out_grid_ok, out_total);
        if (status != 2'b01 || !out_grid_ok) begin
            $display("[TB][FAIL] status/out_grid_ok mismatch");
            fail_cnt = fail_cnt + 1;
        end

        // 收 40 点流逐点比对
        for (k = 0; k < N_out; k = k + 1) begin
            while (!out_valid) @(negedge clk);
            if (out_x !== exp_x[k] || out_y !== exp_y[k]) begin
                fail_cnt = fail_cnt + 1;
                $display("[TB][FAIL][%0d] got %08x,%08x exp %08x,%08x", k,
                         out_x, out_y, exp_x[k], exp_y[k]);
            end
            @(negedge clk);
        end

        $display("==============================================");
        if (fail_cnt == 0)
            $display("TB RESULT: order ALL PASSED (40/40 bit-exact)");
        else
            $display("TB RESULT: order FAIL %0d/40", fail_cnt);
        $finish;
    end

    // 看门狗（2 秒仿真时间上限；vsim 时间刻度 1ps → 2e12 ps）
    initial begin
        #2000000000;
        $display("[TB][FATAL] watchdog timeout");
        $finish;
    end

endmodule
