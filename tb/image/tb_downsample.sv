`timescale 1ns / 1ps
//==============================================================================
// tb_downsample.sv — downsample2x 位级对拍（M6.1）
//------------------------------------------------------------------------------
// 向量：tests/build/vectors/m6_down_<scene>.bin（主控生成，只读，禁止修改）
//   格式（小端）：u32 W, u32 H, W*H 字节源灰度, (W/2)*(H/2) 字节期望缩图
// 场景（TB 内循环，每场景单独 fopen 读入，避免大图超内存）：
//   board5x8(96x272) big(1280x720) s64(64x64) odd33x17(33x17)
//   s127x63(127x63) s5x3(5x3) s2x2(2x2) s3x1(3x1) s1x4(1x4)
//   后两个为 0 输出像素场景（s3x1: H/2=0；s1x4: W/2=0），必须验证
//   "消费全部输入、0 输出、done 正常"。
// 流程：start 脉冲（置 cfg_w/cfg_h）→ 光栅序流式握手喂入 in_gray →
//   输出侧周期性背压（每 8 拍停 1 拍，覆盖 out_ready 反驱）→ 逐字节比对
//   out_gray 与期望，统计消费输入数/产出输出数并断言 == W*H / == (W/2)*(H/2)。
// 每场景打一行 `[scene] WxH -> w2xh2 got=... exp=...`；
// 最终 TB RESULT: ALL DOWNSAMPLE TESTS PASSED / FAILED 并 $finish；
// 超时保护：喂入/done 等待超限打印 FATAL，另有全局看门狗。
//==============================================================================
module tb_downsample;

    reg clk = 1'b0;
    always #5 clk = ~clk;                 // 10ns 周期

    reg rst_n;

    //--------------------------------------------------------------------
    // DUT 例化（ADDR_W = $clog2(2048) = 11）
    //--------------------------------------------------------------------
    reg               start;
    reg [10:0]        cfg_w, cfg_h;
    wire              busy, done;
    reg               in_valid;
    wire              in_ready;
    reg  [7:0]        in_gray;
    wire              out_valid;
    reg               out_ready;
    wire [7:0]        out_gray;

    downsample2x u_dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (start),
        .cfg_w    (cfg_w),
        .cfg_h    (cfg_h),
        .busy     (busy),
        .done     (done),
        .in_valid (in_valid),
        .in_ready (in_ready),
        .in_gray  (in_gray),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_gray (out_gray)
    );

    // 背压：每 8 拍停 1 拍（覆盖 out_ready 反驱）
    reg [3:0] bp_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) bp_cnt <= 4'd0;
        else        bp_cnt <= bp_cnt + 4'd1;
    end
    assign out_ready = (bp_cnt[2:0] == 3'd0) ? 1'b0 : 1'b1;

    //--------------------------------------------------------------------
    // 向量读取（小端 u32/u8），单场景缓冲 2MB（big 场景约 1.38MB）
    //--------------------------------------------------------------------
    reg [7:0] fbuf[0:2097151];
    integer fd, code;

    function automatic [31:0] rd32(input integer base);
        rd32 = {fbuf[base+3], fbuf[base+2], fbuf[base+1], fbuf[base+0]};
    endfunction

    //--------------------------------------------------------------------
    // 场景上下文（每场景由驱动重置）
    //--------------------------------------------------------------------
    integer W, H, src_base, exp_base, n_in, n_out;
    string scene_name;
    integer oi = 0, err = 0, in_cnt = 0;

    // 输出比对 + 计数
    always @(posedge clk) begin
        if (rst_n && out_valid && out_ready) begin
            if (oi >= n_out || out_gray !== fbuf[exp_base + oi]) begin
                err = err + 1;
                if (err <= 10)
                    $display("[FAIL] %s out[%0d] rtl=%0d exp=%0d",
                             scene_name, oi, out_gray, fbuf[exp_base + oi]);
            end
            oi = oi + 1;
        end
    end

    //--------------------------------------------------------------------
    // 驱动：复位 → 逐场景 start → 流式喂入 → 等 done → 断言
    //--------------------------------------------------------------------
    string scenes[0:8];
    integer i, sc;
    initial begin
        scenes[0] = "board5x8";
        scenes[1] = "big";
        scenes[2] = "s64";
        scenes[3] = "odd33x17";
        scenes[4] = "s127x63";
        scenes[5] = "s5x3";
        scenes[6] = "s2x2";
        scenes[7] = "s3x1";
        scenes[8] = "s1x4";

        rst_n    = 1'b0;
        start    = 1'b0;
        in_valid = 1'b0;
        in_gray  = 8'd0;
        cfg_w    = 11'd0;
        cfg_h    = 11'd0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        for (sc = 0; sc < 9; ++sc) begin
            string vec;
            $sformat(vec, "../../data/image/m6_down_%s.bin", scenes[sc]);
            fd = $fopen(vec, "rb");
            if (fd == 0) begin $display("[FATAL] cannot open %s", vec); $finish; end
            code = $fread(fbuf, fd); $fclose(fd);

            W = rd32(0);
            H = rd32(4);
            src_base = 8;
            exp_base = 8 + W * H;
            n_in     = W * H;
            n_out    = (W / 2) * (H / 2);
            scene_name = scenes[sc];
            oi    = 0;
            in_cnt = 0;

            // 等 DUT 空闲后启动
            while (busy) @(negedge clk);
            cfg_w = W; cfg_h = H;
            in_valid = 1'b0;
            start = 1'b1; @(negedge clk);
            start = 1'b0;

            // 光栅序流式喂入 W*H 像素（in_valid/in_ready 握手）
            for (i = 0; i < n_in; ++i) begin
                while (!in_ready) begin
                    @(negedge clk);
                    if ($time > 300_000_000) begin
                        $display("[FATAL] %s feed timeout at pixel %0d/%0d",
                                 scene_name, i, n_in);
                        $finish;
                    end
                end
                in_gray  <= fbuf[src_base + i];
                in_valid <= 1'b1;
                @(negedge clk);
                in_valid <= 1'b0;
                in_cnt = in_cnt + 1;
            end
            in_valid = 1'b0;

            // 等 done（带超时）
            while (!done) begin
                @(posedge clk);
                if ($time > 300_000_000) begin
                    $display("[FATAL] %s done timeout consumed=%0d/%0d out=%0d/%0d",
                             scene_name, in_cnt, n_in, oi, n_out);
                    $finish;
                end
            end
            repeat (3) @(negedge clk);

            // 计数断言：消费 == W*H，产出 == (W/2)*(H/2)
            if (in_cnt != n_in) begin
                $display("[FAIL] %s consumed_in=%0d exp=%0d", scene_name, in_cnt, n_in);
                err = err + 1;
            end
            if (oi != n_out) begin
                $display("[FAIL] %s produced_out=%0d exp=%0d", scene_name, oi, n_out);
                err = err + 1;
            end
            $display("[%s] %0dx%0d -> %0dx%0d got=%0d exp=%0d",
                     scene_name, W, H, W/2, H/2, oi, n_out);
        end

        repeat (5) @(negedge clk);
        $display("======================================");
        if (err == 0)
            $display("TB RESULT: ALL DOWNSAMPLE TESTS PASSED");
        else
            $display("TB RESULT: FAILED (err=%0d)", err);
        $finish;
    end

    // 全局看门狗（正常情况下总仿真时间远小于 1s）
    initial begin
        #1_000_000_000;
        $display("[FATAL] global watchdog timeout");
        $finish;
    end

endmodule
