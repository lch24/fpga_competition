`timescale 1ns / 1ps
//==============================================================================
// tb_gray.sv — gray_scan 位级对拍（M6.1）
//------------------------------------------------------------------------------
// 向量：../../data/image/m6_gray_<scene>.bin（tests/rtl/export_m6.cpp 生成）
//   格式（小端）：u32 W, u32 H, u32 C(=3/1)，
//                然后 W*H*C 字节输入（C=3 时按 B,G,R 顺序），再 W*H 字节期望灰度。
// 场景：board5x8 / big / copy / s2x2 / rand33x17 / rand127x63 / rand1x1 / rand5x3
// 流程：每场景 start 脉冲（置 cfg_w/cfg_h/cfg_c1）→ 按握手流式驱动 in_byte
//       （输入侧每 8 拍停 1 拍源间隙）→ 输出侧每 8 拍停 1 拍周期性背压
//       （覆盖 out_ready 反驱）→ 逐字节比对 out_gray 与期望。
// 断言：消费字节数 == W*H*C，输出像素数 == W*H，逐字节 == 期望。
// 超时保护：300ms 仿真时间兜底 $finish。
//==============================================================================
module tb_gray;

    reg clk = 1'b0;
    always #5 clk = ~clk;                 // 10ns 周期

    reg rst_n;

    //--------------------------------------------------------------------
    // DUT 例化
    //--------------------------------------------------------------------
    reg        start;
    reg  [10:0] cfg_w, cfg_h;
    reg        cfg_c1;
    wire       busy, done;
    reg        in_valid;
    wire       in_ready;
    reg  [7:0] in_byte;
    wire       out_valid;
    wire       out_ready;
    wire [7:0] out_gray;

    gray_scan #(
        .MAX_W (2048),
        .MAX_H (2048)
    ) u_dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (start),
        .cfg_w    (cfg_w),
        .cfg_h    (cfg_h),
        .cfg_c1   (cfg_c1),
        .busy     (busy),
        .done     (done),
        .in_valid (in_valid),
        .in_ready (in_ready),
        .in_byte  (in_byte),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_gray (out_gray)
    );

    // 输出背压：每 8 拍停 1 拍
    reg [3:0] bp_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) bp_cnt <= 4'd0;
        else        bp_cnt <= bp_cnt + 4'd1;
    end
    assign out_ready = (bp_cnt[2:0] == 3'd0) ? 1'b0 : 1'b1;

    //--------------------------------------------------------------------
    // 向量读取（小端 u32）+ 场景表
    //--------------------------------------------------------------------
    localparam integer FBUF_SZ = 4 * 1024 * 1024;   // big 场景文件约 3.7MB
    reg [7:0] fbuf [0:FBUF_SZ-1];
    integer fd, code;

    function automatic [31:0] rd32(input integer base);
        rd32 = {fbuf[base+3], fbuf[base+2], fbuf[base+1], fbuf[base+0]};
    endfunction

    string scene_names[0:7] = '{
        "board5x8", "big", "copy", "s2x2",
        "rand33x17", "rand127x63", "rand1x1", "rand5x3"
    };

    integer W, H, C;
    integer total_bytes, total_pix;
    integer si;          // 输入字节游标（主块）
    integer oi, err;     // 输出像素 / 错误（比对块独占写）
    integer oi_prev, err_prev, got, e;
    integer cycle_cnt;
    integer all_pass = 1;
    reg [3:0] cur;
    reg scene_active;
    string vec_dir = "../../data/image/";

    //--------------------------------------------------------------------
    // 输出比对（posedge Active 区读 = 模块采样时刻值，与握手一致）
    // 期望索引用场景内序号 (oi - oi_prev)，避免跨场景累计错位
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst_n && scene_active && out_valid && out_ready) begin
            if ((oi - oi_prev) >= total_pix ||
                out_gray !== fbuf[12 + total_bytes + (oi - oi_prev)]) begin
                err = err + 1;
                if (err - err_prev <= 10)
                    $display("[FAIL] %s out[%0d] rtl=0x%02x exp=0x%02x",
                             scene_names[cur], oi - oi_prev, out_gray,
                             fbuf[12 + total_bytes + (oi - oi_prev)]);
            end
            oi = oi + 1;
        end
    end

    //--------------------------------------------------------------------
    // 全局超时保护
    //--------------------------------------------------------------------
    initial begin
        #300_000_000;    // 300ms 仿真时间
        $display("[FATAL] GLOBAL TIMEOUT at %0t", $time);
        $finish;
    end

    //--------------------------------------------------------------------
    // 主流程：复位 → 逐场景 start + 输入驱动 + 完成检查
    //--------------------------------------------------------------------
    initial begin
        if (!$value$plusargs("VECDIR=%s", vec_dir))
            vec_dir = "../../data/image/";

        rst_n = 1'b0;
        start = 1'b0; in_valid = 1'b0; in_byte = 8'd0;
        cfg_w = 11'd0; cfg_h = 11'd0; cfg_c1 = 1'b0;
        scene_active = 1'b0; cur = 4'd0;
        oi = 0; err = 0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        for (int s = 0; s < 8; s = s + 1) begin
            cur = s;
            fd = $fopen({vec_dir, "m6_gray_", scene_names[s], ".bin"}, "rb");
            if (fd == 0) begin
                $display("[FATAL] cannot open %s", {vec_dir, "m6_gray_", scene_names[s], ".bin"});
                $finish;
            end
            code = $fread(fbuf, fd); $fclose(fd);
            W = rd32(0); H = rd32(4); C = rd32(8);
            total_bytes = W * H * C;
            total_pix   = W * H;
            si = 0;
            oi_prev = oi;      // 记录本场景前累计（比对块独占写 oi/err）
            err_prev = err;
            scene_active = 1'b1;
            $display("[TB] %s: %0dx%0d C=%0d total_bytes=%0d total_pix=%0d",
                     scene_names[s], W, H, C, total_bytes, total_pix);

            // start 脉冲（锁存配置）
            cfg_w = W[10:0]; cfg_h = H[10:0]; cfg_c1 = (C == 1);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            wait (busy === 1'b1);

            // 输入驱动：握手（源 valid 保持直到接受）+ 输入侧每 8 拍停 1 拍
            cycle_cnt = 0;
            in_valid = 1'b0;
            while (si < total_bytes) begin
                if (in_valid) begin
                    // 待发数据：等 posedge 确认握手（Active 区读 = 模块采样值）
                    @(posedge clk);
                    if (in_ready) begin
                        si = si + 1;
                        in_valid = 1'b0;
                    end
                    @(negedge clk);
                end else if (cycle_cnt % 8 == 7) begin
                    cycle_cnt = cycle_cnt + 1;   // 源间隙停 1 拍
                    @(negedge clk);
                end else if (in_ready) begin
                    in_byte = fbuf[12 + si];
                    in_valid = 1'b1;
                    cycle_cnt = cycle_cnt + 1;
                    // 直接回循环顶检查握手（不在 negedge 挂起，避免双送）
                end else begin
                    cycle_cnt = cycle_cnt + 1;   // 模块反压等待
                    @(negedge clk);
                end
            end
            in_valid = 1'b0;

            // 等流水排空 → done
            wait (done === 1'b1);
            repeat (3) @(negedge clk);

            // 场景统计
            got = oi - oi_prev;
            e   = err - err_prev;
            $display("[%s] bytes=%0d/%0d out=%0d/%0d err=%0d -> %s",
                     scene_names[s], si, total_bytes, got, total_pix, e,
                     (si == total_bytes && got == total_pix && e == 0) ? "PASS" : "FAIL");
            if (si != total_bytes || got != total_pix || e != 0)
                all_pass = 0;
            scene_active = 1'b0;
        end

        $display("======================================");
        if (all_pass)
            $display("TB RESULT: ALL GRAY TESTS PASSED");
        else
            $display("TB RESULT: FAILED");
        $finish;
    end

endmodule
