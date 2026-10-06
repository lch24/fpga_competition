`timescale 1ns / 1ps
//==============================================================================
// tb_pyramid.sv — pyramid_ctrl 位级对拍（M6.2）
//------------------------------------------------------------------------------
// 向量（主控生成，只读，禁止修改）：
//   m6_big_gray.bin      u8[1280*720] big 场景原始灰度（L0 预载）
//   m6_down_big.bin      u32 W,H + W*H 源 + (W/2)*(H/2) 期望缩图（L1 校验）
//   m5_board5x8_gray.bin u8[272*96] 小场景灰度（L0 预载，不应产生缩图）
// 场景（TB 内循环，每场景单独 fopen / 单独合成）：
//   big  ：1280x720 → level_count=2，L1{base=921600,w=640,h=360}，
//          L1 灰度逐字节 == m6_down_big.bin 期望段
//   small：272x96  → level_count=1，无任何 gray 写（wr_events==0）
//   deep ：2000x1200 合成棋盘（格 40px，180/40 交替）→ level_count=3，
//          L1{2400000,1000,600} L2{3000000,500,300}，
//          L1/L2 灰度用 C++ 同款四像素平均公式自算逐字节比对
//          （L1 期望独立自算自原始棋盘；L2 期望链式自算自 RTL 生成的 L1，
//           与 C++ 递归逐层一致）
// gray RAM：TB 内同步 RAM（registered 读：rd_en 的下一拍 rd_data 出数；
//           同步写：wr_en 拍写入）。背压覆盖：pyramid_ctrl 内部 downsample2x
//           气泡（in_ready=0）驱动读流暂停（gray_rd_en 间隙、rd_addr 保持），
//           输出侧每 8 拍停 1 拍反驱 out_ready——两类握手反驱均在 DUT 内部
//           被真实背压驱动。
// 每场景打统计行；最终 TB RESULT: ALL PYRAMID TESTS PASSED / FAILED 并 $finish；
// 超时保护：done 等待超时打印 FATAL，另有全局看门狗。
//==============================================================================
module tb_pyramid;

    reg clk = 1'b0;
    always #5 clk = ~clk;                 // 10ns 周期

    reg rst_n;

    //--------------------------------------------------------------------
    // DUT 例化（MAX_W=2560, MAX_H=1440, MAX_DEPTH=4, GRAY_ADDR_W=26）
    //--------------------------------------------------------------------
    reg               start;
    reg  [10:0]       cfg_w0, cfg_h0;
    reg  [25:0]       cfg_base0;
    wire              busy, done;
    wire              gray_rd_en;
    wire [25:0]       gray_rd_addr;
    reg  [7:0]        gray_rd_data;
    wire              gray_wr_en;
    wire [25:0]       gray_wr_addr;
    wire [7:0]        gray_wr_data;
    wire [2:0]        level_count;
    wire [25:0]       level_base [0:3];
    wire [10:0]       level_w    [0:3];
    wire [10:0]       level_h    [0:3];

    pyramid_ctrl #(
        .MAX_W      (2560),
        .MAX_H      (1440),
        .MAX_DEPTH  (4),
        .GRAY_ADDR_W(26)
    ) u_dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (start),
        .cfg_w0       (cfg_w0),
        .cfg_h0       (cfg_h0),
        .cfg_base0    (cfg_base0),
        .busy         (busy),
        .done         (done),
        .gray_rd_en   (gray_rd_en),
        .gray_rd_addr (gray_rd_addr),
        .gray_rd_data (gray_rd_data),
        .gray_wr_en   (gray_wr_en),
        .gray_wr_addr (gray_wr_addr),
        .gray_wr_data (gray_wr_data),
        .level_count  (level_count),
        .level_base   (level_base),
        .level_w      (level_w),
        .level_h      (level_h)
    );

    //--------------------------------------------------------------------
    // gray RAM：registered 读（rd_en 下一拍出数据）+ 同步写（wr_en 拍写入）
    // 尺寸 4M 字节（deep 场景最大地址 3000000+150000=3150000 < 4194304）
    //--------------------------------------------------------------------
    reg [7:0] gmem[0:4194303];
    always @(posedge clk) begin
        if (gray_wr_en) gmem[gray_wr_addr] <= gray_wr_data;
        if (gray_rd_en) gray_rd_data <= gmem[gray_rd_addr];
    end

    //--------------------------------------------------------------------
    // 向量读取（小端 u32/u8），单场景缓冲 2MB（m6_down_big.bin 约 1.15MB）
    //--------------------------------------------------------------------
    reg [7:0] fbuf[0:2097151];
    integer fd, code;

    function automatic [31:0] rd32(input integer base);
        rd32 = {fbuf[base+3], fbuf[base+2], fbuf[base+1], fbuf[base+0]};
    endfunction

    integer err = 0;

    // 写事件监视（small 场景验证"无任何 gray 写"）
    integer wr_events = 0;
    always @(posedge clk) if (gray_wr_en) wr_events = wr_events + 1;

    //--------------------------------------------------------------------
    // 公共流程：等空闲 → 单拍 start（锁存配置）
    //--------------------------------------------------------------------
    task fire_start(input [10:0] w, input [10:0] h, input [25:0] base);
        begin
            while (busy) @(negedge clk);
            cfg_w0 = w; cfg_h0 = h; cfg_base0 = base;
            start = 1'b1; @(negedge clk);
            start = 1'b0;
        end
    endtask

    // 等 done（电平保持，带超时保护）
    task wait_done(input string tag);
        begin
            while (!done) begin
                @(posedge clk);
                if ($time > 2_000_000_000) begin
                    $display("[FATAL] %s done timeout", tag);
                    $finish;
                end
            end
            repeat (3) @(negedge clk);
        end
    endtask

    //--------------------------------------------------------------------
    // 场景 big：1280x720（预载 m6_big_gray.bin）→ L1 640x360@921600
    //--------------------------------------------------------------------
    task run_big;
        integer i, w, h, n_src, n_exp, errs;
        begin
            errs = 0;
            wr_events = 0;
            fd = $fopen("../../data/image/m6_big_gray.bin", "rb");
            if (fd == 0) begin $display("[FATAL] cannot open m6_big_gray.bin"); $finish; end
            code = $fread(gmem, fd); $fclose(fd);

            fd = $fopen("../../data/image/m6_down_big.bin", "rb");
            if (fd == 0) begin $display("[FATAL] cannot open m6_down_big.bin"); $finish; end
            code = $fread(fbuf, fd); $fclose(fd);
            w = rd32(0); h = rd32(4);
            n_src = w * h; n_exp = (w / 2) * (h / 2);

            fire_start(1280, 720, 26'd0);
            wait_done("big");

            if (level_count !== 3'd2) begin
                errs = errs + 1;
                $display("[FAIL] big level_count=%0d exp=2", level_count);
            end
            if (level_base[1] !== 26'd921600) begin
                errs = errs + 1;
                $display("[FAIL] big L1 base=%0d exp=921600", level_base[1]);
            end
            if (level_w[1] !== 11'd640 || level_h[1] !== 11'd360) begin
                errs = errs + 1;
                $display("[FAIL] big L1 wh=%0dx%0d exp=640x360", level_w[1], level_h[1]);
            end
            // L1 灰度逐字节比对（期望段位于头部后 W*H 字节）
            for (i = 0; i < n_exp; ++i) begin
                if (gmem[921600 + i] !== fbuf[8 + n_src + i]) begin
                    errs = errs + 1;
                    if (errs <= 10)
                        $display("[FAIL] big L1[%0d] rtl=%0d exp=%0d",
                                 i, gmem[921600+i], fbuf[8+n_src+i]);
                end
            end
            $display("[big] 1280x720 -> L1 640x360 base=%0d cmp=%0d writes=%0d errs=%0d",
                     level_base[1], n_exp, wr_events, errs);
            err = err + errs;
        end
    endtask

    //--------------------------------------------------------------------
    // 场景 small：272x96（预载 m5_board5x8_gray.bin）→ 无缩图
    //--------------------------------------------------------------------
    task run_small;
        integer errs;
        begin
            errs = 0;
            wr_events = 0;
            fd = $fopen("../../data/image/m5_board5x8_gray.bin", "rb");
            if (fd == 0) begin $display("[FATAL] cannot open m5_board5x8_gray.bin"); $finish; end
            code = $fread(gmem, fd); $fclose(fd);

            fire_start(272, 96, 26'd0);
            wait_done("small");

            if (level_count !== 3'd1) begin
                errs = errs + 1;
                $display("[FAIL] small level_count=%0d exp=1", level_count);
            end
            if (level_base[0] !== 26'd0) begin
                errs = errs + 1;
                $display("[FAIL] small L0 base=%0d exp=0", level_base[0]);
            end
            if (wr_events != 0) begin
                errs = errs + 1;
                $display("[FAIL] small gray writes=%0d exp=0", wr_events);
            end
            $display("[small] 272x96 level_count=%0d writes=%0d errs=%0d",
                     level_count, wr_events, errs);
            err = err + errs;
        end
    endtask

    //--------------------------------------------------------------------
    // 场景 deep：2000x1200 合成棋盘（格 40px，180/40 交替）→ L1 + L2
    //--------------------------------------------------------------------
    task run_deep;
        integer x, y, i, errs, a, b, c, d, exp;
        begin
            errs = 0;
            wr_events = 0;
            // 合成棋盘预载 L0（与 export_m6 fill_checker 同构）
            for (y = 0; y < 1200; ++y)
                for (x = 0; x < 2000; ++x)
                    gmem[y*2000 + x] = (((x/40) + (y/40)) & 1) ? 8'd180 : 8'd40;

            fire_start(2000, 1200, 26'd0);
            wait_done("deep");

            if (level_count !== 3'd3) begin
                errs = errs + 1;
                $display("[FAIL] deep level_count=%0d exp=3", level_count);
            end
            if (level_base[1] !== 26'd2400000 || level_w[1] !== 11'd1000 ||
                level_h[1] !== 11'd600) begin
                errs = errs + 1;
                $display("[FAIL] deep L1 desc base=%0d w=%0d h=%0d exp=2400000/1000/600",
                         level_base[1], level_w[1], level_h[1]);
            end
            if (level_base[2] !== 26'd3000000 || level_w[2] !== 11'd500 ||
                level_h[2] !== 11'd300) begin
                errs = errs + 1;
                $display("[FAIL] deep L2 desc base=%0d w=%0d h=%0d exp=3000000/500/300",
                         level_base[2], level_w[2], level_h[2]);
            end
            // L1 期望：独立自算（四像素平均，与 C++ 同公式）
            for (y = 0; y < 600; ++y)
                for (x = 0; x < 1000; ++x) begin
                    a = gmem[(2*y)*2000 + 2*x];
                    b = gmem[(2*y)*2000 + 2*x + 1];
                    c = gmem[(2*y+1)*2000 + 2*x];
                    d = gmem[(2*y+1)*2000 + 2*x + 1];
                    exp = (a + b + c + d + 2) >> 2;
                    if (gmem[2400000 + y*1000 + x] !== exp[7:0]) begin
                        errs = errs + 1;
                        if (errs <= 10)
                            $display("[FAIL] deep L1[%0d] rtl=%0d exp=%0d",
                                     y*1000+x, gmem[2400000+y*1000+x], exp);
                    end
                end
            // L2 期望：链式自算（源 = RTL 生成的 L1，与 C++ 递归逐层一致）
            for (y = 0; y < 300; ++y)
                for (x = 0; x < 500; ++x) begin
                    a = gmem[2400000 + (2*y)*1000 + 2*x];
                    b = gmem[2400000 + (2*y)*1000 + 2*x + 1];
                    c = gmem[2400000 + (2*y+1)*1000 + 2*x];
                    d = gmem[2400000 + (2*y+1)*1000 + 2*x + 1];
                    exp = (a + b + c + d + 2) >> 2;
                    if (gmem[3000000 + y*500 + x] !== exp[7:0]) begin
                        errs = errs + 1;
                        if (errs <= 10)
                            $display("[FAIL] deep L2[%0d] rtl=%0d exp=%0d",
                                     y*500+x, gmem[3000000+y*500+x], exp);
                    end
                end
            $display("[deep] 2000x1200 -> L1 1000x600 L2 500x300 writes=%0d errs=%0d",
                     wr_events, errs);
            err = err + errs;
        end
    endtask

    //--------------------------------------------------------------------
    // 主流程：复位 → 逐场景 → 汇总
    //--------------------------------------------------------------------
    initial begin
        rst_n    = 1'b0;
        start    = 1'b0;
        cfg_w0   = 11'd0;
        cfg_h0   = 11'd0;
        cfg_base0= 26'd0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        run_big();
        run_small();
        run_deep();

        repeat (5) @(negedge clk);
        $display("======================================");
        if (err == 0)
            $display("TB RESULT: ALL PYRAMID TESTS PASSED");
        else
            $display("TB RESULT: FAILED (err=%0d)", err);
        $finish;
    end

    // 全局看门狗（正常情况下总仿真时间远小于 10s）
    initial begin
        #(64'd10_000_000_000);
        $display("[FATAL] global watchdog timeout");
        $finish;
    end

endmodule
