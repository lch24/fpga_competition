`timescale 1ns / 1ps
//==============================================================================
// tb_detect_full.sv — M6.2 detect_ctrl 全链位级对拍（big 金字塔 + board5x8 native）
//------------------------------------------------------------------------------
// 向量（只读，主控生成）：
//   tests/build/vectors/m6_chain_big.bin         u32 valid + u32 N + N×{x,y} fp32
//   tests/build/vectors/m6_chain_board5x8.bin    同格式
//   tests/build/vectors/m6_big_gray.bin          1280×720 灰度（L0）
//   tests/build/vectors/m6_down_big.bin          u32 W,H + 源 + 期望缩图（期望段作 L1）
//   tests/build/vectors/m6_down_board5x8.bin     u32 W,H + 源（源段作 96×272 灰度）
//
// 场景：
//   1) big（DEPTH=2，金字塔路径）：gray RAM base0=m6_big_gray.bin、
//      base921600=m6_down_big.bin 期望缩图段；期望 m6_chain_big.bin。
//   2) board5x8（DEPTH=1，native 路径）：权威向量 m6_chain_board5x8.bin 由
//      export_m6.cpp 的 96×272 场景生成（GrayImage(96,272)，注意与 M5.2 的
//      272×96 m5_board5x8_gray.bin 互为转置），故本场景用 W0=96,H0=272，
//      base0 预载 m6_down_board5x8.bin 的源灰度段；期望 m6_chain_board5x8.bin。
//      （native 链在 272×96 + m5_board5x8_gray.bin 下的输出经离线核对与
//        m5_board5x8_refined.bin 40/40 位级一致，集成正确性双保险。）
// gray RAM 同步读（rd_en 下一拍出数据）；输出侧周期背压（每 8 拍停 1 拍）。
// +SCENE=big|b5x8|both（默认 both）。
//==============================================================================
module tb_detect_full;

    localparam integer CLK_PERIOD = 10;
    logic clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    logic rst_n;

    //--------------------------------------------------------------------
    // big 场景（DEPTH=2）
    //--------------------------------------------------------------------
    localparam integer B_W0 = 1280, B_H0 = 720, B_NPIX0 = B_W0 * B_H0;
    localparam integer B_L1   = (B_W0/2) * (B_H0/2);        // 230400
    localparam integer G_AW   = 21;                         // 2^21 ≥ 总灰度
    logic        b_start, b_busy, b_done; logic [1:0] b_status;
    logic        b_gray_en; logic [25:0] b_gray_addr; logic [7:0] b_gray_data;
    logic        b_out_v; logic b_out_r; logic [31:0] b_out_x, b_out_y;
    logic [15:0] b_out_t; logic b_out_gok;
    logic        b_collect;
    logic        bg_wr_en; logic [G_AW-1:0] bg_wr_a; logic [7:0] bg_wr_d;

    detect_ctrl #(
        .W0(1280), .H0(720), .DEPTH(2), .GRAY_ADDR_W(26),
        .CORNER_N(40), .CORNER_AW(8)
    ) u_big (
        .clk(clk), .rst_n(rst_n),
        .start(b_start), .cfg_base0(26'd0),
        .busy(b_busy), .done(b_done), .status(b_status),
        .gray_rd_en(b_gray_en), .gray_rd_addr(b_gray_addr), .gray_rd_data(b_gray_data),
        .out_valid(b_out_v), .out_ready(b_out_r), .out_x(b_out_x), .out_y(b_out_y),
        .out_total(b_out_t), .out_grid_ok(b_out_gok)
    );

    dual_port_ram #(.DATA_WIDTH(8), .ADDR_WIDTH(G_AW)) u_gray_big (
        .clk(clk), .rst_n(rst_n),
        .wr_en(bg_wr_en), .wr_addr(bg_wr_a), .wr_data(bg_wr_d),
        .rd_en(b_gray_en), .rd_addr(b_gray_addr[G_AW-1:0]), .rd_data(b_gray_data)
    );

    //--------------------------------------------------------------------
    // board5x8 场景（DEPTH=1；权威向量来自 96×272 转置场景）
    //--------------------------------------------------------------------
    localparam integer S_W0 = 96, S_H0 = 272, S_NPIX = S_W0 * S_H0;   // 26112
    logic        s_start, s_busy, s_done; logic [1:0] s_status;
    logic        s_gray_en; logic [25:0] s_gray_addr; logic [7:0] s_gray_data;
    logic        s_out_v; logic s_out_r; logic [31:0] s_out_x, s_out_y;
    logic [15:0] s_out_t; logic s_out_gok;
    logic        s_collect;
    logic        sg_wr_en; logic [G_AW-1:0] sg_wr_a; logic [7:0] sg_wr_d;

    detect_ctrl #(
        .W0(96), .H0(272), .DEPTH(1), .GRAY_ADDR_W(26),
        .CORNER_N(40), .CORNER_AW(8)
    ) u_b5 (
        .clk(clk), .rst_n(rst_n),
        .start(s_start), .cfg_base0(26'd0),
        .busy(s_busy), .done(s_done), .status(s_status),
        .gray_rd_en(s_gray_en), .gray_rd_addr(s_gray_addr), .gray_rd_data(s_gray_data),
        .out_valid(s_out_v), .out_ready(s_out_r), .out_x(s_out_x), .out_y(s_out_y),
        .out_total(s_out_t), .out_grid_ok(s_out_gok)
    );

    dual_port_ram #(.DATA_WIDTH(8), .ADDR_WIDTH(G_AW)) u_gray_small (
        .clk(clk), .rst_n(rst_n),
        .wr_en(sg_wr_en), .wr_addr(sg_wr_a), .wr_data(sg_wr_d),
        .rd_en(s_gray_en), .rd_addr(s_gray_addr[G_AW-1:0]), .rd_data(s_gray_data)
    );

    //--------------------------------------------------------------------
    // 公共缓冲 / 工具
    //--------------------------------------------------------------------
    reg [7:0] fbuf [0:2097151];
    integer   fd, code;
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

    reg [31:0] got_x[0:255], got_y[0:255];
    integer    got_cnt;
    reg [31:0] exp_x[0:255], exp_y[0:255];
    integer    exp_valid, exp_N;

    // 周期背压：收集期间计数 0..7，out_ready 在第 7 拍拉低（每 8 拍停 1 拍）
    integer bp_b, bp_s;
    always @(posedge clk) begin
        if (!rst_n) bp_b <= 0;
        else if (b_collect) bp_b <= (bp_b == 7) ? 0 : bp_b + 1;
    end
    always @(posedge clk) begin
        if (!rst_n) bp_s <= 0;
        else if (s_collect) bp_s <= (bp_s == 7) ? 0 : bp_s + 1;
    end
    assign b_out_r = b_collect && (bp_b != 7);
    assign s_out_r = s_collect && (bp_s != 7);

    // 收集（big）
    always @(posedge clk) begin
        if (b_collect && b_out_v && b_out_r) begin
            got_x[got_cnt] <= b_out_x;
            got_y[got_cnt] <= b_out_y;
            got_cnt <= got_cnt + 1;
        end
    end
    // 收集（board5x8）
    always @(posedge clk) begin
        if (s_collect && s_out_v && s_out_r) begin
            got_x[got_cnt] <= s_out_x;
            got_y[got_cnt] <= s_out_y;
            got_cnt <= got_cnt + 1;
        end
    end

    //--------------------------------------------------------------------
    // 校验任务
    //--------------------------------------------------------------------
    integer fail_total;
    integer k;

    task check_scene(input string name, input logic [1:0] st,
                     input logic gok, input logic [15:0] tot);
        begin
            $display("======================================================");
            $display("[%s] status=%b out_total=%0d out_grid_ok=%b got=%0d",
                     name, st, tot, gok, got_cnt);
            if (st !== 2'b01) begin
                fail_total = fail_total + 1;
                $display("[%s][FAIL] status=%b exp=01", name, st);
            end
            if (got_cnt != exp_N) begin
                fail_total = fail_total + 1;
                $display("[%s][FAIL] got %0d / exp %0d points", name, got_cnt, exp_N);
            end
            if (!gok) begin
                fail_total = fail_total + 1;
                $display("[%s][FAIL] out_grid_ok=%b exp=1", name, gok);
            end
            if (tot != exp_N) begin
                fail_total = fail_total + 1;
                $display("[%s][FAIL] out_total=%0d exp=%0d", name, tot, exp_N);
            end
            if (got_cnt > 255) got_cnt = 255;
            for (k = 0; k < exp_N; k = k + 1) begin
                if (got_x[k] !== exp_x[k] || got_y[k] !== exp_y[k]) begin
                    fail_total = fail_total + 1;
                    $display("[%s][FAIL][%0d] got %08x,%08x exp %08x,%08x",
                             name, k, got_x[k], got_y[k], exp_x[k], exp_y[k]);
                end
            end
            if (fail_total == 0)
                $display("[%s] 40/40 bit-exact, status/grid_ok OK", name);
        end
    endtask

    //--------------------------------------------------------------------
    // 主流程
    //--------------------------------------------------------------------
    string run_scene;

    initial begin
        b_start = 1'b0; s_start = 1'b0;
        bg_wr_en = 1'b0; bg_wr_a = 0; bg_wr_d = 8'd0;
        sg_wr_en = 1'b0; sg_wr_a = 0; sg_wr_d = 8'd0;
        b_collect = 1'b0; s_collect = 1'b0;
        got_cnt = 0; fail_total = 0;

        if (!$value$plusargs("SCENE=%s", run_scene)) run_scene = "both";
        $display("[TB] SCENE=%s", run_scene);

        rst_n = 1'b0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        //======================== SCENE 1: big ========================
        if (run_scene == "both" || run_scene == "big") begin
            $display("========== SCENE 1: big (DEPTH=2, pyramid path) ==========");
            // L0 灰度
            read_file("../../data/image/m6_big_gray.bin");
            for (integer kk = 0; kk < B_NPIX0; kk = kk + 1) begin
                bg_wr_en = 1'b1;
                bg_wr_a  = kk[G_AW-1:0];
                bg_wr_d  = fbuf[kk];
                @(negedge clk);
            end
            // L1 灰度（m6_down_big.bin 期望缩图段，文件偏移 8 + W*H）
            read_file("../../data/image/m6_down_big.bin");
            for (integer kk = 0; kk < B_L1; kk = kk + 1) begin
                bg_wr_en = 1'b1;
                bg_wr_a  = B_NPIX0 + kk;
                bg_wr_d  = fbuf[8 + B_NPIX0 + kk];
                @(negedge clk);
            end
            bg_wr_en = 1'b0;
            @(negedge clk);
            $display("[TB][big] gray loaded: L0=%0d px, L1=%0d px @ base %0d",
                     B_NPIX0, B_L1, B_NPIX0);

            // 期望
            read_file("../../data/image/m6_chain_big.bin");
            exp_valid = rd32(0); exp_N = rd32(4);
            for (integer kk = 0; kk < exp_N; kk = kk + 1) begin
                exp_x[kk] = rd32(8 + kk * 8);
                exp_y[kk] = rd32(8 + kk * 8 + 4);
            end
            $display("[TB][big] exp valid=%0d N=%0d first=%08x,%08x",
                     exp_valid, exp_N, exp_x[0], exp_y[0]);
            if (exp_valid != 1) begin
                $display("[TB][FATAL] big exp valid=0"); $finish;
            end

            // start + 收集
            got_cnt = 0;
            b_collect = 1'b1;
            b_start = 1'b1;
            @(negedge clk);
            b_start = 1'b0;
            $display("[TB][big] start @t=%0t", $time);
            while (!b_done) @(negedge clk);
            b_collect = 1'b0;
            @(negedge clk);
            $display("[TB][big] done @t=%0t busy=%b", $time, b_busy);
            // 调试：子级状态
            $display("[dbg][big] L1: shi.st=%b filter.st=%b order.st=%b order.gok=%b refine.valid=%b",
                     u_big.g_slot[1].g_stored.u_shi.status, u_big.g_slot[1].g_private_backend.u_filter.status,
                     u_big.g_slot[1].g_private_backend.u_order.status, u_big.g_slot[1].g_private_backend.u_order.out_grid_ok,
                     u_big.g_slot[1].g_private_backend.u_refine.valid_out);
            $display("[dbg][big] L0: shi.st=%b filter.st=%b order.st=%b order.gok=%b refine.valid=%b",
                     u_big.g_slot[0].g_stored.u_shi.status, u_big.g_slot[0].g_private_backend.u_filter.status,
                     u_big.g_slot[0].g_private_backend.u_order.status, u_big.g_slot[0].g_private_backend.u_order.out_grid_ok,
                     u_big.g_slot[0].g_private_backend.u_refine.valid_out);
            $display("[dbg][big] child_valid=%b stage=%0d", u_big.child_valid, u_big.stage);
            check_scene("big", b_status, b_out_gok, b_out_t);
        end

        //===================== SCENE 2: board5x8 =====================
        if (run_scene == "both" || run_scene == "b5x8") begin
            $display("========== SCENE 2: board5x8 (DEPTH=1, native path) ==========");
            // 权威向量 m6_chain_board5x8.bin 由 export_m6.cpp 的 96×272 场景生成，
            // 故本场景灰度取 m6_down_board5x8.bin 源段（字节 8..8+W*H）
            read_file("../../data/image/m6_down_board5x8.bin");
            for (integer kk = 0; kk < S_NPIX; kk = kk + 1) begin
                sg_wr_en = 1'b1;
                sg_wr_a  = kk[G_AW-1:0];
                sg_wr_d  = fbuf[8 + kk];
                @(negedge clk);
            end
            sg_wr_en = 1'b0;
            @(negedge clk);
            $display("[TB][b5x8] gray loaded: %0d px (%0dx%0d)", S_NPIX, S_W0, S_H0);

            read_file("../../data/image/m6_chain_board5x8.bin");
            exp_valid = rd32(0); exp_N = rd32(4);
            for (integer kk = 0; kk < exp_N; kk = kk + 1) begin
                exp_x[kk] = rd32(8 + kk * 8);
                exp_y[kk] = rd32(8 + kk * 8 + 4);
            end
            $display("[TB][b5x8] exp valid=%0d N=%0d first=%08x,%08x",
                     exp_valid, exp_N, exp_x[0], exp_y[0]);
            if (exp_valid != 1) begin
                $display("[TB][FATAL] b5x8 exp valid=0"); $finish;
            end

            got_cnt = 0;
            s_collect = 1'b1;
            s_start = 1'b1;
            @(negedge clk);
            s_start = 1'b0;
            $display("[TB][b5x8] start @t=%0t", $time);
            while (!s_done) @(negedge clk);
            s_collect = 1'b0;
            @(negedge clk);
            $display("[TB][b5x8] done @t=%0t busy=%b", $time, s_busy);
            // 调试：子级状态
            $display("[dbg][b5x8] shi.st=%b filter.st=%b order.st=%b order.gok=%b ",
                     u_b5.g_slot[0].g_stored.u_shi.status, u_b5.g_slot[0].g_private_backend.u_filter.status,
                     u_b5.g_slot[0].g_private_backend.u_order.status, u_b5.g_slot[0].g_private_backend.u_order.out_grid_ok);
            $display("[dbg][b5x8] refine.valid=%b refine.done=%b child_valid=%b stage=%0d",
                     u_b5.g_slot[0].g_private_backend.u_refine.valid_out, u_b5.g_slot[0].g_private_backend.u_refine.done,
                     u_b5.child_valid, u_b5.stage);
            check_scene("b5x8", s_status, s_out_gok, s_out_t);
        end

        //======================== 汇总 ========================
        $display("======================================================");
        if (fail_total == 0)
            $display("TB RESULT: ALL DETECT-FULL TESTS PASSED");
        else
            $display("TB RESULT: DETECT-FULL TESTS FAILED (%0d)", fail_total);
        $finish;
    end

    // 看门狗（600ms 仿真时间；全链两场景预估 ~50ms）
    initial begin
        #600_000_000;
        $display("[TB][FATAL] watchdog timeout b_done=%b s_done=%b", b_done, s_done);
        $finish;
    end

endmodule
