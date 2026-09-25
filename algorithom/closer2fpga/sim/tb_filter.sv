`timescale 1ns / 1ps
//==============================================================================
// tb_filter.sv — M5 候选后处理全链位级对拍（candidate_filter_ctrl + subpixel_ctrl）
//------------------------------------------------------------------------------
// 向量（tests/build/vectors/m5_board5x8_*，export_m5.cpp export_fullchain）：
//   m5_board5x8_gray.bin   272×96 灰度（W=272 H=96，行主序；17格×16 棋盘）
//   m5_board5x8_spx_in.bin u32 N + N×{x,y} fp32（merge5 后 = subpixel 输入）
//   m5_board5x8_spx_out.bin u32 N + N×{x,y} fp32（subpixel 后）
//   m5_board5x8_merge3.bin u32 N + N×{x,y} fp32（merge3 后）
//   m5_board5x8_inner.bin  u32 N + N×{x,y} fp32（ring 后 inner）
// 流程：
//   1) 读 spx_in 坐标 → lround → NMS 候选流喂 ctrl
//   2) 等 ctrl.done；逐阶段实时探针位级对拍：
//      · S_MERGE5 输出流（u_merge res）  ↔ spx_in.bin
//      · S_SUBPX  输出流（u_subpx out）  ↔ spx_out.bin
//      · S_MERGE3 输出流（u_merge res）  ↔ merge3.bin
//      · inner 流                        ↔ inner.bin
//   3) 全链点数核对 + ALL PASSED
//==============================================================================
module tb_filter;
    localparam integer CLK_PERIOD = 10;
    logic clk = 1'b0;
    logic rst_n;
    always #(CLK_PERIOD/2) clk = ~clk;

    import lround_pkg::*;

    integer fd, code;
    reg [7:0]  bytes [0:2097151];
    reg [31:0] words [0:524287];
    integer N;

    function automatic [31:0] le32(input integer i, input reg [7:0] arr[]);
        le32 = {arr[i+3], arr[i+2], arr[i+1], arr[i]};
    endfunction

    //--------------------------------------------------------------------
    // board5x8 场景参数（272×96 棋盘格：W = 17格×16，H = 6格×16）
    //--------------------------------------------------------------------
    localparam W = 272, H = 96;
    localparam GRAY_AW = 15;
    reg [7:0]  gray_mem [0:32767];

    logic       gray_en;
    logic [GRAY_AW-1:0] gray_addr;
    logic [7:0] gray_q;

    // ctrl 信号
    logic        start, cand_valid, cand_done;
    logic        cand_ready, busy, done;
    logic [1:0]  status;
    logic [10:0] cand_x, cand_y;
    logic        inner_valid;
    logic [31:0] inner_x, inner_y;
    logic [15:0] inner_total;
    logic        probe_valid;
    logic [31:0] probe_hi, probe_lo, probe_thr, probe_ntrans, probe_opp_err;
    logic        probe_sector_ok, probe_pass;

    candidate_filter_ctrl #(
        .IMG_W (W), .IMG_H (H), .GRAY_ADDR_W (GRAY_AW)
    ) u_ctrl (
        .clk (clk), .rst_n (rst_n),
        .start (start), .busy (busy), .done (done), .status (status),
        .cand_valid (cand_valid), .cand_ready (cand_ready),
        .cand_x (cand_x), .cand_y (cand_y), .cand_done (cand_done),
        .gray_rd_en (gray_en), .gray_rd_addr (gray_addr), .gray_rd_data (gray_q),
        .inner_valid (inner_valid), .inner_ready (1'b1),
        .inner_x (inner_x), .inner_y (inner_y), .inner_total (inner_total),
        .probe_valid (probe_valid),
        .probe_hi (probe_hi), .probe_lo (probe_lo), .probe_thr (probe_thr),
        .probe_ntrans (probe_ntrans), .probe_opp_err (probe_opp_err),
        .probe_sector_ok (probe_sector_ok), .probe_pass (probe_pass)
    );
    // gray 读口（TB 提供同步 RAM：rd_en=1 的下一拍出数据，同 dual_port_ram）
    always @(posedge clk) begin
        if (!rst_n) gray_q <= 8'd0;
        else if (gray_en) gray_q <= gray_mem[gray_addr];
    end

    //--------------------------------------------------------------------
    // 期望向量加载
    //--------------------------------------------------------------------
    integer N5, N3, NI;
    reg [31:0] exp_spx_in  [0:16383];
    reg [31:0] exp_spx_out [0:16383];
    reg [31:0] exp_merge3  [0:16383];
    reg [31:0] exp_inner   [0:16383];
    integer n_in;
    reg [31:0] in_x [0:16383];
    reg [31:0] in_y [0:16383];

    task automatic load_u32_file(input string path, output integer nw,
                                 output reg [31:0] arr[0:16383]);
        integer i;
        begin
            fd = $fopen(path, "rb");
            if (fd == 0) begin
                $display("[FATAL] no %s", path);
                $finish;
            end
            code = $fread(bytes, fd);
            $fclose(fd);
            // 文件布局：word0=N，其后 nw 个数据 word（nw = 总字数 - 1）
            nw = (code / 4) - 1;
            for (i = 0; i < nw; ++i)
                arr[i] = le32((i+1)*4, bytes);
        end
    endtask

    task automatic load_pts_bin(input string path, output integer n,
                                output reg [31:0] xs[0:16383],
                                output reg [31:0] ys[0:16383]);
        integer i, nw;
        reg [31:0] w [0:16383];
        begin
            load_u32_file(path, nw, w);
            n = nw / 2;                 // 点数
            for (i = 0; i < n; ++i) begin
                xs[i] = w[2*i];
                ys[i] = w[2*i+1];
            end
        end
    endtask

    integer fail_cnt = 0;

    //--------------------------------------------------------------------
    // 实时探针（逐阶段位级）
    //--------------------------------------------------------------------
    integer m5_cnt = 0, spx_cnt = 0, m3_cnt = 0, inner_cnt = 0;

    // S_MERGE5 输出流（= subpixel 输入 spx_in；期望 = spx_in.bin）
    always @(posedge clk) begin
        if (u_ctrl.state == 4'd2 &&
            u_ctrl.u_merge.res_valid && u_ctrl.u_merge.res_ready) begin
            if (m5_cnt < N5 && (u_ctrl.u_merge.res_x !== exp_spx_in[2*m5_cnt] ||
                                u_ctrl.u_merge.res_y !== exp_spx_in[2*m5_cnt+1])) begin
                fail_cnt = fail_cnt + 1;
                if (fail_cnt <= 90)
                    $display("[FAIL] merge5(spx_in) #%0d rtl=(%08x,%08x) exp=(%08x,%08x)",
                             m5_cnt, u_ctrl.u_merge.res_x, u_ctrl.u_merge.res_y,
                             exp_spx_in[2*m5_cnt], exp_spx_in[2*m5_cnt+1]);
            end
            m5_cnt = m5_cnt + 1;
        end
    end

    // S_SUBPX 输出流（subpixel 后 = spx_out）
    always @(posedge clk) begin
        if (u_ctrl.state == 4'd3 &&
            u_ctrl.u_subpx.out_valid && u_ctrl.u_subpx.out_ready) begin
            if (spx_cnt < N5 && (u_ctrl.u_subpx.out_x !== exp_spx_out[2*spx_cnt] ||
                                 u_ctrl.u_subpx.out_y !== exp_spx_out[2*spx_cnt+1])) begin
                fail_cnt = fail_cnt + 1;
                if (fail_cnt <= 20)
                    $display("[FAIL] spx_out #%0d rtl=(%08x,%08x) exp=(%08x,%08x)",
                             spx_cnt, u_ctrl.u_subpx.out_x, u_ctrl.u_subpx.out_y,
                             exp_spx_out[2*spx_cnt], exp_spx_out[2*spx_cnt+1]);
            end
            spx_cnt = spx_cnt + 1;
        end
    end

    // S_MERGE3 输出流
    always @(posedge clk) begin
        if (u_ctrl.state == 4'd4 &&
            u_ctrl.u_merge.res_valid && u_ctrl.u_merge.res_ready) begin
            if (m3_cnt < N3 && (u_ctrl.u_merge.res_x !== exp_merge3[2*m3_cnt] ||
                                u_ctrl.u_merge.res_y !== exp_merge3[2*m3_cnt+1])) begin
                fail_cnt = fail_cnt + 1;
                if (fail_cnt <= 20)
                    $display("[FAIL] merge3 #%0d rtl=(%08x,%08x) exp=(%08x,%08x)",
                             m3_cnt, u_ctrl.u_merge.res_x, u_ctrl.u_merge.res_y,
                             exp_merge3[2*m3_cnt], exp_merge3[2*m3_cnt+1]);
            end
            m3_cnt = m3_cnt + 1;
        end
    end

    // inner 流（ring 后）
    always @(posedge clk) begin
        if (inner_valid) begin
            if (inner_cnt < NI && (inner_x !== exp_inner[2*inner_cnt] ||
                                   inner_y !== exp_inner[2*inner_cnt+1])) begin
                fail_cnt = fail_cnt + 1;
                if (fail_cnt <= 20)
                    $display("[FAIL] inner #%0d rtl=(%08x,%08x) exp=(%08x,%08x)",
                             inner_cnt, inner_x, inner_y,
                             exp_inner[2*inner_cnt], exp_inner[2*inner_cnt+1]);
            end
            inner_cnt = inner_cnt + 1;
        end
    end

    //--------------------------------------------------------------------
    // 调试探针（RING/NEAR 输入输出；定位 inner=0 用）
    //--------------------------------------------------------------------
    integer dbg_near = 0, dbg_ring = 0, ring_pass_cnt = 0, p_probe = 0;
    always @(posedge clk) begin
        if (probe_valid) begin
            p_probe = p_probe + 1;
            if (probe_pass) ring_pass_cnt = ring_pass_cnt + 1;
        end
        if (u_ctrl.state == 4'd5 && u_ctrl.u_near.out_valid && dbg_near < 80) begin
            $display("[DBG-near] #%0d radius=%08x", dbg_near, u_ctrl.u_near.out_radius);
            dbg_near = dbg_near + 1;
        end
        if (u_ctrl.state == 4'd6 && u_ctrl.u_ring.out_valid && dbg_ring < 80) begin
            $display("[DBG-ring] #%0d in=(%08x,%08x) r=%08x pass=%b hi=%08x lo=%08x thr=%08x ntr=%0d opp=%08x",
                     dbg_ring, u_ctrl.u_ring.in_x, u_ctrl.u_ring.in_y, u_ctrl.u_ring.in_radius,
                     u_ctrl.u_ring.out_pass, u_ctrl.u_ring.out_hi, u_ctrl.u_ring.out_lo,
                     u_ctrl.u_ring.out_thr, u_ctrl.u_ring.out_ntrans, u_ctrl.u_ring.out_opp_err);
            dbg_ring = dbg_ring + 1;
        end
    end

    //--------------------------------------------------------------------
    // 主流程
    //--------------------------------------------------------------------
    integer nw5, nw3, nwi;
    integer i;
    reg [63:0] m64;

    initial begin
        // 1) 加载期望与输入
        //    输入 = shi_tomasi 原始候选（cand.bin，320 点整数坐标，merge5 输入）
        //    merge5 期望 = spx_in.bin（80 点）；subpixel 期望 = spx_out.bin
        load_pts_bin("../tests/build/vectors/m5_board5x8_cand.bin", n_in, in_x, in_y);
        load_u32_file("../tests/build/vectors/m5_board5x8_spx_in.bin", nw5, exp_spx_in);
        N5 = nw5 / 2;
        load_u32_file("../tests/build/vectors/m5_board5x8_spx_out.bin", nw5, exp_spx_out);
        load_u32_file("../tests/build/vectors/m5_board5x8_merge3.bin", nw3, exp_merge3);
        N3 = nw3 / 2;
        load_u32_file("../tests/build/vectors/m5_board5x8_inner.bin", nwi, exp_inner);
        NI = nwi / 2;
        $display("========== scene board5x8: N_cand=%0d (w=%0d h=%0d) ==========", n_in, W, H);

        // 2) 灰度灌入
        fd = $fopen("../tests/build/vectors/m5_board5x8_gray.bin", "rb");
        if (fd == 0) begin $display("[FATAL] no gray"); $finish; end
        code = $fread(bytes, fd);
        $fclose(fd);
        for (i = 0; i < W*H; ++i)
            gray_mem[i] = bytes[i];

        // 3) 复位
        rst_n = 1'b0;
        start = 0; cand_valid = 0; cand_done = 0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // 4) 启动
        start <= 1'b1;
        @(negedge clk);
        start <= 1'b0;
        while (!busy) @(posedge clk);

        // 5) 喂候选（整数坐标：fp32 位模式 → lround）
        for (i = 0; i < n_in; ++i) begin
            cand_x <= lround_f32(in_x[i])[10:0];
            cand_y <= lround_f32(in_y[i])[10:0];
            cand_valid <= 1'b1;
            do @(posedge clk); while (!cand_ready);
            cand_valid <= 1'b0;
        end
        @(negedge clk);
        cand_done <= 1'b1;
        @(negedge clk);
        cand_done <= 1'b0;

        // 6) 等完成
        while (!done) @(posedge clk);
        @(negedge clk);

        // 6b) dump store RAM 内容（定位 ring 输入数据）
        for (i = 0; i < 6; ++i) begin
            m64 = u_ctrl.u_store.u_ram_a.mem[i];
            $display("[RAM-A] #%0d = (%08x,%08x)", i, m64[31:0], m64[63:32]);
            m64 = u_ctrl.u_store.u_ram_b.mem[i];
            $display("[RAM-B] #%0d = (%08x,%08x)", i, m64[31:0], m64[63:32]);
        end
        $display("[ctrl] N_reg=%0d cnt_a=%0d cnt_b=%0d",
                 u_ctrl.N_reg, u_ctrl.u_store.cnt_a, u_ctrl.u_store.cnt_b);

        // 7) 汇总
        $display("[scene board5x8] status=%b inner_total=%0d (exp inner=%0d)", status, inner_total, NI);
        $display("[counts] merge5(spx_in)=%0d (exp %0d)  spx_out=%0d (exp %0d)  merge3=%0d (exp %0d)  inner=%0d (exp %0d)",
                 m5_cnt, N5, spx_cnt, N5, m3_cnt, N3, inner_cnt, NI);
        $display("[dbg] near_first=%0d ring_first=%0d ring_pass_total=%0d probe_points=%0d",
                 dbg_near, dbg_ring, ring_pass_cnt, p_probe);
        if (status !== 2'b01) begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] status=%02b exp=01", status);
        end
        if (m5_cnt !== N5 || spx_cnt !== N5 || m3_cnt !== N3 || inner_cnt !== NI) begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] point count mismatch");
        end

        $display("======================================");
        $display("FILTER CTRL TB: fail=%0d", fail_cnt);
        if (fail_cnt == 0)
            $display("TB RESULT: ALL FILTER TESTS PASSED (bit-exact)");
        else
            $display("TB RESULT: FAILED");
        $finish;
    end

    // 看门狗（仿真时间上限 2000s；subpixel 全链迭代量大）
    initial begin
        #2000000000;
        $display("[TB][FATAL] watchdog timeout");
        $finish;
    end

endmodule
