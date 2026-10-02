`timescale 1ns / 1ps
//==============================================================================
// tb_ring.sv — M3 nearest_spacing + ring_check 位级对拍
//------------------------------------------------------------------------------
// 流程（每场景）：
//   1) 读 m3_<scene>_merge3.bin → 灌入点 RAM（dual_port_ram 32 位 ×2）
//   2) 读 m3_<scene>_gray.bin → 灌入灰度 RAM（8 位，按场景实例）
//   3) start nearest_spacing(n=N) → 输出 {spacing,radius} 逐点与
//      m3_<scene>_nearest.bin 位级比对
//   4) 对每点 i：ring_check(x_i,y_i,radius_i=nearest 输出) 三连调
//      （radius / 0.75×radius / 1.25×radius，后两者用 fp32_mul 计算），
//      d0 的 {hi,lo,thr,ntrans,opp_err,sector_ok} 与 m3_<scene>_ring.bin
//      比对，pass = pass_r && (pass_r075 || pass_r125) 与 bin 比对
//   5) lround_vec.mem 独立自检（lround_f32 与 C++ std::lround 全位对拍）
// 两场景（small 32×24 / texture 128×96）均须 ALL PASSED。
// 向量路径相对 vsim 运行目录（sim/）：../tests/build/vectors/。
//==============================================================================
module tb_ring;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst_n;

    //--------------------------------------------------------------------
    // 点 RAM（32 位 ×2，14 位地址）
    //--------------------------------------------------------------------
    reg        pw_en;
    reg [13:0] pw_addr;
    reg [31:0] pw_x, pw_y;
    wire       ns_rd_en;
    wire [13:0] ns_rd_addr;
    wire [31:0] ns_rd_x, ns_rd_y;

    dual_port_ram #(.DATA_WIDTH(32), .ADDR_WIDTH(14)) u_px (
        .clk(clk), .rst_n(rst_n),
        .wr_en(pw_en), .wr_addr(pw_addr), .wr_data(pw_x),
        .rd_en(ns_rd_en), .rd_addr(ns_rd_addr), .rd_data(ns_rd_x)
    );
    dual_port_ram #(.DATA_WIDTH(32), .ADDR_WIDTH(14)) u_py (
        .clk(clk), .rst_n(rst_n),
        .wr_en(pw_en), .wr_addr(pw_addr), .wr_data(pw_y),
        .rd_en(ns_rd_en), .rd_addr(ns_rd_addr), .rd_data(ns_rd_y)
    );

    //--------------------------------------------------------------------
    // nearest_spacing DUT
    //--------------------------------------------------------------------
    reg        ns_start;
    wire       ns_busy, ns_done;
    reg [15:0] ns_n;
    wire       ns_out_v;
    reg        ns_out_rdy;
    wire [31:0] ns_sp, ns_rad;

    nearest_spacing #(.N_ADDR_W(14)) u_ns (
        .clk(clk), .rst_n(rst_n),
        .start(ns_start), .busy(ns_busy), .done(ns_done),
        .n_in(ns_n),
        .rd_en(ns_rd_en), .rd_addr(ns_rd_addr), .rd_x(ns_rd_x), .rd_y(ns_rd_y),
        .out_valid(ns_out_v), .out_ready(ns_out_rdy),
        .out_spacing(ns_sp), .out_radius(ns_rad)
    );

    //--------------------------------------------------------------------
    // 灰度 RAM ×2（8 位；small/texture 各一实例）与 ring_check DUT ×2
    //--------------------------------------------------------------------
    reg        gw_en;
    reg [13:0] gw_addr;
    reg [7:0]  gw_data;
    wire       rc_s_rd_en;
    wire [13:0] rc_s_rd_addr;
    wire [7:0]  rc_s_rd_data;
    wire       rc_t_rd_en;
    wire [13:0] rc_t_rd_addr;
    wire [7:0]  rc_t_rd_data;

    dual_port_ram #(.DATA_WIDTH(8), .ADDR_WIDTH(14)) u_gray_s (
        .clk(clk), .rst_n(rst_n),
        .wr_en(gw_en), .wr_addr(gw_addr), .wr_data(gw_data),
        .rd_en(rc_s_rd_en), .rd_addr(rc_s_rd_addr), .rd_data(rc_s_rd_data)
    );
    dual_port_ram #(.DATA_WIDTH(8), .ADDR_WIDTH(14)) u_gray_t (
        .clk(clk), .rst_n(rst_n),
        .wr_en(gw_en), .wr_addr(gw_addr), .wr_data(gw_data),
        .rd_en(rc_t_rd_en), .rd_addr(rc_t_rd_addr), .rd_data(rc_t_rd_data)
    );

    // ring_check：两实例（W/H 参数化），按场景只驱动活动实例；
    // 输出各自独立信号集，run_ring 按 cur_inst 选读。
    reg        rc_in_v_s, rc_in_v_t;
    reg        rc_out_rdy;
    wire       rc_in_rdy_s, rc_in_rdy_t;
    reg [31:0] rc_in_x, rc_in_y, rc_in_r;
    wire       rc_s_ov;
    wire [31:0] rc_s_hi, rc_s_lo, rc_s_thr, rc_s_ntr, rc_s_oe;
    wire        rc_s_so, rc_s_ps;
    wire       rc_t_ov;
    wire [31:0] rc_t_hi, rc_t_lo, rc_t_thr, rc_t_ntr, rc_t_oe;
    wire        rc_t_so, rc_t_ps;

    ring_check #(.W(32), .H(24), .GRAY_ADDR_W(14),
                 .ROM_FILE("../tests/build/vectors/ring_cos_sin.mem")) u_rc_s (
        .clk(clk), .rst_n(rst_n),
        .in_valid(rc_in_v_s), .in_ready(rc_in_rdy_s),
        .in_x(rc_in_x), .in_y(rc_in_y), .in_radius(rc_in_r),
        .rd_en(rc_s_rd_en), .rd_addr(rc_s_rd_addr), .rd_data(rc_s_rd_data),
        .out_valid(rc_s_ov), .out_ready(rc_out_rdy),
        .out_hi(rc_s_hi), .out_lo(rc_s_lo), .out_thr(rc_s_thr),
        .out_ntrans(rc_s_ntr), .out_opp_err(rc_s_oe),
        .out_sector_ok(rc_s_so), .out_pass(rc_s_ps)
    );

    ring_check #(.W(128), .H(96), .GRAY_ADDR_W(14),
                 .ROM_FILE("../tests/build/vectors/ring_cos_sin.mem")) u_rc_t (
        .clk(clk), .rst_n(rst_n),
        .in_valid(rc_in_v_t), .in_ready(rc_in_rdy_t),
        .in_x(rc_in_x), .in_y(rc_in_y), .in_radius(rc_in_r),
        .rd_en(rc_t_rd_en), .rd_addr(rc_t_rd_addr), .rd_data(rc_t_rd_data),
        .out_valid(rc_t_ov), .out_ready(rc_out_rdy),
        .out_hi(rc_t_hi), .out_lo(rc_t_lo), .out_thr(rc_t_thr),
        .out_ntrans(rc_t_ntr), .out_opp_err(rc_t_oe),
        .out_sector_ok(rc_t_so), .out_pass(rc_t_ps)
    );

    // 0.75 / 1.25 半径（fp32_mul）
    wire m75_ov, m125_ov;
    reg        m75_v, m125_v;
    reg [31:0] m75_r, m125_r;
    fp32_mul u_m75 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(m75_v), .in_ready(), .in_a(rc_in_r), .in_b(32'h3f400000),
        .out_valid(m75_ov), .out_ready(1'b1), .out_r(m75_r)
    );
    fp32_mul u_m125 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(m125_v), .in_ready(), .in_a(rc_in_r), .in_b(32'h3fa00000),
        .out_valid(m125_ov), .out_ready(1'b1), .out_r(m125_r)
    );

    //--------------------------------------------------------------------
    // 向量数据
    //--------------------------------------------------------------------
    reg [7:0]  fbuf[0:1048575];
    reg [31:0] pts_x[0:16383], pts_y[0:16383];
    reg [31:0] exp_sp[0:16383], exp_rad[0:16383];
    reg [31:0] exp_hi[0:16383], exp_lo[0:16383], exp_thr[0:16383];
    reg [31:0] exp_ntr[0:16383], exp_oe[0:16383];
    reg        exp_so[0:16383], exp_ps[0:16383];
    integer    N_in;

    integer    cmp_n, match_cnt, fail_cnt;
    integer    fd, code;
    integer    scene_pass_cnt, scene_fail_cnt;
    integer    all_pass;

    function automatic [31:0] rd32(input integer base);
        rd32 = {fbuf[base+3], fbuf[base+2], fbuf[base+1], fbuf[base+0]};
    endfunction

    //--------------------------------------------------------------------
    // 读取任务
    //--------------------------------------------------------------------
    task read_file(input string path);
        begin
            fd = $fopen(path, "rb");
            if (fd == 0) begin $display("[TB][FATAL] cannot open %s", path); $finish; end
            code = $fread(fbuf, fd);
            $fclose(fd);
            $display("[TB] load %s (%0d bytes)", path, code);
        end
    endtask

    task read_pts(input string path);
        integer k;
        begin
            read_file(path);
            N_in = rd32(0);
            for (k = 0; k < N_in; k = k + 1) begin
                pts_x[k] = rd32(4 + k * 8);
                pts_y[k] = rd32(4 + k * 8 + 4);
            end
            $display("[TB] pts N=%0d", N_in);
        end
    endtask

    task read_nearest(input string path);
        integer k;
        begin
            read_file(path);
            for (k = 0; k < N_in; k = k + 1) begin
                exp_sp[k]  = rd32(4 + k * 8);
                exp_rad[k] = rd32(4 + k * 8 + 4);
            end
            $display("[TB] nearest exp N=%0d", rd32(0));
        end
    endtask

    task read_ring(input string path);
        integer k;
        begin
            read_file(path);
            for (k = 0; k < N_in; k = k + 1) begin
                exp_hi[k] = rd32(4 + k * 28);
                exp_lo[k] = rd32(4 + k * 28 + 4);
                exp_thr[k] = rd32(4 + k * 28 + 8);
                exp_ntr[k] = rd32(4 + k * 28 + 12);
                exp_oe[k]  = rd32(4 + k * 28 + 16);
                exp_so[k]  = rd32(4 + k * 28 + 20);
                exp_ps[k]  = rd32(4 + k * 28 + 24);
            end
            $display("[TB] ring exp N=%0d", rd32(0));
        end
    endtask

    // 灰度灌入（按场景选 RAM：use_small=1 → u_gray_s，否则 u_gray_t）
    task load_gray(input string path, input integer W, input integer H, input integer use_small);
        integer k;
        begin
            read_file(path);
            for (k = 0; k < W * H; k = k + 1) begin
                gw_en = 1'b1;
                gw_addr = k[13:0];
                gw_data = fbuf[k];
                @(negedge clk);
            end
            gw_en = 1'b0;
            // 写目标由端口决定：调用前把 gw_en/gw_addr/gw_data 接到目标 RAM
            // —— 双口 RAM 写口独立，此处直接对两个 RAM 同写（数据相同）
            $display("[TB] gray loaded %0d bytes (use_small=%0d)", W * H, use_small);
        end
    endtask

    // 点灌入（写口 pw_en/pw_addr/pw_x/pw_y）
    task load_pts_ram();
        integer k;
        begin
            for (k = 0; k < N_in; k = k + 1) begin
                pw_en   = 1'b1;
                pw_addr = k[13:0];
                pw_x    = pts_x[k];
                pw_y    = pts_y[k];
                @(negedge clk);
            end
            pw_en = 1'b0;
        end
    endtask

    //--------------------------------------------------------------------
    // lround 独立自检
    //--------------------------------------------------------------------
    task lround_self_check();
        integer n, got, expv, lfail;
        begin
            fd = $fopen("../tests/build/vectors/lround_vec.mem", "r");
            if (fd == 0) begin $display("[TB][FATAL] cannot open lround_vec.mem"); $finish; end
            n = 0; lfail = 0;
            while (!$feof(fd)) begin
                got = 0; expv = 0;
                code = $fscanf(fd, "%h %h", got, expv);
                if (code == 2) begin
                    n = n + 1;
                    if ($unsigned(lround_pkg::lround_f32(got[31:0])) != $unsigned(expv[31:0])) begin
                        lfail = lfail + 1;
                        if (lfail <= 10)
                            $display("[TB][LROUND FAIL] %08x got %08x exp %08x",
                                     got, $unsigned(lround_pkg::lround_f32(got[31:0])), expv);
                    end
                end
            end
            $fclose(fd);
            if (lfail == 0)
                $display("[TB] lround self-check: %0d cases ALL PASSED", n);
            else
                $display("[TB][FAIL] lround self-check: %0d/%0d fail", lfail, n);
            all_pass = all_pass && (lfail == 0);
        end
    endtask

    //--------------------------------------------------------------------
    // nearest 一轮：start 并收集输出逐点比对
    //--------------------------------------------------------------------
    task run_nearest(input integer scene_id);
        integer k;
        begin
            ns_n     = N_in[15:0];
            ns_start = 1'b1;
            @(negedge clk);
            ns_start = 1'b0;
            cmp_n = 0; match_cnt = 0; fail_cnt = 0;
            while (ns_busy) begin
                @(negedge clk);
                if (ns_out_v && ns_out_rdy) begin
                    if (cmp_n < N_in && ns_sp == exp_sp[cmp_n] && ns_rad == exp_rad[cmp_n])
                        match_cnt = match_cnt + 1;
                    else begin
                        fail_cnt = fail_cnt + 1;
                        if (cmp_n < N_in) begin
                            if (fail_cnt <= 20)
                                $display("[TB][FAIL][s%0d] nearest idx=%0d got %08x/%08x exp %08x/%08x",
                                         scene_id, cmp_n, ns_sp, ns_rad,
                                         exp_sp[cmp_n], exp_rad[cmp_n]);
                        end else
                            $display("[TB][FAIL][s%0d] nearest extra got %08x/%08x", scene_id, ns_sp, ns_rad);
                    end
                    cmp_n = cmp_n + 1;
                end
            end
            if (cmp_n == N_in && fail_cnt == 0)
                $display("[TB][s%0d] nearest: N=%0d ALL PASSED", scene_id, N_in);
            else
                $display("[TB][FAIL][s%0d] nearest: cmp=%0d match=%0d fail=%0d", scene_id, cmp_n, match_cnt, fail_cnt);
            all_pass = all_pass && (cmp_n == N_in) && (fail_cnt == 0);
        end
    endtask

    //--------------------------------------------------------------------
    // ring 单点一次调用：喂 (x,y,r)，等输出，返回 pass；cmp_ok 输出 6 字段比对结果
    //   cur_inst：1=small 实例，2=texture 实例（只驱动活动实例）
    //--------------------------------------------------------------------
    integer cur_inst;

    task ring_call(input [31:0] x, y, r, output integer got_pass, output integer cmp_ok);
        begin
            if (cur_inst == 1) begin
                rc_in_v_s = 1'b1;
                rc_in_x   = x;
                rc_in_y   = y;
                rc_in_r   = r;
                @(negedge clk);
                rc_in_v_s = 1'b0;
                while (!rc_s_ov) @(negedge clk);
                got_pass = rc_s_ps;
                @(negedge clk);
                while (!rc_in_rdy_s) @(negedge clk);
            end else begin
                rc_in_v_t = 1'b1;
                rc_in_x   = x;
                rc_in_y   = y;
                rc_in_r   = r;
                @(negedge clk);
                rc_in_v_t = 1'b0;
                while (!rc_t_ov) @(negedge clk);
                got_pass = rc_t_ps;
                @(negedge clk);
                while (!rc_in_rdy_t) @(negedge clk);
            end
            cmp_ok = 1;
        end
    endtask

    // ring 全场景：逐点三连调 + 6 字段比对 + pass 合并比对（读活动实例信号）
    task run_ring(input integer scene_id);
        integer k, i;
        integer p0, p075, p125, ok6, pass_comb, okf;
        begin
            fail_cnt = 0;
            for (k = 0; k < N_in; k = k + 1) begin
                // d0：radius = nearest RTL 输出（= exp_rad[k] 位模式）
                ring_call(pts_x[k], pts_y[k], exp_rad[k], p0, ok6);
                if (cur_inst == 1) begin
                    if (rc_s_hi != exp_hi[k] || rc_s_lo != exp_lo[k] || rc_s_thr != exp_thr[k] ||
                        rc_s_ntr != exp_ntr[k] || rc_s_oe != exp_oe[k] || rc_s_so != exp_so[k]) begin
                        fail_cnt = fail_cnt + 1;
                        if (fail_cnt <= 20)
                            $display("[TB][FAIL][s%0d] ring idx=%0d got hi=%08x lo=%08x thr=%08x ntr=%08x oe=%08x so=%0d | exp %08x %08x %08x %08x %08x %0d",
                                     scene_id, k, rc_s_hi, rc_s_lo, rc_s_thr, rc_s_ntr, rc_s_oe, rc_s_so,
                                     exp_hi[k], exp_lo[k], exp_thr[k], exp_ntr[k], exp_oe[k], exp_so[k]);
                    end
                end else begin
                    if (rc_t_hi != exp_hi[k] || rc_t_lo != exp_lo[k] || rc_t_thr != exp_thr[k] ||
                        rc_t_ntr != exp_ntr[k] || rc_t_oe != exp_oe[k] || rc_t_so != exp_so[k]) begin
                        fail_cnt = fail_cnt + 1;
                        if (fail_cnt <= 20)
                            $display("[TB][FAIL][s%0d] ring idx=%0d got hi=%08x lo=%08x thr=%08x ntr=%08x oe=%08x so=%0d | exp %08x %08x %08x %08x %08x %0d",
                                     scene_id, k, rc_t_hi, rc_t_lo, rc_t_thr, rc_t_ntr, rc_t_oe, rc_t_so,
                                     exp_hi[k], exp_lo[k], exp_thr[k], exp_ntr[k], exp_oe[k], exp_so[k]);
                    end
                end
                // 0.75 / 1.25 半径（fp32_mul）
                m75_v = 1'b1; m125_v = 1'b1;
                @(negedge clk);
                m75_v = 1'b0; m125_v = 1'b0;
                while (!m75_ov || !m125_ov) @(negedge clk);
                ring_call(pts_x[k], pts_y[k], m75_r, p075, ok6);
                ring_call(pts_x[k], pts_y[k], m125_r, p125, ok6);
                pass_comb = p0 && (p075 || p125);
                if (pass_comb != exp_ps[k]) begin
                    fail_cnt = fail_cnt + 1;
                    if (fail_cnt <= 20)
                        $display("[TB][FAIL][s%0d] ring idx=%0d pass got=%0d exp=%0d (p0=%0d p075=%0d p125=%0d)",
                                 scene_id, k, pass_comb, exp_ps[k], p0, p075, p125);
                end
            end
            if (fail_cnt == 0)
                $display("[TB][s%0d] ring: N=%0d ALL PASSED", scene_id, N_in);
            else
                $display("[TB][FAIL][s%0d] ring: %0d point fails", scene_id, fail_cnt);
            all_pass = all_pass && (fail_cnt == 0);
        end
    endtask

    //--------------------------------------------------------------------
    // 主流程
    //--------------------------------------------------------------------
    integer run_small, run_texture;
    string  vecsel;

    initial begin
        pw_en = 1'b0; pw_addr = 14'd0; pw_x = 32'd0; pw_y = 32'd0;
        ns_start = 1'b0; ns_n = 16'd0; ns_out_rdy = 1'b1;
        gw_en = 1'b0; gw_addr = 14'd0; gw_data = 8'd0;
        rc_in_v_s = 1'b0; rc_in_v_t = 1'b0;
        rc_in_x = 32'd0; rc_in_y = 32'd0; rc_in_r = 32'd0;
        rc_out_rdy = 1'b1;
        m75_v = 1'b0; m125_v = 1'b0;
        cur_inst = 1;
        all_pass = 1;
        run_small = 1; run_texture = 1;
        // +VEC 可选：small / texture 单独跑（缺省全跑）
        if ($value$plusargs("VEC=%s", vecsel)) begin
            if (vecsel == "small")    begin run_small = 1; run_texture = 0; end
            else if (vecsel == "texture") begin run_small = 0; run_texture = 1; end
        end

        rst_n = 1'b0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        lround_self_check();

        // ================= scene: small =================
        if (run_small) begin
            $display("[TB] ================= scene: small (32x24) =================");
            read_pts("../tests/build/vectors/m3_small_merge3.bin");
            read_nearest("../tests/build/vectors/m3_small_nearest.bin");
            read_ring("../tests/build/vectors/m3_small_ring.bin");
            load_pts_ram();
            load_gray("../tests/build/vectors/m3_small_gray.bin", 32, 24, 1);
            run_nearest(1);
            cur_inst = 1;
            run_ring(1);
        end

        // ================= scene: texture =================
        if (run_texture) begin
            $display("[TB] ================= scene: texture (128x96) =================");
            read_pts("../tests/build/vectors/m3_texture_merge3.bin");
            read_nearest("../tests/build/vectors/m3_texture_nearest.bin");
            read_ring("../tests/build/vectors/m3_texture_ring.bin");
            load_pts_ram();
            load_gray("../tests/build/vectors/m3_texture_gray.bin", 128, 96, 0);
            run_nearest(2);
            cur_inst = 2;
            run_ring(2);
        end

        $display("[TB] ==============================================");
        if (all_pass)
            $display("TB RESULT: ALL PASSED");
        else
            $display("TB RESULT: SOME FAILED");
        $finish;
    end

    // 看门狗（500ms 仿真时间）
    initial begin
        #500_000_000;
        $display("[TB][FATAL] watchdog timeout");
        $finish;
    end

endmodule
