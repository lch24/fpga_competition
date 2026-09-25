`timescale 1ns / 1ps
//==============================================================================
// tb_filter.sv — M3 候选后处理全链位级对拍（candidate_filter_ctrl）
//------------------------------------------------------------------------------
// 流程（每场景一轮）：
//   1) 读 m3_<scene>_in.bin（fp32 坐标）→ 转整数（lround_pkg）→ NMS 候选流
//   2) 喂 candidate_filter_ctrl（start → cand 流 → cand_done）
//   3) 等 ctrl.done；层次引用内部 RAM 对拍：
//      · u_store.u_ram_b.mem[0..N5)  ↔ m3_<scene>_merge5.bin   （merge5 结果）
//      · u_store.u_ram_a.mem[0..N3)  ↔ m3_<scene>_merge3.bin   （merge3 结果）
//      · u_radram.mem[0..N3)          ↔ m3_<scene>_nearest.bin （radius 字段）
//   4) 实时探针：probe 流 ↔ m3_<scene>_ring.bin（每点 7 字段）；
//      inner 流  ↔ m3_<scene>_inner.bin
// 场景：small(32×24) / texture(128×96)。+VEC=small|texture 选一，缺省全跑。
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
    // 两个 ctrl 实例（场景参数化）+ 各自灰度 RAM
    //--------------------------------------------------------------------
    localparam GRAY_AW = 14;
    reg [7:0]  gray_mem_s [0:16383];
    reg [7:0]  gray_mem_t [0:16383];

    // small 的 gray 读口信号（先于 ctrl 例化声明）
    logic       s_gray_en;
    logic [GRAY_AW-1:0] s_gray_addr;
    logic [7:0] s_gray_q;
    // texture 的 gray 读口信号
    logic       t_gray_en;
    logic [GRAY_AW-1:0] t_gray_addr;
    logic [7:0] t_gray_q;

    // small 实例
    logic        s_start, s_cand_valid, s_cand_done;
    logic        s_cand_ready, s_busy, s_done;
    logic [1:0]  s_status;
    logic [10:0] s_cand_x, s_cand_y;
    logic        s_inner_valid;
    logic [31:0] s_inner_x, s_inner_y;
    logic [15:0] s_inner_total;
    logic        s_probe_valid;
    logic [31:0] s_probe_hi, s_probe_lo, s_probe_thr, s_probe_ntrans, s_probe_opp_err;
    logic        s_probe_sector_ok, s_probe_pass;

    candidate_filter_ctrl #(
        .IMG_W (32), .IMG_H (24), .GRAY_ADDR_W (GRAY_AW)
    ) u_ctrl_s (
        .clk (clk), .rst_n (rst_n),
        .start (s_start), .busy (s_busy), .done (s_done), .status (s_status),
        .cand_valid (s_cand_valid), .cand_ready (s_cand_ready),
        .cand_x (s_cand_x), .cand_y (s_cand_y), .cand_done (s_cand_done),
        .gray_rd_en (s_gray_en), .gray_rd_addr (s_gray_addr), .gray_rd_data (s_gray_q),
        .inner_valid (s_inner_valid), .inner_ready (1'b1),
        .inner_x (s_inner_x), .inner_y (s_inner_y), .inner_total (s_inner_total),
        .probe_valid (s_probe_valid),
        .probe_hi (s_probe_hi), .probe_lo (s_probe_lo), .probe_thr (s_probe_thr),
        .probe_ntrans (s_probe_ntrans), .probe_opp_err (s_probe_opp_err),
        .probe_sector_ok (s_probe_sector_ok), .probe_pass (s_probe_pass)
    );
    // gray 读口（TB 提供同步 RAM：rd_en=1 的下一拍出数据，同 dual_port_ram）
    always @(posedge clk) begin
        if (!rst_n) s_gray_q <= 8'd0;
        else if (s_gray_en) s_gray_q <= gray_mem_s[s_gray_addr];
    end

    // texture 实例
    logic        t_start, t_cand_valid, t_cand_done;
    logic        t_cand_ready, t_busy, t_done;
    logic [1:0]  t_status;
    logic [10:0] t_cand_x, t_cand_y;
    logic        t_inner_valid;
    logic [31:0] t_inner_x, t_inner_y;
    logic [15:0] t_inner_total;
    logic        t_probe_valid;
    logic [31:0] t_probe_hi, t_probe_lo, t_probe_thr, t_probe_ntrans, t_probe_opp_err;
    logic        t_probe_sector_ok, t_probe_pass;

    candidate_filter_ctrl #(
        .IMG_W (128), .IMG_H (96), .GRAY_ADDR_W (GRAY_AW)
    ) u_ctrl_t (
        .clk (clk), .rst_n (rst_n),
        .start (t_start), .busy (t_busy), .done (t_done), .status (t_status),
        .cand_valid (t_cand_valid), .cand_ready (t_cand_ready),
        .cand_x (t_cand_x), .cand_y (t_cand_y), .cand_done (t_cand_done),
        .gray_rd_en (t_gray_en), .gray_rd_addr (t_gray_addr), .gray_rd_data (t_gray_q),
        .inner_valid (t_inner_valid), .inner_ready (1'b1),
        .inner_x (t_inner_x), .inner_y (t_inner_y), .inner_total (t_inner_total),
        .probe_valid (t_probe_valid),
        .probe_hi (t_probe_hi), .probe_lo (t_probe_lo), .probe_thr (t_probe_thr),
        .probe_ntrans (t_probe_ntrans), .probe_opp_err (t_probe_opp_err),
        .probe_sector_ok (t_probe_sector_ok), .probe_pass (t_probe_pass)
    );
    always @(posedge clk) begin
        if (!rst_n) t_gray_q <= 8'd0;
        else if (t_gray_en) t_gray_q <= gray_mem_t[t_gray_addr];
    end

    //--------------------------------------------------------------------
    // 期望向量加载
    //--------------------------------------------------------------------
    integer N5, N3, NR, NI;
    reg [31:0] exp_merge5 [0:16383];
    reg [31:0] exp_merge3 [0:16383];
    reg [31:0] exp_near   [0:16383];   // {spacing, radius} 交错
    reg [31:0] exp_ring   [0:16383];   // 每点 7 字段交错
    reg [31:0] exp_inner  [0:16383];

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

    // 运行一个场景：输入候选（fp32 位模式）→ 期望 merge5/merge3/nearest/ring/inner
    integer fail_cnt = 0;

    task run_scene(
        input integer use_small,
        input string scene,
        input integer W,
        input integer H
    );
        integer i, k;
        integer n_in;
        integer nw5, nw3, nwn, nwr, nwi;
        reg [31:0] in_x [0:16383];
        reg [31:0] in_y [0:16383];
        integer ring_idx, inner_idx;
        reg [31:0] rv;
        reg [63:0] mem64;
        reg [31:0] xx, yy, sp, rd;
        integer probe_cnt, inner_cnt;

        begin
            // 1) 加载输入与期望（nw = 数据 word 数；点数按文件布局换算）
            load_pts_bin({"../tests/build/vectors/m3_", scene, "_in.bin"}, n_in, in_x, in_y);
            load_u32_file({"../tests/build/vectors/m3_", scene, "_merge5.bin"}, nw5, exp_merge5);
            N5 = nw5 / 2;
            load_u32_file({"../tests/build/vectors/m3_", scene, "_merge3.bin"}, nw3, exp_merge3);
            N3 = nw3 / 2;
            load_u32_file({"../tests/build/vectors/m3_", scene, "_nearest.bin"}, nwn, exp_near);
            NR = nwn / 2;
            load_u32_file({"../tests/build/vectors/m3_", scene, "_ring.bin"}, nwr, exp_ring);
            load_u32_file({"../tests/build/vectors/m3_", scene, "_inner.bin"}, nwi, exp_inner);
            NI = nwi / 2;
            // 2) 灰度灌入
            fd = $fopen({"../tests/build/vectors/m3_", scene, "_gray.bin"}, "rb");
            if (fd == 0) begin $display("[FATAL] no gray"); $finish; end
            code = $fread(bytes, fd);
            $fclose(fd);
            for (i = 0; i < W*H; ++i)
                if (use_small) gray_mem_s[i] = bytes[i];
                else           gray_mem_t[i] = bytes[i];

            $display("========== scene %s: N_in=%0d (w=%0d h=%0d) ==========", scene, n_in, W, H);

            // 3) 启动
            if (use_small) begin
                s_cand_valid <= 0; s_cand_done <= 0;
                s_start <= 1'b1;
                @(negedge clk);
                s_start <= 1'b0;
                while (!s_busy) @(posedge clk);
                // 4) 喂候选（整数坐标：fp32 位模式 → lround）
                for (i = 0; i < n_in; ++i) begin
                    s_cand_x <= lround_f32(in_x[i])[10:0];
                    s_cand_y <= lround_f32(in_y[i])[10:0];
                    s_cand_valid <= 1'b1;
                    do @(posedge clk); while (!s_cand_ready);
                    s_cand_valid <= 1'b0;
                end
                @(negedge clk);
                s_cand_done <= 1'b1;
                @(negedge clk);
                s_cand_done <= 1'b0;
                // 5) 等 done
                while (!s_done) @(posedge clk);
                @(negedge clk);
                // 6) 对拍 merge5（RAM B）
                for (i = 0; i < N5; ++i) begin
                    mem64 = u_ctrl_s.u_store.u_ram_b.mem[i];
                    xx = mem64[31:0]; yy = mem64[63:32];
                    if (xx !== exp_merge5[2*i] || yy !== exp_merge5[2*i+1]) begin
                        fail_cnt = fail_cnt + 1;
                        if (fail_cnt <= 10)
                            $display("[FAIL] merge5 #%0d rtl=(%08x,%08x) exp=(%08x,%08x)",
                                     i, xx, yy, exp_merge5[2*i], exp_merge5[2*i+1]);
                    end
                end
                // 7) 对拍 merge3（RAM A）
                for (i = 0; i < N3; ++i) begin
                    mem64 = u_ctrl_s.u_store.u_ram_a.mem[i];
                    xx = mem64[31:0]; yy = mem64[63:32];
                    if (xx !== exp_merge3[2*i] || yy !== exp_merge3[2*i+1]) begin
                        fail_cnt = fail_cnt + 1;
                        if (fail_cnt <= 10)
                            $display("[FAIL] merge3 #%0d rtl=(%08x,%08x) exp=(%08x,%08x)",
                                     i, xx, yy, exp_merge3[2*i], exp_merge3[2*i+1]);
                    end
                end
                // 8) 对拍 radius（radius RAM ↔ nearest.bin radius 字段）
                for (i = 0; i < N3; ++i) begin
                    rd = u_ctrl_s.u_radram.mem[i];
                    if (rd !== exp_near[2*i+1]) begin
                        fail_cnt = fail_cnt + 1;
                        if (fail_cnt <= 10)
                            $display("[FAIL] radius #%0d rtl=%08x exp=%08x", i, rd, exp_near[2*i+1]);
                    end
                end
                // 9) probe/inner 已在实时逻辑记录（见探针 always）
            end else begin
                t_cand_valid <= 0; t_cand_done <= 0;
                t_start <= 1'b1;
                @(negedge clk);
                t_start <= 1'b0;
                while (!t_busy) @(posedge clk);
                for (i = 0; i < n_in; ++i) begin
                    t_cand_x <= lround_f32(in_x[i])[10:0];
                    t_cand_y <= lround_f32(in_y[i])[10:0];
                    t_cand_valid <= 1'b1;
                    do @(posedge clk); while (!t_cand_ready);
                    t_cand_valid <= 1'b0;
                end
                @(negedge clk);
                t_cand_done <= 1'b1;
                @(negedge clk);
                t_cand_done <= 1'b0;
                while (!t_done) @(posedge clk);
                @(negedge clk);
                for (i = 0; i < N5; ++i) begin
                    mem64 = u_ctrl_t.u_store.u_ram_b.mem[i];
                    xx = mem64[31:0]; yy = mem64[63:32];
                    if (xx !== exp_merge5[2*i] || yy !== exp_merge5[2*i+1]) begin
                        fail_cnt = fail_cnt + 1;
                        if (fail_cnt <= 10)
                            $display("[FAIL] merge5 #%0d rtl=(%08x,%08x) exp=(%08x,%08x)",
                                     i, xx, yy, exp_merge5[2*i], exp_merge5[2*i+1]);
                    end
                end
                for (i = 0; i < N3; ++i) begin
                    mem64 = u_ctrl_t.u_store.u_ram_a.mem[i];
                    xx = mem64[31:0]; yy = mem64[63:32];
                    if (xx !== exp_merge3[2*i] || yy !== exp_merge3[2*i+1]) begin
                        fail_cnt = fail_cnt + 1;
                        if (fail_cnt <= 10)
                            $display("[FAIL] merge3 #%0d rtl=(%08x,%08x) exp=(%08x,%08x)",
                                     i, xx, yy, exp_merge3[2*i], exp_merge3[2*i+1]);
                    end
                end
                for (i = 0; i < N3; ++i) begin
                    rd = u_ctrl_t.u_radram.mem[i];
                    if (rd !== exp_near[2*i+1]) begin
                        fail_cnt = fail_cnt + 1;
                        if (fail_cnt <= 10)
                            $display("[FAIL] radius #%0d rtl=%08x exp=%08x", i, rd, exp_near[2*i+1]);
                    end
                end
            end

            // 汇总本场景 inner 数
            if (use_small) begin
                $display("[scene %s] status=%b inner_total=%0d (exp inner=%0d)", scene, s_status, s_inner_total, NI);
                $display("[diag] N_reg=%0d cnt_a=%0d cnt_b=%0d probe_s=%0d",
                         u_ctrl_s.N_reg, u_ctrl_s.u_store.cnt_a,
                         u_ctrl_s.u_store.cnt_b, p_s);
            end
            else
                $display("[scene %s] status=%b inner_total=%0d (exp inner=%0d)", scene, t_status, t_inner_total, NI);
        end
    endtask

    //--------------------------------------------------------------------
    // 实时探针（probe ↔ ring.bin；inner ↔ inner.bin）
    //--------------------------------------------------------------------
    integer p_s = 0, p_t = 0, in_s = 0, in_t = 0;

    // small 探针
    always @(posedge clk) begin
        if (s_probe_valid) begin
            if (p_s < NR && (
                s_probe_hi      !== exp_ring[7*p_s+0] ||
                s_probe_lo      !== exp_ring[7*p_s+1] ||
                s_probe_thr     !== exp_ring[7*p_s+2] ||
                s_probe_ntrans  !== exp_ring[7*p_s+3] ||
                s_probe_opp_err !== exp_ring[7*p_s+4] ||
                s_probe_sector_ok !== exp_ring[7*p_s+5] ||
                s_probe_pass    !== exp_ring[7*p_s+6])) begin
                fail_cnt = fail_cnt + 1;
                if (fail_cnt <= 20)
                    $display("[FAIL] ring_s #%0d rtl=(%08x,%08x,%08x,%08x,%08x,%b,%b) exp=(%08x,%08x,%08x,%08x,%08x,%b,%b)",
                             p_s, s_probe_hi, s_probe_lo, s_probe_thr, s_probe_ntrans, s_probe_opp_err,
                             s_probe_sector_ok, s_probe_pass,
                             exp_ring[7*p_s+0], exp_ring[7*p_s+1], exp_ring[7*p_s+2],
                             exp_ring[7*p_s+3], exp_ring[7*p_s+4],
                             exp_ring[7*p_s+5], exp_ring[7*p_s+6]);
            end
            p_s = p_s + 1;
        end
    end
    always @(posedge clk) begin
        if (s_inner_valid) begin
            if (in_s < NI && (s_inner_x !== exp_inner[2*in_s] || s_inner_y !== exp_inner[2*in_s+1])) begin
                fail_cnt = fail_cnt + 1;
                if (fail_cnt <= 20)
                    $display("[FAIL] inner_s #%0d rtl=(%08x,%08x) exp=(%08x,%08x)",
                             in_s, s_inner_x, s_inner_y, exp_inner[2*in_s], exp_inner[2*in_s+1]);
            end
            in_s = in_s + 1;
        end
    end

    // texture 探针
    always @(posedge clk) begin
        if (t_probe_valid) begin
            if (p_t < NR && (
                t_probe_hi      !== exp_ring[7*p_t+0] ||
                t_probe_lo      !== exp_ring[7*p_t+1] ||
                t_probe_thr     !== exp_ring[7*p_t+2] ||
                t_probe_ntrans  !== exp_ring[7*p_t+3] ||
                t_probe_opp_err !== exp_ring[7*p_t+4] ||
                t_probe_sector_ok !== exp_ring[7*p_t+5] ||
                t_probe_pass    !== exp_ring[7*p_t+6])) begin
                fail_cnt = fail_cnt + 1;
                if (fail_cnt <= 20)
                    $display("[FAIL] ring_t #%0d rtl=(%08x,%08x,%08x,%08x,%08x,%b,%b) exp=(%08x,%08x,%08x,%08x,%08x,%b,%b)",
                             p_t, t_probe_hi, t_probe_lo, t_probe_thr, t_probe_ntrans, t_probe_opp_err,
                             t_probe_sector_ok, t_probe_pass,
                             exp_ring[7*p_t+0], exp_ring[7*p_t+1], exp_ring[7*p_t+2],
                             exp_ring[7*p_t+3], exp_ring[7*p_t+4],
                             exp_ring[7*p_t+5], exp_ring[7*p_t+6]);
            end
            p_t = p_t + 1;
        end
    end
    always @(posedge clk) begin
        if (t_inner_valid) begin
            if (in_t < NI && (t_inner_x !== exp_inner[2*in_t] || t_inner_y !== exp_inner[2*in_t+1])) begin
                fail_cnt = fail_cnt + 1;
                if (fail_cnt <= 20)
                    $display("[FAIL] inner_t #%0d rtl=(%08x,%08x) exp=(%08x,%08x)",
                             in_t, t_inner_x, t_inner_y, exp_inner[2*in_t], exp_inner[2*in_t+1]);
            end
            in_t = in_t + 1;
        end
    end

    //--------------------------------------------------------------------
    // 主流程
    //--------------------------------------------------------------------
    string vecsel = "both";
    initial begin
        if ($value$plusargs("VEC=%s", vecsel)) ;
        rst_n = 1'b0;
        s_start = 0; s_cand_valid = 0; s_cand_done = 0;
        t_start = 0; t_cand_valid = 0; t_cand_done = 0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        if (vecsel == "small" || vecsel == "both") begin
            p_s = 0; in_s = 0;
            run_scene(1, "small", 32, 24);
            $display("[scene small] probe %0d ring_points / inner %0d", p_s, in_s);
        end
        if (vecsel == "texture" || vecsel == "both") begin
            p_t = 0; in_t = 0;
            run_scene(0, "texture", 128, 96);
            $display("[scene texture] probe %0d ring_points / inner %0d", p_t, in_t);
        end

        $display("======================================");
        $display("FILTER CTRL TB: fail=%0d", fail_cnt);
        if (fail_cnt == 0)
            $display("TB RESULT: ALL FILTER TESTS PASSED");
        else
            $display("TB RESULT: FAILED");
        $finish;
    end

endmodule
