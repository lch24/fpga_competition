`timescale 1ns / 1ps
//==============================================================================
// tb_subpixel.sv — M5 subpixel_ctrl 全链位级对拍（3 场景 s7/s2/s15）
//------------------------------------------------------------------------------
// 向量（tests/build/vectors/）：
//   m5_<s>_pts.bin  = u32 N + N×{x,y}(fp32 位模式)      → 点读口内存
//   m5_<s>_gray.bin = W*H = 128×128 字节灰度             → 灰度读口内存
//   m5_<s>_out.bin  = u32 N + N×{x,y,reliable}(fp32/标量) → 期望输出
// 流程（每场景）：读点→灌点 RAM；读灰度→灌灰度 RAM；读期望；start(n_in, half_win)
//   → 并行收集 out 流（out_valid 在 done 之前逐点流完）→ 等 done → 逐点
//   {x,y,reliable} 位级比对，打印逐点 PASS/FAIL。
// 要求：23 点 × 3 场景全部位级一致 → ALL PASSED。
// 点/灰度读口均为 1 拍延迟（dual_port_ram 模板，与 ring_check 同款）。
// 看门狗设长（r=15 场景仿真量大，同 tb_order 的 2e12 ns 写法）。
//==============================================================================
module tb_subpixel;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst_n;

    //--------------------------------------------------------------------
    // DUT 信号
    //--------------------------------------------------------------------
    reg        start;
    wire       busy, done;
    reg  [15:0] n_in;
    reg  [7:0]  half_win;
    wire       pt_rd_en;
    wire [7:0] pt_rd_addr;
    reg  [31:0] pt_rd_x, pt_rd_y;
    wire       gray_rd_en;
    wire [15:0] gray_rd_addr;
    reg  [7:0]  gray_rd_data;
    wire       out_valid;
    reg        out_ready;
    wire [31:0] out_x, out_y;
    wire        out_reliable;

    subpixel_ctrl #(.IMG_W(128), .IMG_H(128), .GRAY_ADDR_W(16), .N_ADDR_W(8),
                    .PATCH_HW(16),
                    .ROM_FILE("../tests/build/vectors/gaussian_weights.mem")) u_dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .busy(busy), .done(done),
        .n_in(n_in), .half_win(half_win),
        .pt_rd_en(pt_rd_en), .pt_rd_addr(pt_rd_addr),
        .pt_rd_x(pt_rd_x), .pt_rd_y(pt_rd_y),
        .gray_rd_en(gray_rd_en), .gray_rd_addr(gray_rd_addr),
        .gray_rd_data(gray_rd_data),
        .out_valid(out_valid), .out_ready(out_ready),
        .out_x(out_x), .out_y(out_y), .out_reliable(out_reliable)
    );

    //--------------------------------------------------------------------
    // 点内存（32 位 ×2，N_ADDR_W=8）
    //--------------------------------------------------------------------
    reg        pw_en;
    reg [7:0]  pw_addr;
    reg [31:0] pw_x, pw_y;

    dual_port_ram #(.DATA_WIDTH(32), .ADDR_WIDTH(8)) u_ptx (
        .clk(clk), .rst_n(rst_n),
        .wr_en(pw_en), .wr_addr(pw_addr), .wr_data(pw_x),
        .rd_en(pt_rd_en), .rd_addr(pt_rd_addr), .rd_data(pt_rd_x)
    );
    dual_port_ram #(.DATA_WIDTH(32), .ADDR_WIDTH(8)) u_pty (
        .clk(clk), .rst_n(rst_n),
        .wr_en(pw_en), .wr_addr(pw_addr), .wr_data(pw_y),
        .rd_en(pt_rd_en), .rd_addr(pt_rd_addr), .rd_data(pt_rd_y)
    );

    //--------------------------------------------------------------------
    // 灰度内存（8 位，128×128 = 16384 项）
    //--------------------------------------------------------------------
    reg        gw_en;
    reg [13:0] gw_addr;
    reg [7:0]  gw_data;

    dual_port_ram #(.DATA_WIDTH(8), .ADDR_WIDTH(14)) u_gray (
        .clk(clk), .rst_n(rst_n),
        .wr_en(gw_en), .wr_addr(gw_addr), .wr_data(gw_data),
        .rd_en(gray_rd_en), .rd_addr(gray_rd_addr[13:0]), .rd_data(gray_rd_data)
    );

    //--------------------------------------------------------------------
    // 输出收集器（out 流在 done 之前逐点流完，须并行收集）
    //--------------------------------------------------------------------
    reg        collecting;
    reg [31:0] got_x[0:255], got_y[0:255];
    reg        got_rel[0:255];
    integer    got_cnt;

    always @(posedge clk) begin
        if (collecting && out_valid && out_ready) begin
            got_x[got_cnt]   <= out_x;
            got_y[got_cnt]   <= out_y;
            got_rel[got_cnt] <= out_reliable;
            got_cnt <= got_cnt + 1;
        end
    end

    //--------------------------------------------------------------------
    // 向量缓冲（小端）
    //--------------------------------------------------------------------
    reg [7:0]  fbuf[0:1048575];
    integer    N_in;
    reg [31:0] pts_x[0:255], pts_y[0:255];
    reg [31:0] exp_x[0:255], exp_y[0:255];
    reg        exp_rel[0:255];

    integer    fd, code, k;
    integer    scene_pass_cnt, scene_fail_cnt, all_pass;

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

    // 点灌入（写口 pw_en/pw_addr/pw_x/pw_y）
    task load_pts_ram();
        integer k;
        begin
            for (k = 0; k < N_in; k = k + 1) begin
                pw_en   = 1'b1;
                pw_addr = k[7:0];
                pw_x    = pts_x[k];
                pw_y    = pts_y[k];
                @(negedge clk);
            end
            pw_en = 1'b0;
            @(negedge clk);
        end
    endtask

    // 灰度灌入（128×128）
    task load_gray(input string path);
        integer k;
        begin
            read_file(path);
            for (k = 0; k < 128 * 128; k = k + 1) begin
                gw_en   = 1'b1;
                gw_addr = k[13:0];
                gw_data = fbuf[k];
                @(negedge clk);
            end
            gw_en = 1'b0;
            @(negedge clk);
        end
    endtask

    //--------------------------------------------------------------------
    // 单场景：读向量 → 灌内存 → start → 等 done → 收流比对
    //--------------------------------------------------------------------
    task run_scene(input string name, input integer hw);
        integer k, pfail;
        begin
            // 输入点
            read_file({"../tests/build/vectors/m5_", name, "_pts.bin"});
            N_in = rd32(0);
            for (k = 0; k < N_in; k = k + 1) begin
                pts_x[k] = rd32(4 + k * 8);
                pts_y[k] = rd32(4 + k * 8 + 4);
            end
            $display("[TB][%s] pts N=%0d", name, N_in);
            load_pts_ram();

            // 灰度
            load_gray({"../tests/build/vectors/m5_", name, "_gray.bin"});

            // 期望输出
            read_file({"../tests/build/vectors/m5_", name, "_out.bin"});
            for (k = 0; k < N_in; k = k + 1) begin
                exp_x[k]   = rd32(4 + k * 12);
                exp_y[k]   = rd32(4 + k * 12 + 4);
                exp_rel[k] = rd32(4 + k * 12 + 8) != 0;
            end
            $display("[TB][%s] out exp N=%0d", name, rd32(0));

            // start + 并行收集
            got_cnt = 0;
            collecting = 1'b1;
            n_in     = N_in[15:0];
            half_win = hw[7:0];
            start    = 1'b1;
            @(negedge clk);
            start = 1'b0;
            $display("[TB][%s] start (half_win=%0d) ...", name, hw);

            while (!done) @(negedge clk);
            collecting = 1'b0;
            @(negedge clk);

            // 逐点比对
            pfail = 0;
            for (k = 0; k < N_in; k = k + 1) begin
                if (got_x[k] !== exp_x[k] || got_y[k] !== exp_y[k] ||
                    got_rel[k] !== exp_rel[k]) begin
                    pfail = pfail + 1;
                    $display("[TB][FAIL][%s] pt%0d got %08x/%08x rel=%0d exp %08x/%08x rel=%0d",
                             name, k, got_x[k], got_y[k], got_rel[k],
                             exp_x[k], exp_y[k], exp_rel[k]);
                end else begin
                    $display("[TB][PASS][%s] pt%0d x=%08x y=%08x rel=%0d",
                             name, k, got_x[k], got_y[k], got_rel[k]);
                end
            end
            if (got_cnt != N_in)
                $display("[TB][FAIL][%s] collected %0d / expected %0d points", name, got_cnt, N_in);

            if (pfail == 0 && got_cnt == N_in)
                $display("[TB][%s] N=%0d ALL PASSED", name, N_in);
            else
                $display("[TB][FAIL][%s] %0d/%0d point fails", name, pfail, N_in);
            scene_pass_cnt = scene_pass_cnt + ((pfail == 0 && got_cnt == N_in) ? 1 : 0);
            scene_fail_cnt = scene_fail_cnt + ((pfail == 0 && got_cnt == N_in) ? 0 : 1);
            all_pass = all_pass && (pfail == 0) && (got_cnt == N_in);
        end
    endtask

    //--------------------------------------------------------------------
    // 主流程
    //--------------------------------------------------------------------
    integer run_s7, run_s2, run_s15;
    string  vecsel;

    initial begin
        start = 1'b0; n_in = 16'd0; half_win = 8'd0;
        out_ready = 1'b1;
        collecting = 1'b0; got_cnt = 0;
        pw_en = 1'b0; pw_addr = 8'd0; pw_x = 32'd0; pw_y = 32'd0;
        gw_en = 1'b0; gw_addr = 14'd0; gw_data = 8'd0;
        scene_pass_cnt = 0; scene_fail_cnt = 0; all_pass = 1;
        run_s7 = 1; run_s2 = 1; run_s15 = 1;
        if ($value$plusargs("SCENE=%s", vecsel)) begin
            run_s7 = 0; run_s2 = 0; run_s15 = 0;
            if (vecsel == "s7")   run_s7 = 1;
            else if (vecsel == "s2")  run_s2 = 1;
            else if (vecsel == "s15") run_s15 = 1;
        end

        rst_n = 1'b0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // 3 场景：s7 / s2 / s15
        if (run_s7)  run_scene("s7", 7);
        if (run_s2)  run_scene("s2", 2);
        if (run_s15) run_scene("s15", 15);

        $display("======================================");
        $display("[TB] scenes passed=%0d failed=%0d", scene_pass_cnt, scene_fail_cnt);
        if (all_pass)
            $display("TB RESULT: ALL PASSED");
        else
            $display("TB RESULT: SOME FAILED");
        $finish;
    end

    // 看门狗（2e12 ns = 2000s 仿真上限；r=15 场景量大）
    initial begin
        #(64'd2_000_000_000_000);
        $display("[TB][FATAL] watchdog timeout");
        $finish;
    end

endmodule
