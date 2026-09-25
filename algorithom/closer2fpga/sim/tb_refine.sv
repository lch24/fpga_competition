`timescale 1ns / 1ps
//==============================================================================
// tb_refine.sv — M5.2 grid_refine_ctrl 全链位级对拍（refine_grid_ref）
//------------------------------------------------------------------------------
// 向量（tests/build/vectors/m5_board5x8_*，export_m5.cpp export_fullchain）：
//   m5_board5x8_grid.bin    u32 ok + u32 N + N×{x,y} fp32（organize 输出 = 输入）
//   m5_board5x8_refined.bin u32 valid + u32 N + N×{x,y} fp32（期望输出）
//   m5_board5x8_gray.bin    96×272 灰度（W=272 H=96，行主序）
// 流程：复位 → 灌点 RAM（1 拍读，grid 40 点）→ 灌灰度 → 读 refined 期望 →
//   start → 并行收 out 流（out_valid/out_ready）→ 等 done → 校验
//   valid_out/out_valid_flag + 40 点 {x,y} 逐点位级比对。
// 要求：valid=1 + 40/40 位级一致 → ALL PASSED。
//==============================================================================
module tb_refine;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst_n;

    //--------------------------------------------------------------------
    // board5x8 场景参数（272×96 棋盘格：W=17格×16，H=6格×16）
    //--------------------------------------------------------------------
    localparam W = 272, H = 96;
    localparam GRAY_AW = 15;     // ≥ $clog2(272*96)=15
    localparam N_AW    = 8;

    //--------------------------------------------------------------------
    // DUT 信号
    //--------------------------------------------------------------------
    reg        start;
    wire       busy, done, valid_out;
    wire       pt_rd_en;
    wire [N_AW-1:0] pt_rd_addr;
    reg  [31:0] pt_rd_x, pt_rd_y;
    wire       gray_rd_en;
    wire [GRAY_AW-1:0] gray_rd_addr;
    reg  [7:0]  gray_rd_data;
    wire       out_valid;
    reg        out_ready;
    wire [31:0] out_x, out_y;
    wire       out_valid_flag;

    grid_refine_ctrl #(
        .ROWS(5), .COLS(8), .N_ADDR_W(N_AW),
        .IMG_W(W), .IMG_H(H), .GRAY_ADDR_W(GRAY_AW),
        .ROM_FILE("../tests/build/vectors/gaussian_weights.mem")
    ) u_dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .busy(busy), .done(done), .valid_out(valid_out),
        .pt_rd_en(pt_rd_en), .pt_rd_addr(pt_rd_addr),
        .pt_rd_x(pt_rd_x), .pt_rd_y(pt_rd_y),
        .gray_rd_en(gray_rd_en), .gray_rd_addr(gray_rd_addr),
        .gray_rd_data(gray_rd_data),
        .out_valid(out_valid), .out_ready(out_ready),
        .out_x(out_x), .out_y(out_y), .out_valid_flag(out_valid_flag)
    );

    //--------------------------------------------------------------------
    // 点内存（32 位 ×2，N_AW=8，1 拍读延迟）
    //--------------------------------------------------------------------
    reg        pw_en;
    reg [N_AW-1:0] pw_addr;
    reg [31:0] pw_x, pw_y;

    dual_port_ram #(.DATA_WIDTH(32), .ADDR_WIDTH(N_AW)) u_ptx (
        .clk(clk), .rst_n(rst_n),
        .wr_en(pw_en), .wr_addr(pw_addr), .wr_data(pw_x),
        .rd_en(pt_rd_en), .rd_addr(pt_rd_addr), .rd_data(pt_rd_x)
    );
    dual_port_ram #(.DATA_WIDTH(32), .ADDR_WIDTH(N_AW)) u_pty (
        .clk(clk), .rst_n(rst_n),
        .wr_en(pw_en), .wr_addr(pw_addr), .wr_data(pw_y),
        .rd_en(pt_rd_en), .rd_addr(pt_rd_addr), .rd_data(pt_rd_y)
    );

    //--------------------------------------------------------------------
    // 灰度内存（8 位，272×96 = 26112 项）
    //--------------------------------------------------------------------
    reg        gw_en;
    reg [GRAY_AW-1:0] gw_addr;
    reg [7:0]  gw_data;

    dual_port_ram #(.DATA_WIDTH(8), .ADDR_WIDTH(GRAY_AW)) u_gray (
        .clk(clk), .rst_n(rst_n),
        .wr_en(gw_en), .wr_addr(gw_addr), .wr_data(gw_data),
        .rd_en(gray_rd_en), .rd_addr(gray_rd_addr), .rd_data(gray_rd_data)
    );

    //--------------------------------------------------------------------
    // 输出收集器（out 流在 done 之前逐点流完，须并行收集）
    //--------------------------------------------------------------------
    reg        collecting;
    reg [31:0] got_x[0:255], got_y[0:255];
    integer    got_cnt;
    reg        flag_seen;

    always @(posedge clk) begin
        if (collecting && out_valid && out_ready) begin
            got_x[got_cnt] <= out_x;
            got_y[got_cnt] <= out_y;
            got_cnt <= got_cnt + 1;
        end
        if (collecting && out_valid)
            flag_seen <= 1'b1;
    end

    //--------------------------------------------------------------------
    // 向量缓冲（小端）
    //--------------------------------------------------------------------
    reg [7:0]  fbuf[0:1048575];
    integer    N_in, N_out, ok_exp, k, fd, code, fail_cnt;
    reg [31:0] grid_x[0:255], grid_y[0:255];
    reg [31:0] exp_x[0:255], exp_y[0:255];

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

    task load_pts_ram();
        integer k;
        begin
            for (k = 0; k < N_in; k = k + 1) begin
                pw_en   = 1'b1;
                pw_addr = k[N_AW-1:0];
                pw_x    = grid_x[k];
                pw_y    = grid_y[k];
                @(negedge clk);
            end
            pw_en = 1'b0;
            @(negedge clk);
        end
    endtask

    task load_gray();
        integer k;
        begin
            read_file("../tests/build/vectors/m5_board5x8_gray.bin");
            for (k = 0; k < W*H; k = k + 1) begin
                gw_en   = 1'b1;
                gw_addr = k[GRAY_AW-1:0];
                gw_data = fbuf[k];
                @(negedge clk);
            end
            gw_en = 1'b0;
            @(negedge clk);
        end
    endtask

    //--------------------------------------------------------------------
    // 主流程
    //--------------------------------------------------------------------
    initial begin
        start = 1'b0; out_ready = 1'b1;
        collecting = 1'b0; got_cnt = 0; flag_seen = 1'b0;
        pw_en = 1'b0; pw_addr = 8'd0; pw_x = 32'd0; pw_y = 32'd0;
        gw_en = 1'b0; gw_addr = 15'd0; gw_data = 8'd0;
        fail_cnt = 0;

        rst_n = 1'b0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // 输入 grid（organize 输出 40 点，ok 应为 1）
        read_file("../tests/build/vectors/m5_board5x8_grid.bin");
        ok_exp = rd32(0);
        N_in   = rd32(4);
        for (k = 0; k < N_in; k = k + 1) begin
            grid_x[k] = rd32(8 + k * 8);
            grid_y[k] = rd32(8 + k * 8 + 4);
        end
        $display("[TB] grid ok=%0d N=%0d", ok_exp, N_in);
        if (ok_exp != 1) begin
            $display("[TB][FATAL] expected ok=1"); $finish;
        end
        load_pts_ram();

        // 灰度
        load_gray();

        // 期望输出（refined）
        read_file("../tests/build/vectors/m5_board5x8_refined.bin");
        N_out = rd32(4);
        for (k = 0; k < N_out; k = k + 1) begin
            exp_x[k] = rd32(8 + k * 8);
            exp_y[k] = rd32(8 + k * 8 + 4);
        end
        $display("[TB] refined valid=%0d N=%0d", rd32(0), N_out);

        // start + 并行收集
        got_cnt = 0;
        flag_seen = 1'b0;
        collecting = 1'b1;
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        $display("[TB] start ...");

        while (!done) @(negedge clk);
        collecting = 1'b0;
        @(negedge clk);

        // 校验 valid 标志 + 40 点流
        if (valid_out !== 1'b1) begin
            fail_cnt = fail_cnt + 1;
            $display("[TB][FAIL] valid_out=%0d exp=1", valid_out);
        end
        if (!flag_seen) begin
            fail_cnt = fail_cnt + 1;
            $display("[TB][FAIL] out_valid_flag not seen");
        end
        if (got_cnt != N_out) begin
            fail_cnt = fail_cnt + 1;
            $display("[TB][FAIL] collected %0d / expected %0d points", got_cnt, N_out);
        end
        for (k = 0; k < N_out; k = k + 1) begin
            if (got_x[k] !== exp_x[k] || got_y[k] !== exp_y[k]) begin
                fail_cnt = fail_cnt + 1;
                $display("[TB][FAIL][%0d] got %08x,%08x exp %08x,%08x", k,
                         got_x[k], got_y[k], exp_x[k], exp_y[k]);
            end else begin
                $display("[TB][PASS][%0d] x=%08x y=%08x", k, got_x[k], got_y[k]);
            end
        end

        $display("======================================");
        if (fail_cnt == 0)
            $display("TB RESULT: refine ALL PASSED (valid=1, 40/40 bit-exact)");
        else
            $display("TB RESULT: refine FAIL %0d", fail_cnt);
        $finish;
    end

    // 看门狗（2000s 仿真上限；subpixel 迭代量大）
    initial begin
        #2000000000;
        $display("[TB][FATAL] watchdog timeout");
        $finish;
    end

endmodule
