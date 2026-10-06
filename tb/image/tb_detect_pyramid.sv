`timescale 1ns / 1ps
//==============================================================================
// tb_detect_pyramid.sv — M6.2 集成：pyramid_ctrl → detect_ctrl 端到端全链对拍
//------------------------------------------------------------------------------
// 场景：
//   1) big（1280×720，金字塔路径）：
//      · 预载 L0 灰度 m6_big_gray.bin @base0
//      · pyramid_ctrl(1280×720, base0=0) → 生成 L1(640×360) @921600，与
//        m6_down_big.bin 期望段逐字节比对（覆盖 downsample 全链）
//      · detect_ctrl(W0=1280,H0=720,DEPTH=2) → 40 点 vs m6_chain_big.bin
//        金字塔路径：native@L1 → 2p+0.5 → refine@L0
//   2) board5x8（272×96，native 路径）：
//      · 预载 m5_board5x8_gray.bin @base0
//      · detect_ctrl(W0=272,H0=96,DEPTH=1) → 40 点 vs m6_chain_board5x8.bin
//
// 权威：tests/rtl/export_m6.cpp（detect_chessboard_ref 确定性变体）。
// 向量（只读）：tests/build/vectors/{m6_big_gray,m6_down_big,m6_chain_big,
//              m6_chain_board5x8}.bin、tests/build/vectors/m5_board5x8_gray.bin
//==============================================================================
module tb_detect_pyramid;

    localparam integer CLK_PERIOD = 10;
    logic clk = 1'b0;
    logic rst_n;
    always #(CLK_PERIOD/2) clk = ~clk;

    integer fd, code;
    reg [7:0]  fbuf [0:4194303];           // 4MB 向量缓冲（big L0 921600 + L1 230400）
    reg [31:0] wbuf [0:1048575];

    function automatic [31:0] le32(input integer i);
        le32 = {fbuf[i+3], fbuf[i+2], fbuf[i+1], fbuf[i]};
    endfunction

    //--------------------------------------------------------------------
    // 灰度 RAM（同步：写使能拍更新，读使能下一拍出数据，read-first）
    //--------------------------------------------------------------------
    localparam integer GRAY_AW = 21;       // ≥ $clog2(1152000)
    reg [7:0]  gray_mem [0:(1<<GRAY_AW)-1];
    logic            gray_rd_en;
    logic [GRAY_AW-1:0] gray_rd_addr;
    logic [7:0]       gray_rd_data;
    wire             gray_wr_en;            // pyramid 组合驱动
    wire [GRAY_AW-1:0] gray_wr_addr;
    wire [7:0]       gray_wr_data;

    always @(posedge clk) begin
        if (!rst_n) gray_rd_data <= 8'd0;
        else if (gray_rd_en) gray_rd_data <= gray_mem[gray_rd_addr];
    end
    always @(posedge clk) begin
        if (gray_wr_en) gray_mem[gray_wr_addr] <= gray_wr_data;
    end

    //--------------------------------------------------------------------
    // pyramid_ctrl（big 场景：1280×720 → L1 640×360）
    //--------------------------------------------------------------------
    logic        pyr_start, pyr_busy, pyr_done;
    logic [2:0]  pyr_level_count;          // $clog2(MAX_DEPTH=4)+1 = 3 位
    logic [20:0] pyr_lvl_base [0:3];
    logic [10:0] pyr_lvl_w    [0:3];
    logic [10:0] pyr_lvl_h    [0:3];

    //--------------------------------------------------------------------
    // detect_ctrl（两个实例：big DEPTH=2 / board5x8 DEPTH=1）
    //--------------------------------------------------------------------
    logic        det_start, det_busy, det_done;
    logic [1:0]  det_status;
    logic        det_out_valid, det_out_ready, det_out_grid_ok;
    logic [31:0] det_out_x, det_out_y;
    logic [15:0] det_out_total;

    logic        det2_start, det2_busy, det2_done;
    logic [1:0]  det2_status;
    logic        det2_out_valid, det2_out_ready, det2_out_grid_ok;
    logic [31:0] det2_out_x, det2_out_y;
    logic [15:0] det2_out_total;

    // gray 读口仲裁：pyramid 阶段归 pyramid，detect 阶段归 detect_ctrl
    logic pyr_gray_en;
    logic [20:0] pyr_gray_addr;
    logic [7:0]  pyr_gray_q;

    pyramid_ctrl #(
        .MAX_W (2560), .MAX_H (1440), .MAX_DEPTH (4), .GRAY_ADDR_W (21)
    ) u_pyr (
        .clk (clk), .rst_n (rst_n),
        .start (pyr_start), .busy (pyr_busy), .done (pyr_done),
        .cfg_w0 (11'd1280), .cfg_h0 (11'd720), .cfg_base0 (21'd0),
        .gray_rd_en (pyr_gray_en), .gray_rd_addr (pyr_gray_addr),
        .gray_rd_data (gray_rd_data),
        .gray_wr_en (gray_wr_en), .gray_wr_addr (gray_wr_addr),
        .gray_wr_data (gray_wr_data),
        .level_count (pyr_level_count),
        .level_base (pyr_lvl_base), .level_w (pyr_lvl_w), .level_h (pyr_lvl_h)
    );

    logic det_gray_en;
    logic [20:0] det_gray_addr;
    logic det2_gray_en;
    logic [20:0] det2_gray_addr;

    detect_ctrl #(
        .W0 (1280), .H0 (720), .DEPTH (2), .GRAY_ADDR_W (21)
    ) u_det (
        .clk (clk), .rst_n (rst_n),
        .start (det_start), .busy (det_busy), .done (det_done), .status (det_status),
        .cfg_base0 (21'd0),
        .gray_rd_en (det_gray_en), .gray_rd_addr (det_gray_addr),
        .gray_rd_data (gray_rd_data),
        .out_valid (det_out_valid), .out_ready (det_out_ready),
        .out_x (det_out_x), .out_y (det_out_y),
        .out_total (det_out_total), .out_grid_ok (det_out_grid_ok)
    );

    detect_ctrl #(
        .W0 (272), .H0 (96), .DEPTH (1), .GRAY_ADDR_W (21)
    ) u_det2 (
        .clk (clk), .rst_n (rst_n),
        .start (det2_start), .busy (det2_busy), .done (det2_done), .status (det2_status),
        .cfg_base0 (21'd0),
        .gray_rd_en (det2_gray_en), .gray_rd_addr (det2_gray_addr),
        .gray_rd_data (gray_rd_data),
        .out_valid (det2_out_valid), .out_ready (det2_out_ready),
        .out_x (det2_out_x), .out_y (det2_out_y),
        .out_total (det2_out_total), .out_grid_ok (det2_out_grid_ok)
    );

    // 灰度读口复用：detect 阶段只允许一个实例活动
    reg pyr_active, det_active, det2_active;
    assign gray_rd_en   = pyr_active ? pyr_gray_en : det_active ? det_gray_en : det2_active ? det2_gray_en : 1'b0;
    assign gray_rd_addr = pyr_active ? pyr_gray_addr : det_active ? det_gray_addr : det2_gray_addr;

    //--------------------------------------------------------------------
    // 期望向量
    //--------------------------------------------------------------------
    reg [31:0] exp_chain [0:1023];         // 每场景 2 + 40*2 = 82 词
    integer    exp_N, exp_valid;
    integer    err_cnt = 0;

    //--------------------------------------------------------------------
    // 输出采集（两实例共用逻辑：按活动实例切换）
    //--------------------------------------------------------------------
    integer oi = 0, oi2 = 0;

    always @(posedge clk) begin
        if (det_out_valid && det_out_ready && det_active) begin
            if (oi >= exp_N || det_out_x !== exp_chain[2 + oi*2] ||
                det_out_y !== exp_chain[3 + oi*2]) begin
                err_cnt = err_cnt + 1;
                if (err_cnt <= 10)
                    $display("[FAIL][big] pt%0d got=(%08x,%08x) exp=(%08x,%08x)",
                             oi, det_out_x, det_out_y, exp_chain[2+oi*2], exp_chain[3+oi*2]);
            end
            oi = oi + 1;
        end
        if (det2_out_valid && det2_out_ready && det2_active) begin
            if (oi2 >= exp_N || det2_out_x !== exp_chain[2 + oi2*2] ||
                det2_out_y !== exp_chain[3 + oi2*2]) begin
                err_cnt = err_cnt + 1;
                if (err_cnt <= 10)
                    $display("[FAIL][board5x8] pt%0d got=(%08x,%08x) exp=(%08x,%08x)",
                             oi2, det2_out_x, det2_out_y, exp_chain[2+oi2*2], exp_chain[3+oi2*2]);
            end
            oi2 = oi2 + 1;
        end
    end

    assign det_out_ready  = 1'b1;
    assign det2_out_ready = 1'b1;

    //--------------------------------------------------------------------
    // 向量加载
    //--------------------------------------------------------------------
    task automatic load_vec(input string path);
        begin
            fd = $fopen(path, "rb");
            if (fd == 0) begin $display("[FATAL] no %s", path); $finish; end
            code = $fread(fbuf, fd);
            $fclose(fd);
        end
    endtask

    task automatic load_chain(input string path);
        integer i;
        begin
            load_vec(path);
            exp_valid = (le32(0) == 1) ? 1 : 0;
            exp_N     = le32(4);
            for (i = 0; i < 2 + exp_N*2; ++i)
                exp_chain[i] = le32(i*4);
        end
    endtask

    //--------------------------------------------------------------------
    // 运行
    //--------------------------------------------------------------------
    integer t;
    reg w_d_prev;

    initial begin
        automatic string vec_dir = "../../data/image/";
        rst_n = 1'b0;
        pyr_start = 1'b0; det_start = 1'b0; det2_start = 1'b0;
        pyr_active = 1'b0; det_active = 1'b0; det2_active = 1'b0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // ================= 场景 1：big（金字塔 + 全链） =================
        $display("=== scene: big (1280x720 pyramid) ===");
        // 预载 L0 灰度
        load_vec({vec_dir, "m6_big_gray.bin"});
        for (t = 0; t < 1280*720; ++t) gray_mem[t] = fbuf[t];
        // pyramid_ctrl 生成 L1
        pyr_active = 1'b1;
        @(negedge clk);
        pyr_start = 1'b1;
        @(negedge clk);
        pyr_start = 1'b0;
        // 等待 pyr_done 电平 0→1 沿（前一负沿值判沿，防漏沿/防冻结）
        w_d_prev = 1'b0;
        forever begin
            @(negedge clk);
            if (pyr_done && !w_d_prev) break;
            w_d_prev = pyr_done;
        end
        $display("[big] pyramid done level_count=%0d L1=(base=%0d W=%0d H=%0d)",
                 pyr_level_count, pyr_lvl_base[1], pyr_lvl_w[1], pyr_lvl_h[1]);
        if (pyr_level_count != 2 || pyr_lvl_base[1] != 921600 || pyr_lvl_w[1] != 640 || pyr_lvl_h[1] != 360) begin
            $display("[FAIL][big] pyramid descriptor mismatch"); err_cnt = err_cnt + 1;
        end
        // 比对 L1 灰度 vs m6_down_big.bin 期望段
        load_vec({vec_dir, "m6_down_big.bin"});       // u32 W,u32 H,921600 src,230400 exp
        begin
            automatic integer l1err = 0, k;
            for (k = 0; k < 640*360; ++k) begin
                if (gray_mem[921600 + k] !== fbuf[8 + 921600 + k]) l1err = l1err + 1;
            end
            $display("[big] L1 gray bytes err=%0d / %0d", l1err, 640*360);
            if (l1err != 0) err_cnt = err_cnt + 1;
        end
        // detect_ctrl DEPTH=2
        load_chain({vec_dir, "m6_chain_big.bin"});
        pyr_active = 1'b0;
        det_active = 1'b1;
        @(negedge clk);
        det_start = 1'b1;
        @(negedge clk);
        det_start = 1'b0;
        w_d_prev = 1'b0;
        forever begin
            @(negedge clk);
            if (det_done && !w_d_prev) break;
            w_d_prev = det_done;
        end
        $display("[big] detect done status=%02b total=%0d grid_ok=%b got_pts=%0d",
                 det_status, det_out_total, det_out_grid_ok, oi);
        if (det_status !== 2'b01 || det_out_grid_ok !== 1'b1 || oi != 40 ||
            oi != exp_N || exp_valid != 1) begin
            $display("[FAIL][big] detect result mismatch status=%02b got=%0d expN=%0d expV=%0d",
                     det_status, oi, exp_N, exp_valid);
            err_cnt = err_cnt + 1;
        end
        det_active = 1'b0;
        repeat (10) @(negedge clk);

        // ================= 场景 2：board5x8（native 路径） =================
        $display("=== scene: board5x8 (272x96 native) ===");
        load_vec({vec_dir, "m5_board5x8_gray.bin"});
        for (t = 0; t < 272*96; ++t) gray_mem[t] = fbuf[t];
        load_chain({vec_dir, "m6_chain_board5x8.bin"});
        det2_active = 1'b1;
        @(negedge clk);
        det2_start = 1'b1;
        @(negedge clk);
        det2_start = 1'b0;
        w_d_prev = 1'b0;
        forever begin
            @(negedge clk);
            if (det2_done && !w_d_prev) break;
            w_d_prev = det2_done;
        end
        $display("[board5x8] detect done status=%02b total=%0d grid_ok=%b got_pts=%0d",
                 det2_status, det2_out_total, det2_out_grid_ok, oi2);
        if (det2_status !== 2'b01 || det2_out_grid_ok !== 1'b1 || oi2 != 40 ||
            oi2 != exp_N || exp_valid != 1) begin
            $display("[FAIL][board5x8] detect result mismatch status=%02b got=%0d expN=%0d expV=%0d",
                     det2_status, oi2, exp_N, exp_valid);
            err_cnt = err_cnt + 1;
        end
        det2_active = 1'b0;

        // ================= 结果 =================
        repeat (20) @(negedge clk);
        $display("==============================================");
        $display("DETECT-PYRAMID TB: err=%0d (big_pts=%0d board_pts=%0d)", err_cnt, oi, oi2);
        if (err_cnt == 0 && oi == 40 && oi2 == 40)
            $display("TB RESULT: ALL DETECT-PYRAMID TESTS PASSED");
        else
            $display("TB RESULT: FAILED");
        $finish;
    end

    // 超时看门狗（big 全链 ~63ms 仿真 + 金字塔 ~25ms + board5x8，给足余量）
    initial begin
        #500_000_000;
        $display("[FATAL] global timeout err=%0d oi=%0d oi2=%0d", err_cnt, oi, oi2);
        $finish;
    end

endmodule
