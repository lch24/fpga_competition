`timescale 1ns / 1ps
//==============================================================================
// tb_resp.sv — M2 集成全链对拍（BGR → resp 全图 → 候选角点）
//------------------------------------------------------------------------------
// 链：bgr_to_gray → window3x3(CLAMP) → sobel → tensor → tensor_window_sum
//     → s32_to_f32 ×3（A/B/C 锁步）→ min_eigen_core → shi_tomasi_ctrl
//       （PASS1 store+rmax / PASS2 重读+窗口NMS）
// 握手链：me.out_ready=ctrl.resp_rdy；tws/s32.out_ready=me.in_ready；
//   逐级 in_ready(输出) 反驱上级 out_ready(输入)，与 tb_front 完全同构。
//
// 对拍（export_vectors.cpp v3；+VEC= 前缀，默认 ../tests/build/vectors/test0）：
//   C1  resp 全图：min_eigen_core 输出逐像素位级 vs _resp.bin（W*H f32）
//   C2  rmax：PASS1 末 u_ctrl.u_store.rmax vs TB 自算最大（IEEE 序位变换）
//   C3  候选：cand_x/y 时序 vs _candidates.bin（数量+坐标全等、顺序=光栅）
//   C4  计数：resp fire==W*H、cand_total==N、done 置位、status==01
// thr = rmax*0.08f（fp32_mul，0.08f=0x3DA3D70A；C++ 语义 rmax*clamp(0.08f)），
//   在 pass1_done 脉冲启动乘法器，结果闩存进 ctrl.thr —— 先于 PASS2 首窗口判据。
// 采样范式：激励 negedge 驱动、检测 posedge fire（M1 教训）。
//==============================================================================
module tb_resp;

    localparam integer W = 32, H = 24, NPIX = W * H;
    localparam integer CLK_PERIOD = 10;

    logic clk = 1'b0;
    logic rst_n;
    always #(CLK_PERIOD/2) clk = ~clk;

    //--------------------------------------------------------------------
    // 向量装载（字节数组→小端字）
    //--------------------------------------------------------------------
    string   vec_prefix;
    integer  fd, code;
    reg [7:0]  bgr_b  [0:NPIX*3-1];
    reg [7:0]  resp_b [0:NPIX*4-1];
    reg [7:0]  cand_b [0:8191];
    reg [31:0] resp_w [0:NPIX-1];
    reg [31:0] cand_w [0:8191];
    integer candN = 0;
    reg [31:0] exp_rmax;

    function automatic [31:0] le32_b(input integer i, input reg [7:0] arr[]);
        le32_b = {arr[i+3], arr[i+2], arr[i+1], arr[i]};
    endfunction

    // IEEE 序位变换（B 修正版）：fp32 比较 ⟺ ./ 无符号比较
    function automatic [31:0] f2o(input [31:0] v);
        f2o = v[31] ? ~v : (v | 32'h8000_0000);
    endfunction

    //--------------------------------------------------------------------
    // 前端链（握手同 tb_front）
    //--------------------------------------------------------------------
    logic        pin_valid, pin_ready;
    logic [7:0]  pin_b, pin_g, pin_r;
    logic        gv, grdy;   logic [7:0]  gd;
    logic        w1v, w1rdy; logic [71:0] w1d; logic [10:0] w1x, w1y;
    logic        sv, trdy;   logic [15:0] sgx, sgy; logic [10:0] sx, sy;
    logic        tv, tws_rdy; logic [31:0] txx, txy, tyy; logic [10:0] tx, ty;
    logic        av;         logic [31:0] s_a, s_b_, s_c;
    // s32→fp32 锁步 + min_eigen 中间信号（必须先于首次端口引用声明，
    //   否则触发隐式 net 与显式声明冲突）
    logic        mrdy, fa_v, fb_v, fc_v;
    logic [31:0] fa_r, fb_r, fc_r;
    logic        mev, ctrl_resp_rdy;   logic [31:0] me_resp;

    bgr_to_gray u_gray (
        .clk(clk), .rst_n(rst_n),
        .in_valid(pin_valid), .in_ready(pin_ready),
        .in_b(pin_b), .in_g(pin_g), .in_r(pin_r),
        .out_valid(gv), .out_ready(grdy), .out_gray(gd)
    );
    window3x3 #(.CH(1),.DW(8),.IMG_W(W),.IMG_H(H),.BORDER_CLAMP(1'b1)) u_win1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(gv), .in_ready(grdy), .in_data(gd),
        .out_valid(w1v), .out_ready(w1rdy), .out_data(w1d),
        .out_x(w1x), .out_y(w1y)
    );
    sobel_core u_sob (
        .clk(clk), .rst_n(rst_n),
        .in_valid(w1v), .in_ready(w1rdy), .in_win(w1d),
        .in_x(w1x), .in_y(w1y),
        .out_valid(sv), .out_ready(trdy), .out_gx(sgx), .out_gy(sgy),
        .out_x(sx), .out_y(sy)
    );
    tensor_core u_tc (
        .clk(clk), .rst_n(rst_n),
        .in_valid(sv), .in_ready(trdy), .in_gx(sgx), .in_gy(sgy),
        .in_x(sx), .in_y(sy),
        .out_valid(tv), .out_ready(tws_rdy), .out_xx(txx), .out_xy(txy), .out_yy(tyy),
        .out_x(tx), .out_y(ty)
    );
    tensor_window_sum #(.IMG_W(W), .IMG_H(H)) u_tws (
        .clk(clk), .rst_n(rst_n),
        .in_valid(tv), .in_ready(tws_rdy),
        .in_xx(txx), .in_xy(txy), .in_yy(tyy),
        .in_x(tx), .in_y(ty),
        .out_valid(av), .out_ready(mrdy), .out_a(s_a), .out_b(s_b_), .out_c(s_c),
        .out_x(), .out_y()
    );

    //--------------------------------------------------------------------
    // s32→fp32 ×3（锁步）+ min_eigen_core
    //--------------------------------------------------------------------
    s32_to_f32 u_fa (
        .clk(clk), .rst_n(rst_n),
        .in_valid(av), .in_ready(), .in_data(s_a),
        .out_valid(fa_v), .out_ready(mrdy), .out_r(fa_r)
    );
    s32_to_f32 u_fb (
        .clk(clk), .rst_n(rst_n),
        .in_valid(av), .in_ready(), .in_data(s_b_),
        .out_valid(fb_v), .out_ready(mrdy), .out_r(fb_r)
    );
    s32_to_f32 u_fc (
        .clk(clk), .rst_n(rst_n),
        .in_valid(av), .in_ready(), .in_data(s_c),
        .out_valid(fc_v), .out_ready(mrdy), .out_r(fc_r)
    );

    min_eigen_core u_me (
        .clk(clk), .rst_n(rst_n),
        .in_valid(fa_v), .in_ready(mrdy),
        .in_a(fa_r), .in_b(fb_r), .in_c(fc_r),
        .out_valid(mev), .out_ready(me2f_rdy), .out_resp(me_resp)
    );

    //--------------------------------------------------------------------
    // me → ctrl 桥接 FIFO：把 me 的弹性输出转成 store 可吸收的固定流。
    //   me.out_ready = fifo.in_ready（水位）；fifo 输出直连 ctrl.resp_valid；
    //   ctrl 的 resp_in_ready 接 fifo.in_ready —— ctrl 只见 fifo 的简单握手，
    //   store 的恒1 in_ready 不再外泄成伪握手。
    //--------------------------------------------------------------------
    logic        fifo_out_v, fifo_out_rdy;
    logic [31:0] fifo_out_d;
    logic [4:0]  fifo_cnt;
    sync_fifo #(.DATA_WIDTH(32), .ADDR_WIDTH(5)) u_resp_fifo (
        .clk(clk), .rst_n(rst_n),
        .in_valid(mev), .in_ready(me2f_rdy), .in_data(me_resp),
        .out_valid(fifo_out_v), .out_ready(fifo_out_rdy), .out_data(fifo_out_d),
        .count(fifo_cnt), .empty(), .full()
    );

    //--------------------------------------------------------------------
    // shi_tomasi_ctrl
    //--------------------------------------------------------------------
    logic ctrl_start, ctrl_busy, ctrl_done;
    logic [1:0] ctrl_status;
    logic ctrl_cand_valid;
    logic [10:0] ctrl_cx, ctrl_cy;
    logic [15:0] ctrl_total;
    logic [31:0] ctrl_thr;
    wire  [31:0] rmax_rtl = u_ctrl.u_store.rmax;
    wire         p1d      = u_ctrl.u_store.pass1_done;

    shi_tomasi_ctrl #(
        .IMG_W(W), .IMG_H(H)
    ) u_ctrl (
        .clk(clk), .rst_n(rst_n),
        .start(ctrl_start), .busy(ctrl_busy),
        .thr(ctrl_thr),
        .resp_valid(fifo_out_v), .resp_rdy(fifo_out_rdy), .resp_data(fifo_out_d),
        .resp_in_ready(me2f_rdy),
        .mem_addr(), .mem_data(),
        .done(ctrl_done), .status(ctrl_status),
        .cand_valid(ctrl_cand_valid), .cand_ready(1'b1),
        .cand_x(ctrl_cx), .cand_y(ctrl_cy),
        .cand_total(ctrl_total)
    );

    // thr = rmax * 0.08f（pass1_done 脉冲启动；fp32_mul 延迟约 2 拍）
    logic       thr_mul_v;   logic [31:0] thr_mul_r;
    fp32_mul u_thr (
        .clk(clk), .rst_n(rst_n),
        .in_valid(p1d), .in_ready(),
        .in_a(rmax_rtl), .in_b(32'h3DA3D70A),
        .out_valid(thr_mul_v), .out_ready(1'b1), .out_r(thr_mul_r)
    );
    always @(posedge clk) begin
        if (!rst_n)
            ctrl_thr <= 32'd0;
        else if (thr_mul_v)
            ctrl_thr <= thr_mul_r;
    end

    //--------------------------------------------------------------------
    // 对拍探针
    //--------------------------------------------------------------------
    integer fails = 0;
    integer resp_cnt = 0, cand_cnt = 0;
    integer cand_errs = 0, resp_errs = 0;

    // C1: resp 逐像素（在 ctrl 接收侧成拍 = fifo_out_v && fifo_out_rdy）
    always @(posedge clk) begin
        if (rst_n && fifo_out_v && fifo_out_rdy) begin
            if (resp_cnt < NPIX && fifo_out_d !== resp_w[resp_cnt]) begin
                fails = fails + 1;
                resp_errs = resp_errs + 1;
                if (resp_errs <= 20)
                    $display("[TB][FAIL] C1 resp[%0d] = 0x%08x != 0x%08x",
                             resp_cnt, fifo_out_d, resp_w[resp_cnt]);
            end
            resp_cnt = resp_cnt + 1;
        end
    end

    // C3: 候选（顺序）
    always @(posedge clk) begin
        if (rst_n && ctrl_cand_valid) begin
            if (cand_cnt >= candN ||
                ctrl_cx !== cand_w[2*cand_cnt+1] || ctrl_cy !== cand_w[2*cand_cnt+2]) begin
                if (cand_errs < 30) begin
                    fails = fails + 1;
                    cand_errs = cand_errs + 1;
                    $display("[TB][FAIL] C3 cand[%0d] = (%0d,%0d) != exp (%0d,%0d)",
                             cand_cnt, ctrl_cx, ctrl_cy,
                             cand_w[2*cand_cnt+1], cand_w[2*cand_cnt+2]);
                end
            end
            cand_cnt = cand_cnt + 1;
        end
    end

    //--------------------------------------------------------------------
    // 激励
    //--------------------------------------------------------------------
    integer p_cnt = 0;

    initial begin
        if (!$value$plusargs("VEC=%s", vec_prefix))
            vec_prefix = "../tests/build/vectors/small";

        fd = $fopen({vec_prefix, "_bgr.bin"}, "rb");
        if (fd == 0) begin $display("[TB][FATAL] no bgr"); $finish; end
        code = $fread(bgr_b, fd); $fclose(fd);
        fd = $fopen({vec_prefix, "_resp.bin"}, "rb");
        if (fd == 0) begin $display("[TB][FATAL] no resp"); $finish; end
        code = $fread(resp_b, fd); $fclose(fd);
        fd = $fopen({vec_prefix, "_candidates.bin"}, "rb");
        if (fd == 0) begin $display("[TB][FATAL] no cand"); $finish; end
        code = $fread(cand_b, fd); $fclose(fd);
        candN = le32_b(0, cand_b);   // 手动小端拼（$fread 会按字节序填数组）
        for (integer i = 0; i < candN && i < 4096; ++i) begin
            cand_w[2*i+1] = le32_b(4 + i*8, cand_b);
            cand_w[2*i+2] = le32_b(8 + i*8, cand_b);
        end
        for (integer i = 0; i < NPIX; ++i)
            resp_w[i] = le32_b(i*4, resp_b);
        $display("[TB] loaded %s: candN=%0d", vec_prefix, candN);

        // 期望 rmax
        exp_rmax = 32'hFF800000;
        for (integer i = 0; i < NPIX; ++i)
            if (f2o(resp_w[i]) > f2o(exp_rmax))
                exp_rmax = resp_w[i];

        pin_valid = 1'b0; pin_b = 0; pin_g = 0; pin_r = 0;
        ctrl_start = 1'b0;
        rst_n = 1'b0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // start 必须先于首批 resp 到达（ctrl 需处 PASS1）
        ctrl_start = 1'b1;
        @(negedge clk);
        ctrl_start = 1'b0;
        repeat (4) @(negedge clk);

        // 喂 BGR
        pin_valid = 1'b1;
        while (p_cnt < NPIX) begin
            pin_b = bgr_b[p_cnt*3+0];
            pin_g = bgr_b[p_cnt*3+1];
            pin_r = bgr_b[p_cnt*3+2];
            @(negedge clk);
            if (pin_valid && pin_ready)
                p_cnt = p_cnt + 1;
        end
        pin_valid = 1'b0;
        $display("[TB] fed %0d pixels", NPIX);

        wait (ctrl_done);
        repeat (5) @(negedge clk);

        $display("==================================================");
        $display("TB SUMMARY: fails=%0d", fails);
        $display("  resp fire=%0d / %0d (err %0d)", resp_cnt, NPIX, resp_errs);
        $display("  rmax     RTL=0x%08x EXP=0x%08x", rmax_rtl, exp_rmax);
        $display("  cand     cnt=%0d / %0d (err %0d) total=%0d status=2'b%b",
                 cand_cnt, candN, cand_errs, ctrl_total, ctrl_status);
        if (fails == 0 && resp_cnt == NPIX && cand_cnt == candN
                       && ctrl_total == candN && ctrl_status == 2'b01 && ctrl_done)
            $display("TB RESULT: ALL RESP/DETECT TESTS PASSED");
        else
            $display("TB RESULT: TESTS FAILED");
        $display("==================================================");
        $finish;
    end

    initial begin
        #10_000_000;   // 10ms 看门狗（32×24 小图足够；全图用会超时）
        $display("[TB][TIMEOUT] resp=%0d/%0d cand=%0d done=%b st=%0d fifo=%0d mev=%b p1d=%b wcnt=%0d rmax=0x%08x",
                 resp_cnt, NPIX, cand_cnt, ctrl_done, u_ctrl.state, fifo_cnt, mev,
                 u_ctrl.u_store.pass1_done, u_ctrl.u_store.wcnt, u_ctrl.u_store.rmax);
        $finish;
    end

endmodule