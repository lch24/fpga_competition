`timescale 1ns / 1ps
//==============================================================================
// tb_me_chain.sv — min_eigen 全图流验证（不含 ctrl，定位集成死锁来源）
//------------------------------------------------------------------------------
// 链：前端（bgr→win→sobel→tensor→tws）→ s32×3 → min_eigen_core
// me 输出 out_ready 恒 1（无背压）——验证 me 本身在连续流下能走通。
// 统计 me 输出 valid 数，期望 == W*H（921600）。
//==============================================================================
module tb_me_chain;

    localparam integer W = 1280, H = 720, NPIX = W * H;
    localparam integer CLK_PERIOD = 10;

    logic clk = 1'b0;
    logic rst_n;
    always #(CLK_PERIOD/2) clk = ~clk;

    string   vec_prefix;
    integer  fd, code;
    reg [7:0]  bgr_b [0:NPIX*3-1];

    // 前端链（同 tb_resp 前半）
    logic        pin_valid, pin_ready;
    logic [7:0]  pin_b, pin_g, pin_r;
    logic        gv, grdy;   logic [7:0]  gd;
    logic        w1v, w1rdy; logic [71:0] w1d; logic [10:0] w1x, w1y;
    logic        sv, trdy;   logic [15:0] sgx, sgy; logic [10:0] sx, sy;
    logic        tv, tws_rdy; logic [31:0] txx, txy, tyy; logic [10:0] tx, ty;
    logic        av;         logic [31:0] s_a, s_b_, s_c;
    logic        mrdy, fa_v, fb_v, fc_v;
    logic [31:0] fa_r, fb_r, fc_r;
    logic        mev, me_rdy;   logic [31:0] me_resp;

    bgr_to_gray u_gray (.clk(clk), .rst_n(rst_n),
        .in_valid(pin_valid), .in_ready(pin_ready),
        .in_b(pin_b), .in_g(pin_g), .in_r(pin_r),
        .out_valid(gv), .out_ready(grdy), .out_gray(gd));
    window3x3 #(.CH(1),.DW(8),.IMG_W(W),.IMG_H(H),.BORDER_CLAMP(1'b1)) u_win1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(gv), .in_ready(grdy), .in_data(gd),
        .out_valid(w1v), .out_ready(w1rdy), .out_data(w1d),
        .out_x(w1x), .out_y(w1y));
    sobel_core u_sob (.clk(clk), .rst_n(rst_n),
        .in_valid(w1v), .in_ready(w1rdy), .in_win(w1d), .in_x(w1x), .in_y(w1y),
        .out_valid(sv), .out_ready(trdy), .out_gx(sgx), .out_gy(sgy), .out_x(sx), .out_y(sy));
    tensor_core u_tc (.clk(clk), .rst_n(rst_n),
        .in_valid(sv), .in_ready(trdy), .in_gx(sgx), .in_gy(sgy), .in_x(sx), .in_y(sy),
        .out_valid(tv), .out_ready(tws_rdy), .out_xx(txx), .out_xy(txy), .out_yy(tyy),
        .out_x(tx), .out_y(ty));
    tensor_window_sum #(.IMG_W(W), .IMG_H(H)) u_tws (.clk(clk), .rst_n(rst_n),
        .in_valid(tv), .in_ready(tws_rdy), .in_xx(txx), .in_xy(txy), .in_yy(tyy),
        .in_x(tx), .in_y(ty),
        .out_valid(av), .out_ready(mrdy), .out_a(s_a), .out_b(s_b_), .out_c(s_c),
        .out_x(), .out_y());
    s32_to_f32 u_fa (.clk(clk), .rst_n(rst_n), .in_valid(av), .in_ready(), .in_data(s_a),
        .out_valid(fa_v), .out_ready(mrdy), .out_r(fa_r));
    s32_to_f32 u_fb (.clk(clk), .rst_n(rst_n), .in_valid(av), .in_ready(), .in_data(s_b_),
        .out_valid(fb_v), .out_ready(mrdy), .out_r(fb_r));
    s32_to_f32 u_fc (.clk(clk), .rst_n(rst_n), .in_valid(av), .in_ready(), .in_data(s_c),
        .out_valid(fc_v), .out_ready(mrdy), .out_r(fc_r));
    min_eigen_core u_me (.clk(clk), .rst_n(rst_n),
        .in_valid(fa_v), .in_ready(mrdy), .in_a(fa_r), .in_b(fb_r), .in_c(fc_r),
        .out_valid(mev), .out_ready(me_rdy), .out_resp(me_resp));

    assign me_rdy = 1'b1;   // 无背压

    integer me_cnt = 0;
    always @(posedge clk)
        if (rst_n && mev && me_rdy)
            me_cnt = me_cnt + 1;

    integer p_cnt = 0;
    initial begin
        if (!$value$plusargs("VEC=%s", vec_prefix))
            vec_prefix = "../tests/build/vectors/test0";
        fd = $fopen({vec_prefix, "_bgr.bin"}, "rb");
        if (fd == 0) begin $display("[TB][FATAL] no bgr"); $finish; end
        code = $fread(bgr_b, fd); $fclose(fd);

        pin_valid = 1'b0; pin_b = 0; pin_g = 0; pin_r = 0;
        rst_n = 1'b0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        pin_valid = 1'b1;
        while (p_cnt < NPIX) begin
            pin_b = bgr_b[p_cnt*3+0]; pin_g = bgr_b[p_cnt*3+1]; pin_r = bgr_b[p_cnt*3+2];
            @(negedge clk);
            if (pin_valid && pin_ready)
                p_cnt = p_cnt + 1;
        end
        pin_valid = 1'b0;
        $display("[TB] fed %0d pixels", NPIX);

        wait (me_cnt >= NPIX);
        repeat (5) @(negedge clk);
        $display("==============================================");
        $display("ME-CHAIN: me_cnt=%0d / %0d", me_cnt, NPIX);
        if (me_cnt == NPIX)
            $display("TB RESULT: ME CHAIN PASSED (no deadlock, full flow)");
        else
            $display("TB RESULT: ME CHAIN FAILED");
        $finish;
    end

    initial begin
        #60_000_000;   // 60ms 看门狗
        $display("[TB][FATAL] timeout me_cnt=%0d / %0d", me_cnt, NPIX);
        $finish;
    end

endmodule