`timescale 1ns / 1ps
//==============================================================================
// tb_front_mid.sv — 完整链中等尺寸对拍（W×H 可参数化，含 tensor 窗口）
//------------------------------------------------------------------------------
// 与 tb_front_small 的区别：链尾接 tensor_core + tensor_window_sum（带
// 虚拟节拍背压），复现/排除大图窗口丢失问题。
//==============================================================================
module tb_front_mid;

    localparam integer W = 1280, H = 6, NPIX = W * H;
    localparam integer CLK_PERIOD = 10;

    logic clk = 1'b0;
    logic rst_n;
    always #(CLK_PERIOD/2) clk = ~clk;

    function automatic [7:0] pix(input integer x, input integer y);
        pix = (x * 7 + y * 13) & 8'hFF;
    endfunction

    logic        pin_valid, pin_ready;
    logic [7:0]  pin_b, pin_g, pin_r;

    logic        gv, grdy;
    logic [7:0]  gd;

    logic        w1v, w1rdy;
    logic [71:0] w1d;
    logic [10:0] w1x, w1y;

    logic        sv;
    logic        trdy;
    logic [15:0] sgx, sgy;
    logic [10:0] sx, sy;

    logic        tv;
    logic        tws_rdy;
    logic [31:0] txx, txy, tyy;
    logic [10:0] tx, ty;

    logic        av;
    logic [31:0] sum_a, sum_b_, sum_c;
    logic [10:0] ax, ay;

    bgr_to_gray u_gray (
        .clk(clk), .rst_n(rst_n),
        .in_valid(pin_valid), .in_ready(pin_ready),
        .in_b(pin_b), .in_g(pin_g), .in_r(pin_r),
        .out_valid(gv), .out_ready(grdy), .out_gray(gd)
    );

    window3x3 #(
        .CH(1), .DW(8), .IMG_W(W), .IMG_H(H), .BORDER_CLAMP(1'b1)
    ) u_win1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(gv), .in_ready(grdy), .in_data(gd),
        .out_valid(w1v), .out_ready(w1rdy),
        .out_data(w1d), .out_x(w1x), .out_y(w1y)
    );

    sobel_core u_sob (
        .clk(clk), .rst_n(rst_n),
        .in_valid(w1v), .in_ready(w1rdy), .in_win(w1d), .in_x(w1x), .in_y(w1y),
        .out_valid(sv), .out_ready(trdy),
        .out_gx(sgx), .out_gy(sgy), .out_x(sx), .out_y(sy)
    );

    tensor_core u_tc (
        .clk(clk), .rst_n(rst_n),
        .in_valid(sv), .in_ready(trdy), .in_gx(sgx), .in_gy(sgy),
        .in_x(sx), .in_y(sy),
        .out_valid(tv), .out_ready(tws_rdy),
        .out_xx(txx), .out_xy(txy), .out_yy(tyy), .out_x(tx), .out_y(ty)
    );

    tensor_window_sum #(
        .IMG_W(W), .IMG_H(H)
    ) u_tws (
        .clk(clk), .rst_n(rst_n),
        .in_valid(tv), .in_ready(tws_rdy),
        .in_xx(txx), .in_xy(txy), .in_yy(tyy), .in_x(tx), .in_y(ty),
        .out_valid(av), .out_ready(1'b1),
        .out_a(sum_a), .out_b(sum_b_), .out_c(sum_c), .out_x(ax), .out_y(ay)
    );

    integer w_cnt = 0, s_cnt = 0, t_cnt = 0, a_cnt = 0;
    integer w_errs = 0, s_errs = 0, a_errs = 0;

    // 丢失窗口拍附近的逐拍内部状态
    always @(posedge clk) begin
        if (rst_n && w_cnt >= 1276 && w_cnt <= 1288)
            $display("[%0t] st=%0d x_t=%0d row_t=%0d fire=%b fire_d=%b fire_d2=%b x_d1=%0d row_d1=%0d fifo_cnt=%0d ov=%b wrdy=%b trdy=%b | srT=%0d,%0d,%0d",
                     $time, u_win1.state, u_win1.x_t, u_win1.row_t,
                     u_win1.scan_fire, u_win1.fire_d, u_win1.fire_d2,
                     u_win1.x_d1, u_win1.row_d1, u_win1.fifo_count,
                     u_win1.out_valid, w1rdy, trdy,
                     u_win1.sr_top[2], u_win1.sr_top[1], u_win1.sr_top[0]);
    end

    always @(posedge clk) begin
        if (rst_n) begin
            if (w1v && w1rdy) begin
                if (w1x !== (w_cnt % W) || w1y !== (w_cnt / W)) begin
                    w_errs = w_errs + 1;
                    if (w_errs <= 10)
                        $display("[WIN %0d][MISMATCH] (%0d,%0d) exp (%0d,%0d) | x_t=%0d row_t=%0d x_d1=%0d fifo=%0d",
                                 w_cnt, w1x, w1y, w_cnt % W, w_cnt / W,
                                 u_win1.x_t, u_win1.row_t, u_win1.x_d1, u_win1.fifo_count);
                end
                w_cnt = w_cnt + 1;
            end
            if (sv && trdy) begin
                if (sx !== (s_cnt % W) || sy !== (s_cnt / W)) begin
                    s_errs = s_errs + 1;
                    if (s_errs <= 10)
                        $display("[SOB %0d][MISMATCH] (%0d,%0d) exp (%0d,%0d)",
                                 s_cnt, sx, sy, s_cnt % W, s_cnt / W);
                end
                s_cnt = s_cnt + 1;
            end
            if (tv && tws_rdy)
                t_cnt = t_cnt + 1;
            if (av) begin
                if (ax !== (a_cnt % W) || ay !== (a_cnt / W)) begin
                    a_errs = a_errs + 1;
                    if (a_errs <= 10)
                        $display("[SUM %0d][MISMATCH] (%0d,%0d) exp (%0d,%0d)",
                                 a_cnt, ax, ay, a_cnt % W, a_cnt / W);
                end
                a_cnt = a_cnt + 1;
            end
        end
    end

    integer p_cnt = 0;
    initial begin
        pin_valid = 1'b0;
        pin_b = 0; pin_g = 0; pin_r = 0;
        rst_n = 1'b0;
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        pin_valid = 1'b1;
        while (p_cnt < NPIX) begin
            pin_b = pix(p_cnt % W, p_cnt / W);
            pin_g = pin_b;
            pin_r = pin_b;
            @(negedge clk);
            if (pin_valid && pin_ready)
                p_cnt = p_cnt + 1;
        end
        pin_valid = 1'b0;
        $display("[TB] fed %0d pixels", NPIX);
    end

    initial begin
        wait (a_cnt >= NPIX);
        repeat (10) @(negedge clk);
        $display("=== mid TB: win=%0d(err %0d) sob=%0d(err %0d) ten=%0d sum=%0d(err %0d) expect=%0d ===",
                 w_cnt, w_errs, s_cnt, s_errs, t_cnt, a_cnt, a_errs, NPIX);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("[TB][FATAL] timeout: w=%0d s=%0d t=%0d a=%0d", w_cnt, s_cnt, t_cnt, a_cnt);
        $finish;
    end

endmodule
