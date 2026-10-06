`timescale 1ns / 1ps
//==============================================================================
// tb_front_small.sv — window3x3 小图调试平台（8×6）
//------------------------------------------------------------------------------
// 目的：定位"每行丢 1 个窗口"的节拍。打印每个窗口输出的坐标与内部
//       流水状态（x_d2/row_d2/win_d2/scan_d1），全链共 48 个窗口。
//==============================================================================
module tb_front_small;

    localparam integer W = 1280, H = 3, NPIX = W * H;
    localparam integer CLK_PERIOD = 10;

    logic clk = 1'b0;
    logic rst_n;
    always #(CLK_PERIOD/2) clk = ~clk;

    // 用可辨识的像素值：gray(x,y) = y*16 + x（BGR 直接构造）
    function automatic [7:0] pix(input integer x, input integer y);
        pix = (y * 16 + x) & 8'hFF;
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

    // 终端：恒就绪
    assign trdy = 1'b1;

    integer w_cnt = 0, s_cnt = 0;

    // 窗口输出追踪：坐标错位时打印（前 20 个错误）
    integer coord_errs = 0;
    always @(posedge clk) begin
        if (rst_n && w1v && w1rdy) begin
            if (w1x !== (w_cnt % W) || w1y !== (w_cnt / W)) begin
                coord_errs = coord_errs + 1;
                if (coord_errs <= 20)
                    $display("[WIN %0d][MISMATCH] coord=(%0d,%0d) expected=(%0d,%0d) | x_t=%0d row_t=%0d x_d2=%0d row_d2=%0d win_d2=%0b",
                             w_cnt, w1x, w1y, w_cnt % W, w_cnt / W,
                             u_win1.x_t, u_win1.row_t, u_win1.x_d2, u_win1.row_d2, u_win1.win_d2);
            end
            // 行边界打印
            if (w_cnt > 0 && (w_cnt % W) == 0)
                $display("[ROW] cy=%0d complete at win %0d", w_cnt / W - 1, w_cnt);
            w_cnt = w_cnt + 1;
        end
    end

    // sobel 输出坐标追踪
    always @(posedge clk) begin
        if (rst_n && sv && trdy) begin
            $display("[SOB %0d] coord=(%0d,%0d) gx=%0d gy=%0d",
                     s_cnt, sx, sy, $signed(sgx), $signed(sgy));
            s_cnt = s_cnt + 1;
        end
    end

    // 激励
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

    // 结束
    initial begin
        wait (w_cnt >= NPIX);
        repeat (10) @(negedge clk);
        $display("=== small TB: windows=%0d sobel=%0d (expect %0d) ===",
                 w_cnt, s_cnt, NPIX);
        $finish;
    end

    initial begin
        #100_000;
        $display("[TB][FATAL] timeout: w=%0d s=%0d", w_cnt, s_cnt);
        $finish;
    end

endmodule
