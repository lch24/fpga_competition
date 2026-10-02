`timescale 1ns / 1ps
//==============================================================================
// tb_front.sv — 前端数据流全链对拍（M2）
//------------------------------------------------------------------------------
// DUT 链：
//   TB(BGR像素流) → bgr_to_gray → window3x3(CLAMP) → sobel_core
//                → tensor_core → tensor_window_sum(ZERO)
//
// 对拍向量（export_vectors.cpp v2 导出，默认 test0，+VEC= 前缀可换）：
//   C1  gray 流  ↔ _gray.bin（逐字节）
//   C2  gx 流    ↔ _ix.bin（f32 位模式；整数值域内位级一致）
//   C3  gy 流    ↔ _iy.bin（同上）
//   C4  窗口和 A/B/C ↔ _sum.bin（每像素 3 个 f32，光栅序）
//   C5  各级输出像素计数 == W*H 且坐标严格光栅序
//
// 流控：TB 激励无输入背压（连续 valid）；window3x3 的虚拟节拍会产生
//   in_ready 间隙——链式背压本身是被验证的行为之一。
// 文件字节序：向量均为小端；$fread 到 8bit 数组后按小端拼字。
//
// 通过标准：fails==0 且四级计数均为 W*H。
//==============================================================================
module tb_front;

    localparam integer W = 1280, H = 720, NPIX = W * H;
    localparam integer CLK_PERIOD = 10;

    logic clk = 1'b0;
    logic rst_n;

    always #(CLK_PERIOD/2) clk = ~clk;

    //--------------------------------------------------------------------
    // 向量存储（字节数组，避免 $fread 多字节端序问题）
    //--------------------------------------------------------------------
    string  vec_prefix;
    integer fd, code;

    reg [7:0] bgr_b  [0:NPIX*3-1];
    reg [7:0] gray_b [0:NPIX-1];
    reg [7:0] ixb    [0:NPIX*4-1];
    reg [7:0] iyb    [0:NPIX*4-1];
    reg [7:0] sumb   [0:NPIX*12-1];

    // 小端取字（向量文件均为小端）
    function automatic [31:0] le32_ix(input integer i);
        le32_ix = {ixb[i*4+3], ixb[i*4+2], ixb[i*4+1], ixb[i*4]};
    endfunction
    function automatic [31:0] le32_iy(input integer i);
        le32_iy = {iyb[i*4+3], iyb[i*4+2], iyb[i*4+1], iyb[i*4]};
    endfunction
    function automatic [31:0] le32_sum(input integer i);
        le32_sum = {sumb[i*4+3], sumb[i*4+2], sumb[i*4+1], sumb[i*4]};
    endfunction

    //--------------------------------------------------------------------
    // 整数 → IEEE754 f32 位模式（值域 |v| < 2^24 内精确，无舍入）
    //--------------------------------------------------------------------
    function automatic [31:0] i2f(input signed [31:0] v);
        reg [31:0] mag;
        integer e;
        begin
            if (v == 0) begin
                i2f = 32'h0000_0000;
            end else begin
                mag = (v < 0) ? (-v) : v;
                e   = 31;
                while (e > 0 && !mag[e])
                    e = e - 1;
                i2f[31]    = (v < 0);
                i2f[30:23] = e + 127;
                i2f[22:0]  = (mag << (23 - e)) & 23'h7FFFFF;
            end
        end
    endfunction

    //--------------------------------------------------------------------
    // DUT 链
    //--------------------------------------------------------------------
    logic        pin_valid, pin_ready;
    logic [7:0]  pin_b, pin_g, pin_r;

    logic        gv, grdy;
    logic [7:0]  gd;

    logic        w1v, w1rdy;
    logic [71:0] w1d;
    logic [10:0] w1x, w1y;

    logic        sv, srdy;
    logic [15:0] sgx, sgy;
    logic [10:0] sx, sy;

    logic        tv;
    logic        trdy;        // u_tc.in_ready 驱动；u_sob.out_ready 接收
    logic [31:0] txx, txy, tyy;
    logic [10:0] tx, ty;

    logic        av;
    logic        tws_rdy;     // u_tws.in_ready 驱动；u_tc.out_ready 接收
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

    //--------------------------------------------------------------------
    // 采集与比对
    //--------------------------------------------------------------------
    integer fails = 0;
    integer g_cnt = 0, s_cnt = 0, t_cnt = 0, a_cnt = 0;

    task automatic check_coord(input string stage, input integer cnt,
                               input [10:0] cx, input [10:0] cy);
        if (cx !== (cnt % W) || cy !== (cnt / W)) begin
            fails = fails + 1;
            if (fails < 30)
                $display("[TB][FAIL] %s coord (%0d,%0d) != expected (%0d,%0d) @cnt=%0d",
                         stage, cx, cy, cnt % W, cnt / W, cnt);
        end
    endtask

    always @(posedge clk) begin
        if (rst_n) begin
            // C1: gray
            if (gv && grdy) begin
                if (gd !== gray_b[g_cnt]) begin
                    fails = fails + 1;
                    if (fails < 30)
                        $display("[TB][FAIL] C1 gray[%0d]=0x%02x != 0x%02x",
                                 g_cnt, gd, gray_b[g_cnt]);
                end
                g_cnt = g_cnt + 1;
            end
            // C2/C3: gx/gy（成交拍判定：sv && trdy）
            if (sv && trdy) begin
                check_coord("sobel", s_cnt, sx, sy);
                if (i2f($signed(sgx)) !== le32_ix(s_cnt)) begin
                    fails = fails + 1;
                    if (fails < 30)
                        $display("[TB][FAIL] C2 ix[%0d]=%0d(0x%08x) != 0x%08x",
                                 s_cnt, sgx, i2f($signed(sgx)), le32_ix(s_cnt));
                end
                if (i2f($signed(sgy)) !== le32_iy(s_cnt)) begin
                    fails = fails + 1;
                    if (fails < 30)
                        $display("[TB][FAIL] C3 iy[%0d]=%0d(0x%08x) != 0x%08x",
                                 s_cnt, sgy, i2f($signed(sgy)), le32_iy(s_cnt));
                end
                s_cnt = s_cnt + 1;
            end
            // tensor 流计数（成交拍：tv && tws_rdy）
            if (tv && tws_rdy)
                t_cnt = t_cnt + 1;
            // C5: sum
            if (av) begin
                check_coord("sum", a_cnt, ax, ay);
                if (i2f($signed(sum_a)) !== le32_sum(a_cnt*3)) begin
                    fails = fails + 1;
                    if (fails < 30)
                        $display("[TB][FAIL] C4 sumA[%0d]=%0d(0x%08x) != 0x%08x",
                                 a_cnt, sum_a, i2f($signed(sum_a)), le32_sum(a_cnt*3));
                end
                if (i2f($signed(sum_b_)) !== le32_sum(a_cnt*3+1)) begin
                    fails = fails + 1;
                    if (fails < 30)
                        $display("[TB][FAIL] C4 sumB[%0d]=%0d(0x%08x) != 0x%08x",
                                 a_cnt, sum_b_, i2f($signed(sum_b_)), le32_sum(a_cnt*3+1));
                end
                if (i2f($signed(sum_c)) !== le32_sum(a_cnt*3+2)) begin
                    fails = fails + 1;
                    if (fails < 30)
                        $display("[TB][FAIL] C4 sumC[%0d]=%0d(0x%08x) != 0x%08x",
                                 a_cnt, sum_c, i2f($signed(sum_c)), le32_sum(a_cnt*3+2));
                end
                a_cnt = a_cnt + 1;
            end
        end
    end

    //--------------------------------------------------------------------
    // 激励：BGR 像素流（连续 valid）
    //--------------------------------------------------------------------
    integer p_cnt = 0;

    initial begin
        if (!$value$plusargs("VEC=%s", vec_prefix))
            vec_prefix = "../tests/build/vectors/test0";

        fd = $fopen({vec_prefix, "_bgr.bin"}, "rb");
        if (fd == 0) begin $display("[TB][FATAL] cannot open bgr vector"); $finish; end
        code = $fread(bgr_b, fd); $fclose(fd);
        fd = $fopen({vec_prefix, "_gray.bin"}, "rb");
        if (fd == 0) begin $display("[TB][FATAL] cannot open gray vector"); $finish; end
        code = $fread(gray_b, fd); $fclose(fd);
        fd = $fopen({vec_prefix, "_ix.bin"}, "rb");
        if (fd == 0) begin $display("[TB][FATAL] cannot open ix vector"); $finish; end
        code = $fread(ixb, fd); $fclose(fd);
        fd = $fopen({vec_prefix, "_iy.bin"}, "rb");
        if (fd == 0) begin $display("[TB][FATAL] cannot open iy vector"); $finish; end
        code = $fread(iyb, fd); $fclose(fd);
        fd = $fopen({vec_prefix, "_sum.bin"}, "rb");
        if (fd == 0) begin $display("[TB][FATAL] cannot open sum vector"); $finish; end
        code = $fread(sumb, fd); $fclose(fd);
        $display("[TB] vectors loaded from %s", vec_prefix);

        pin_valid = 1'b0;
        pin_b = 8'd0; pin_g = 8'd0; pin_r = 8'd0;
        rst_n = 1'b0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        pin_valid = 1'b1;
        while (p_cnt < NPIX) begin
            pin_b = bgr_b[p_cnt*3 + 0];
            pin_g = bgr_b[p_cnt*3 + 1];
            pin_r = bgr_b[p_cnt*3 + 2];
            @(negedge clk);
            if (pin_valid && pin_ready)
                p_cnt = p_cnt + 1;
        end
        pin_valid = 1'b0;
        $display("[TB] all %0d pixels fed", NPIX);
    end

    //--------------------------------------------------------------------
    // 结束判定与看门狗
    //--------------------------------------------------------------------
    initial begin
        wait (a_cnt >= NPIX);
        repeat (20) @(negedge clk);
        $display("==================================================");
        $display("TB SUMMARY: fails=%0d", fails);
        $display("  gray  count=%0d / %0d", g_cnt, NPIX);
        $display("  sobel count=%0d / %0d", s_cnt, NPIX);
        $display("  tensor count=%0d / %0d", t_cnt, NPIX);
        $display("  sum   count=%0d / %0d", a_cnt, NPIX);
        if (fails == 0 && g_cnt == NPIX && s_cnt == NPIX
                       && t_cnt == NPIX && a_cnt == NPIX)
            $display("TB RESULT: ALL FRONT-END TESTS PASSED");
        else
            $display("TB RESULT: TESTS FAILED");
        $display("==================================================");
        $finish;
    end

    initial begin
        #200_000_000;
        $display("[TB][FATAL] global timeout: g=%0d s=%0d t=%0d a=%0d",
                 g_cnt, s_cnt, t_cnt, a_cnt);
        $finish;
    end

endmodule
