`timescale 1ns / 1ps
//==============================================================================
// tb_fp32.sv — fp32_hypot / fp32_div / fp32_sqrt 单元对拍（M3 基础件）
//------------------------------------------------------------------------------
// 向量：../tests/build/vectors/test_fp32.bin（gen_fp32_vectors.cpp 生成）
//   格式（小端）：u32 N，随后 N×4 {kind, a_bits, b_bits, exp_bits}
//     kind: 0=hypot(a,b) 1=div(a,b) 2=sqrt(a)
// 三个模块独立驱动、独立比对；期望按 kind 预分组。
//==============================================================================
module tb_fp32;
    localparam integer CLK_PERIOD = 10;
    logic clk = 1'b0;
    logic rst_n;
    always #(CLK_PERIOD/2) clk = ~clk;

    integer fd, code;
    reg [7:0]  bytes [0:2097151];
    reg [31:0] words [0:524287];
    integer N;

    function automatic [31:0] le32(input integer i, input reg [7:0] arr[]);
        le32 = {arr[i+3], arr[i+2], arr[i+1], arr[i]};
    endfunction

    // 三 DUT（out_valid 与 out_r 分开，避免 1bit 连 32bit 宽度错位）
    logic        hv, hrdy, hr;
    logic [31:0] ha, hb, hr_d;
    logic        dv, drdy, dr;
    logic [31:0] da, db, dr_d;
    logic        sv, srdy, sr;
    logic [31:0] sa, sr_d;
    fp32_hypot u_hypot (.clk(clk), .rst_n(rst_n),
        .in_valid(hv), .in_ready(hrdy), .in_a(ha), .in_b(hb),
        .out_valid(hr), .out_ready(1'b1), .out_r(hr_d));
    fp32_div u_div (.clk(clk), .rst_n(rst_n),
        .in_valid(dv), .in_ready(drdy), .in_a(da), .in_b(db),
        .out_valid(dr), .out_ready(1'b1), .out_r(dr_d));
    fp32_sqrt u_sqrt (.clk(clk), .rst_n(rst_n),
        .in_valid(sv), .in_ready(srdy), .in_x(sa),
        .out_valid(sr), .out_ready(1'b1), .out_r(sr_d));

    // 期望分组（按 kind 顺序）
    integer src_h [0:131071]; integer src_d [0:131071]; integer src_s [0:131071];
    integer exp_h [0:131071]; integer exp_d [0:131071]; integer exp_s [0:131071];
    integer cnt_h = 0, cnt_d = 0, cnt_s = 0;

    // 输出比对 + 计数
    integer err = 0, oh = 0, od = 0, os_ = 0;
    always @(posedge clk) begin
        if (rst_n) begin
            if (hr) begin
                if (oh >= cnt_h || hr_d !== exp_h[oh]) begin
                    err = err + 1;
                    if (err <= 10) $display("[FAIL] hypot #%0d rtl=0x%08x exp=0x%08x", oh, hr_d, exp_h[oh]);
                end
                oh = oh + 1;
            end
            if (dr) begin
                if (od >= cnt_d || dr_d !== exp_d[od]) begin
                    err = err + 1;
                    if (err <= 10) $display("[FAIL] div #%0d rtl=0x%08x exp=0x%08x", od, dr_d, exp_d[od]);
                end
                od = od + 1;
            end
            if (sr) begin
                if (os_ >= cnt_s || sr_d !== exp_s[os_]) begin
                    err = err + 1;
                    if (err <= 10) $display("[FAIL] sqrt #%0d rtl=0x%08x exp=0x%08x", os_, sr_d, exp_s[os_]);
                end
                os_ = os_ + 1;
            end
        end
    end

    // 三路独立驱动（等待向量加载完成 + 复位释放）
    integer pi_h = 0, pi_d = 0, pi_s = 0;
    reg loaded_done = 1'b0;
    initial begin
        rst_n = 1'b0; hv = 0; dv = 0; sv = 0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);
    end
    initial begin
        wait (rst_n && loaded_done);
        while (pi_h < cnt_h) begin
            if (!hv || hrdy) begin
                ha <= words[2 + src_h[pi_h]*4];
                hb <= words[3 + src_h[pi_h]*4];
                hv <= 1'b1;
                @(negedge clk);
                pi_h = pi_h + 1;
                hv <= 1'b0;
            end else begin
                @(negedge clk);
            end
        end
        hv <= 1'b0;
        $display("[TB] hypot fed %0d", cnt_h);
    end
    initial begin
        wait (rst_n && loaded_done);
        while (pi_d < cnt_d) begin
            if (!dv || drdy) begin
                da <= words[2 + src_d[pi_d]*4];
                db <= words[3 + src_d[pi_d]*4];
                dv <= 1'b1;
                @(negedge clk);
                pi_d = pi_d + 1;
                dv <= 1'b0;
            end else begin
                @(negedge clk);
            end
        end
        dv <= 1'b0;
        $display("[TB] div fed %0d", cnt_d);
    end
    initial begin
        wait (rst_n && loaded_done);
        while (pi_s < cnt_s) begin
            if (!sv || srdy) begin
                sa <= words[2 + src_s[pi_s]*4];
                sv <= 1'b1;
                @(negedge clk);
                pi_s = pi_s + 1;
                sv <= 1'b0;
            end else begin
                @(negedge clk);
            end
        end
        sv <= 1'b0;
        $display("[TB] sqrt fed %0d", cnt_s);
    end

    // 加载向量（先于驱动；+VEC= 可选向量文件）
    initial begin
        string vec = "../tests/build/vectors/test_fp32.bin";
        if (!$value$plusargs("VEC=%s", vec))
            vec = "../tests/build/vectors/test_fp32.bin";
        fd = $fopen(vec, "rb");
        if (fd == 0) begin $display("[FATAL] no %s", vec); $finish; end
        code = $fread(bytes, fd); $fclose(fd);
        N = le32(0, bytes);
        for (integer i = 0; i <= N*4; ++i)   // 1+4N 个 word，含最后一组 exp 的 word[4N]
            words[i] = le32(i*4, bytes);
        for (integer i = 0; i < N; ++i) begin
            case (words[1 + i*4])      // 布局：word0=N, word[1+4k]=kind, [2+4k]=a, [3+4k]=b, [4+4k]=exp
                32'd0: begin src_h[cnt_h] = i; exp_h[cnt_h] = words[4 + i*4]; cnt_h = cnt_h + 1; end
                32'd1: begin src_d[cnt_d] = i; exp_d[cnt_d] = words[4 + i*4]; cnt_d = cnt_d + 1; end
                32'd2: begin src_s[cnt_s] = i; exp_s[cnt_s] = words[4 + i*4]; cnt_s = cnt_s + 1; end
            endcase
        end
        $display("[TB] loaded N=%0d (h=%0d d=%0d s=%0d)", N, cnt_h, cnt_d, cnt_s);
        loaded_done = 1'b1;
    end

    // 完成判定
    initial begin
        while (oh < cnt_h || od < cnt_d || os_ < cnt_s) begin
            @(posedge clk);
            if ($time > 50_000_000) begin
                $display("[FATAL] timeout h=%0d/%0d d=%0d/%0d s=%0d/%0d", oh, cnt_h, od, cnt_d, os_, cnt_s);
                $finish;
            end
        end
        repeat (20) @(negedge clk);
        $display("======================================");
        $display("FP32 UNIT TB: err=%0d (h %0d/%0d d %0d/%0d s %0d/%0d)", err, oh, cnt_h, od, cnt_d, os_, cnt_s);
        if (err == 0 && oh == cnt_h && od == cnt_d && os_ == cnt_s)
            $display("TB RESULT: ALL FP32 UNIT TESTS PASSED");
        else
            $display("TB RESULT: FAILED");
        $finish;
    end

endmodule
