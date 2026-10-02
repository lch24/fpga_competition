`timescale 1ns / 1ps
//==============================================================================
// tb_fp64_adddiv.sv — fp64_add / fp64_div 单元对拍（M5 基础件）
//------------------------------------------------------------------------------
// 向量：../tests/build/vectors/test_fp64.bin（gen_fp64_vectors.cpp 生成）
//   格式（小端）：u32 N，随后 N×7 {kind, a_lo, a_hi, b_lo, b_hi, exp_lo, exp_hi}
//     kind: 0=add(a,b) 1=div(a,b)；u64 = {hi, lo}
// 两个模块独立驱动、独立比对；期望按 kind 预分组。
//==============================================================================
module tb_fp64_adddiv;
    localparam integer CLK_PERIOD = 10;
    logic clk = 1'b0;
    logic rst_n;
    always #(CLK_PERIOD/2) clk = ~clk;

    integer fd, code;
    reg [7:0]  bytes [0:8388607];
    reg [31:0] words [0:2097151];
    integer N;

    function automatic [31:0] le32(input integer i, input reg [7:0] arr[]);
        le32 = {arr[i+3], arr[i+2], arr[i+1], arr[i]};
    endfunction

    // 两 DUT（out_valid 与 out_r 分开）
    logic        av, ardy, ar;
    logic [63:0] aa, ab, ar_d;
    logic        dv, drdy, dr;
    logic [63:0] da, db, dr_d;
    fp64_add u_add (.clk(clk), .rst_n(rst_n),
        .in_valid(av), .in_ready(ardy), .in_a(aa), .in_b(ab),
        .out_valid(ar), .out_ready(1'b1), .out_r(ar_d));
    fp64_div u_div (.clk(clk), .rst_n(rst_n),
        .in_valid(dv), .in_ready(drdy), .in_a(da), .in_b(db),
        .out_valid(dr), .out_ready(1'b1), .out_r(dr_d));

    // 期望分组（按 kind 顺序）
    integer src_a [0:262143]; integer src_d [0:262143];
    logic [63:0] exp_a [0:262143]; logic [63:0] exp_d [0:262143];
    integer cnt_a = 0, cnt_d = 0;

    // 输出比对 + 计数
    integer err = 0, oa = 0, od = 0;
    always @(posedge clk) begin
        if (rst_n) begin
            if (ar) begin
                if (oa >= cnt_a || ar_d !== exp_a[oa]) begin
                    err = err + 1;
                    if (err <= 10) $display("[FAIL] add #%0d rtl=0x%016x exp=0x%016x", oa, ar_d, exp_a[oa]);
                end
                oa = oa + 1;
            end
            if (dr) begin
                if (od >= cnt_d || dr_d !== exp_d[od]) begin
                    err = err + 1;
                    if (err <= 10) $display("[FAIL] div #%0d rtl=0x%016x exp=0x%016x", od, dr_d, exp_d[od]);
                end
                od = od + 1;
            end
        end
    end

    // 两路独立驱动（等待向量加载完成 + 复位释放）
    integer pi_a = 0, pi_d = 0;
    reg loaded_done = 1'b0;
    initial begin
        rst_n = 1'b0; av = 0; dv = 0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);
    end
    initial begin
        wait (rst_n && loaded_done);
        while (pi_a < cnt_a) begin
            if (!av || ardy) begin
                aa <= {words[3 + src_a[pi_a]*7], words[2 + src_a[pi_a]*7]};
                ab <= {words[5 + src_a[pi_a]*7], words[4 + src_a[pi_a]*7]};
                av <= 1'b1;
                @(negedge clk);
                pi_a = pi_a + 1;
                av <= 1'b0;
            end else begin
                @(negedge clk);
            end
        end
        av <= 1'b0;
        $display("[TB] add fed %0d", cnt_a);
    end
    initial begin
        wait (rst_n && loaded_done);
        while (pi_d < cnt_d) begin
            if (!dv || drdy) begin
                da <= {words[3 + src_d[pi_d]*7], words[2 + src_d[pi_d]*7]};
                db <= {words[5 + src_d[pi_d]*7], words[4 + src_d[pi_d]*7]};
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

    // 加载向量（先于驱动；+VEC= 可选向量文件）
    initial begin
        string vec = "../tests/build/vectors/test_fp64.bin";
        if (!$value$plusargs("VEC=%s", vec))
            vec = "../tests/build/vectors/test_fp64.bin";
        fd = $fopen(vec, "rb");
        if (fd == 0) begin $display("[FATAL] no %s", vec); $finish; end
        code = $fread(bytes, fd); $fclose(fd);
        N = le32(0, bytes);
        for (integer i = 0; i <= N*7; ++i)   // 1+7N 个 word，含最后一组 exp 的 word[7N]
            words[i] = le32(i*4, bytes);
        for (integer i = 0; i < N; ++i) begin
            case (words[1 + i*7])      // 布局：word0=N, word[1+7k]=kind, [2..3]=a, [4..5]=b, [6..7]=exp
                32'd0: begin src_a[cnt_a] = i; exp_a[cnt_a] = {words[7 + i*7], words[6 + i*7]}; cnt_a = cnt_a + 1; end
                32'd1: begin src_d[cnt_d] = i; exp_d[cnt_d] = {words[7 + i*7], words[6 + i*7]}; cnt_d = cnt_d + 1; end
            endcase
        end
        $display("[TB] loaded N=%0d (a=%0d d=%0d)", N, cnt_a, cnt_d);
        loaded_done = 1'b1;
    end

    // 完成判定
    initial begin
        while (oa < cnt_a || od < cnt_d) begin
            @(posedge clk);
            if ($time > 100_000_000) begin
                $display("[FATAL] timeout a=%0d/%0d d=%0d/%0d", oa, cnt_a, od, cnt_d);
                $finish;
            end
        end
        repeat (20) @(negedge clk);
        $display("======================================");
        $display("FP64 UNIT TB: err=%0d (a %0d/%0d d %0d/%0d)", err, oa, cnt_a, od, cnt_d);
        if (err == 0 && oa == cnt_a && od == cnt_d)
            $display("TB RESULT: ALL FP64 ADD/DIV TESTS PASSED");
        else
            $display("TB RESULT: FAILED");
        $finish;
    end

endmodule
