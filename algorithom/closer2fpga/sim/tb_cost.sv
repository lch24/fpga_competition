`timescale 1ns / 1ps
//==============================================================================
// tb_cost.sv — M4 grid_validate 位级对拍
//------------------------------------------------------------------------------
// 向量：tests/build/vectors/m4_cost_<cs>.bin（cs=0..5）=
//   u32 n + n×{x,y} + u32 cost_bits（1e30 表示无效）。
// 流程：点集灌入网格 RAM → start grid_validate(n_in=n) → 比对 cost_out。
// 要求：6 组全过（含合法 0.0185 与非法 1e30）。
//==============================================================================
module tb_cost;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg        rst_n;
    // 网格 RAM（x/y 双口，N_ADDR_W=6 → 64 深度）
    reg        pw_en;
    reg [5:0]  pw_addr;
    reg [31:0] pw_x, pw_y;
    wire       rd_en;
    wire [5:0] rd_addr;
    wire [31:0] rd_x, rd_y;

    dual_port_ram #(.DATA_WIDTH(32), .ADDR_WIDTH(6)) u_rx (
        .clk(clk), .rst_n(rst_n),
        .wr_en(pw_en), .wr_addr(pw_addr), .wr_data(pw_x),
        .rd_en(rd_en), .rd_addr(rd_addr), .rd_data(rd_x)
    );
    dual_port_ram #(.DATA_WIDTH(32), .ADDR_WIDTH(6)) u_ry (
        .clk(clk), .rst_n(rst_n),
        .wr_en(pw_en), .wr_addr(pw_addr), .wr_data(pw_y),
        .rd_en(rd_en), .rd_addr(rd_addr), .rd_data(rd_y)
    );

    reg        start;
    wire       busy, done;
    reg  [15:0] n_in;
    wire       valid_out;
    wire [31:0] cost_out;

    grid_validate #(.ROWS(5), .COLS(8), .N_ADDR_W(6)) u_dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .busy(busy), .done(done),
        .n_in(n_in),
        .rd_en(rd_en), .rd_addr(rd_addr), .rd_x(rd_x), .rd_y(rd_y),
        .valid_out(valid_out), .cost_out(cost_out)
    );

    //--------------------------------------------------------------------
    reg [7:0] fbuf[0:1048575];
    reg [31:0] pts_x[0:63], pts_y[0:63];
    integer   N_in, k, cs, fd, code, fail_cnt;
    reg [31:0] exp_cost;

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

    // 灌点入 RAM
    task load_pts();
        integer m;
        begin
            for (m = 0; m < N_in; m = m + 1) begin
                pw_en   = 1'b1;
                pw_addr = m[5:0];
                pw_x    = pts_x[m];
                pw_y    = pts_y[m];
                @(negedge clk);
            end
            pw_en = 1'b0;
        end
    endtask

    // 单组：读文件 → 灌 RAM → start → 等 done → 比对
    task run_case(input integer cs_id);
        string path;
        begin
            path = $sformatf("../tests/build/vectors/m4_cost_%0d.bin", cs_id);
            read_file(path);
            N_in = rd32(0);
            for (k = 0; k < N_in; k = k + 1) begin
                pts_x[k] = rd32(4 + k * 8);
                pts_y[k] = rd32(4 + k * 8 + 4);
            end
            exp_cost = rd32(4 + N_in * 8);
            load_pts();
            n_in   = N_in[15:0];
            start  = 1'b1;
            @(negedge clk);
            start  = 1'b0;
            while (!done) @(negedge clk);
            if (valid_out && cost_out == exp_cost) begin
                $display("[TB][cs%0d] cost got=%08x exp=%08x MATCH", cs_id, cost_out, exp_cost);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[TB][FAIL][cs%0d] cost got=%08x exp=%08x", cs_id, cost_out, exp_cost);
            end
            @(negedge clk);
        end
    endtask

    initial begin
        pw_en    = 1'b0;
        pw_addr  = 6'd0;
        pw_x     = 32'd0;
        pw_y     = 32'd0;
        start    = 1'b0;
        n_in     = 16'd0;
        fail_cnt = 0;

        rst_n = 1'b0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        for (cs = 0; cs < 6; cs = cs + 1)
            run_case(cs);

        $display("==============================================");
        if (fail_cnt == 0)
            $display("TB RESULT: cost ALL PASSED (6/6)");
        else
            $display("TB RESULT: cost FAIL %0d/6", fail_cnt);
        $finish;
    end

    // 看门狗
    initial begin
        #500_000_000;
        $display("[TB][FATAL] watchdog timeout");
        $finish;
    end

endmodule
