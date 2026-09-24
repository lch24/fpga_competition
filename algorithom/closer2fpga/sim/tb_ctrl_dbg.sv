`timescale 1ns / 1ps
//==============================================================================
// tb_ctrl_dbg.sv — ctrl 死锁最小复现（W=4,H=4，直连假 resp 源，逐拍打印）
//------------------------------------------------------------------------------
// 目的：定位 shi_tomasi_ctrl 在 Pass2 的死锁。resp 由 TB 顺序给出（绕过 me），
//   rmax 直接用外部值（0x41880000=16，>0 进入 Pass2），thr 用低值保证出候选。
// 打印：state / r_addr / wout_n / window out / fifo / 是否 done。
//==============================================================================
module tb_ctrl_dbg;

    localparam integer W = 4, H = 4, PIXELS = W * H;
    localparam integer CLK_PERIOD = 10;

    logic clk = 1'b0;
    logic rst_n;
    always #(CLK_PERIOD/2) clk = ~clk;

    logic        resp_v, resp_rdy, resp_iready;
    logic [31:0] resp_d;
    integer      resp_n = 0;

    logic        start, busy, done;
    logic [1:0]  status;
    logic        cand_v;
    logic [10:0] cand_x, cand_y;
    logic [15:0] cand_total;

    shi_tomasi_ctrl #(.IMG_W(W), .IMG_H(H)) u_ctrl (
        .clk(clk), .rst_n(rst_n),
        .start(start), .busy(busy), .thr(32'h3F000000),   // 0.5
        .resp_valid(resp_v), .resp_rdy(resp_rdy), .resp_data(resp_d),
        .resp_in_ready(resp_iready),
        .mem_addr(), .mem_data(),
        .done(done), .status(status),
        .cand_valid(cand_v), .cand_ready(1'b1),
        .cand_x(cand_x), .cand_y(cand_y),
        .cand_total(cand_total)
    );

    // resp 源：固定值 16.0（fp32 0x41800000），4 个角点（>0.5）
    // 喂满 PIXELS 后停。
    always @(posedge clk) begin
        if (!rst_n) begin
            resp_n <= 0;
            resp_v <= 1'b0;
        end else begin
            if (busy && resp_rdy && resp_n < PIXELS) begin
                resp_v  <= 1'b1;
                resp_d  <= 32'h41800000;   // 16.0
                resp_n  <= resp_n + 1;
            end else if (resp_n >= PIXELS) begin
                resp_v <= 1'b0;
            end
        end
    end
    assign resp_iready = 1'b1;

    // 逐拍打印（前 400 拍）
    integer dbg_n = 0;
    always @(posedge clk) begin
        if (rst_n && dbg_n < 400) begin
            if (busy || done)
                $display("[%0t] st=%0d r_addr=%0d wout_n=%0d resp_n=%0d resp_rdy=%b w_v=%b w_rdy=%b nms_v=%b busy=%b done=%b status=%b",
                         $time, u_ctrl.state, u_ctrl.r_addr, u_ctrl.wout_n, resp_n,
                         resp_rdy, u_ctrl.win2w_in_valid, u_ctrl.win2w_in_ready,
                         u_ctrl.wout_valid, busy, done, status);
            dbg_n = dbg_n + 1;
        end
    end

    initial begin
        start = 1'b0;
        resp_v = 1'b0; resp_d = 0;
        rst_n = 1'b0;
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        wait (done);
        repeat (3) @(negedge clk);
        $display("=== CTRL-DBG DONE: status=%b cand_total=%0d ===", status, cand_total);
        $finish;
    end

    initial begin
        #100_000;
        $display("[TB][FATAL] timeout: st=%0d r_addr=%0d wout_n=%0d", u_ctrl.state, u_ctrl.r_addr, u_ctrl.wout_n);
        $finish;
    end

endmodule