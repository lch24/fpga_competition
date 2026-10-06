`timescale 1ns/1ps
`include "calib_defs.vh"
module tb_corner_store;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0, clear = 0;
    reg corner_valid = 0, corner_last = 0;
    wire corner_ready;
    reg [7:0] corner_view_id = 0, corner_point_index = 0;
    reg [31:0] corner_x_fp32 = 0, corner_y_fp32 = 0;
    reg view_rsp_valid = 0;
    wire view_rsp_ready;
    reg [7:0] view_rsp_view_id = 0, view_rsp_status = 0;
    wire [2:0] view_done, view_format_error, view_usable;
    wire [23:0] view_status;
    wire [17:0] view_point_count;
    reg rd_en = 0;
    reg [1:0] rd_view_id = 0;
    reg [5:0] rd_point_index = 0;
    wire rd_valid;
    wire [31:0] rd_x_fp32, rd_y_fp32;
    integer errors = 0, checks = 0, scenarios = 0;
    reg tb_done = 0;
    integer report;
    integer v, p, k, round_id;
    reg [31:0] random_state = 32'h12345678;
    reg [31:0] expected_x [0:119];
    reg [31:0] expected_y [0:119];

    corner_store dut (
        .clk(clk), .rst_n(rst_n), .clear(clear),
        .corner_valid(corner_valid), .corner_ready(corner_ready),
        .corner_view_id(corner_view_id), .corner_point_index(corner_point_index),
        .corner_x_fp32(corner_x_fp32), .corner_y_fp32(corner_y_fp32),
        .corner_last(corner_last),
        .view_rsp_valid(view_rsp_valid), .view_rsp_ready(view_rsp_ready),
        .view_rsp_view_id(view_rsp_view_id), .view_rsp_status(view_rsp_status),
        .view_done(view_done), .view_status(view_status),
        .view_point_count(view_point_count), .view_format_error(view_format_error),
        .view_usable(view_usable), .rd_en(rd_en), .rd_view_id(rd_view_id),
        .rd_point_index(rd_point_index), .rd_valid(rd_valid),
        .rd_x_fp32(rd_x_fp32), .rd_y_fp32(rd_y_fp32)
    );

    task check;
        input condition;
        input [1023:0] message;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL %0t: %0s", $time, message);
                $fdisplay(report, "FAIL %0t: %0s", $time, message);
            end
        end
    endtask

    task start_case;
        input [1023:0] name;
        begin
            scenarios = scenarios + 1;
            $fdisplay(report, "CASE %0d: %0s", scenarios, name);
        end
    endtask

    task check_empty;
        begin
            check(view_done === 0 && view_status === 0 &&
                  view_point_count === 0 && view_format_error === 0 &&
                  view_usable === 0 && rd_valid === 0, "empty control state");
        end
    endtask

    task clear_store;
        begin
            @(negedge clk);
            corner_valid = 0; view_rsp_valid = 0; rd_en = 0; clear = 1;
            #1;
            check(!corner_ready && !view_rsp_ready && !rd_valid, "clear suppresses interfaces");
            @(posedge clk); #1; check_empty;
            @(negedge clk); clear = 0;
        end
    endtask

    task send_point;
        input integer view_num;
        input integer point_num;
        input [31:0] x, y;
        input last;
        begin
            @(negedge clk);
            corner_valid = 1; corner_view_id = view_num;
            corner_point_index = point_num; corner_x_fp32 = x;
            corner_y_fp32 = y; corner_last = last;
            #1; check(corner_ready, "point accepted without unexpected backpressure");
            @(posedge clk); #1;
            @(negedge clk); corner_valid = 0;
        end
    endtask

    task send_result;
        input integer view_num;
        input [7:0] status;
        begin
            @(negedge clk);
            view_rsp_valid = 1; view_rsp_view_id = view_num; view_rsp_status = status;
            #1; check(view_rsp_ready, "result accepted even when point count is incomplete");
            @(posedge clk); #1;
            check(view_done[view_num] === 1'b1, "view done");
            check(view_status[8*view_num +: 8] === status, "raw detector status preserved");
            @(negedge clk); view_rsp_valid = 0;
        end
    endtask

    task read_point;
        input integer view_num;
        input integer point_num;
        input [31:0] x, y;
        begin
            @(negedge clk);
            rd_en = 1; rd_view_id = view_num; rd_point_index = point_num;
            #1; check(rd_valid === 0, "no combinational read response");
            @(posedge clk); #1;
            check(rd_valid === 1 && rd_x_fp32 === x && rd_y_fp32 === y,
                  "one-cycle read matches independent expected coordinates");
            @(negedge clk); rd_en = 0;
            @(posedge clk); #1; check(rd_valid === 0, "read valid is a pulse");
        end
    endtask

    // Deliberately illegal reads test defensive suppression, not normal API use.
    task rejected_read;
        input integer view_num;
        input integer point_num;
        begin
            @(negedge clk); rd_en = 1; rd_view_id = view_num; rd_point_index = point_num;
            @(posedge clk); #1; check(rd_valid === 0, "illegal/uncommitted read suppressed");
            @(negedge clk); rd_en = 0;
        end
    endtask

    task bad_point_case;
        input integer point_num;
        input [31:0] x, y;
        input last;
        begin
            clear_store;
            send_point(0, point_num, x, y, last);
            check(view_format_error === 3'b001 && view_point_count === 0 &&
                  view_usable === 0, "malformed point rejected without RAM count advance");
            check(!corner_ready, "poisoned view blocks further points");
            send_result(0, `PAR_NO_BOARD);
            check(view_format_error === 3'b001 && !view_usable[0],
                  "failure preserves sticky format error");
        end
    endtask

    initial begin
        report = $fopen("results.txt", "w");
        if (!report) begin $display("Cannot open results.txt"); $stop; end
        start_case("reset");
        repeat (2) @(posedge clk);
        #1; check_empty;
        check(!corner_ready && !view_rsp_ready, "reset gates inputs");
        @(negedge clk); rst_n = 1;

        // Four jobs, all three views, idle gaps, different finite FP32 bit patterns.
        for (round_id = 0; round_id < 4; round_id = round_id + 1) begin
            start_case("complete job with gaps and permuted random access");
            clear_store;
            for (v = 0; v < 3; v = v + 1) begin
                for (p = 0; p < 40; p = p + 1) begin
                    random_state = random_state * 32'd1664525 + 32'd1013904223;
                    repeat (random_state[1:0]) @(negedge clk);
                    expected_x[v*40+p] = 32'h40000000 | (random_state & 32'h007fffff);
                    expected_y[v*40+p] = 32'h42000000 | ((random_state >> 3) & 32'h007fffff);
                    send_point(v, p, expected_x[v*40+p], expected_y[v*40+p], p == 39);
                    check(view_point_count[6*v +: 6] == p+1, "per-view count increments");
                    check(view_done[v] === 0 && view_usable[v] === 0, "not committed before result");
                end
                send_result(v, `PAR_OK);
                check(view_usable[v] === 1 && view_format_error[v] === 0, "view committed");
            end
            check(view_done === 7 && view_usable === 7, "all three views usable");
            for (k = 0; k < 120; k = k + 1) begin
                // 37 and 120 are coprime: visits each stored point exactly once.
                p = (k * 37 + round_id * 11) % 120;
                read_point(p/40, p%40, expected_x[p], expected_y[p]);
            end
            check(view_usable === 7 && view_done === 7, "reads preserve debug state");
        end

        start_case("duplicate completion and writes after done");
        @(negedge clk);
        view_rsp_valid = 1; view_rsp_view_id = 0; view_rsp_status = `PAR_NO_BOARD;
        corner_valid = 1; corner_view_id = 0; corner_point_index = 0;
        #1; check(!corner_ready && !view_rsp_ready, "done view backpressures duplicates");
        repeat (3) @(posedge clk);
        #1; check(view_status === 0 && view_usable === 7, "duplicates do not overwrite success");
        @(negedge clk); view_rsp_valid = 0; corner_valid = 0;
        rejected_read(3, 0); rejected_read(0, 40); rejected_read(2, 63);

        start_case("clear cancels read and concurrent writes/results");
        @(negedge clk);
        rd_en = 1; rd_view_id = 0; rd_point_index = 0;
        @(posedge clk); #1; check(rd_valid, "read present before clear");
        @(negedge clk);
        clear = 1; corner_valid = 1; view_rsp_valid = 1;
        #1; check(!rd_valid && !corner_ready && !view_rsp_ready, "clear has priority");
        @(posedge clk); #1; check_empty;
        @(negedge clk); clear = 0; corner_valid = 0; view_rsp_valid = 0; rd_en = 0;
        rejected_read(0, 0);

        start_case("partial detector failure and per-view independence");
        for (p = 0; p < 40; p = p + 1)
            send_point(0, p, 32'h3f800000+p, 32'h40000000+p, p == 39);
        send_result(0, `PAR_OK);
        for (p = 0; p < 7; p = p + 1)
            send_point(1, p, 32'h3f800000, 32'h40000000, 0);
        send_result(1, `PAR_NO_BOARD);
        check(view_done === 3 && view_usable === 1 && view_format_error === 0,
              "failed view leaves prior successful view intact");
        check(view_point_count[11:6] == 7 && view_status[15:8] == `PAR_NO_BOARD,
              "partial count and failure retained");
        read_point(0, 39, 32'h3f800027, 32'h40000027);
        rejected_read(1, 0);
        send_result(2, `PAR_NO_BOARD);
        check(view_done === 7 && view_point_count[17:12] == 0, "zero-point failure completes");

        start_case("early success notification");
        clear_store; send_point(0, 0, 32'h3f800000, 32'h40000000, 0);
        send_result(0, `PAR_OK);
        check(view_done[0] && view_status[7:0] == 0 && view_format_error[0] &&
              !view_usable[0], "early success is format error, raw status preserved");

        start_case("wrong index"); bad_point_case(1, 32'h3f800000, 32'h40000000, 0);
        start_case("out-of-range point"); bad_point_case(255, 32'h3f800000, 32'h40000000, 0);
        start_case("early last"); bad_point_case(0, 32'h3f800000, 32'h40000000, 1);
        start_case("NaN x"); bad_point_case(0, 32'h7fc00001, 32'h40000000, 0);
        start_case("positive Inf y"); bad_point_case(0, 32'h3f800000, 32'h7f800000, 0);
        start_case("negative Inf x"); bad_point_case(0, 32'hff800000, 32'h40000000, 0);

        start_case("missing last");
        clear_store;
        for (p = 0; p < 39; p = p + 1) send_point(0, p, 0, 32'h80000000, 0);
        send_point(0, 39, 0, 32'h80000000, 0);
        check(view_point_count[5:0] == 39 && view_format_error[0], "bad last not written");

        start_case("extra point before completion");
        clear_store;
        for (p = 0; p < 40; p = p + 1) send_point(0, p, 32'h00000001, 0, p == 39);
        send_point(0, 40, 0, 0, 0);
        check(view_point_count[5:0] == 40 && view_format_error[0], "count saturates at forty");
        send_result(0, `PAR_OK); check(!view_usable[0], "overflow invalidates view");

        start_case("out-of-order view data and result");
        clear_store; send_point(1, 0, 0, 0, 0);
        check(view_format_error === 2 && view_point_count === 0, "wrong view attributed safely");
        clear_store; send_result(2, `PAR_NO_BOARD);
        check(view_done === 4 && view_format_error === 4, "out-of-order result flagged");

        start_case("illegal view IDs never alias legal view");
        clear_store;
        @(negedge clk);
        corner_valid = 1; corner_view_id = 8'd255; corner_point_index = 0;
        view_rsp_valid = 1; view_rsp_view_id = 8'd3; view_rsp_status = 0;
        #1; check(!corner_ready && !view_rsp_ready, "illegal view IDs backpressured");
        @(posedge clk); #1; check_empty;
        @(negedge clk); corner_valid = 0; view_rsp_valid = 0;

        start_case("simultaneous final point and success: write priority");
        clear_store;
        for (p = 0; p < 39; p = p + 1) send_point(0, p, 0, 0, 0);
        @(negedge clk);
        corner_valid = 1; corner_view_id = 0; corner_point_index = 39;
        corner_last = 1; corner_x_fp32 = 32'h3f800000; corner_y_fp32 = 32'h40000000;
        view_rsp_valid = 1; view_rsp_view_id = 0; view_rsp_status = 0;
        #1; check(corner_ready && !view_rsp_ready, "same-view result held behind point");
        @(posedge clk); #1;
        check(view_point_count[5:0] == 40 && !view_done[0], "last point accepted first");
        @(negedge clk); corner_valid = 0;
        #1; check(view_rsp_ready, "held result accepted next");
        @(posedge clk); #1; check(view_usable[0] && !view_format_error[0], "concurrent boundary succeeds");
        @(negedge clk); view_rsp_valid = 0;
        read_point(0, 39, 32'h3f800000, 32'h40000000);

        start_case("asynchronous reset cancels read and restores next task");
        @(negedge clk); rd_en = 1; rd_view_id = 0; rd_point_index = 39;
        @(posedge clk); #1; check(rd_valid, "read in flight before reset");
        #1; rst_n = 0;
        #1; check_empty; check(!corner_ready && !view_rsp_ready, "reset blocks handshakes");
        @(negedge clk); rd_en = 0; rst_n = 1;
        send_point(0, 0, 32'h80000000, 32'h00000001, 0);
        check(view_point_count[5:0] == 1 && !view_format_error[0],
              "new task after reset; negative zero and subnormal are finite");
        send_result(0, `PAR_NO_BOARD);

        $fdisplay(report, "RESULT scenarios=%0d checks=%0d errors=%0d", scenarios, checks, errors);
        $fclose(report);
        tb_done = 1;
    end

    initial begin
        #100000;
        if (!tb_done) begin
            check(0, "watchdog timeout");
            $fclose(report);
            tb_done = 1;
        end
    end
endmodule
