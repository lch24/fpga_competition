`timescale 1ns/1ps

module tb_map_build_ctrl;
    logic clk = 1'b0;
    logic rst_n = 1'b0;
    always #5 clk = ~clk;

    logic cmd_valid;
    wire  cmd_ready;
    logic [31:0] cmd_job_id, cmd_calib_id;
    logic [15:0] cmd_width, cmd_height;
    logic cmd_camera_valid;
    logic [31:0] cmd_map_x_base, cmd_map_y_base, cmd_map_stride_bytes;
    logic [31:0] cmd_fx, cmd_fy, cmd_cx, cmd_cy;
    logic [31:0] cmd_k1, cmd_k2, cmd_k3, cmd_p1, cmd_p2;

    wire core_cfg_valid;
    logic core_cfg_ready;
    wire [31:0] core_cfg_fx, core_cfg_fy, core_cfg_cx, core_cfg_cy;
    wire [31:0] core_cfg_k1, core_cfg_k2, core_cfg_k3, core_cfg_p1, core_cfg_p2;
    wire core_in_valid;
    wire core_in_ready;
    wire [15:0] core_in_x, core_in_y;
    wire [31:0] core_in_pixel_id;
    wire core_in_last;
    logic core_out_valid;
    wire core_out_ready;
    logic [31:0] core_out_src_x, core_out_src_y;
    logic [15:0] core_out_dst_x, core_out_dst_y;
    logic [31:0] core_out_pixel_id;
    logic core_out_last, core_out_error;

    wire writer_cfg_valid;
    logic writer_cfg_ready;
    wire [31:0] writer_cfg_job_id, writer_cfg_calib_id;
    wire [15:0] writer_cfg_width, writer_cfg_height;
    wire [31:0] writer_cfg_map_x_base, writer_cfg_map_y_base, writer_cfg_stride_bytes;
    wire map_valid;
    logic map_ready;
    wire [31:0] map_src_x, map_src_y;
    wire [15:0] map_dst_x, map_dst_y;
    wire [31:0] map_pixel_id;
    wire map_last, map_error;
    logic writer_rsp_valid;
    wire writer_rsp_ready;
    logic writer_rsp_error;

    wire rsp_valid;
    logic rsp_ready;
    wire [31:0] rsp_job_id, rsp_calib_id;
    wire [7:0] rsp_status;
    wire busy;

    map_build_ctrl dut (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .cmd_job_id(cmd_job_id), .cmd_calib_id(cmd_calib_id),
        .cmd_width(cmd_width), .cmd_height(cmd_height),
        .cmd_camera_valid(cmd_camera_valid),
        .cmd_map_x_base(cmd_map_x_base), .cmd_map_y_base(cmd_map_y_base),
        .cmd_map_stride_bytes(cmd_map_stride_bytes),
        .cmd_fx(cmd_fx), .cmd_fy(cmd_fy), .cmd_cx(cmd_cx), .cmd_cy(cmd_cy),
        .cmd_k1(cmd_k1), .cmd_k2(cmd_k2), .cmd_k3(cmd_k3),
        .cmd_p1(cmd_p1), .cmd_p2(cmd_p2),
        .core_cfg_valid(core_cfg_valid), .core_cfg_ready(core_cfg_ready),
        .core_cfg_fx(core_cfg_fx), .core_cfg_fy(core_cfg_fy),
        .core_cfg_cx(core_cfg_cx), .core_cfg_cy(core_cfg_cy),
        .core_cfg_k1(core_cfg_k1), .core_cfg_k2(core_cfg_k2),
        .core_cfg_k3(core_cfg_k3), .core_cfg_p1(core_cfg_p1),
        .core_cfg_p2(core_cfg_p2),
        .core_in_valid(core_in_valid), .core_in_ready(core_in_ready),
        .core_in_x(core_in_x), .core_in_y(core_in_y),
        .core_in_pixel_id(core_in_pixel_id), .core_in_last(core_in_last),
        .core_out_valid(core_out_valid), .core_out_ready(core_out_ready),
        .core_out_src_x(core_out_src_x), .core_out_src_y(core_out_src_y),
        .core_out_dst_x(core_out_dst_x), .core_out_dst_y(core_out_dst_y),
        .core_out_pixel_id(core_out_pixel_id), .core_out_last(core_out_last),
        .core_out_error(core_out_error),
        .writer_cfg_valid(writer_cfg_valid), .writer_cfg_ready(writer_cfg_ready),
        .writer_cfg_job_id(writer_cfg_job_id), .writer_cfg_calib_id(writer_cfg_calib_id),
        .writer_cfg_width(writer_cfg_width), .writer_cfg_height(writer_cfg_height),
        .writer_cfg_map_x_base(writer_cfg_map_x_base),
        .writer_cfg_map_y_base(writer_cfg_map_y_base),
        .writer_cfg_stride_bytes(writer_cfg_stride_bytes),
        .map_valid(map_valid), .map_ready(map_ready),
        .map_src_x(map_src_x), .map_src_y(map_src_y),
        .map_dst_x(map_dst_x), .map_dst_y(map_dst_y),
        .map_pixel_id(map_pixel_id), .map_last(map_last), .map_error(map_error),
        .writer_rsp_valid(writer_rsp_valid), .writer_rsp_ready(writer_rsp_ready),
        .writer_rsp_error(writer_rsp_error),
        .rsp_valid(rsp_valid), .rsp_ready(rsp_ready),
        .rsp_job_id(rsp_job_id), .rsp_calib_id(rsp_calib_id),
        .rsp_status(rsp_status), .busy(busy)
    );

    // One-entry mock coordinate core. It preserves tags and makes the payload
    // visibly dependent on x/y; arithmetic correctness is tested separately.
    assign core_in_ready = !core_out_valid || core_out_ready;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            core_out_valid    <= 1'b0;
            core_out_src_x    <= 32'd0;
            core_out_src_y    <= 32'd0;
            core_out_dst_x    <= 16'd0;
            core_out_dst_y    <= 16'd0;
            core_out_pixel_id <= 32'd0;
            core_out_last     <= 1'b0;
            core_out_error    <= 1'b0;
        end else begin
            if (core_out_valid && core_out_ready)
                core_out_valid <= 1'b0;
            if (core_in_valid && core_in_ready) begin
                core_out_valid    <= 1'b1;
                core_out_src_x    <= {16'd0, core_in_x};
                core_out_src_y    <= {16'd0, core_in_y};
                core_out_dst_x    <= core_in_x;
                core_out_dst_y    <= core_in_y;
                core_out_pixel_id <= core_in_pixel_id;
                core_out_last     <= core_in_last;
                core_out_error    <= 1'b0;
            end
        end
    end

    logic [15:0] lfsr;
    integer accepted_count;
    integer writer_countdown;

    // Map-writer mock applies deterministic backpressure and checks raster order.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lfsr             <= 16'h1ace;
            map_ready        <= 1'b0;
            accepted_count   <= 0;
            writer_countdown <= -1;
            writer_rsp_valid <= 1'b0;
            writer_rsp_error <= 1'b0;
        end else begin
            lfsr      <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
            map_ready <= lfsr[0] | lfsr[3];

            if (map_valid && map_ready) begin
                if (map_pixel_id !== accepted_count[31:0])
                    $fatal(1, "pixel_id mismatch: got %0d expected %0d", map_pixel_id, accepted_count);
                if (map_dst_x !== {14'd0, accepted_count[1:0]} ||
                    map_dst_y !== accepted_count[17:2])
                    $fatal(1, "raster coordinate mismatch at pixel %0d", accepted_count);
                if (map_last !== (accepted_count == 11))
                    $fatal(1, "last mismatch at pixel %0d", accepted_count);
                accepted_count <= accepted_count + 1;
                if (map_last)
                    writer_countdown <= 3;
            end

            if (writer_countdown > 0)
                writer_countdown <= writer_countdown - 1;
            else if (writer_countdown == 0) begin
                writer_rsp_valid <= 1'b1;
                writer_countdown <= -1;
            end

            if (writer_rsp_valid && writer_rsp_ready)
                writer_rsp_valid <= 1'b0;
        end
    end

    task automatic send_command(
        input logic [31:0] job_id,
        input logic [15:0] width,
        input logic [15:0] height,
        input logic camera_valid,
        input logic [31:0] stride
    );
        begin
            @(negedge clk);
            cmd_job_id          = job_id;
            cmd_calib_id        = 32'h1234_5678;
            cmd_width           = width;
            cmd_height          = height;
            cmd_camera_valid    = camera_valid;
            cmd_map_stride_bytes = stride;
            cmd_valid           = 1'b1;
            do @(posedge clk); while (!cmd_ready);
            @(negedge clk);
            cmd_valid = 1'b0;
        end
    endtask

    initial begin
        cmd_valid = 1'b0;
        cmd_job_id = 32'd0;
        cmd_calib_id = 32'd0;
        cmd_width = 16'd0;
        cmd_height = 16'd0;
        cmd_camera_valid = 1'b0;
        cmd_map_x_base = 32'h0100_0000;
        cmd_map_y_base = 32'h0140_0000;
        cmd_map_stride_bytes = 32'd0;
        cmd_fx = 32'h44ab_ed92;
        cmd_fy = 32'h44ab_52f3;
        cmd_cx = 32'h4425_794f;
        cmd_cy = 32'h4351_2f4f;
        cmd_k1 = 32'hbe82_16e0;
        cmd_k2 = 32'h3d27_a3d3;
        cmd_k3 = 32'h0000_0000;
        cmd_p1 = 32'h3c27_14ed;
        cmd_p2 = 32'hba3f_1f0f;
        core_cfg_ready = 1'b1;
        writer_cfg_ready = 1'b1;
        rsp_ready = 1'b1;

        repeat (4) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        send_command(32'd7, 16'd4, 16'd3, 1'b1, 32'd16);
        wait (rsp_valid);
        if (rsp_status !== 8'd0 || rsp_job_id !== 32'd7 || accepted_count !== 12)
            $fatal(1, "valid map job failed");
        @(posedge clk);

        // Stride is smaller than 4*width, so no core/writer traffic is allowed.
        send_command(32'd8, 16'd4, 16'd3, 1'b1, 32'd12);
        wait (rsp_valid);
        if (rsp_status !== 8'd1)
            $fatal(1, "bad stride was not rejected");
        @(posedge clk);

        send_command(32'd9, 16'd4, 16'd3, 1'b0, 32'd16);
        wait (rsp_valid);
        if (rsp_status !== 8'd4)
            $fatal(1, "invalid camera packet was not rejected");

        $display("PASS: map_build_ctrl raster, backpressure, and completion barrier");
        $finish;
    end

endmodule
