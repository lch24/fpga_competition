`timescale 1ns/1ps
`include "vision_defs.vh"

// Controls one complete map-build job after a valid camera parameter packet
// has already been received. This module contains no floating-point arithmetic
// and no DDR-controller-specific logic.
//
// The coordinate core and map writer are configured once per job. Raster input
// tokens may be backpressured independently from result tokens. A successful
// response is emitted only after the map writer reports that both DDR planes
// have completed.
module map_build_ctrl (
    input  wire        clk,
    input  wire        rst_n,

    // ---------------- Job command ----------------
    input  wire        cmd_valid,
    output wire        cmd_ready,
    input  wire [31:0] cmd_job_id,
    input  wire [31:0] cmd_calib_id,
    input  wire [15:0] cmd_width,
    input  wire [15:0] cmd_height,
    input  wire        cmd_camera_valid,
    input  wire [31:0] cmd_map_x_base,
    input  wire [31:0] cmd_map_y_base,
    input  wire [31:0] cmd_map_stride_bytes,
    input  wire [31:0] cmd_fx,
    input  wire [31:0] cmd_fy,
    input  wire [31:0] cmd_cx,
    input  wire [31:0] cmd_cy,
    input  wire [31:0] cmd_k1,
    input  wire [31:0] cmd_k2,
    input  wire [31:0] cmd_k3,
    input  wire [31:0] cmd_p1,
    input  wire [31:0] cmd_p2,

    // ---------------- Coordinate-core configuration ----------------
    output wire        core_cfg_valid,
    input  wire        core_cfg_ready,
    output wire [31:0] core_cfg_fx,
    output wire [31:0] core_cfg_fy,
    output wire [31:0] core_cfg_cx,
    output wire [31:0] core_cfg_cy,
    output wire [31:0] core_cfg_k1,
    output wire [31:0] core_cfg_k2,
    output wire [31:0] core_cfg_k3,
    output wire [31:0] core_cfg_p1,
    output wire [31:0] core_cfg_p2,

    // ---------------- Raster tokens into coordinate core ----------------
    output wire        core_in_valid,
    input  wire        core_in_ready,
    output wire [15:0] core_in_x,
    output wire [15:0] core_in_y,
    output wire [31:0] core_in_pixel_id,
    output wire        core_in_last,

    // ---------------- Results returned by coordinate core ----------------
    input  wire        core_out_valid,
    output wire        core_out_ready,
    input  wire [31:0] core_out_src_x,
    input  wire [31:0] core_out_src_y,
    input  wire [15:0] core_out_dst_x,
    input  wire [15:0] core_out_dst_y,
    input  wire [31:0] core_out_pixel_id,
    input  wire        core_out_last,
    input  wire        core_out_error,

    // ---------------- Map-writer configuration ----------------
    output wire        writer_cfg_valid,
    input  wire        writer_cfg_ready,
    output wire [31:0] writer_cfg_job_id,
    output wire [31:0] writer_cfg_calib_id,
    output wire [15:0] writer_cfg_width,
    output wire [15:0] writer_cfg_height,
    output wire [31:0] writer_cfg_map_x_base,
    output wire [31:0] writer_cfg_map_y_base,
    output wire [31:0] writer_cfg_stride_bytes,

    // ---------------- Ordered coordinate stream to map writer ----------------
    output wire        map_valid,
    input  wire        map_ready,
    output wire [31:0] map_src_x,
    output wire [31:0] map_src_y,
    output wire [15:0] map_dst_x,
    output wire [15:0] map_dst_y,
    output wire [31:0] map_pixel_id,
    output wire        map_last,
    output wire        map_error,

    // Writer response means all required DDR write completions were observed.
    input  wire        writer_rsp_valid,
    output wire        writer_rsp_ready,
    input  wire        writer_rsp_error,

    // ---------------- Job response ----------------
    output wire        rsp_valid,
    input  wire        rsp_ready,
    output wire [31:0] rsp_job_id,
    output wire [31:0] rsp_calib_id,
    output wire [7:0]  rsp_status,
    output wire        busy
);

    localparam [2:0] ST_IDLE       = 3'd0;
    localparam [2:0] ST_CORE_CFG   = 3'd1;
    localparam [2:0] ST_WRITER_CFG = 3'd2;
    localparam [2:0] ST_RUN        = 3'd3;
    localparam [2:0] ST_WAIT_WRITE = 3'd4;
    localparam [2:0] ST_RESPONSE   = 3'd5;

    reg [2:0] state;

    reg [31:0] job_id_reg;
    reg [31:0] calib_id_reg;
    reg [15:0] width_reg;
    reg [15:0] height_reg;
    reg [31:0] map_x_base_reg;
    reg [31:0] map_y_base_reg;
    reg [31:0] map_stride_reg;
    reg [31:0] fx_reg;
    reg [31:0] fy_reg;
    reg [31:0] cx_reg;
    reg [31:0] cy_reg;
    reg [31:0] k1_reg;
    reg [31:0] k2_reg;
    reg [31:0] k3_reg;
    reg [31:0] p1_reg;
    reg [31:0] p2_reg;

    reg [15:0] issue_x;
    reg [15:0] issue_y;
    reg [31:0] issue_pixel_id;
    reg        issue_done;
    reg        math_error_seen;
    reg [7:0]  response_status_reg;

    wire [31:0] minimum_stride;
    wire        command_bad_config;
    wire        command_invalid_camera;
    wire        issue_is_last;
    wire        input_fire;
    wire        output_fire;

    assign minimum_stride       = {14'd0, cmd_width, 2'b00};
    assign command_bad_config   = (cmd_width == 16'd0) ||
                                  (cmd_height == 16'd0) ||
                                  (cmd_map_stride_bytes < minimum_stride);
    assign command_invalid_camera = ~cmd_camera_valid;

    assign cmd_ready = (state == ST_IDLE);
    assign busy      = (state != ST_IDLE);

    assign core_cfg_valid = (state == ST_CORE_CFG);
    assign core_cfg_fx = fx_reg;
    assign core_cfg_fy = fy_reg;
    assign core_cfg_cx = cx_reg;
    assign core_cfg_cy = cy_reg;
    assign core_cfg_k1 = k1_reg;
    assign core_cfg_k2 = k2_reg;
    assign core_cfg_k3 = k3_reg;
    assign core_cfg_p1 = p1_reg;
    assign core_cfg_p2 = p2_reg;

    assign writer_cfg_valid        = (state == ST_WRITER_CFG);
    assign writer_cfg_job_id       = job_id_reg;
    assign writer_cfg_calib_id     = calib_id_reg;
    assign writer_cfg_width        = width_reg;
    assign writer_cfg_height       = height_reg;
    assign writer_cfg_map_x_base   = map_x_base_reg;
    assign writer_cfg_map_y_base   = map_y_base_reg;
    assign writer_cfg_stride_bytes = map_stride_reg;

    assign issue_is_last   = (issue_x == (width_reg - 16'd1)) &&
                             (issue_y == (height_reg - 16'd1));
    assign core_in_valid   = (state == ST_RUN) && !issue_done;
    assign core_in_x       = issue_x;
    assign core_in_y       = issue_y;
    assign core_in_pixel_id = issue_pixel_id;
    assign core_in_last    = issue_is_last;
    assign input_fire      = core_in_valid && core_in_ready;

    assign map_valid       = (state == ST_RUN) && core_out_valid;
    assign core_out_ready  = (state == ST_RUN) && map_ready;
    assign map_src_x       = core_out_src_x;
    assign map_src_y       = core_out_src_y;
    assign map_dst_x       = core_out_dst_x;
    assign map_dst_y       = core_out_dst_y;
    assign map_pixel_id    = core_out_pixel_id;
    assign map_last        = core_out_last;
    assign map_error       = core_out_error;
    assign output_fire     = map_valid && map_ready;

    assign writer_rsp_ready = (state == ST_WAIT_WRITE);

    assign rsp_valid    = (state == ST_RESPONSE);
    assign rsp_job_id   = job_id_reg;
    assign rsp_calib_id = calib_id_reg;
    assign rsp_status   = response_status_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state               <= ST_IDLE;
            job_id_reg          <= 32'd0;
            calib_id_reg        <= 32'd0;
            width_reg           <= 16'd0;
            height_reg          <= 16'd0;
            map_x_base_reg      <= 32'd0;
            map_y_base_reg      <= 32'd0;
            map_stride_reg      <= 32'd0;
            fx_reg              <= `VISION_FP32_ZERO;
            fy_reg              <= `VISION_FP32_ZERO;
            cx_reg              <= `VISION_FP32_ZERO;
            cy_reg              <= `VISION_FP32_ZERO;
            k1_reg              <= `VISION_FP32_ZERO;
            k2_reg              <= `VISION_FP32_ZERO;
            k3_reg              <= `VISION_FP32_ZERO;
            p1_reg              <= `VISION_FP32_ZERO;
            p2_reg              <= `VISION_FP32_ZERO;
            issue_x             <= 16'd0;
            issue_y             <= 16'd0;
            issue_pixel_id      <= 32'd0;
            issue_done          <= 1'b0;
            math_error_seen     <= 1'b0;
            response_status_reg <= `VISION_STATUS_OK;
        end
        else begin
            case (state)
                ST_IDLE: begin
                    if (cmd_valid && cmd_ready) begin
                        job_id_reg     <= cmd_job_id;
                        calib_id_reg   <= cmd_calib_id;
                        width_reg      <= cmd_width;
                        height_reg     <= cmd_height;
                        map_x_base_reg <= cmd_map_x_base;
                        map_y_base_reg <= cmd_map_y_base;
                        map_stride_reg <= cmd_map_stride_bytes;
                        fx_reg         <= cmd_fx;
                        fy_reg         <= cmd_fy;
                        cx_reg         <= cmd_cx;
                        cy_reg         <= cmd_cy;
                        k1_reg         <= cmd_k1;
                        k2_reg         <= cmd_k2;
                        k3_reg         <= cmd_k3;
                        p1_reg         <= cmd_p1;
                        p2_reg         <= cmd_p2;
                        issue_x        <= 16'd0;
                        issue_y        <= 16'd0;
                        issue_pixel_id <= 32'd0;
                        issue_done     <= 1'b0;
                        math_error_seen <= 1'b0;

                        if (command_bad_config) begin
                            response_status_reg <= `VISION_STATUS_BAD_CONFIG;
                            state <= ST_RESPONSE;
                        end
                        else if (command_invalid_camera) begin
                            response_status_reg <= `VISION_STATUS_CALIB_INVALID;
                            state <= ST_RESPONSE;
                        end
                        else begin
                            response_status_reg <= `VISION_STATUS_OK;
                            state <= ST_CORE_CFG;
                        end
                    end
                end

                ST_CORE_CFG: begin
                    if (core_cfg_valid && core_cfg_ready)
                        state <= ST_WRITER_CFG;
                end

                ST_WRITER_CFG: begin
                    if (writer_cfg_valid && writer_cfg_ready)
                        state <= ST_RUN;
                end

                ST_RUN: begin
                    if (input_fire) begin
                        if (issue_is_last) begin
                            issue_done <= 1'b1;
                        end
                        else begin
                            issue_pixel_id <= issue_pixel_id + 32'd1;
                            if (issue_x == (width_reg - 16'd1)) begin
                                issue_x <= 16'd0;
                                issue_y <= issue_y + 16'd1;
                            end
                            else begin
                                issue_x <= issue_x + 16'd1;
                            end
                        end
                    end

                    if (output_fire) begin
                        if (core_out_error)
                            math_error_seen <= 1'b1;
                        if (core_out_last)
                            state <= ST_WAIT_WRITE;
                    end
                end

                ST_WAIT_WRITE: begin
                    if (writer_rsp_valid && writer_rsp_ready) begin
                        if (writer_rsp_error)
                            response_status_reg <= `VISION_STATUS_MEM_ERROR;
                        else if (math_error_seen)
                            response_status_reg <= `VISION_STATUS_CALIB_INVALID;
                        else
                            response_status_reg <= `VISION_STATUS_OK;
                        state <= ST_RESPONSE;
                    end
                end

                ST_RESPONSE: begin
                    if (rsp_valid && rsp_ready)
                        state <= ST_IDLE;
                end

                default: begin
                    response_status_reg <= `VISION_STATUS_BAD_CONFIG;
                    state <= ST_RESPONSE;
                end
            endcase
        end
    end

endmodule
