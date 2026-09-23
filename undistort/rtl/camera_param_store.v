`timescale 1ns/1ps

// Receives one complete calibration result and switches it into the active
// parameter bank only while the remap generator is idle.
//
// Interface assumptions:
//   * All signals are synchronous to clk. A CDC bridge is required otherwise.
//   * fx/fy/cx/cy use signed Q16.16.
//   * k1/k2/p1/p2/k3 use signed Q4.28.
//   * RMS error uses unsigned Q16.16 pixels.
//   * The producer must keep the entire s_* bundle stable while
//     s_param_valid is high and s_param_ready is low.
module camera_param_store (
    input  wire                clk,
    input  wire                rst_n,

    input  wire                s_param_valid,
    output wire                s_param_ready,

    input  wire signed [31:0]  s_fx,
    input  wire signed [31:0]  s_fy,
    input  wire signed [31:0]  s_cx,
    input  wire signed [31:0]  s_cy,
    input  wire signed [31:0]  s_k1,
    input  wire signed [31:0]  s_k2,
    input  wire signed [31:0]  s_p1,
    input  wire signed [31:0]  s_p2,
    input  wire signed [31:0]  s_k3,
    input  wire        [15:0]  s_calib_width,
    input  wire        [15:0]  s_calib_height,
    input  wire        [15:0]  s_calib_id,
    input  wire        [31:0]  s_rms_error,

    // A new parameter set may be received while the remap generator is busy,
    // but it is not made active until map_busy becomes low.
    input  wire                map_busy,

    output reg                 active_valid,
    output reg                 active_update,
    output reg  signed [31:0]  active_fx,
    output reg  signed [31:0]  active_fy,
    output reg  signed [31:0]  active_cx,
    output reg  signed [31:0]  active_cy,
    output reg  signed [31:0]  active_k1,
    output reg  signed [31:0]  active_k2,
    output reg  signed [31:0]  active_p1,
    output reg  signed [31:0]  active_p2,
    output reg  signed [31:0]  active_k3,
    output reg          [15:0] active_calib_width,
    output reg          [15:0] active_calib_height,
    output reg          [15:0] active_calib_id,
    output reg          [31:0] active_rms_error,

    // Debug/status signal: one complete result is waiting to become active.
    output wire                shadow_pending
);

    reg                        shadow_valid;
    reg signed [31:0]          shadow_fx;
    reg signed [31:0]          shadow_fy;
    reg signed [31:0]          shadow_cx;
    reg signed [31:0]          shadow_cy;
    reg signed [31:0]          shadow_k1;
    reg signed [31:0]          shadow_k2;
    reg signed [31:0]          shadow_p1;
    reg signed [31:0]          shadow_p2;
    reg signed [31:0]          shadow_k3;
    reg        [15:0]          shadow_calib_width;
    reg        [15:0]          shadow_calib_height;
    reg        [15:0]          shadow_calib_id;
    reg        [31:0]          shadow_rms_error;

    assign s_param_ready = ~shadow_valid;
    assign shadow_pending = shadow_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shadow_valid        <= 1'b0;
            active_valid        <= 1'b0;
            active_update       <= 1'b0;

            shadow_fx           <= 32'sd0;
            shadow_fy           <= 32'sd0;
            shadow_cx           <= 32'sd0;
            shadow_cy           <= 32'sd0;
            shadow_k1           <= 32'sd0;
            shadow_k2           <= 32'sd0;
            shadow_p1           <= 32'sd0;
            shadow_p2           <= 32'sd0;
            shadow_k3           <= 32'sd0;
            shadow_calib_width  <= 16'd0;
            shadow_calib_height <= 16'd0;
            shadow_calib_id     <= 16'd0;
            shadow_rms_error    <= 32'd0;

            active_fx           <= 32'sd0;
            active_fy           <= 32'sd0;
            active_cx           <= 32'sd0;
            active_cy           <= 32'sd0;
            active_k1           <= 32'sd0;
            active_k2           <= 32'sd0;
            active_p1           <= 32'sd0;
            active_p2           <= 32'sd0;
            active_k3           <= 32'sd0;
            active_calib_width  <= 16'd0;
            active_calib_height <= 16'd0;
            active_calib_id     <= 16'd0;
            active_rms_error    <= 32'd0;
        end
        else begin
            active_update <= 1'b0;

            // Capture the whole result atomically.
            if (s_param_valid && s_param_ready) begin
                shadow_fx           <= s_fx;
                shadow_fy           <= s_fy;
                shadow_cx           <= s_cx;
                shadow_cy           <= s_cy;
                shadow_k1           <= s_k1;
                shadow_k2           <= s_k2;
                shadow_p1           <= s_p1;
                shadow_p2           <= s_p2;
                shadow_k3           <= s_k3;
                shadow_calib_width  <= s_calib_width;
                shadow_calib_height <= s_calib_height;
                shadow_calib_id     <= s_calib_id;
                shadow_rms_error    <= s_rms_error;
                shadow_valid        <= 1'b1;
            end

            // Commit only between mapping jobs. All active registers switch on
            // the same edge, so the mapper never observes a mixed parameter set.
            if (shadow_valid && !map_busy) begin
                active_fx           <= shadow_fx;
                active_fy           <= shadow_fy;
                active_cx           <= shadow_cx;
                active_cy           <= shadow_cy;
                active_k1           <= shadow_k1;
                active_k2           <= shadow_k2;
                active_p1           <= shadow_p1;
                active_p2           <= shadow_p2;
                active_k3           <= shadow_k3;
                active_calib_width  <= shadow_calib_width;
                active_calib_height <= shadow_calib_height;
                active_calib_id     <= shadow_calib_id;
                active_rms_error    <= shadow_rms_error;
                active_valid        <= 1'b1;
                active_update       <= 1'b1;
                shadow_valid        <= 1'b0;
            end
        end
    end

endmodule
