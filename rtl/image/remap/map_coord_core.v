`timescale 1ns/1ps
`include "vision_defs.vh"

// Bring-up implementation of the Brown reverse-map coordinate calculation.
//
// Numeric contract:
//   - Camera parameters and outputs are IEEE 754 FP32 bit patterns.
//   - dst_x/dst_y are unsigned integer pixel coordinates.
//   - One abstract FP32 service operation may be outstanding at a time.
//   - The operation order is explicit and must be mirrored by the numeric model.
//
// Throughput note:
//   This version intentionally serializes 38 FP operations per pixel. It is a
//   correctness-first implementation and will not meet real-time 720p/1080p.
//   After bit-accurate validation, independent operations can be pipelined or
//   replicated without changing this module's external stream contract.
module map_coord_core (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        cfg_valid,
    output wire        cfg_ready,
    input  wire [31:0] cfg_fx,
    input  wire [31:0] cfg_fy,
    input  wire [31:0] cfg_cx,
    input  wire [31:0] cfg_cy,
    input  wire [31:0] cfg_k1,
    input  wire [31:0] cfg_k2,
    input  wire [31:0] cfg_k3,
    input  wire [31:0] cfg_p1,
    input  wire [31:0] cfg_p2,

    input  wire        in_valid,
    output wire        in_ready,
    input  wire [15:0] in_dst_x,
    input  wire [15:0] in_dst_y,
    input  wire [31:0] in_pixel_id,
    input  wire        in_last,

    output wire        out_valid,
    input  wire        out_ready,
    output wire [31:0] out_src_x,
    output wire [31:0] out_src_y,
    output wire [15:0] out_dst_x,
    output wire [15:0] out_dst_y,
    output wire [31:0] out_pixel_id,
    output wire        out_last,
    output wire        out_error,

    // Abstract, in-order FP32 operation service.
    output wire        fp_req_valid,
    input  wire        fp_req_ready,
    output wire [2:0]  fp_req_op,
    output wire [31:0] fp_req_a,
    output wire [31:0] fp_req_b,
    input  wire        fp_rsp_valid,
    output wire        fp_rsp_ready,
    input  wire [31:0] fp_rsp_result,
    input  wire        fp_rsp_error
);

    localparam [5:0] S_IDLE        = 6'd0;
    localparam [5:0] S_CVT_X       = 6'd1;
    localparam [5:0] S_CVT_Y       = 6'd2;
    localparam [5:0] S_DX          = 6'd3;
    localparam [5:0] S_NX          = 6'd4;
    localparam [5:0] S_DY          = 6'd5;
    localparam [5:0] S_NY          = 6'd6;
    localparam [5:0] S_X2          = 6'd7;
    localparam [5:0] S_Y2          = 6'd8;
    localparam [5:0] S_R2          = 6'd9;
    localparam [5:0] S_R4          = 6'd10;
    localparam [5:0] S_R6          = 6'd11;
    localparam [5:0] S_K1R2        = 6'd12;
    localparam [5:0] S_K2R4        = 6'd13;
    localparam [5:0] S_K3R6        = 6'd14;
    localparam [5:0] S_RADIAL_A    = 6'd15;
    localparam [5:0] S_RADIAL_B    = 6'd16;
    localparam [5:0] S_RADIAL      = 6'd17;
    localparam [5:0] S_X_RADIAL    = 6'd18;
    localparam [5:0] S_Y_RADIAL    = 6'd19;
    localparam [5:0] S_XY          = 6'd20;
    localparam [5:0] S_P1XY        = 6'd21;
    localparam [5:0] S_TX1         = 6'd22;
    localparam [5:0] S_TWO_X2      = 6'd23;
    localparam [5:0] S_R2_2X2      = 6'd24;
    localparam [5:0] S_TX2         = 6'd25;
    localparam [5:0] S_XD_A        = 6'd26;
    localparam [5:0] S_XD          = 6'd27;
    localparam [5:0] S_TWO_Y2      = 6'd28;
    localparam [5:0] S_R2_2Y2      = 6'd29;
    localparam [5:0] S_TY1         = 6'd30;
    localparam [5:0] S_P2XY        = 6'd31;
    localparam [5:0] S_TY2         = 6'd32;
    localparam [5:0] S_YD_A        = 6'd33;
    localparam [5:0] S_YD          = 6'd34;
    localparam [5:0] S_SX_MUL      = 6'd35;
    localparam [5:0] S_SX          = 6'd36;
    localparam [5:0] S_SY_MUL      = 6'd37;
    localparam [5:0] S_SY          = 6'd38;

    reg [5:0] state;
    reg       cfg_loaded;
    reg       op_inflight;
    reg       out_valid_reg;
    reg       error_seen;

    reg [31:0] fx, fy, cx, cy, k1, k2, k3, p1, p2;
    reg [15:0] dst_x, dst_y;
    reg [31:0] pixel_id;
    reg        pixel_last;

    reg [31:0] x_fp, y_fp, dx, dy, nx, ny;
    reg [31:0] x2, y2, xy, r2, r4, r6;
    reg [31:0] k1r2, k2r4, k3r6;
    reg [31:0] radial_a, radial_b, radial;
    reg [31:0] x_radial, y_radial;
    reg [31:0] p1xy, tx1, two_x2, r2_2x2, tx2, xd_a, xd;
    reg [31:0] two_y2, r2_2y2, ty1, p2xy, ty2, yd_a, yd;
    reg [31:0] sx_mul, sy_mul;
    reg [31:0] src_x_reg, src_y_reg;
    reg        out_error_reg;

    reg [2:0]  req_op_comb;
    reg [31:0] req_a_comb;
    reg [31:0] req_b_comb;

    wire cfg_fire = cfg_valid && cfg_ready;
    wire in_fire  = in_valid && in_ready;
    wire req_fire = fp_req_valid && fp_req_ready;
    wire rsp_fire = fp_rsp_valid && fp_rsp_ready;

    assign cfg_ready = (state == S_IDLE) && !out_valid_reg;
    // Configuration wins if cfg_valid and in_valid arrive together.
    assign in_ready  = (state == S_IDLE) && cfg_loaded && !out_valid_reg && !cfg_valid;

    assign out_valid    = out_valid_reg;
    assign out_src_x    = src_x_reg;
    assign out_src_y    = src_y_reg;
    assign out_dst_x    = dst_x;
    assign out_dst_y    = dst_y;
    assign out_pixel_id = pixel_id;
    assign out_last     = pixel_last;
    assign out_error    = out_error_reg;

    assign fp_req_valid = (state != S_IDLE) && !op_inflight;
    assign fp_req_op    = req_op_comb;
    assign fp_req_a     = req_a_comb;
    assign fp_req_b     = req_b_comb;
    assign fp_rsp_ready = op_inflight;

    always @* begin
        req_op_comb = `VISION_FP_OP_ADD;
        req_a_comb  = `VISION_FP32_ZERO;
        req_b_comb  = `VISION_FP32_ZERO;
        case (state)
            S_CVT_X:    begin req_op_comb = `VISION_FP_OP_U32_TO_FP32; req_a_comb = {16'd0, dst_x}; end
            S_CVT_Y:    begin req_op_comb = `VISION_FP_OP_U32_TO_FP32; req_a_comb = {16'd0, dst_y}; end
            S_DX:       begin req_op_comb = `VISION_FP_OP_SUB; req_a_comb = x_fp; req_b_comb = cx; end
            S_NX:       begin req_op_comb = `VISION_FP_OP_DIV; req_a_comb = dx; req_b_comb = fx; end
            S_DY:       begin req_op_comb = `VISION_FP_OP_SUB; req_a_comb = y_fp; req_b_comb = cy; end
            S_NY:       begin req_op_comb = `VISION_FP_OP_DIV; req_a_comb = dy; req_b_comb = fy; end
            S_X2:       begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = nx; req_b_comb = nx; end
            S_Y2:       begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = ny; req_b_comb = ny; end
            S_R2:       begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = x2; req_b_comb = y2; end
            S_R4:       begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = r2; req_b_comb = r2; end
            S_R6:       begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = r4; req_b_comb = r2; end
            S_K1R2:     begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = k1; req_b_comb = r2; end
            S_K2R4:     begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = k2; req_b_comb = r4; end
            S_K3R6:     begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = k3; req_b_comb = r6; end
            S_RADIAL_A: begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = `VISION_FP32_ONE; req_b_comb = k1r2; end
            S_RADIAL_B: begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = radial_a; req_b_comb = k2r4; end
            S_RADIAL:   begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = radial_b; req_b_comb = k3r6; end
            S_X_RADIAL: begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = nx; req_b_comb = radial; end
            S_Y_RADIAL: begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = ny; req_b_comb = radial; end
            S_XY:       begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = nx; req_b_comb = ny; end
            S_P1XY:     begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = p1; req_b_comb = xy; end
            S_TX1:      begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = p1xy; req_b_comb = p1xy; end
            S_TWO_X2:   begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = x2; req_b_comb = x2; end
            S_R2_2X2:   begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = r2; req_b_comb = two_x2; end
            S_TX2:      begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = p2; req_b_comb = r2_2x2; end
            S_XD_A:     begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = x_radial; req_b_comb = tx1; end
            S_XD:       begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = xd_a; req_b_comb = tx2; end
            S_TWO_Y2:   begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = y2; req_b_comb = y2; end
            S_R2_2Y2:   begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = r2; req_b_comb = two_y2; end
            S_TY1:      begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = p1; req_b_comb = r2_2y2; end
            S_P2XY:     begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = p2; req_b_comb = xy; end
            S_TY2:      begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = p2xy; req_b_comb = p2xy; end
            S_YD_A:     begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = y_radial; req_b_comb = ty1; end
            S_YD:       begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = yd_a; req_b_comb = ty2; end
            S_SX_MUL:   begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = fx; req_b_comb = xd; end
            S_SX:       begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = sx_mul; req_b_comb = cx; end
            S_SY_MUL:   begin req_op_comb = `VISION_FP_OP_MUL; req_a_comb = fy; req_b_comb = yd; end
            S_SY:       begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = sy_mul; req_b_comb = cy; end
            default:    begin req_op_comb = `VISION_FP_OP_ADD; req_a_comb = `VISION_FP32_ZERO; req_b_comb = `VISION_FP32_ZERO; end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            cfg_loaded    <= 1'b0;
            op_inflight   <= 1'b0;
            out_valid_reg <= 1'b0;
            error_seen    <= 1'b0;
            out_error_reg <= 1'b0;
            fx <= 32'd0; fy <= 32'd0; cx <= 32'd0; cy <= 32'd0;
            k1 <= 32'd0; k2 <= 32'd0; k3 <= 32'd0; p1 <= 32'd0; p2 <= 32'd0;
            dst_x <= 16'd0; dst_y <= 16'd0; pixel_id <= 32'd0; pixel_last <= 1'b0;
            x_fp <= 32'd0; y_fp <= 32'd0; dx <= 32'd0; dy <= 32'd0; nx <= 32'd0; ny <= 32'd0;
            x2 <= 32'd0; y2 <= 32'd0; xy <= 32'd0; r2 <= 32'd0; r4 <= 32'd0; r6 <= 32'd0;
            k1r2 <= 32'd0; k2r4 <= 32'd0; k3r6 <= 32'd0;
            radial_a <= 32'd0; radial_b <= 32'd0; radial <= 32'd0;
            x_radial <= 32'd0; y_radial <= 32'd0;
            p1xy <= 32'd0; tx1 <= 32'd0; two_x2 <= 32'd0; r2_2x2 <= 32'd0;
            tx2 <= 32'd0; xd_a <= 32'd0; xd <= 32'd0;
            two_y2 <= 32'd0; r2_2y2 <= 32'd0; ty1 <= 32'd0; p2xy <= 32'd0;
            ty2 <= 32'd0; yd_a <= 32'd0; yd <= 32'd0;
            sx_mul <= 32'd0; sy_mul <= 32'd0; src_x_reg <= 32'd0; src_y_reg <= 32'd0;
        end else begin
            if (cfg_fire) begin
                fx <= cfg_fx; fy <= cfg_fy; cx <= cfg_cx; cy <= cfg_cy;
                k1 <= cfg_k1; k2 <= cfg_k2; k3 <= cfg_k3; p1 <= cfg_p1; p2 <= cfg_p2;
                cfg_loaded <= 1'b1;
            end

            if (out_valid_reg && out_ready)
                out_valid_reg <= 1'b0;

            if (in_fire) begin
                dst_x       <= in_dst_x;
                dst_y       <= in_dst_y;
                pixel_id    <= in_pixel_id;
                pixel_last  <= in_last;
                error_seen  <= 1'b0;
                out_error_reg <= 1'b0;
                state       <= S_CVT_X;
            end

            if (req_fire)
                op_inflight <= 1'b1;

            if (rsp_fire) begin
                op_inflight <= 1'b0;
                if (fp_rsp_error)
                    error_seen <= 1'b1;
                case (state)
                    S_CVT_X:    begin x_fp      <= fp_rsp_result; state <= S_CVT_Y; end
                    S_CVT_Y:    begin y_fp      <= fp_rsp_result; state <= S_DX; end
                    S_DX:       begin dx        <= fp_rsp_result; state <= S_NX; end
                    S_NX:       begin nx        <= fp_rsp_result; state <= S_DY; end
                    S_DY:       begin dy        <= fp_rsp_result; state <= S_NY; end
                    S_NY:       begin ny        <= fp_rsp_result; state <= S_X2; end
                    S_X2:       begin x2        <= fp_rsp_result; state <= S_Y2; end
                    S_Y2:       begin y2        <= fp_rsp_result; state <= S_R2; end
                    S_R2:       begin r2        <= fp_rsp_result; state <= S_R4; end
                    S_R4:       begin r4        <= fp_rsp_result; state <= S_R6; end
                    S_R6:       begin r6        <= fp_rsp_result; state <= S_K1R2; end
                    S_K1R2:     begin k1r2      <= fp_rsp_result; state <= S_K2R4; end
                    S_K2R4:     begin k2r4      <= fp_rsp_result; state <= S_K3R6; end
                    S_K3R6:     begin k3r6      <= fp_rsp_result; state <= S_RADIAL_A; end
                    S_RADIAL_A: begin radial_a  <= fp_rsp_result; state <= S_RADIAL_B; end
                    S_RADIAL_B: begin radial_b  <= fp_rsp_result; state <= S_RADIAL; end
                    S_RADIAL:   begin radial    <= fp_rsp_result; state <= S_X_RADIAL; end
                    S_X_RADIAL: begin x_radial  <= fp_rsp_result; state <= S_Y_RADIAL; end
                    S_Y_RADIAL: begin y_radial  <= fp_rsp_result; state <= S_XY; end
                    S_XY:       begin xy        <= fp_rsp_result; state <= S_P1XY; end
                    S_P1XY:     begin p1xy      <= fp_rsp_result; state <= S_TX1; end
                    S_TX1:      begin tx1       <= fp_rsp_result; state <= S_TWO_X2; end
                    S_TWO_X2:   begin two_x2    <= fp_rsp_result; state <= S_R2_2X2; end
                    S_R2_2X2:   begin r2_2x2    <= fp_rsp_result; state <= S_TX2; end
                    S_TX2:      begin tx2       <= fp_rsp_result; state <= S_XD_A; end
                    S_XD_A:     begin xd_a      <= fp_rsp_result; state <= S_XD; end
                    S_XD:       begin xd        <= fp_rsp_result; state <= S_TWO_Y2; end
                    S_TWO_Y2:   begin two_y2    <= fp_rsp_result; state <= S_R2_2Y2; end
                    S_R2_2Y2:   begin r2_2y2    <= fp_rsp_result; state <= S_TY1; end
                    S_TY1:      begin ty1       <= fp_rsp_result; state <= S_P2XY; end
                    S_P2XY:     begin p2xy      <= fp_rsp_result; state <= S_TY2; end
                    S_TY2:      begin ty2       <= fp_rsp_result; state <= S_YD_A; end
                    S_YD_A:     begin yd_a      <= fp_rsp_result; state <= S_YD; end
                    S_YD:       begin yd        <= fp_rsp_result; state <= S_SX_MUL; end
                    S_SX_MUL:   begin sx_mul    <= fp_rsp_result; state <= S_SX; end
                    S_SX:       begin src_x_reg <= fp_rsp_result; state <= S_SY_MUL; end
                    S_SY_MUL:   begin sy_mul    <= fp_rsp_result; state <= S_SY; end
                    S_SY: begin
                        src_y_reg    <= fp_rsp_result;
                        out_error_reg <= error_seen | fp_rsp_error;
                        out_valid_reg <= 1'b1;
                        state         <= S_IDLE;
                    end
                    default: begin
                        out_error_reg <= 1'b1;
                        out_valid_reg <= 1'b1;
                        state         <= S_IDLE;
                    end
                endcase
            end
        end
    end

endmodule
