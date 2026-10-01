`timescale 1ns/1ps

module camera_calibration_rtl (
    input  wire                clk,
    input  wire                rst_n,

    // 标定图像/角点等输入接口暂略
    // ...

    // ============================================================
    // 标定结果输出
    // param_valid && param_ready 时，九参数被接收
    // ============================================================
    output wire                param_valid,
    input  wire                param_ready,

    // 相机内参：IEEE-754 FP32原始位模式
    output wire        [31:0]  param_fx,
    output wire        [31:0]  param_fy,
    output wire        [31:0]  param_cx,
    output wire        [31:0]  param_cy,

    // 畸变参数：IEEE-754 FP32原始位模式
    output wire        [31:0]  param_k1,
    output wire        [31:0]  param_k2,
    output wire        [31:0]  param_k3,
    output wire        [31:0]  param_p1,
    output wire        [31:0]  param_p2,

    // 标定时使用的图像分辨率
    output wire [15:0]         calib_width,
    output wire [15:0]         calib_height,

    // 本次标定结果编号
    output wire [31:0]         calib_id,

    // 标定状态
    output wire                calib_busy,
    output wire                calib_success,
    output wire                calib_failed,
    output wire [7:0]          calib_error_code,

    // 可选：标定误差，IEEE-754 FP64原始位模式
    output wire [63:0]         calib_rms_error
);

endmodule
