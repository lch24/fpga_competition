`timescale 1ns / 1ps
//==============================================================================
// frame_task_ctrl.v — M8 帧级任务流状态机（一键帧调度）
//------------------------------------------------------------------------------
// 功能：把"一帧完整处理"编排为串行流水：
//   IDLE(process) → GF(gray_fetch：DDR 灰度→片上 gray RAM)
//   → PYR(pyramid_ctrl：片上缩图，cfg_pyr_en=1 才执行)
//   → DET(detect_ctrl：40 点检测 + 响应图帧级导出)
//   → RESP(resp_ddr_writer：响应图写 DDR，cfg_resp_dump_en=1 才执行)
//   → done。
//
// 状态决策（M8 契约）：
//   - gf_status≠01（读错误）→ status=10，直接 done（不进后续阶段）。
//   - det_status 为 01（有角点）或 10（无角点）都视为正常帧完成；
//     仅 11 或非法 status 算错误（status=10）。
//   - det_status=01 且 cfg_resp_dump_en=1 → 进入 RESP 等 w_done；
//     否则（无角点，或 dump 关闭）→ 直接 done。
//
// 握手纪律（M6/M7 教训）：
//   - 所有等待用"前一拍电平值判 0→1 沿"（done_d 无条件每拍跟踪，
//     子模块 start 时 done 先清零，保证每帧有新鲜沿）。
//   - start 脉冲 1 拍；busy 全程电平保持；done 电平保持（下次 process 清零）。
//   - 响应导出时序要点：detect_ctrl 的 ST_DUMP 需要 resp_dump_ready 才推进
//     （resp_dump_done → det done），而 resp_ddr_writer 必须先 start 才拉
//     in_ready —— 因此 cfg_resp_dump_en=1 时 w_start 在进入 DET 阶段与
//     det_start 同拍发出（与 M7 tb_resp_ddr 同步启动手法一致），RESP 阶段
//     只负责等 w_done 沿并检查 w_status。
// 注：SV 关键字 process 在 ModelSim 10.6e -sv 模式下被保留，帧命令端口
//   命名为 process_frame（= 契约 process，语义一致）。
//==============================================================================
module frame_task_ctrl #(
    parameter ADDR_W      = 32,
    parameter LEN_W       = 32,
    parameter GRAY_ADDR_W = 26          // cfg_ram_base 位宽（gray RAM 地址）
) (
    input  wire                    clk,
    input  wire                    rst_n,
    // ---- 帧命令 ----
    input  wire                    process_frame,  // busy=0 时单拍，锁存 cfg 并启动一帧
    output reg                     busy,
    output reg                     done,           // 电平保持（下次 process 清零）
    output reg  [1:0]              status,         // 01=成功 10=子模块错误
    // ---- 帧级配置（process 时锁存）----
    input  wire [ADDR_W-1:0]       cfg_gray_base,   // DDR 灰度区首字节地址
    input  wire [ADDR_W-1:0]       cfg_gray_stride, // DDR 行跨度（字节）
    input  wire [15:0]             cfg_gray_w,      // 每行像素/字节
    input  wire [15:0]             cfg_gray_h,      // 行数
    input  wire [GRAY_ADDR_W-1:0]  cfg_ram_base,    // 片上 gray RAM 基址
    input  wire [ADDR_W-1:0]       cfg_resp_base,   // 响应图写 DDR 首字节地址
    input  wire                    cfg_resp_dump_en,// 帧级响应图导出使能
    input  wire                    cfg_pyr_en,      // 金字塔缩图使能
    // ---- 子模块接口 ----
    output reg                     gf_start,
    input  wire                    gf_busy,
    input  wire                    gf_done,
    input  wire [1:0]              gf_status,
    output reg                     pyr_start,
    input  wire                    pyr_busy,
    input  wire                    pyr_done,
    output reg                     det_start,
    input  wire                    det_busy,
    input  wire                    det_done,
    input  wire [1:0]              det_status,
    output reg                     w_start,
    input  wire                    w_busy,
    input  wire                    w_done,
    input  wire [1:0]              w_status
);

    //--------------------------------------------------------------------
    // 状态
    //--------------------------------------------------------------------
    localparam [2:0] S_IDLE = 3'd0;
    localparam [2:0] S_GF   = 3'd1;
    localparam [2:0] S_PYR  = 3'd2;
    localparam [2:0] S_DET  = 3'd3;
    localparam [2:0] S_RESP = 3'd4;
    reg [2:0] st;

    //--------------------------------------------------------------------
    // 配置锁存（process 时采样；cfg_resp_base 保留以维持契约语义，
    //   实际写 DDR 基址由 top 直连 cfg 总线给 resp_ddr_writer）
    //--------------------------------------------------------------------
    reg [ADDR_W-1:0]      cfg_gray_base_r, cfg_gray_stride_r, cfg_resp_base_r;
    reg [15:0]            cfg_gray_w_r, cfg_gray_h_r;
    reg [GRAY_ADDR_W-1:0] cfg_ram_base_r;
    reg                   cfg_resp_dump_en_r, cfg_pyr_en_r;

    //--------------------------------------------------------------------
    // 完成沿检测（无条件每拍跟踪：子模块 start 时 done 清零 → 新鲜 0→1 沿）
    //--------------------------------------------------------------------
    reg  gf_done_d, pyr_done_d, det_done_d, w_done_d;
    wire gf_done_rise  = gf_done  && !gf_done_d;
    wire pyr_done_rise = pyr_done && !pyr_done_d;
    wire det_done_rise = det_done && !det_done_d;
    wire w_done_rise   = w_done   && !w_done_d;

    //--------------------------------------------------------------------
    // 主状态机
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            st                  <= S_IDLE;
            busy                <= 1'b0;
            done                <= 1'b0;
            status              <= 2'b00;
            cfg_gray_base_r     <= {ADDR_W{1'b0}};
            cfg_gray_stride_r   <= {ADDR_W{1'b0}};
            cfg_resp_base_r     <= {ADDR_W{1'b0}};
            cfg_gray_w_r        <= 16'd0;
            cfg_gray_h_r        <= 16'd0;
            cfg_ram_base_r      <= {GRAY_ADDR_W{1'b0}};
            cfg_resp_dump_en_r  <= 1'b0;
            cfg_pyr_en_r        <= 1'b0;
            gf_done_d           <= 1'b0;
            pyr_done_d          <= 1'b0;
            det_done_d          <= 1'b0;
            w_done_d            <= 1'b0;
            gf_start            <= 1'b0;
            pyr_start           <= 1'b0;
            det_start           <= 1'b0;
            w_start             <= 1'b0;
        end else begin
            // 沿跟踪（无条件，防漏沿）
            gf_done_d  <= gf_done;
            pyr_done_d <= pyr_done;
            det_done_d <= det_done;
            w_done_d   <= w_done;
            // start 脉冲默认清零（各状态按需拉高 1 拍）
            gf_start  <= 1'b0;
            pyr_start <= 1'b0;
            det_start <= 1'b0;
            w_start   <= 1'b0;

            case (st)
                //-------------------------------------------- IDLE
                S_IDLE: begin
                    if (process_frame && !busy) begin
                        busy                <= 1'b1;
                        done                <= 1'b0;
                        status              <= 2'b00;
                        cfg_gray_base_r     <= cfg_gray_base;
                        cfg_gray_stride_r   <= cfg_gray_stride;
                        cfg_resp_base_r     <= cfg_resp_base;
                        cfg_gray_w_r        <= cfg_gray_w;
                        cfg_gray_h_r        <= cfg_gray_h;
                        cfg_ram_base_r      <= cfg_ram_base;
                        cfg_resp_dump_en_r  <= cfg_resp_dump_en;
                        cfg_pyr_en_r        <= cfg_pyr_en;
                        gf_start            <= 1'b1;      // 启动 gray_fetch
                        st                  <= S_GF;
                    end
                end
                //-------------------------------------------- GF（等 gray_fetch done 沿）
                S_GF: begin
                    if (gf_done_rise) begin
                        if (gf_status != 2'b01) begin
                            // 读错误：status=10 直接结束
                            status <= 2'b10;
                            done   <= 1'b1;
                            busy   <= 1'b0;
                            st     <= S_IDLE;
                        end else if (cfg_pyr_en_r) begin
                            pyr_start <= 1'b1;
                            st        <= S_PYR;
                        end else begin
                            det_start <= 1'b1;
                            if (cfg_resp_dump_en_r) w_start <= 1'b1;  // 同步启动 writer（见头注释）
                            st        <= S_DET;
                        end
                    end
                end
                //-------------------------------------------- PYR（等 pyramid done 沿）
                S_PYR: begin
                    if (pyr_done_rise) begin
                        det_start <= 1'b1;
                        if (cfg_resp_dump_en_r) w_start <= 1'b1;
                        st        <= S_DET;
                    end
                end
                //-------------------------------------------- DET（等 detect done 沿）
                S_DET: begin
                    if (det_done_rise) begin
                        if (det_status == 2'b11) begin
                            // 检测内部错误：status=10 直接结束
                            status <= 2'b10;
                            done   <= 1'b1;
                            busy   <= 1'b0;
                            st     <= S_IDLE;
                        end else if (cfg_resp_dump_en_r && (det_status == 2'b01)) begin
                            // 有角点且导出使能：进 RESP 等写 DDR 完成
                            st <= S_RESP;
                        end else begin
                            // det_status=01/10 均视为正常帧完成（无角点不导出）
                            status <= 2'b01;
                            done   <= 1'b1;
                            busy   <= 1'b0;
                            st     <= S_IDLE;
                        end
                    end
                end
                //-------------------------------------------- RESP（等 resp writer done 沿）
                S_RESP: begin
                    if (w_done_rise) begin
                        status <= (w_status != 2'b01) ? 2'b10 : 2'b01;
                        done   <= 1'b1;
                        busy   <= 1'b0;
                        st     <= S_IDLE;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
