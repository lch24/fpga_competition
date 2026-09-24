`timescale 1ns / 1ps
//==============================================================================
// shi_tomasi_ctrl.sv — Shi-Tomasi 检测"两遍扫描"控制器与存储调度
//------------------------------------------------------------------------------
// 调度两流程：
//   Pass1：把外部 resp 流转发给 response_store_max（顺序写全图 + rmax），
//          收满 PIXELS 个后读 rmax；rmax≤0 → 无角点，置 status=2'b10 结束。
//   Pass2：顺序读回 RAM，喂 window3x3 → nms_candidates，候选直出 top。
//          全部窗口消费完且候选 FIFO 排空后置 done。
//
// 读口 registered（延迟 1 拍）：mem_addr 施加后一拍出 mem_data。为保证喂给
// window 的每个像素位序正确，首拍先预取地址（P2INIT 一拍），随后每次
// window 接受一个像素则读地址 +1；因 mem_data 与读地址同延迟对齐，故无论
// window 如何背压都不会丢/重像素。
//
// 不做任何 fp32 算术（thr 按输入位直通 nms）。
//==============================================================================
module shi_tomasi_ctrl #(
    parameter IMG_W   = 1280,
    parameter IMG_H   = 720,
    parameter PIXELS  = IMG_W * IMG_H,
    parameter ADDR_W  = $clog2(PIXELS)
) (
    input                                  clk,
    input                                  rst_n,
    input                                  start,      // IDLE 时启动
    output reg                             busy,
    input  [31:0]                          thr,        // 外部算好 rmax*0.08f
    // Pass1 resp 流（握手反向）
    input                                  resp_valid,
    output                                 resp_rdy,
    input  [31:0]                          resp_data,
    input                                  resp_in_ready,   // 上游 me.in_ready
    // Pass2 resp RAM 读口
    output [ADDR_W-1:0]                    mem_addr,
    input  [31:0]                          mem_data,
    // 输出
    output reg                             done,
    output reg [1:0]                       status,     //01=有角点 10=无 11=内部错
    output                                 cand_valid,
    input                                  cand_ready,
    output [10:0]                          cand_x,
    output [10:0]                          cand_y,
    output reg [15:0]                      cand_total
);

    localparam S_IDLE  = 3'd0;
    localparam S_PASS1 = 3'd1;
    localparam S_P2INIT= 3'd2;
    localparam S_P2RUN = 3'd3;
    localparam S_WAIT  = 3'd4;
    reg [2:0] state;

    //--------------------------------------------------------------------
    // 内部子模块例化
    //--------------------------------------------------------------------
    wire store_pass1_done;
    wire [31:0] store_rmax;
    wire [31:0] store_rd_data;
    // Pass1 门控（先在例化前声明，避免隐式 net 与显式声明冲突）
    wire resp_valid_to_store;
    wire store_in_ready;                       // store in_ready（恒 1）

    response_store_max #(
        .PIXELS (PIXELS),
        .ADDR_W (ADDR_W)
    ) u_store (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_valid   (resp_valid_to_store),
        .in_ready   (store_in_ready),
        .in_resp    (resp_data),
        .pass1_done (store_pass1_done),
        .rmax       (store_rmax),
        .rd_addr    (mem_addr),
        .rd_data    (store_rd_data)
    );

    // window3x3 → nms 连接
    wire win2w_in_valid, win2w_in_ready;
    wire [31:0]     win2w_data;
    wire            wout_valid, wout_ready;
    wire [287:0]    wout_data;
    wire [10:0]     wout_x, wout_y;

    window3x3 #(
        .CH           (1),
        .DW           (32),
        .IMG_W        (IMG_W),
        .IMG_H        (IMG_H),
        .BORDER_CLAMP (0)
    ) u_window (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (win2w_in_valid),
        .in_ready (win2w_in_ready),
        .in_data  (win2w_data),
        .out_valid(wout_valid),
        .out_ready(wout_ready),
        .out_data (wout_data),
        .out_x    (wout_x),
        .out_y    (wout_y)
    );

    nms_candidates #(
        .IMG_W (IMG_W),
        .IMG_H (IMG_H)
    ) u_nms (
        .clk        (clk),
        .rst_n      (rst_n),
        .win_valid  (wout_valid),
        .win_ready  (wout_ready),
        .win_data   (wout_data),
        .cx         (wout_x),
        .cy         (wout_y),
        .thr        (thr),
        .cand_valid (cand_valid),
        .cand_ready (cand_ready),
        .cand_x     (cand_x),
        .cand_y     (cand_y)
    );

    //--------------------------------------------------------------------
    // Pass1 → store：握手必须真实反映"上游 me 是否接受"（me.in_ready），
    //   不能拿 store 恒 1 的 in_ready 当 ready 传播——否则 me 未就绪拍被
    //   当作接受，像素静默丢弃，PASS1 计数错位死锁。
    //   resp_rdy = 处于 PASS1 && me.in_ready（PASS2 时不给上游就绪，冻结 me）
    //--------------------------------------------------------------------
    assign resp_valid_to_store = (state == S_PASS1) ? resp_valid : 1'b0;
    assign store_in_ready      = 1'b1;
    assign resp_rdy            = (state == S_PASS1) ? resp_in_ready : 1'b0;

    //--------------------------------------------------------------------
    // Pass2 控制寄存器
    //--------------------------------------------------------------------
    reg [ADDR_W:0] r_addr;       // 下一个要送给 window 的 resp 序号
    reg [ADDR_W:0] wout_n;       // 已从 window 输出接受的窗口数
    reg [15:0]     wait_idle;    // 排空稳定计数

    wire win_pix_fire = win2w_in_valid && win2w_in_ready;   // window 接受一像素
    wire win_out_fire = wout_valid && wout_ready;           // nms 接受一窗口

    assign mem_addr = r_addr + 1'b1;     // 送 resp(k) 时地址 = k+1（registered read 提前1拍）
    assign win2w_in_valid = (state == S_P2RUN);
    assign win2w_data     = store_rd_data;

    // 候选直出（passthrough 握手）：cand_ready 由外部控制（顶层恒 1 即可）
    always @(posedge clk) begin
        if (!rst_n)
            cand_total <= 16'd0;
        else if (cand_valid && cand_ready)
            cand_total <= cand_total + 16'd1;
    end

    //--------------------------------------------------------------------
    // 主状态机
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            busy       <= 1'b0;
            done       <= 1'b0;
            status     <= 2'b00;
            r_addr     <= 0;
            wout_n     <= 0;
            wait_idle  <= 0;
        end else begin
            case (state)
                //-------------------------------------------- IDLE
                S_IDLE: begin
                    if (start) begin
                        state       <= S_PASS1;
                        busy        <= 1'b1;
                        done        <= 1'b0;
                        status      <= 2'b00;
                        cand_total  <= 16'd0;
                        r_addr      <= {(ADDR_W+1){1'b0}};
                        wout_n      <= {(ADDR_W+1){1'b0}};
                        wait_idle   <= 16'd0;
                    end
                end
                //-------------------------------------------- PASS1
                S_PASS1: begin
                    if (store_pass1_done) begin
                        if (store_rmax[31] || (store_rmax == 32'h0)) begin
                            // rmax≤0：无角点
                            status <= 2'b10;
                            done   <= 1'b1;
                            busy   <= 1'b0;
                            state  <= S_IDLE;
                        end else begin
                            // 有角点，进 Pass2；P2INIT 一拍预取首个地址（r_addr=-1 → mem_addr=0）
                            status <= 2'b01;
                            r_addr <= {ADDR_W+1{1'b1}};   // = -1
                            state  <= S_P2INIT;
                        end
                    end
                end
                //-------------------------------------------- P2INIT
                S_P2INIT: begin
                    // registered 读：P2INIT 拍呈现地址0 -> P2RUN 首拍出 resp[0]。
                    // 此处再 +1，使 P2RUN 首拍地址为1，保证首两个 resp 背靠背。
                    r_addr <= r_addr + 1'b1;
                    state  <= S_P2RUN;
                end
                //-------------------------------------------- P2RUN
                S_P2RUN: begin
                    if (win_pix_fire)
                        r_addr <= r_addr + 1'b1;
                    if (win_out_fire) begin
                        if (wout_n >= PIXELS-1)
                            state <= S_WAIT;
                        else
                            wout_n <= wout_n + 1'b1;
                    end
                end
                //-------------------------------------------- WAIT（排空候选）
                S_WAIT: begin
                    if (cand_valid && cand_ready)
                        cand_total <= cand_total + 16'd1;
                    if (!cand_valid) begin
                        if (wait_idle >= 16'd2) begin
                            done   <= 1'b1;
                            busy   <= 1'b0;
                            state  <= S_IDLE;
                        end else
                            wait_idle <= wait_idle + 16'd1;
                    end else
                        wait_idle <= 16'd0;
                end
                default: state <= S_IDLE;   // 内部错误兜底
            endcase
        end
    end

endmodule