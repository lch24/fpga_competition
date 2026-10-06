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
module shi_tomasi_ctrl #(parameter USE_CE=0,
    parameter IMG_W   = 1280,
    parameter IMG_H   = 720,
    parameter PIXELS  = IMG_W * IMG_H,
    parameter ADDR_W  = $clog2(PIXELS)
) (
    // Global synchronous stall for variable-latency backing memory.
    input wire ce,

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
    // 响应图读回口（dump：外部在 busy=0/done 后驱动；S_IDLE 期间 store 只读不写无冲突。
    //   registered 读：dump_addr 施加拍 N，dump_data 拍 N+1 有效。
    //   PASS1/PASS2 期间外部必须保持 dump_en=0，本模块仅做组合 mux 不干预状态机）
    input  wire                            dump_en,
    input  wire [ADDR_W-1:0]               dump_addr,
    output wire [31:0]                     dump_data,
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
    // 响应图读回口 mux：dump_en=1 时读口交给外部 dump，否则照旧给 Pass2 mem_addr
    wire [ADDR_W-1:0] store_rd_addr_mux;
    assign store_rd_addr_mux = dump_en ? dump_addr : mem_addr;
    assign dump_data         = store_rd_data;   // registered 读，dump 地址拍 N → 数据拍 N+1
    // Pass1 门控（先在例化前声明，避免隐式 net 与显式声明冲突）
    wire resp_valid_to_store;
    wire store_in_ready;                       // store in_ready（恒 1）

    response_store_max #(.USE_CE(USE_CE),
        .PIXELS (PIXELS),
        .ADDR_W (ADDR_W)
    ) u_store (.ce(ce),
        .clk        (clk),
        .rst_n      (rst_n),
        .in_valid   (resp_valid_to_store),
        .in_ready   (store_in_ready),
        .in_resp    (resp_data),
        .pass1_done (store_pass1_done),
        .rmax       (store_rmax),
        .rd_addr    (store_rd_addr_mux),
        .rd_data    (store_rd_data)
    );

    // window3x3 → nms 连接
    wire win2w_in_valid, win2w_in_ready;
    wire [31:0]     win2w_data;
    wire            wout_valid, wout_ready;
    wire [287:0]    wout_data;
    wire [10:0]     wout_x, wout_y;

    window3x3 #(.USE_CE(USE_CE),
        .CH           (1),
        .DW           (32),
        .IMG_W        (IMG_W),
        .IMG_H        (IMG_H),
        .BORDER_CLAMP (0)
    ) u_window (.ce(ce),
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

    nms_candidates #(.USE_CE(USE_CE),
        .IMG_W (IMG_W),
        .IMG_H (IMG_H)
    ) u_nms (.ce(ce),
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
        if(!USE_CE || ce) begin
        if (!rst_n || (state == S_IDLE && start))
            cand_total <= 16'd0;
        else if (cand_valid && cand_ready)
            cand_total <= cand_total + 16'd1;
    end // synchronous clock enable
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
        end else if(!USE_CE || ce) begin begin
            case (state)
                //-------------------------------------------- IDLE
                S_IDLE: begin
                    if (start) begin
                        state       <= S_PASS1;
                        busy        <= 1'b1;
                        done        <= 1'b0;
                        status      <= 2'b00;
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
    end // synchronous clock enable
    end

endmodule

// Low-memory alternative used by the board. Pass 1 consumes responses only
// for the parent's global maximum/threshold calculation. Once thr_valid arrives,
// reset and restart the SAME gray-to-response pipeline. Pass 2 feeds responses
// directly into the existing line-window and NMS; no frame response RAM exists.
// The parent must keep gray pixels stable across both scans. filter may use the
// gray port only after done, when the replay reader has fully drained.
module shi_tomasi_replay #(parameter USE_CE=0,
    parameter IMG_W=1280, IMG_H=720,
    parameter PIXELS=IMG_W*IMG_H,
    parameter COUNT_W=$clog2(PIXELS+1)
) (
    // Global synchronous stall for variable-latency backing memory.
    input wire ce,

    input wire clk, rst_n, start,
    input wire [31:0] thr,
    input wire thr_valid,
    input wire resp_valid,
    output wire resp_ready,
    input wire [31:0] resp_data,
    output wire first_pass,
    output wire replay_reset,
    output wire replay_start,
    output reg done,
    output wire cand_valid,
    input wire cand_ready,
    output wire [10:0] cand_x, cand_y,
    output reg [15:0] cand_total
);
    localparam IDLE=0, MAXIMUM=1, THRESHOLD=2, RESET_PIPE=3,
               START_PIPE=4, SCAN=5, DRAIN=6;
    reg [2:0] state;
    reg [COUNT_W-1:0] received, emitted;
    wire window_ready, win_valid, win_ready;
    wire [287:0] win_data;
    wire [10:0] win_x, win_y;
    wire local_rst_n = rst_n && !start;
    assign first_pass = state==MAXIMUM;
    assign replay_reset = state==RESET_PIPE;
    assign replay_start = state==START_PIPE;
    assign resp_ready = first_pass ||
        (state==SCAN && received<PIXELS && window_ready);

    window3x3 #(.USE_CE(USE_CE),.CH(1),.DW(32),.IMG_W(IMG_W),.IMG_H(IMG_H),
                .BORDER_CLAMP(0)) u_window (.ce(ce),
        .clk(clk),.rst_n(local_rst_n),
        .in_valid(state==SCAN && received<PIXELS && resp_valid),
        .in_ready(window_ready),.in_data(resp_data),
        .out_valid(win_valid),.out_ready(win_ready),.out_data(win_data),
        .out_x(win_x),.out_y(win_y));
    nms_candidates #(.USE_CE(USE_CE),.IMG_W(IMG_W),.IMG_H(IMG_H)) u_nms (.ce(ce),
        .clk(clk),.rst_n(local_rst_n),
        .win_valid(win_valid),.win_ready(win_ready),.win_data(win_data),
        .cx(win_x),.cy(win_y),.thr(thr),
        .cand_valid(cand_valid),.cand_ready(cand_ready),
        .cand_x(cand_x),.cand_y(cand_y));

    always @(posedge clk) begin
        if(!rst_n) begin
            state<=IDLE; received<=0; emitted<=0; done<=0; cand_total<=0;
        end else if(!USE_CE || ce) begin if(start) begin
            state<=MAXIMUM; received<=0; emitted<=0; done<=0; cand_total<=0;
        end else begin
            if(cand_valid && cand_ready) cand_total<=cand_total+1'b1;
            case(state)
                MAXIMUM: if(resp_valid && resp_ready) begin
                    if(received==PIXELS-1) begin
                        received<=0; state<=THRESHOLD;
                    end else received<=received+1'b1;
                end
                // thr_valid is the parent's multiplier response; thr is latched
                // on this edge and is stable before the second scan starts.
                THRESHOLD: if(thr_valid) state<=RESET_PIPE;
                RESET_PIPE: state<=START_PIPE;
                START_PIPE: state<=SCAN;
                SCAN: begin
                    if(resp_valid && resp_ready) received<=received+1'b1;
                    if(win_valid && win_ready) begin
                        if(emitted==PIXELS-1) state<=DRAIN;
                        else emitted<=emitted+1'b1;
                    end
                end
                // The last window's candidate is already in the NMS FIFO.
                // Wait for actual acceptance, including arbitrarily long stalls.
                DRAIN: if(!cand_valid) begin done<=1; state<=IDLE; end
                default: begin end
            endcase
        end
    end // synchronous clock enable
    end
endmodule
