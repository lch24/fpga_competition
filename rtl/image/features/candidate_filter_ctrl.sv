`timescale 1ns / 1ps
// Candidate pipeline: capture -> merge(5) -> subpixel(7) -> merge(3)
// -> fixed integer ring(6) AND (ring(4) OR ring(8)). Merge and subpixel
// time-share the candidate store; the ring reads one coordinate at a time.
// No nearest-neighbor radius search or per-candidate radius RAM.
module candidate_filter_ctrl #(parameter FIXED_BILINEAR=0, parameter FIXED_ACCUM=0, parameter USE_CE=0, parameter DDR_CANDIDATES=0, parameter VARIABLE_SCALE=0, parameter SHARED_SUBPIXEL=0,
    parameter IMG_W       = 1280,
    parameter IMG_H       = 720,
    parameter GRAY_ADDR_W = 20,          // ≥ $clog2(W*H)
    parameter N_ADDR_W    = 14,          // 候选点深度 2**14 = 16384
    parameter MIN_CAND    = 40,
    parameter MAX_CAND    = 12000,
    parameter ROM_FILE    = "data/rom/ring_cos_sin.mem",
    parameter SUB_ROM_FILE = "data/rom/gaussian_weights.mem"
) (
    // Global synchronous stall for variable-latency backing memory.
    output wire sp_start,
    input wire sp_busy,
    input wire sp_done,
    output wire [16-1:0] sp_n_in,
    output wire [8-1:0] sp_half_win,
    input wire sp_pt_rd_en,
    input wire [N_ADDR_W-1:0] sp_pt_rd_addr,
    output wire [32-1:0] sp_pt_rd_x,
    output wire [32-1:0] sp_pt_rd_y,
    input wire sp_gray_rd_en,
    input wire [GRAY_ADDR_W-1:0] sp_gray_rd_addr,
    input wire sp_out_valid,
    output wire sp_out_ready,
    input wire [32-1:0] sp_out_x,
    input wire [32-1:0] sp_out_y,
    input wire ce,
    output wire scratch_rd_en,scratch_wr_en,
    output wire [14:0] scratch_rd_addr,scratch_wr_addr,
    output wire [63:0] scratch_wr_data,
    input wire [63:0] scratch_rd_data,

    input wire [2:0] cfg_scale,

    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,          // IDLE 时启动
    output reg                     busy,
    output reg                     done,
    output reg  [1:0]              status,         //01=完成 10=候选不足 11=候选超限
    // NMS 候选流（整数坐标，来自 shi_tomasi_ctrl）
    input  wire                    cand_valid,
    output reg                     cand_ready,
    input  wire [10:0]             cand_x,
    input  wire [10:0]             cand_y,
    input  wire                    cand_done,      // 候选流结束（shi_tomasi_ctrl.done）
    // 灰度随机读口（外部存储，1 拍延迟；透传 ring_check）
    output wire                    gray_rd_en,
    output wire [GRAY_ADDR_W-1:0]  gray_rd_addr,
    input  wire [7:0]              gray_rd_data,
    // inner 输出（fp32 坐标流）
    output reg                     inner_valid,
    input  wire                    inner_ready,
    output reg  [31:0]             inner_x,
    output reg  [31:0]             inner_y,
    output reg  [15:0]             inner_total,
    // ring 探针（对拍：每点 d0 中间量 + 综合 pass，无背压）
    output reg                     probe_valid,
    output reg  [31:0]             probe_hi,
    output reg  [31:0]             probe_lo,
    output reg  [31:0]             probe_thr,
    output reg  [31:0]             probe_ntrans,
    output reg  [31:0]             probe_opp_err,
    output reg                     probe_sector_ok,
    output reg                     probe_pass
);

    localparam C_R5    = 32'h40a00000;   // 5.0f（merge5）
    localparam C_R3    = 32'h40400000;   // 3.0f（merge3）

    //--------------------------------------------------------------------
    // 主阶段状态
    //--------------------------------------------------------------------
    localparam S_IDLE   = 4'd0;
    localparam S_CAP    = 4'd1;   // 收 NMS 候选写 store A
    localparam S_MERGE5 = 4'd2;   // merge 读 A 写 B（phase=1）
    localparam S_SUBPX  = 4'd3;   // subpixel：读 B 写 A（phase=0），half_win=7
    localparam S_MERGE3 = 4'd4;   // merge 读 A 写 B（phase=1）
    localparam S_RING   = 4'd6;   // 逐点 ring 判定（读 B）
    localparam S_DONE   = 4'd7;
    reg [3:0] state;

    // subpixel 子状态
    localparam SPX_CLR = 2'd0;   // 清 store 写侧（回卷）+ 发 subpixel start
    localparam SPX_RUN = 2'd1;   // 等 subpixel 输出流写回 store 写侧 → done
    reg [1:0] spx_sub;

    // ring 子状态
    localparam RG_RDPT = 4'd0;   // 发读请求（store + radram）
    localparam RG_WPT  = 4'd1;   // 锁存点 + r0
    localparam RG_CALL = 4'd4;   // 喂 ring 一次调用
    localparam RG_WAIT = 4'd5;   // 等 ring 结果 → 短路判定
    localparam RG_END  = 4'd6;   // 输出 probe（+inner）
    reg [2:0] rg_sub;
    reg ring_finish;             // RING 完成（跨块电平）

    //--------------------------------------------------------------------
    // 控制寄存器
    //--------------------------------------------------------------------
    reg [15:0] cnt_cand;
    reg [15:0] N_reg;            // 当前列表点数
    reg [15:0] ring_i;           // RING 当前点索引
    reg        m5_started, m3_started;
    reg [1:0]  rsel;             // 0=6 pixels, 1=4 pixels, 2=8 pixels
    reg [31:0] ring_x_reg, ring_y_reg;
    reg        rg_pass;
    reg [31:0] d0_hi, d0_lo, d0_thr, d0_ntrans, d0_opp_err;
    reg        d0_sector_ok;
    reg        inner_arm;        // RG_END 时是否要发 inner

    //--------------------------------------------------------------------
    // 子模块互连（先声明，避免隐式 net）
    //--------------------------------------------------------------------
    wire        store_wr_ready;
    wire [31:0] store_rd_x, store_rd_y;
    wire [15:0] store_count;
    wire        merge_res_valid, merge_res_ready;
    wire [31:0] merge_res_x, merge_res_y;
    wire        merge_busy, merge_done;
    wire        merge_rd_en;
    wire [N_ADDR_W-1:0] merge_rd_addr;
    wire        ring_in_ready, ring_out_valid;
    wire [31:0] ring_out_hi, ring_out_lo, ring_out_thr, ring_out_ntrans;
    wire [31:0] ring_out_opp_err;
    wire        ring_out_sector_ok, ring_out_pass;
    wire        c2fx_v, c2fy_v, c2fx_rdy, c2fy_rdy;
    wire [31:0] c2fx_r, c2fy_r;
    // subpixel_ctrl 互连
    wire        spx_busy, spx_done;
    wire        spx_pt_rd_en;
    wire [N_ADDR_W-1:0] spx_pt_rd_addr;
    wire        spx_gray_rd_en;
    wire [GRAY_ADDR_W-1:0] spx_gray_rd_addr;
    wire        spx_out_valid, spx_out_ready;
    wire [31:0] spx_out_x, spx_out_y;
    // ring_check 内部 gray 读口（经仲裁后对外）
    wire        ring_gray_rd_en;
    wire [GRAY_ADDR_W-1:0] ring_gray_rd_addr;
    // 组合 assign 目标（显式声明位宽，避免隐式 1bit net）
    wire        store_wr_valid;
    wire [31:0] store_wr_x, store_wr_y;
    wire        phase_sel, cand_fire;
    wire        spx_start, store_clr;
    wire [31:0] merge_radius;
    wire        merge_start;
    wire        ring_fire;
    wire [31:0] ring_in_x, ring_in_y, ring_in_radius;

    // 阶段互斥的 store 读口多路（subpixel / merge / nearest / RING）
    wire ctrl_rd_en = (rg_sub == RG_RDPT);
    wire [N_ADDR_W-1:0] ctrl_rd_addr = ring_i[N_ADDR_W-1:0];
    wire        store_rd_en = (state == S_SUBPX) ? spx_pt_rd_en :
                              (state == S_RING) ? ctrl_rd_en :
                              (((state == S_MERGE5) || (state == S_MERGE3)) && merge_rd_en);
    wire [N_ADDR_W-1:0] store_rd_addr =
        (state == S_SUBPX) ? spx_pt_rd_addr :
        (state == S_RING) ? ctrl_rd_addr : merge_rd_addr;

    // 阶段互斥的 gray 读口多路（subpixel / ring_check）；端口已声明，仅 assign 驱动
    assign gray_rd_en = (state == S_SUBPX) ? spx_gray_rd_en : ring_gray_rd_en;
    assign gray_rd_addr = (state == S_SUBPX) ? spx_gray_rd_addr : ring_gray_rd_addr;

    //--------------------------------------------------------------------
    // 例化
    //--------------------------------------------------------------------
    candidate_store #(.EXTERNAL_RAM(DDR_CANDIDATES),.USE_CE(USE_CE),.N_ADDR_W(N_ADDR_W)) u_store (.ce(ce),.scratch_rd_en(scratch_rd_en),.scratch_wr_en(scratch_wr_en),
        .scratch_rd_addr(scratch_rd_addr),.scratch_wr_addr(scratch_wr_addr),
        .scratch_wr_data(scratch_wr_data),.scratch_rd_data(scratch_rd_data),
        .clk (clk), .rst_n (rst_n),
        // 写侧回卷：start 清两侧；S_SUBPX 进入拍清一次写侧（SPX_CLR 脉冲），
        // 使 subpixel 结果从 A 地址 0 起写（覆盖 NMS 候选旧数据）。
        .clr (store_clr), .phase (phase_sel),
        .wr_valid (store_wr_valid), .wr_ready (store_wr_ready),
        .wr_x (store_wr_x), .wr_y (store_wr_y),
        .count (store_count),
        .rd_en (store_rd_en), .rd_addr (store_rd_addr),
        .rd_x (store_rd_x), .rd_y (store_rd_y)
    );

    // NMS 整数坐标 → fp32（x/y 各一条，同步流水）
    s32_to_f32 #(.USE_CE(USE_CE)) u_c2fx (.ce(ce),
        .clk (clk), .rst_n (rst_n),
        .in_valid (cand_fire), .in_ready (c2fx_rdy),
        .in_data ({21'd0, cand_x}),
        .out_valid (c2fx_v), .out_ready (store_wr_ready),
        .out_r (c2fx_r)
    );
    s32_to_f32 #(.USE_CE(USE_CE)) u_c2fy (.ce(ce),
        .clk (clk), .rst_n (rst_n),
        .in_valid (cand_fire), .in_ready (c2fy_rdy),
        .in_data ({21'd0, cand_y}),
        .out_valid (c2fy_v), .out_ready (store_wr_ready),
        .out_r (c2fy_r)
    );

    // One distance engine serves both merge passes; ring uses integers.
    wire md_valid,md_ready,md_rvalid;
    wire md_rready;
    wire [31:0] md_x,md_y,distance_result;
    wire distance_ready,distance_valid;
    wire merge_owner=(state==S_MERGE5 || state==S_MERGE3);
    assign md_ready=merge_owner && distance_ready;
    assign md_rvalid=merge_owner && distance_valid;
    fp32_hypot #(.USE_CE(USE_CE)) shared_distance(.ce(ce),.clk(clk),.rst_n(rst_n),
        .in_valid(merge_owner && md_valid),
        .in_ready(distance_ready),.in_a(md_x),.in_b(md_y),
        .out_valid(distance_valid),.out_ready(merge_owner&&md_rready),
        .out_r(distance_result));

    candidate_merge #(.SHARED_DISTANCE(1),.USE_CE(USE_CE),.N_ADDR_W(N_ADDR_W)) u_merge (.ce(ce),

        .distance_valid(md_valid),.distance_ready(md_ready),
        .distance_x(md_x),.distance_y(md_y),
        .distance_result_valid(md_rvalid),.distance_result_ready(md_rready),
        .distance_result(distance_result),
        .clk (clk), .rst_n (rst_n),
        .start (merge_start), .busy (merge_busy), .done (merge_done),
        .n_in (N_reg), .radius (merge_radius),
        .rd_en (merge_rd_en), .rd_addr (merge_rd_addr),
        .rd_x (store_rd_x), .rd_y (store_rd_y),
        .res_valid (merge_res_valid), .res_ready (merge_res_ready),
        .res_x (merge_res_x), .res_y (merge_res_y),
        .res_count ()
    );

    ring_check #(.VARIABLE_SCALE(VARIABLE_SCALE),.USE_CE(USE_CE),
        .W (IMG_W), .H (IMG_H),
        .GRAY_ADDR_W (GRAY_ADDR_W),
        .ROM_FILE (ROM_FILE)
    ) u_ring (.cfg_scale(cfg_scale),.ce(ce),
        .clk (clk), .rst_n (rst_n),
        .in_valid (ring_fire), .in_ready (ring_in_ready),
        .in_x (ring_in_x), .in_y (ring_in_y), .in_radius (ring_in_radius),
        .rd_en (ring_gray_rd_en), .rd_addr (ring_gray_rd_addr), .rd_data (gray_rd_data),
        .out_valid (ring_out_valid), .out_ready (1'b1),
        .out_hi (ring_out_hi), .out_lo (ring_out_lo), .out_thr (ring_out_thr),
        .out_ntrans (ring_out_ntrans), .out_opp_err (ring_out_opp_err),
        .out_sector_ok (ring_out_sector_ok), .out_pass (ring_out_pass)
    );

    // M5 亚像素精定位：SUBPX 期间读 store B（输入）、灰度，输出流写回 store A
    generate if(SHARED_SUBPIXEL) begin : g_subpixel_service
        assign sp_start=spx_start;
        assign spx_busy=sp_busy;
        assign spx_done=sp_done;
        assign sp_n_in=N_reg;
        assign sp_half_win=8'd7;
        assign spx_pt_rd_en=sp_pt_rd_en;
        assign spx_pt_rd_addr=sp_pt_rd_addr;
        assign sp_pt_rd_x=store_rd_x;
        assign sp_pt_rd_y=store_rd_y;
        assign spx_gray_rd_en=sp_gray_rd_en;
        assign spx_gray_rd_addr=sp_gray_rd_addr;
        assign spx_out_valid=sp_out_valid;
        assign sp_out_ready=spx_out_ready;
        assign spx_out_x=sp_out_x;
        assign spx_out_y=sp_out_y;
    end else begin : g_private_subpixel
    subpixel_ctrl #(.FIXED_ACCUM(FIXED_ACCUM),.FIXED_BILINEAR(FIXED_BILINEAR),.VARIABLE_SCALE(VARIABLE_SCALE),.USE_CE(USE_CE),
        .IMG_W (IMG_W), .IMG_H (IMG_H),
        .GRAY_ADDR_W (GRAY_ADDR_W),
        .N_ADDR_W (N_ADDR_W),
        .ROM_FILE (SUB_ROM_FILE)
    ) u_subpx (.cfg_scale(cfg_scale),.ce(ce),
        .clk (clk), .rst_n (rst_n),
        .start (spx_start), .busy (spx_busy), .done (spx_done),
        .n_in (N_reg), .half_win (8'd7),
        .pt_rd_en (spx_pt_rd_en), .pt_rd_addr (spx_pt_rd_addr),
        .pt_rd_x (store_rd_x), .pt_rd_y (store_rd_y),
        .gray_rd_en (spx_gray_rd_en), .gray_rd_addr (spx_gray_rd_addr),
        .gray_rd_data (gray_rd_data),
        .out_valid (spx_out_valid), .out_ready (spx_out_ready),
        .out_x (spx_out_x), .out_y (spx_out_y),
        .out_reliable ()
    );
        assign sp_start=0;
        assign sp_n_in=0;
        assign sp_half_win=0;
        assign sp_pt_rd_x=0;
        assign sp_pt_rd_y=0;
        assign sp_out_ready=0;
    end endgenerate


    //--------------------------------------------------------------------
    // 组合控制
    //--------------------------------------------------------------------
    // 相位轮换：CAP/SUBPX/NEAR/RING 写 A 读 B；MERGE5/MERGE3 写 B 读 A
    assign phase_sel = ((state == S_MERGE5) || (state == S_MERGE3)) ? 1'b1 : 1'b0;
    assign cand_fire = (state == S_CAP) && cand_valid && cand_ready;
    assign store_wr_valid = ((state == S_CAP) && c2fx_v && c2fy_v) ||
                            (((state == S_MERGE5) || (state == S_MERGE3)) && merge_res_valid) ||
                            ((state == S_SUBPX) && spx_out_valid);
    assign store_wr_x = (state == S_CAP) ? c2fx_r :
                        (state == S_SUBPX) ? spx_out_x : merge_res_x;
    assign store_wr_y = (state == S_CAP) ? c2fy_r :
                        (state == S_SUBPX) ? spx_out_y : merge_res_y;
    // subpixel 启动（SPX_CLR 拍 1 拍脉冲）+ 输出流背压 + 写侧回卷脉冲
    assign spx_start = (state == S_SUBPX) && (spx_sub == SPX_CLR);
    assign spx_out_ready = (state == S_SUBPX) && store_wr_ready;
    assign store_clr = start || ((state == S_SUBPX) && (spx_sub == SPX_CLR));
    assign merge_radius = (state == S_MERGE5) ? C_R5 : C_R3;
    assign merge_start = ((state == S_MERGE5) && !m5_started) ||
                         ((state == S_MERGE3) && !m3_started);
    assign merge_res_ready = (((state == S_MERGE5) || (state == S_MERGE3)) &&
                              store_wr_ready);
    // ring 调用
    assign ring_fire = (state == S_RING) && (rg_sub == RG_CALL) && ring_in_ready;
    assign ring_in_x  = ring_x_reg;
    assign ring_in_y  = ring_y_reg;
    assign ring_in_radius = (rsel == 2'd0) ? 32'h40c00000 : (rsel == 2'd1) ? 32'h40800000 : 32'h41000000;
    assign cand_ready = (state == S_CAP) && c2fx_rdy && c2fy_rdy &&
                        (cnt_cand <= MAX_CAND[15:0]);

    //--------------------------------------------------------------------
    // 块 A：主状态机（state/busy/done/status/N_reg）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state  <= S_IDLE;
            busy   <= 1'b0;
            done   <= 1'b0;
            status <= 2'b00;
            N_reg  <= 16'd0;
            spx_sub <= SPX_CLR;
        end else if(!USE_CE || ce) begin begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        busy   <= 1'b1;
                        done   <= 1'b0;
                        status <= 2'b00;
                        state  <= S_CAP;
                    end
                end

                S_CAP: begin
                    if (cnt_cand > MAX_CAND[15:0]) begin
                        status <= 2'b11;
                        done   <= 1'b1;
                        busy   <= 1'b0;
                        state  <= S_DONE;
                    end else if (cand_done) begin
                        if (cnt_cand < MIN_CAND[15:0]) begin
                            status <= 2'b10;
                            done   <= 1'b1;
                            busy   <= 1'b0;
                            state  <= S_DONE;
                        end else begin
                            N_reg <= cnt_cand;
                            state <= S_MERGE5;
                        end
                    end
                end

                S_MERGE5: begin
                    if (m5_started && merge_done && !merge_busy) begin
                        N_reg <= store_count;   // cnt_b
                        state <= S_SUBPX;
                    end
                end

                S_SUBPX: begin
                    case (spx_sub)
                        SPX_CLR: begin
                            // 本拍组合 clr=1（清 store 写侧）+ spx_start=1（启动 subpixel）
                            spx_sub <= SPX_RUN;
                        end
                        default: begin   // SPX_RUN：等 subpixel 输出流写回 → done
                            if (spx_done) begin
                                spx_sub <= SPX_CLR;
                                state   <= S_MERGE3;   // N 不变（subpixel 不增减点数）
                            end
                        end
                    endcase
                end

                S_MERGE3: begin
                    if (m3_started && merge_done && !merge_busy) begin
                        N_reg <= store_count;   // cnt_b（merge3 写 B）
                        state <= S_RING;
                    end
                end

                S_RING: begin
                    if (ring_finish) begin
                        status <= 2'b01;
                        done   <= 1'b1;
                        busy   <= 1'b0;
                        state  <= S_DONE;
                    end
                end

                S_DONE: begin
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end // synchronous clock enable
    end

    //--------------------------------------------------------------------
    // 块 B：ring 子状态机（含 probe/inner 输出与短路判定）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            rg_sub       <= RG_RDPT;
            ring_i       <= 16'd0;
            ring_x_reg   <= 32'd0;
            ring_y_reg   <= 32'd0;
            rsel         <= 2'd0;
            rg_pass      <= 1'b0;
            d0_hi        <= 32'd0;
            d0_lo        <= 32'd0;
            d0_thr       <= 32'd0;
            d0_ntrans    <= 32'd0;
            d0_opp_err   <= 32'd0;
            d0_sector_ok <= 1'b0;
            inner_arm    <= 1'b0;
            probe_valid  <= 1'b0;
            inner_valid  <= 1'b0;
            inner_x      <= 32'd0;
            inner_y      <= 32'd0;
            ring_finish  <= 1'b0;
        end else if(!USE_CE || ce) begin begin
            // probe/inner 输出自清（握手恒收，1 拍）
            probe_valid <= 1'b0;
            inner_valid <= 1'b0;
            // Keep all ring-substate writes in this clocked process.
            if (state == S_MERGE3 && m3_started && merge_done && !merge_busy) begin
                ring_i <= 0;
                rg_sub <= RG_RDPT;
                rsel <= 2'd0;
            end
            // M8.1：ring_finish 只在复位清零会残留到下一帧——帧 2 进 S_RING 时
            //   ring_finish=1 导致 S_RING 瞬间空跑完成、inner 全丢（order 无点）。
            //   非 RING 阶段清零（S_RING 期间保持由 RG_END 置位）。
            if (state != S_RING)
                ring_finish <= 1'b0;
            if (state == S_RING) begin
                case (rg_sub)
                    RG_RDPT: begin
                        rg_sub <= RG_WPT;
                    end
                    RG_WPT: begin
                        ring_x_reg <= store_rd_x;
                        ring_y_reg <= store_rd_y;
                        rsel       <= 2'd0;
                        rg_sub     <= RG_CALL;
                    end
                    RG_CALL: begin
                        if (ring_fire)
                            rg_sub <= RG_WAIT;
                    end
                    RG_WAIT: begin
                        if (ring_out_valid) begin
                            if (rsel == 2'd0) begin
                                d0_hi        <= ring_out_hi;
                                d0_lo        <= ring_out_lo;
                                d0_thr       <= ring_out_thr;
                                d0_ntrans    <= ring_out_ntrans;
                                d0_opp_err   <= ring_out_opp_err;
                                d0_sector_ok <= ring_out_sector_ok;
                                if (ring_out_pass) begin
                                    rsel   <= 2'd1;
                                    rg_sub <= RG_CALL;
                                end else begin
                                    rg_pass   <= 1'b0;
                                    inner_arm <= 1'b0;
                                    rg_sub    <= RG_END;
                                end
                            end else if (rsel == 2'd1) begin
                                if (ring_out_pass) begin
                                    rg_pass   <= 1'b1;
                                    inner_arm <= 1'b1;
                                    rg_sub    <= RG_END;
                                end else begin
                                    rsel   <= 2'd2;
                                    rg_sub <= RG_CALL;
                                end
                            end else begin
                                rg_pass   <= ring_out_pass;
                                inner_arm <= ring_out_pass;
                                rg_sub    <= RG_END;
                            end
                        end
                    end
                    RG_END: begin
                        probe_valid     <= 1'b1;
                        probe_hi        <= d0_hi;
                        probe_lo        <= d0_lo;
                        probe_thr       <= d0_thr;
                        probe_ntrans    <= d0_ntrans;
                        probe_opp_err   <= d0_opp_err;
                        probe_sector_ok <= d0_sector_ok;
                        probe_pass      <= rg_pass;
                        if (inner_arm) begin
                            inner_valid <= 1'b1;
                            inner_x     <= ring_x_reg;
                            inner_y     <= ring_y_reg;
                        end
                        // 总是推进 rg_sub（即使本点结束），防止 finish 拍重复输出 probe
                        rg_sub <= RG_RDPT;
                        if (ring_i + 16'd1 >= N_reg) begin
                            ring_finish <= 1'b1;
                        end else begin
                            ring_i <= ring_i + 16'd1;
                        end
                    end
                    default: rg_sub <= RG_RDPT;
                endcase
            end
        end
    end // synchronous clock enable
    end

    //--------------------------------------------------------------------
    // 块 C：计数（候选 / merge·near 启动 / nearest 写指针 / inner 总数）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n)
            cnt_cand <= 16'd0;
        else if(!USE_CE || ce) begin if (state == S_IDLE)
            cnt_cand <= 16'd0;
        else if (cand_fire)
            cnt_cand <= cnt_cand + 16'd1;
    end // synchronous clock enable
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            m5_started   <= 1'b0;
            m3_started   <= 1'b0;
        end else if(!USE_CE || ce) begin begin
            if (state == S_MERGE5) m5_started <= 1'b1;
            if (state == S_MERGE3) m3_started <= 1'b1;
            if (state == S_SUBPX)  m5_started <= 1'b0;
            if (state == S_RING)   m3_started <= 1'b0;   // M8.1：m3_started 从不清零会残留
                // 到下一帧——帧 2 的 S_MERGE3 merge_start=0、merge3 永不启动，filter 卡死
        end
    end // synchronous clock enable
    end

    always @(posedge clk) begin
        if (!rst_n)
            inner_total <= 16'd0;
        else if(!USE_CE || ce) begin if (state == S_IDLE)
            inner_total <= 16'd0;
        else if ((state == S_RING) && (rg_sub == RG_END) && inner_arm)
            inner_total <= inner_total + 16'd1;
    end // synchronous clock enable
    end

endmodule
