`timescale 1ns / 1ps
//==============================================================================
// pyramid_ctrl.sv — 金字塔缩图层控制（M6.2）
//------------------------------------------------------------------------------
// 作用：以"有限深度层描述符栈"替代 chessboard.cpp::detect_chessboard 的递归
//       缩图。从 L0（外部灰度 RAM 预载）起逐层生成缩图，直到某层
//       max(W,H)<=960 或 W<32 或 H<32（或层数达 MAX_DEPTH）停止。
//
// 公式（chessboard.cpp 第 14-17 行，位级权威）：
//     half(x,y) = (a + b + c + d + 2) >> 2     （整数 floor 除法）
//   a = src(2x,2y)    b = src(2x+1,2y)
//   c = src(2x,2y+1)  d = src(2x+1,2y+1)
//   输出尺寸 floor(W/2) x floor(H/2)；奇数末行/末列不参与（源多余像素被消费但
//   丢弃）。缩图由基础件 downsample2x 完成（M6.1 已对拍，只读不改）。
//
// 层条件（对最新层 L_{d-1} 判定，任一不满足即停止）：
//     W_{d-1} >= 32 && H_{d-1} >= 32 && max(W_{d-1},H_{d-1}) > 960
//
// 基址布局（GRAY_ADDR_W 位地址空间，连续排布）：
//     base_0 = cfg_base0（外部预载）
//     base_d = base_{d-1} + W_{d-1}*H_{d-1}     (d = 1..level_count-1)
//
// 接口语义：
//   start      ：busy=0 时单拍启动，锁存 cfg；level_count 复位为 1（含 L0）
//   busy       ：启动后置 1；全部缩图层生成完毕置 0
//   done       ：电平保持置 1（下次 start 清零）
//   gray 读口  ：registered 读（gray_rd_en=1 的下一拍 gray_rd_data 有效）。
//                读流经 1-deep 缓冲（buf_data）握手喂给 downsample2x：
//                其 in_ready=0（气泡/输出挂起）时 gray_rd_en 拉低、
//                rd_addr 保持、buf_data 保持，不丢数、不欠速。
//   gray 写口  ：组合输出，out_acc（out_valid&&out_ready）拍 1 拍完成，无背压
//   level_*    ：done 时有效。level_count = 已生成层数（含 L0，>=1）
//
// 时序（10ns 周期示意，clk posedge 采样）：
//   [idle]  start=1 → busy=1, 描述符[0]={cfg_base0,cfg_w0,cfg_h0}
//   [layer] 判定层条件：满足 → 锁存 cur_*、清计数、downsample2x start 脉冲，
//           进入 feed；否则 done=1、busy=0
//   [feed]  registered 读流水：发读拍 gray_rd_en=1/rd_addr 有效 → 下一拍
//           rd_data 到达，经 in-flight 标志（rd_pend，保持到转移完成）锁存
//           buf_data → 再下一拍握手喂出（每拍 1 像素；in_ready=0 时
//           rd_addr/rd_data 保持，数据排队不丢，恢复后继续流水）
//           捕获输出流写 gray RAM 到 base_next+out_cnt（out_ready 每 8 拍
//           停 1 拍，模拟外部写口可能背压的鲁棒性，覆盖 out_ready 反驱）
//   [done]  downsample2x 的 done 为电平保持，打 2 拍检测上升沿（0→1）再推进，
//           避免电平被误认为持续 busy；记录新层描述符 → 回 [layer]
//
// 防御：W0<2 或 H0<2 时 level_count=1 直接 done（不生成任何缩图层）。
// 连续帧复用：每次 start 时 level_count 复位为 1、层索引清零，busy/done 按
// 新配置重来（done 电平在 start 拍清零）。
//==============================================================================
module pyramid_ctrl #(parameter USE_CE=0,
    parameter MAX_W       = 2560,
    parameter MAX_H       = 1440,
    parameter MAX_DEPTH   = 4,          // 层描述符深度（含 L0）
    parameter GRAY_ADDR_W = 26          // >= $clog2(Σ_{d<MAX_DEPTH} W_d*H_d + base0)
) (
    // Global synchronous stall for variable-latency backing memory.
    input wire ce,

    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,        // busy=0 时单拍启动；锁存配置
    input  wire [10:0]             cfg_w0,
    input  wire [10:0]             cfg_h0,
    input  wire [GRAY_ADDR_W-1:0]  cfg_base0,    // L0 灰度基址
    output reg                     busy,
    output reg                     done,         // 电平保持（下次 start 清零）
    // 灰度 RAM 读口（registered 读：rd_en=1 的下一拍出数据）
    output wire                    gray_rd_en,
    output wire [GRAY_ADDR_W-1:0]  gray_rd_addr,
    input  wire [7:0]              gray_rd_data,
    // 灰度 RAM 写口（写各缩图层；1 拍完成，无背压）
    output reg                     gray_wr_en,
    output reg  [GRAY_ADDR_W-1:0]  gray_wr_addr,
    output reg  [7:0]              gray_wr_data,
    // 层描述符（done 时有效）
    output reg  [$clog2(MAX_DEPTH):0] level_count,   // 实际层数（含 L0，>=1）
    output reg  [GRAY_ADDR_W-1:0]  level_base  [0:MAX_DEPTH-1],
    output reg  [10:0]             level_w     [0:MAX_DEPTH-1],
    output reg  [10:0]             level_h     [0:MAX_DEPTH-1]
);

    //--------------------------------------------------------------------
    // 状态机 / 当前层参数
    //--------------------------------------------------------------------
    localparam S_IDLE  = 2'd0, S_LAYER = 2'd1, S_FEED = 2'd2;
    reg [1:0] st;

    reg [GRAY_ADDR_W-1:0] cur_base;     // 正在下采样的源层基址
    reg [10:0]            cur_w, cur_h; // 正在下采样的源层宽/高

    // 层启动条件（对最新层 L_{level_count-1} 判定）
    wire [10:0] d_w = level_w[level_count - 1];
    wire [10:0] d_h = level_h[level_count - 1];
    wire layer_cond = (st == S_LAYER) && (level_count >= 1) &&
                      (d_w >= 32) && (d_h >= 32) &&
                      ((d_w > d_h) ? d_w : d_h) > 960;
    wire layer_start = layer_cond && (level_count < MAX_DEPTH);

    // 本层像素总数 / 下一层基址（base_next = cur_base + cur_w*cur_h）
    wire [GRAY_ADDR_W:0] layer_pixels = {1'b0, cur_w} * {1'b0, cur_h};
    wire [GRAY_ADDR_W-1:0] base_next  = cur_base + layer_pixels[GRAY_ADDR_W-1:0];

    //--------------------------------------------------------------------
    // 读流（gray RAM → downsample2x）：1-deep 缓冲 + registered 读流水
    //   rd_pend 保持到转移完成：in_ready=0（气泡）期间 rd_addr 保持、
    //   rd_data 排队、不丢数；恢复后转移锁存并继续流水
    //--------------------------------------------------------------------
    reg              buf_valid;
    reg [7:0]        buf_data;
    reg              rd_pend;        // registered 读在途标志（保持到转移）
    reg [GRAY_ADDR_W-1:0] rd_addr_r; // 读地址（背压时保持）
    reg [GRAY_ADDR_W:0]   rd_cnt;    // 已发起读请求数（限 N 个）
    reg [GRAY_ADDR_W:0]   fed_cnt;   // 已握手喂出数（debug/统计）

    //--------------------------------------------------------------------
    // downsample2x 例化（只读基础件，M6.1 已对拍）
    //--------------------------------------------------------------------
    reg  d_start;
    wire d_busy, d_done;
    wire d_in_ready, d_out_valid;
    wire [7:0] d_out_gray;
    reg  d_out_ready;

    wire d_in_valid = buf_valid;
    wire [7:0] d_in_gray = buf_data;

    downsample2x #(.USE_CE(USE_CE),
        .MAX_W (MAX_W)
    ) u_down (.ce(ce),
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (d_start),
        .cfg_w    ({1'b0, cur_w}),   // 显式零扩展：ADDR_W=12 > 11 位 cur_w
        .cfg_h    ({1'b0, cur_h}),
        .busy     (d_busy),
        .done     (d_done),
        .in_valid (d_in_valid),
        .in_ready (d_in_ready),
        .in_gray  (d_in_gray),
        .out_valid(d_out_valid),
        .out_ready(d_out_ready),
        .out_gray (d_out_gray)
    );

    //--------------------------------------------------------------------
    // 读流握手控制：can_read 组合（无组合环：d_in_ready 不依赖读口）
    //   rd_pend：registered 读的在途标志，保持到数据成功转移到 buf——
    //   避免"数据在 rd_data 上排队时 rd_addr 被更新冲掉"的丢数（downsample2x
    //   气泡拍 in_ready=0 期间的经典坑）
    //--------------------------------------------------------------------
    wire feed_open   = !buf_valid || (buf_valid && d_in_ready);
    wire accepted_in = buf_valid && d_in_ready;
    wire take        = rd_pend && (!buf_valid || accepted_in);
    wire read_done   = (rd_cnt >= layer_pixels);
    wire can_read    = (st == S_FEED) && !read_done && (!rd_pend || take) && feed_open;

    assign gray_rd_en   = can_read;
    assign gray_rd_addr = rd_addr_r;

    always @(posedge clk) begin
        if (!rst_n) begin
            buf_valid <= 1'b0;
            buf_data  <= 8'd0;
            rd_pend   <= 1'b0;
            rd_addr_r <= {GRAY_ADDR_W{1'b0}};
            rd_cnt    <= {(GRAY_ADDR_W+1){1'b0}};
            fed_cnt   <= {(GRAY_ADDR_W+1){1'b0}};
        end else if(!USE_CE || ce) begin if (layer_start) begin
            buf_valid <= 1'b0;
            buf_data  <= 8'd0;
            rd_pend   <= 1'b0;
            rd_addr_r <= level_base[level_count-1];
            rd_cnt    <= {(GRAY_ADDR_W+1){1'b0}};
            fed_cnt   <= {(GRAY_ADDR_W+1){1'b0}};
        end else begin
            // 在途标志：发读置 1；转移完成清 0；否则保持（数据在 rd_data 排队）
            rd_pend <= can_read || (rd_pend && !take);
            // 转移锁存：数据到达且 buf 可接收 → 锁存；否则背压保持
            if (take) begin
                buf_valid <= 1'b1;
                buf_data  <= gray_rd_data;
            end else if (buf_valid && !d_in_ready) begin
                buf_valid <= 1'b1;
            end else begin
                buf_valid <= 1'b0;
            end
            if (can_read) begin
                rd_addr_r <= rd_addr_r + 1'b1;
                rd_cnt    <= rd_cnt + 1'b1;
            end
            if (accepted_in) fed_cnt <= fed_cnt + 1'b1;
        end
    end // synchronous clock enable
    end

    //--------------------------------------------------------------------
    // 输出捕获（downsample2x → gray RAM）：组合写口，无背压
    //   out_ready 每 8 拍停 1 拍：模拟外部写口可能背压，覆盖 out_ready 反驱
    //--------------------------------------------------------------------
    reg [GRAY_ADDR_W:0] out_cnt;        // 已写输出像素数
    reg [2:0]           bp;             // 背压周期计数

    wire out_acc = d_out_valid && d_out_ready;

    always @* begin
        gray_wr_en   = out_acc;
        gray_wr_addr = base_next + out_cnt;
        gray_wr_data = d_out_gray;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            out_cnt <= {(GRAY_ADDR_W+1){1'b0}};
            bp      <= 3'd0;
        end else if(!USE_CE || ce) begin begin
            bp <= bp + 1'b1;                     // 自由计数，0 拍停 1 拍
            if (layer_start) out_cnt <= {(GRAY_ADDR_W+1){1'b0}};
            else if (out_acc) out_cnt <= out_cnt + 1'b1;
        end
    end // synchronous clock enable
    end

    assign d_out_ready = (bp != 3'd0);

    //--------------------------------------------------------------------
    // downsample2x done 上升沿检测（电平保持 → 打 2 拍检测 0→1）
    //--------------------------------------------------------------------
    reg d_done_d1, d_done_d2;
    always @(posedge clk) begin
        if (!rst_n) begin
            d_done_d1 <= 1'b0;
            d_done_d2 <= 1'b0;
        end else if(!USE_CE || ce) begin begin
            d_done_d1 <= d_done;
            d_done_d2 <= d_done_d1;
        end
    end // synchronous clock enable
    end
    wire d_done_rise = d_done_d1 && !d_done_d2;

    //--------------------------------------------------------------------
    // 主状态机：idle → layer 判定 → feed 下采样 →（done 上升沿）回 layer
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            st          <= S_IDLE;
            busy        <= 1'b0;
            done        <= 1'b0;
            d_start     <= 1'b0;
            level_count <= {( $clog2(MAX_DEPTH)+1){1'b0}};
            cur_base    <= {GRAY_ADDR_W{1'b0}};
            cur_w       <= 11'd0;
            cur_h       <= 11'd0;
        end else if(!USE_CE || ce) begin begin
            case (st)
                S_IDLE: begin
                    if (start) begin
                        level_w[0]    <= cfg_w0;
                        level_h[0]    <= cfg_h0;
                        level_base[0] <= cfg_base0;
                        level_count   <= 1;          // 含 L0，共 1 层
                        done          <= 1'b0;
                        if (cfg_w0 < 2 || cfg_h0 < 2) begin
                            // 防御：不可缩图，直接完成
                            busy <= 1'b0;
                            done <= 1'b1;
                        end else begin
                            busy <= 1'b1;
                            st   <= S_LAYER;
                        end
                    end
                end
                S_LAYER: begin
                    if (layer_start) begin
                        // 锁存源层参数 + 启动 downsample2x（脉冲，下一拍生效）
                        cur_base <= level_base[level_count-1];
                        cur_w    <= level_w[level_count-1];
                        cur_h    <= level_h[level_count-1];
                        d_start  <= 1'b1;
                        st       <= S_FEED;
                    end else begin
                        // 层条件不满足或层数已达上限：停止
                        busy <= 1'b0;
                        done <= 1'b1;
                        st   <= S_IDLE;
                    end
                end
                S_FEED: begin
                    d_start <= 1'b0;
                    if (d_done_rise) begin
                        // 记录新层描述符（done 上升沿时输出已全部写入）
                        level_base[level_count] <= base_next;
                        level_w[level_count]    <= cur_w >> 1;
                        level_h[level_count]    <= cur_h >> 1;
                        level_count             <= level_count + 1'b1;
                        st                      <= S_LAYER;
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end // synchronous clock enable
    end

endmodule
