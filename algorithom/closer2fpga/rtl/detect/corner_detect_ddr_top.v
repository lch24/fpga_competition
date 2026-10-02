`timescale 1ns / 1ps
//==============================================================================
// corner_detect_ddr_top.v — M8 联调顶层（帧级一键处理接口）
//------------------------------------------------------------------------------
// 把 M7 各 DMA 客户端收敛为单一 DDR 端口（ddr_port_arbiter）+ 帧级任务流
// （frame_task_ctrl），形成可交队友联调的顶层。
//
// 结构：
//   ddr_port_arbiter #(N_RD=2, N_WR=2)
//     读客户端 0 = gray_fetch（内部：DDR 灰度 → 片上 gray RAM）
//     读客户端 1 = ext_rd_*  （外部读：下游读回响应图 / 测试）
//     写客户端 0 = resp_ddr_writer（内部：响应字流 → DDR）
//     写客户端 1 = ext_wr_*  （外部写：上游预载灰度 / 测试）
//   frame_task_ctrl：GF → PYR → DET → RESP 串行调度（start/done/busy/status）
//   gray RAM：内部 reg 数组，同步写、registered 读 1 拍；
//     读口由 pyramid/detect 按阶段互斥 mux（子模块 busy 选通），
//     写口由 gray_fetch/pyramid 按阶段互斥 mux（参考 tb_detect_ddr active 选通）
//
// cfg 接线：frame_task_ctrl 在 process 拍锁存 cfg 用于阶段决策（cfg_pyr_en/
//   cfg_resp_dump_en）；各子模块的 cfg_* 由 top 直连 cfg 总线 —— 子模块在
//   自身 start 拍采样（TB 保证 cfg 在帧期间稳定，M7 同款手法）。
//
// 响应导出：detect_ctrl.resp_dump_* 直连 resp_ddr_writer.in_*；
//   cfg_words = (W0>>(DEPTH-1))*(H0>>(DEPTH-1))（参数化 localparam）。
//==============================================================================
module corner_detect_ddr_top #(
    parameter W0          = 1280,
    parameter H0          = 720,
    parameter DEPTH       = 2,          // 层数（含 L0；DEPTH=1 无金字塔）
    parameter GRAY_ADDR_W = 21,         // >= $clog2(Σ层像素 + ram_base)
    parameter MAX_W       = 2560,       // pyramid 上限
    parameter MAX_H       = 1440,
    parameter MAX_DEPTH   = 4,
    parameter ROWS        = 5,
    parameter COLS        = 8
) (
    input  wire                    clk,
    input  wire                    rst_n,

    // ---- 帧命令与状态 ----
    input  wire                    process_frame,  // busy=0 时单拍启动一帧（SV 关键字 process 保留，改名）
    output wire                    busy,
    output wire                    done,           // 电平保持（下次 process 清零）
    output wire [1:0]              status,         // 01=成功 10=子模块错误

    // ---- 帧级配置（process 前稳定）----
    input  wire [31:0]             cfg_gray_base,   // DDR 灰度区首字节地址
    input  wire [31:0]             cfg_gray_stride, // DDR 行跨度（字节）
    input  wire [15:0]             cfg_gray_w,      // 每行像素/字节
    input  wire [15:0]             cfg_gray_h,      // 行数
    input  wire [GRAY_ADDR_W-1:0]  cfg_ram_base,    // 片上 gray RAM 基址
    input  wire [31:0]             cfg_resp_base,   // 响应图写 DDR 首字节地址
    input  wire                    cfg_resp_dump_en, // 响应图帧级导出使能
    input  wire                    cfg_pyr_en,      // 金字塔缩图使能

    // ---- 40 点输出（直通 detect）----
    output wire                    out_valid,
    input  wire                    out_ready,
    output wire [31:0]             out_x,
    output wire [31:0]             out_y,
    output wire [15:0]             out_total,
    output wire                    out_grid_ok,

    // ---- DDR 模型侧单组端口（直通 arbiter）----
    output wire                    m_rd_req_valid,
    input  wire                    m_rd_req_ready,
    output wire [31:0]             m_rd_req_addr,
    output wire [31:0]             m_rd_req_len_bytes,
    output wire [15:0]             m_rd_req_tag,
    input  wire                    m_rd_ret_valid,
    output wire                    m_rd_ret_ready,
    input  wire [31:0]             m_rd_ret_data,
    input  wire [3:0]              m_rd_ret_keep,
    input  wire [15:0]             m_rd_ret_tag,
    input  wire                    m_rd_ret_last,
    input  wire                    m_rd_ret_error,
    output wire                    m_wr_req_valid,
    input  wire                    m_wr_req_ready,
    output wire [31:0]             m_wr_req_addr,
    output wire [31:0]             m_wr_req_len_bytes,
    output wire [15:0]             m_wr_req_tag,
    output wire                    m_wr_dat_valid,
    input  wire                    m_wr_dat_ready,
    output wire [31:0]             m_wr_dat_data,
    output wire [3:0]              m_wr_dat_keep,
    output wire                    m_wr_dat_last,
    input  wire                    m_wr_cplt_valid,
    output wire                    m_wr_cplt_ready,
    input  wire [15:0]             m_wr_cplt_tag,
    input  wire                    m_wr_cplt_error,

    // ---- 外部读客户端（arbiter 读下标 1；req 为输入、ret 为输出）----
    input  wire                    ext_rd_req_valid,
    output wire                    ext_rd_req_ready,
    input  wire [31:0]             ext_rd_req_addr,
    input  wire [31:0]             ext_rd_req_len,
    input  wire [15:0]             ext_rd_req_tag,
    output wire                    ext_rd_ret_valid,
    input  wire                    ext_rd_ret_ready,
    output wire [31:0]             ext_rd_ret_data,
    output wire [3:0]              ext_rd_ret_keep,
    output wire [15:0]             ext_rd_ret_tag,
    output wire                    ext_rd_ret_last,
    output wire                    ext_rd_ret_error,

    // ---- 外部写客户端（arbiter 写下标 1；req/dat 为输入、done 为输出）----
    input  wire                    ext_wr_req_valid,
    output wire                    ext_wr_req_ready,
    input  wire [31:0]             ext_wr_req_addr,
    input  wire [31:0]             ext_wr_req_len,
    input  wire [15:0]             ext_wr_req_tag,
    input  wire                    ext_wr_dat_valid,
    output wire                    ext_wr_dat_ready,
    input  wire [31:0]             ext_wr_dat_data,
    input  wire [3:0]              ext_wr_dat_keep,
    input  wire                    ext_wr_dat_last,
    output wire                    ext_wr_done_valid,
    input  wire                    ext_wr_done_ready,
    output wire [15:0]             ext_wr_done_tag,
    output wire                    ext_wr_done_error
);

    localparam integer PIXELS_DEPTH = (W0 >> (DEPTH-1)) * (H0 >> (DEPTH-1));  // 最深层像素 = cfg_words

    //--------------------------------------------------------------------
    // arbiter 客户端数组（unpacked，[0:N-1]）
    //--------------------------------------------------------------------
    wire        rd_req_valid_a [0:1];  wire        rd_req_ready_a [0:1];
    wire [31:0] rd_req_addr_a  [0:1];  wire [31:0] rd_req_len_a   [0:1];
    wire [15:0] rd_req_tag_a   [0:1];
    wire        rd_ret_valid_a [0:1];  wire        rd_ret_ready_a [0:1];
    wire [31:0] rd_ret_data_a  [0:1];  wire [3:0]  rd_ret_keep_a  [0:1];
    wire [15:0] rd_ret_tag_a   [0:1];  wire        rd_ret_last_a  [0:1];
    wire        rd_ret_error_a [0:1];
    wire        wr_req_valid_a [0:1];  wire        wr_req_ready_a [0:1];
    wire [31:0] wr_req_addr_a  [0:1];  wire [31:0] wr_req_len_a   [0:1];
    wire [15:0] wr_req_tag_a   [0:1];
    wire        wr_dat_valid_a [0:1];  wire        wr_dat_ready_a [0:1];
    wire [31:0] wr_dat_data_a  [0:1];  wire [3:0]  wr_dat_keep_a  [0:1];
    wire        wr_dat_last_a  [0:1];
    wire        wr_done_valid_a[0:1];  wire        wr_done_ready_a[0:1];
    wire [15:0] wr_done_tag_a  [0:1];  wire        wr_done_error_a[0:1];

    //--------------------------------------------------------------------
    // frame_task_ctrl ↔ 子模块控制
    //--------------------------------------------------------------------
    wire f_start, f_busy, f_done;
    wire [1:0] f_status;
    wire pyr_start, pyr_busy, pyr_done;
    wire det_start, det_busy, det_done;
    wire [1:0] det_status;
    wire w_start, w_busy, w_done;
    wire [1:0] w_status;

    frame_task_ctrl #(
        .ADDR_W (32), .LEN_W (32), .GRAY_ADDR_W (GRAY_ADDR_W)
    ) u_frame (
        .clk (clk), .rst_n (rst_n),
        .process_frame (process_frame), .busy (busy), .done (done), .status (status),
        .cfg_gray_base (cfg_gray_base), .cfg_gray_stride (cfg_gray_stride),
        .cfg_gray_w (cfg_gray_w), .cfg_gray_h (cfg_gray_h),
        .cfg_ram_base (cfg_ram_base), .cfg_resp_base (cfg_resp_base),
        .cfg_resp_dump_en (cfg_resp_dump_en), .cfg_pyr_en (cfg_pyr_en),
        .gf_start (f_start), .gf_busy (f_busy), .gf_done (f_done), .gf_status (f_status),
        .pyr_start (pyr_start), .pyr_busy (pyr_busy), .pyr_done (pyr_done),
        .det_start (det_start), .det_busy (det_busy), .det_done (det_done), .det_status (det_status),
        .w_start (w_start), .w_busy (w_busy), .w_done (w_done), .w_status (w_status)
    );

    //--------------------------------------------------------------------
    // gray_fetch（DDR 灰度 → 片上 gray RAM；cfg 直连总线，start 拍采样）
    //--------------------------------------------------------------------
    wire        f_gray_wr_en;
    wire [GRAY_ADDR_W-1:0] f_gray_wr_addr;
    wire [7:0]  f_gray_wr_data;
    wire        f_rd_req_valid, f_rd_req_ready;
    wire [31:0] f_rd_req_addr, f_rd_req_len;
    wire [15:0] f_rd_req_tag;
    wire        f_rd_ret_valid, f_rd_ret_ready, f_rd_ret_last, f_rd_ret_error;
    wire [31:0] f_rd_ret_data;
    wire [3:0]  f_rd_ret_keep;
    wire [15:0] f_rd_ret_tag;

    gray_fetch #(
        .ADDR_W (32), .LEN_W (32), .TAG_W (16), .GRAY_ADDR_W (GRAY_ADDR_W)
    ) u_fetch (
        .clk (clk), .rst_n (rst_n),
        .start (f_start), .busy (f_busy), .done (f_done), .status (f_status),
        .cfg_ddr_base (cfg_gray_base), .cfg_stride (cfg_gray_stride),
        .cfg_w (cfg_gray_w), .cfg_h (cfg_gray_h), .cfg_ram_base (cfg_ram_base),
        .gray_wr_en (f_gray_wr_en), .gray_wr_addr (f_gray_wr_addr), .gray_wr_data (f_gray_wr_data),
        .rd_req_valid (f_rd_req_valid), .rd_req_ready (f_rd_req_ready),
        .rd_req_addr (f_rd_req_addr), .rd_req_len (f_rd_req_len), .rd_req_tag (f_rd_req_tag),
        .rd_ret_valid (f_rd_ret_valid), .rd_ret_ready (f_rd_ret_ready),
        .rd_ret_data (f_rd_ret_data), .rd_ret_keep (f_rd_ret_keep),
        .rd_ret_tag (f_rd_ret_tag), .rd_ret_last (f_rd_ret_last), .rd_ret_error (f_rd_ret_error)
    );

    //--------------------------------------------------------------------
    // 片上 gray RAM 信号（先声明，供 pyramid/detect 例化端口引用）
    //   （同步写、registered 读 1 拍；读写口阶段互斥 mux 见下方块）
    //--------------------------------------------------------------------
    reg [7:0]  gray_mem [0:(1<<GRAY_ADDR_W)-1];
    wire       gray_rd_en;
    wire [GRAY_ADDR_W-1:0] gray_rd_addr;
    reg  [7:0] gray_rd_data;
    wire       gray_wr_en;
    wire [GRAY_ADDR_W-1:0] gray_wr_addr;
    wire [7:0] gray_wr_data;

    //--------------------------------------------------------------------
    // pyramid_ctrl（片上缩图；cfg_w0/h0/base0 = cfg 总线）
    //--------------------------------------------------------------------
    wire        pyr_gray_rd_en, pyr_gray_wr_en;
    wire [GRAY_ADDR_W-1:0] pyr_gray_rd_addr, pyr_gray_wr_addr;
    wire [7:0]  pyr_gray_wr_data;

    pyramid_ctrl #(
        .MAX_W (MAX_W), .MAX_H (MAX_H), .MAX_DEPTH (MAX_DEPTH), .GRAY_ADDR_W (GRAY_ADDR_W)
    ) u_pyr (
        .clk (clk), .rst_n (rst_n),
        .start (pyr_start), .busy (pyr_busy), .done (pyr_done),
        .cfg_w0 (cfg_gray_w[10:0]), .cfg_h0 (cfg_gray_h[10:0]), .cfg_base0 (cfg_ram_base),
        .gray_rd_en (pyr_gray_rd_en), .gray_rd_addr (pyr_gray_rd_addr), .gray_rd_data (gray_rd_data),
        .gray_wr_en (pyr_gray_wr_en), .gray_wr_addr (pyr_gray_wr_addr), .gray_wr_data (pyr_gray_wr_data),
        .level_count (), .level_base (), .level_w (), .level_h ()
    );

    //--------------------------------------------------------------------
    // detect_ctrl（检测全链 + 响应图帧级导出；cfg_base0=cfg_ram_base）
    //   per-frame 软复位：GF/PYR 阶段拉低 det_rst_n，把 detect 内部子模块
    //   残留状态复位回 IDLE —— 连续帧复用的架构保证。M8.1 已根治三个
    //   具体残留 bug（grid_order_ctrl S_DONE 再武装、candidate_filter
    //   m3_started/ring_finish 清零），软复位继续兜底其余潜伏残留与
    //   fp32 弹性模块的罕见背压死锁（见 M8_REPORT §5）；det_start 时已释放。
    //--------------------------------------------------------------------
    wire det_rst_n = rst_n && !(f_busy || pyr_busy);
    wire        det_gray_rd_en;
    wire [GRAY_ADDR_W-1:0] det_gray_rd_addr;
    wire        det_resp_dump_valid, det_resp_dump_ready, det_resp_dump_done;
    wire [31:0] det_resp_dump_data;

    detect_ctrl #(
        .W0 (W0), .H0 (H0), .DEPTH (DEPTH), .GRAY_ADDR_W (GRAY_ADDR_W),
        .ROWS(ROWS), .COLS(COLS)
    ) u_det (
        .clk (clk), .rst_n (det_rst_n),
        .start (det_start), .busy (det_busy), .done (det_done), .status (det_status),
        .cfg_base0 (cfg_ram_base),
        .gray_rd_en (det_gray_rd_en), .gray_rd_addr (det_gray_rd_addr), .gray_rd_data (gray_rd_data),
        .resp_tap_valid (), .resp_tap_data (),
        .cfg_resp_dump_en (cfg_resp_dump_en),
        .resp_dump_valid (det_resp_dump_valid), .resp_dump_ready (det_resp_dump_ready),
        .resp_dump_data (det_resp_dump_data), .resp_dump_done (det_resp_dump_done),
        .out_valid (out_valid), .out_ready (out_ready),
        .out_x (out_x), .out_y (out_y),
        .out_total (out_total), .out_grid_ok (out_grid_ok)
    );

    //--------------------------------------------------------------------
    // resp_ddr_writer（响应字流 → DDR 写；cfg_words = 最深层像素）
    //--------------------------------------------------------------------
    wire        w_req_valid, w_req_ready;
    wire [31:0] w_req_addr, w_req_len;
    wire [15:0] w_req_tag;
    wire        w_dat_valid, w_dat_ready, w_dat_last;
    wire [31:0] w_dat_data;
    wire [3:0]  w_dat_keep;
    wire        w_done_valid, w_done_ready, w_done_error;
    wire [15:0] w_done_tag;

    resp_ddr_writer #(
        .ADDR_W (32), .LEN_W (32), .TAG_W (16)
    ) u_wresp (
        .clk (clk), .rst_n (rst_n),
        .start (w_start), .busy (w_busy), .done (w_done), .status (w_status),
        .cfg_base (cfg_resp_base), .cfg_words (PIXELS_DEPTH[31:0]),
        .in_valid (det_resp_dump_valid), .in_ready (det_resp_dump_ready), .in_data (det_resp_dump_data),
        .wr_req_valid (w_req_valid), .wr_req_ready (w_req_ready),
        .wr_req_addr (w_req_addr), .wr_req_len (w_req_len), .wr_req_tag (w_req_tag),
        .wr_dat_valid (w_dat_valid), .wr_dat_ready (w_dat_ready),
        .wr_dat_data (w_dat_data), .wr_dat_keep (w_dat_keep), .wr_dat_last (w_dat_last),
        .wr_done_valid (w_done_valid), .wr_done_ready (w_done_ready),
        .wr_done_tag (w_done_tag), .wr_done_error (w_done_error)
    );

    //--------------------------------------------------------------------
    // 片上 gray RAM（读写口阶段互斥 mux + 存储行为）
    //--------------------------------------------------------------------
    // 读口：pyramid/detect 按阶段互斥（子模块 busy 选通，同 tb_detect_ddr active 手法）
    wire pyr_active = pyr_busy;
    wire det_active = det_busy;
    assign gray_rd_en   = pyr_active ? pyr_gray_rd_en : det_active ? det_gray_rd_en : 1'b0;
    assign gray_rd_addr = pyr_active ? pyr_gray_rd_addr : det_gray_rd_addr;

    // 写口：gray_fetch/pyramid 按阶段互斥（gf 只写 L0，pyr 写缩图层）
    assign gray_wr_en   = f_busy ? f_gray_wr_en : pyr_active ? pyr_gray_wr_en : 1'b0;
    assign gray_wr_addr = f_busy ? f_gray_wr_addr : pyr_gray_wr_addr;
    assign gray_wr_data = f_busy ? f_gray_wr_data : pyr_gray_wr_data;

    always @(posedge clk) begin
        if (!rst_n) gray_rd_data <= 8'd0;
        else if (gray_rd_en) gray_rd_data <= gray_mem[gray_rd_addr];
    end
    always @(posedge clk) begin
        if (gray_wr_en) gray_mem[gray_wr_addr] <= gray_wr_data;
    end

    //--------------------------------------------------------------------
    // ddr_port_arbiter #(N_RD=2, N_WR=2)（J 交付，端口契约冻结）
    //--------------------------------------------------------------------
    ddr_port_arbiter #(
        .ADDR_W (32), .LEN_W (32), .TAG_W (16), .N_RD (2), .N_WR (2)
    ) u_arb (
        .clk (clk), .rst_n (rst_n),
        .rd_req_valid (rd_req_valid_a), .rd_req_ready (rd_req_ready_a),
        .rd_req_addr (rd_req_addr_a), .rd_req_len (rd_req_len_a), .rd_req_tag (rd_req_tag_a),
        .rd_ret_valid (rd_ret_valid_a), .rd_ret_ready (rd_ret_ready_a),
        .rd_ret_data (rd_ret_data_a), .rd_ret_keep (rd_ret_keep_a),
        .rd_ret_tag (rd_ret_tag_a), .rd_ret_last (rd_ret_last_a), .rd_ret_error (rd_ret_error_a),
        .wr_req_valid (wr_req_valid_a), .wr_req_ready (wr_req_ready_a),
        .wr_req_addr (wr_req_addr_a), .wr_req_len (wr_req_len_a), .wr_req_tag (wr_req_tag_a),
        .wr_dat_valid (wr_dat_valid_a), .wr_dat_ready (wr_dat_ready_a),
        .wr_dat_data (wr_dat_data_a), .wr_dat_keep (wr_dat_keep_a), .wr_dat_last (wr_dat_last_a),
        .wr_done_valid (wr_done_valid_a), .wr_done_ready (wr_done_ready_a),
        .wr_done_tag (wr_done_tag_a), .wr_done_error (wr_done_error_a),
        .m_rd_req_valid (m_rd_req_valid), .m_rd_req_ready (m_rd_req_ready),
        .m_rd_req_addr (m_rd_req_addr), .m_rd_req_len_bytes (m_rd_req_len_bytes), .m_rd_req_tag (m_rd_req_tag),
        .m_rd_ret_valid (m_rd_ret_valid), .m_rd_ret_ready (m_rd_ret_ready),
        .m_rd_ret_data (m_rd_ret_data), .m_rd_ret_keep (m_rd_ret_keep),
        .m_rd_ret_tag (m_rd_ret_tag), .m_rd_ret_last (m_rd_ret_last), .m_rd_ret_error (m_rd_ret_error),
        .m_wr_req_valid (m_wr_req_valid), .m_wr_req_ready (m_wr_req_ready),
        .m_wr_req_addr (m_wr_req_addr), .m_wr_req_len_bytes (m_wr_req_len_bytes), .m_wr_req_tag (m_wr_req_tag),
        .m_wr_dat_valid (m_wr_dat_valid), .m_wr_dat_ready (m_wr_dat_ready),
        .m_wr_dat_data (m_wr_dat_data), .m_wr_dat_keep (m_wr_dat_keep), .m_wr_dat_last (m_wr_dat_last),
        .m_wr_cplt_valid (m_wr_cplt_valid), .m_wr_cplt_ready (m_wr_cplt_ready),
        .m_wr_cplt_tag (m_wr_cplt_tag), .m_wr_cplt_error (m_wr_cplt_error)
    );

    //--------------------------------------------------------------------
    // 客户端 0：gray_fetch 读 + resp_ddr_writer 写（内部）
    //--------------------------------------------------------------------
    assign rd_req_valid_a[0] = f_rd_req_valid;
    assign f_rd_req_ready    = rd_req_ready_a[0];
    assign rd_req_addr_a[0]  = f_rd_req_addr;
    assign rd_req_len_a[0]   = f_rd_req_len;
    assign rd_req_tag_a[0]   = f_rd_req_tag;
    assign f_rd_ret_valid    = rd_ret_valid_a[0];
    assign rd_ret_ready_a[0] = f_rd_ret_ready;
    assign f_rd_ret_data     = rd_ret_data_a[0];
    assign f_rd_ret_keep     = rd_ret_keep_a[0];
    assign f_rd_ret_tag      = rd_ret_tag_a[0];
    assign f_rd_ret_last     = rd_ret_last_a[0];
    assign f_rd_ret_error    = rd_ret_error_a[0];

    assign wr_req_valid_a[0] = w_req_valid;
    assign w_req_ready       = wr_req_ready_a[0];
    assign wr_req_addr_a[0]  = w_req_addr;
    assign wr_req_len_a[0]   = w_req_len;
    assign wr_req_tag_a[0]   = w_req_tag;
    assign wr_dat_valid_a[0] = w_dat_valid;
    assign w_dat_ready       = wr_dat_ready_a[0];
    assign wr_dat_data_a[0]  = w_dat_data;
    assign wr_dat_keep_a[0]  = w_dat_keep;
    assign wr_dat_last_a[0]  = w_dat_last;
    assign w_done_valid      = wr_done_valid_a[0];
    assign wr_done_ready_a[0]= w_done_ready;
    assign w_done_tag        = wr_done_tag_a[0];
    assign w_done_error      = wr_done_error_a[0];

    //--------------------------------------------------------------------
    // 客户端 1：ext_rd 读 + ext_wr 写（外部）
    //--------------------------------------------------------------------
    assign rd_req_valid_a[1] = ext_rd_req_valid;
    assign ext_rd_req_ready  = rd_req_ready_a[1];
    assign rd_req_addr_a[1]  = ext_rd_req_addr;
    assign rd_req_len_a[1]   = ext_rd_req_len;
    assign rd_req_tag_a[1]   = ext_rd_req_tag;
    assign ext_rd_ret_valid  = rd_ret_valid_a[1];
    assign rd_ret_ready_a[1] = ext_rd_ret_ready;
    assign ext_rd_ret_data   = rd_ret_data_a[1];
    assign ext_rd_ret_keep   = rd_ret_keep_a[1];
    assign ext_rd_ret_tag    = rd_ret_tag_a[1];
    assign ext_rd_ret_last   = rd_ret_last_a[1];
    assign ext_rd_ret_error  = rd_ret_error_a[1];

    assign wr_req_valid_a[1] = ext_wr_req_valid;
    assign ext_wr_req_ready  = wr_req_ready_a[1];
    assign wr_req_addr_a[1]  = ext_wr_req_addr;
    assign wr_req_len_a[1]   = ext_wr_req_len;
    assign wr_req_tag_a[1]   = ext_wr_req_tag;
    assign wr_dat_valid_a[1] = ext_wr_dat_valid;
    assign ext_wr_dat_ready  = wr_dat_ready_a[1];
    assign wr_dat_data_a[1]  = ext_wr_dat_data;
    assign wr_dat_keep_a[1]  = ext_wr_dat_keep;
    assign wr_dat_last_a[1]  = ext_wr_dat_last;
    assign ext_wr_done_valid = wr_done_valid_a[1];
    assign wr_done_ready_a[1]= ext_wr_done_ready;
    assign ext_wr_done_tag   = wr_done_tag_a[1];
    assign ext_wr_done_error = wr_done_error_a[1];

endmodule
