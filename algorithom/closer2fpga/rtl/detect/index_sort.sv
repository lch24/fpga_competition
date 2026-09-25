`timescale 1ns / 1ps
//==============================================================================
// index_sort.sv — M4 通用排序核（双 RAM 迭代归并：key + 原索引）
//------------------------------------------------------------------------------
// 语义（与 export_m4.cpp::export_sort_cases 位级一致，不得改动）：
//   全序比较 less(a,b) = (key_a < key_b) || (key_a==key_b && idx_a<idx_b)
//   key 为 fp32 位模式，比较采用位模式无符号序（与 C++ 参考
//   std::vector<uint32_t> 的 operator< 一致）；相等 key 用位模式相等，
//   平局按原索引升序（归并稳定）。输出：排序后的原索引序列（0..n-1）。
//
// 结构：
//   LOAD   ：从外部 key/idx RAM（1 拍延迟读）依次读 n 个 {key,idx} 写内部 RAM A。
//   MERGE  ：迭代归并。run_len 从 1 倍增，每轮从源 RAM 读两有序段（各 ≤
//            run_len），比较写目标 RAM，轮末交换源/目标（src_sel 翻转）。
//            n 不必为 2 的幂：段长 l0/l1 = min(run_len, 剩余)，段对末尾裁剪；
//            最后可能只剩单段（直接拷贝，本身有序）。
//   OUT    ：顺序读最终源 RAM 输出 idx 流；out_ready 反驱（背压暂停）。
//
// 时序（关键）：读使能 rd_en/rd_src_en 为 reg（posedge 生效），RAM 为 1 拍
//   同步读 → 数据在"请求发出后第 2 拍"组合可见。故所有数据吸收/写入都走
//   两级流水（请求 → 转移 → 吸收），避免拿到上一拍的旧值。
// 归并流水：每拍至多一个读请求在途；取走元素后该段 cur 立即失效（v=0），
//   新值经两级 pend 到达后置 v=1，吞吐 1 元素/3 拍，无同拍双写冲突。
//==============================================================================
module index_sort #(
    parameter N_ADDR_W = 7,
    parameter DEPTH    = 1 << N_ADDR_W
) (
    input  wire                clk,
    input  wire                rst_n,
    input  wire                start,
    output reg                 busy,
    output reg                 done,
    input  wire [15:0]         n_in,
    // 外部 key/idx RAM 读口（1 拍延迟：rd_en=1 下一拍 rd_key/rd_idx 有效）
    output reg                 rd_en,
    output reg  [N_ADDR_W-1:0] rd_addr,
    input  wire [31:0]         rd_key,
    input  wire [N_ADDR_W-1:0] rd_idx,
    // 排序后索引流
    output reg                 out_valid,
    input  wire                out_ready,
    output reg  [N_ADDR_W-1:0] out_idx
);

    //--------------------------------------------------------------------
    // 状态
    //--------------------------------------------------------------------
    localparam S_IDLE  = 2'd0;
    localparam S_LOAD  = 2'd1;
    localparam S_MERGE = 2'd2;
    localparam S_OUT   = 2'd3;
    reg [1:0] state;

    //--------------------------------------------------------------------
    // 内部双 RAM（各存 {key[31:0], idx[15:0]} = 48 bit）
    //--------------------------------------------------------------------
    wire [47:0] ra_data, rb_data;
    // 写/读口 net（先声明，避免例化时隐式 net 声明与后续冲突）
    wire                 wr_a_en;
    wire [N_ADDR_W-1:0]  wr_a_addr;
    wire [47:0]          wr_a_data;
    wire                 rd_a_en;
    wire [N_ADDR_W-1:0]  rd_a_addr;
    wire                 wr_b_en_ram;
    wire [N_ADDR_W-1:0]  wr_b_addr_ram;
    wire [47:0]          wr_b_data_ram;
    wire                 rd_b_en;
    wire [N_ADDR_W-1:0]  rd_b_addr;

    dual_port_ram #(.DATA_WIDTH(48), .ADDR_WIDTH(N_ADDR_W)) u_ram_a (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_en   (wr_a_en),
        .wr_addr (wr_a_addr),
        .wr_data (wr_a_data),
        .rd_en   (rd_a_en),
        .rd_addr (rd_a_addr),
        .rd_data (ra_data)
    );

    dual_port_ram #(.DATA_WIDTH(48), .ADDR_WIDTH(N_ADDR_W)) u_ram_b (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_en   (wr_b_en_ram),
        .wr_addr (wr_b_addr_ram),
        .wr_data (wr_b_data_ram),
        .rd_en   (rd_b_en),
        .rd_addr (rd_b_addr),
        .rd_data (rb_data)
    );

    //--------------------------------------------------------------------
    // 控制寄存器
    //--------------------------------------------------------------------
    reg [N_ADDR_W:0] n_reg;        // 元素数
    reg              src_sel;      // 0=源 A（目标 B），1=源 B（目标 A）
    // LOAD（写使能两级延迟：请求→转移→写入，保证 rd_key 为"新值"）
    reg [N_ADDR_W:0] ld_cnt;
    reg              ld_wr_en1, ld_wr_en2;
    reg [N_ADDR_W-1:0] ld_wr_addr1, ld_wr_addr2;
    // MERGE
    reg [N_ADDR_W:0] run_len;
    reg [N_ADDR_W:0] seg0, p0, p1;
    reg [N_ADDR_W:0] l0, l1;
    reg [N_ADDR_W:0] out_pos;
    reg [1:0]        seg_step;     // 0=段对初始化(发段0首) 1=发段1首 2=流水
    reg [47:0]       cur0, cur1;
    reg              v0, v1;
    reg              pend_valid, pend_valid2;   // 请求在途 L1 / 数据在途 L2
    reg              pend_target, pend_target2;
    reg              rd_src_en;
    reg [N_ADDR_W:0] rd_src_addr;
    // OUT（请求两级流水 + 吸收门控）
    reg [N_ADDR_W:0] o_cnt;
    reg              req1, req2;
    reg              out_valid_r;
    reg [N_ADDR_W-1:0] out_idx_r;

    //--------------------------------------------------------------------
    // 组合辅助
    //--------------------------------------------------------------------
    wire seg_init  = (state == S_MERGE) && (seg_step == 2'd0);
    wire seg_prep1 = (state == S_MERGE) && (seg_step == 2'd1);
    wire in_seg    = (state == S_MERGE) && (seg_step == 2'd2);

    // 段长（末尾裁剪）
    wire [N_ADDR_W:0] l0_c = (n_reg - seg0 < run_len) ? (n_reg - seg0) : run_len;
    wire [N_ADDR_W:0] l1_c = (seg0 + run_len < n_reg) ?
        ((n_reg - seg0 - run_len < run_len) ? (n_reg - seg0 - run_len) : run_len) :
        {N_ADDR_W+1{1'b0}};

    // 取元素判定：段1 优先当 less(cur1,cur0)
    // 比较语义：key 为位模式无符号比较（与 export_m4.cpp::export_sort_cases
    //   参考 std::vector<uint32_t> 的 operator< 位级一致，对拍 9/9 全过）。
    //   organize_grid 真实 u/v 投影若为负且需 float 符号语义，由调用侧
    //   预处理 key（见汇报）。
    reg take1_comb;
    always_comb begin
        if (v0 && v1)
            take1_comb = (cur1[47:16] < cur0[47:16]) ||
                         ((cur1[47:16] == cur0[47:16]) && (cur1[15:0] < cur0[15:0]));
        else
            take1_comb = 1'b0;
    end
    // 段已耗尽标记（用于单边取：另一段在途时不能取，避免次序错误）
    wire seg0_empty = (p0 >= l0);
    wire seg1_empty = (l1 == 0) || (p1 >= l1);

    wire take1 = (v0 && v1) ? take1_comb :
                 (!v0 && v1 && seg0_empty) ? 1'b1 : 1'b0;
    wire take0 = (v0 && v1) ? !take1_comb :
                 (v0 && !v1 && seg1_empty) ? 1'b1 : 1'b0;
    wire any_take = take0 || take1;

    // 段对完成 / 读请求
    wire seg_done = any_take && (out_pos + 1 == seg0 + l0 + l1);
    wire taken_more0 = take0 && (p0 + 1 < l0);
    wire taken_more1 = take1 && (p1 + 1 < l1);
    wire issue = seg_init || (seg_prep1 && (l1 > 0)) || taken_more0 || taken_more1;
    wire [N_ADDR_W:0] issue_addr =
        seg_init  ? seg0 :
        seg_prep1 ? (seg0 + run_len) :
        take0     ? (seg0 + p0 + 1) :
                    (seg0 + run_len + p1 + 1);
    wire issue_target = (seg_init || take0) ? 1'b0 : 1'b1;

    // 目标写口组合信号（当拍生效：take 拍即写 RAM，不延迟）
    wire wr_b_en_c  = in_seg && any_take;
    wire [N_ADDR_W-1:0] wr_b_addr_c = out_pos[N_ADDR_W-1:0];
    wire [47:0]      wr_b_data_c = take0 ? cur0 : cur1;

    // 源 RAM 读数据（交换后的源）
    wire [47:0] rd_src_data = src_sel ? rb_data : ra_data;

    // 目标 RAM 写口（LOAD 只写 A；MERGE 写 ~src_sel）
    // 注意：MERGE 写口为组合驱动（当拍即写），避免 reg 延迟导致
    // 段对完成拍（同拍轮结束、src_sel 翻转）的最后一笔写入丢失。
    assign wr_a_en = (state == S_LOAD) ? ld_wr_en2 :
                     (src_sel == 1'b1) ? wr_b_en_c : 1'b0;
    assign wr_a_addr = (state == S_LOAD) ? ld_wr_addr2 : wr_b_addr_c;
    assign wr_a_data = (state == S_LOAD) ?
        {rd_key, {{16-N_ADDR_W{1'b0}}, rd_idx}} : wr_b_data_c;
    assign wr_b_en_ram = (src_sel == 1'b0) ? wr_b_en_c : 1'b0;
    assign wr_b_addr_ram = wr_b_addr_c;
    assign wr_b_data_ram = wr_b_data_c;

    assign rd_a_en = (src_sel == 1'b0) ? rd_src_en : 1'b0;
    assign rd_a_addr = rd_src_addr[N_ADDR_W-1:0];
    assign rd_b_en = (src_sel == 1'b1) ? rd_src_en : 1'b0;
    assign rd_b_addr = rd_src_addr[N_ADDR_W-1:0];

    // OUT 推进条件（含背压：输出口被占且未接收时停）
    wire can_issue = (state == S_OUT) && !req1 && !req2 &&
                     (out_valid_r == 1'b0 || out_ready) && (o_cnt < n_reg);

    //--------------------------------------------------------------------
    // 主状态机
    //--------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            busy         <= 1'b0;
            done         <= 1'b0;
            rd_en        <= 1'b0;
            rd_addr      <= {N_ADDR_W{1'b0}};
            ld_cnt       <= {N_ADDR_W+1{1'b0}};
            ld_wr_en1    <= 1'b0;
            ld_wr_en2    <= 1'b0;
            ld_wr_addr1  <= {N_ADDR_W{1'b0}};
            ld_wr_addr2  <= {N_ADDR_W{1'b0}};
            run_len      <= {N_ADDR_W+1{1'b0}};
            seg0         <= {N_ADDR_W+1{1'b0}};
            p0           <= {N_ADDR_W+1{1'b0}};
            p1           <= {N_ADDR_W+1{1'b0}};
            l0           <= {N_ADDR_W+1{1'b0}};
            l1           <= {N_ADDR_W+1{1'b0}};
            out_pos      <= {N_ADDR_W+1{1'b0}};
            seg_step     <= 2'd0;
            cur0         <= 48'd0;
            cur1         <= 48'd0;
            v0           <= 1'b0;
            v1           <= 1'b0;
            pend_valid   <= 1'b0;
            pend_valid2  <= 1'b0;
            pend_target  <= 1'b0;
            pend_target2 <= 1'b0;
            rd_src_en    <= 1'b0;
            rd_src_addr  <= {N_ADDR_W+1{1'b0}};
            src_sel      <= 1'b0;
            o_cnt        <= {N_ADDR_W+1{1'b0}};
            req1         <= 1'b0;
            req2         <= 1'b0;
            out_valid_r  <= 1'b0;
            out_idx_r    <= {N_ADDR_W{1'b0}};
        end else begin
            case (state)
            //==============================================================
            S_IDLE: begin
                done        <= 1'b0;
                out_valid_r <= 1'b0;
                if (start) begin
                    n_reg      <= n_in[N_ADDR_W:0];
                    busy       <= 1'b1;
                    src_sel    <= 1'b0;
                    ld_cnt     <= {N_ADDR_W+1{1'b0}};
                    ld_wr_en1  <= 1'b0;
                    ld_wr_en2  <= 1'b0;
                    if (n_in[N_ADDR_W:0] == {N_ADDR_W+1{1'b0}}) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                    end else begin
                        state <= S_LOAD;
                    end
                end
            end
            //==============================================================
            S_LOAD: begin
                // 请求（t 末生效）
                if (ld_cnt < n_reg) begin
                    rd_en        <= 1'b1;
                    rd_addr      <= ld_cnt[N_ADDR_W-1:0];
                    ld_wr_en1    <= 1'b1;
                    ld_wr_addr1  <= ld_cnt[N_ADDR_W-1:0];
                    ld_cnt       <= ld_cnt + 1;
                end else begin
                    rd_en     <= 1'b0;
                    ld_wr_en1 <= 1'b0;
                    // 最后一级写（ld_wr_en2 生效拍）已完成 → 转移
                    if (!ld_wr_en2) begin
                        if (n_reg <= 1) begin
                            state       <= S_OUT;
                            o_cnt       <= {N_ADDR_W+1{1'b0}};
                            req1        <= 1'b0;
                            req2        <= 1'b0;
                            out_valid_r <= 1'b0;
                        end else begin
                            state    <= S_MERGE;
                            run_len  <= {{N_ADDR_W{1'b0}}, 1'b1};
                            seg0     <= {N_ADDR_W+1{1'b0}};
                            seg_step <= 2'd0;
                        end
                    end
                end
                // 两级转移（写使能延迟到 rd_key 更新后）
                ld_wr_en2   <= ld_wr_en1;
                ld_wr_addr2 <= ld_wr_addr1;
            end
            //==============================================================
            S_MERGE: begin
                // ---- 1) 段对 / 轮推进 ----
                if (seg_init) begin
                    seg_step <= 2'd1;
                end else if (seg_prep1) begin
                    seg_step <= 2'd2;
                end else if (in_seg && seg_done) begin
                    seg_step <= 2'd0;
                    if (seg0 + (run_len << 1) >= n_reg) begin
                        // 本轮结束：交换 RAM，run_len 倍增
                        run_len <= run_len << 1;
                        src_sel <= ~src_sel;
                        seg0    <= {N_ADDR_W+1{1'b0}};
                        if ((run_len << 1) >= n_reg) begin
                            state <= S_OUT;
                            o_cnt <= {N_ADDR_W+1{1'b0}};
                            req1  <= 1'b0;
                            req2  <= 1'b0;
                            out_valid_r <= 1'b0;
                        end
                    end else begin
                        seg0 <= seg0 + (run_len << 1);
                    end
                end
                // ---- 2) 段对初始化 ----
                if (seg_init) begin
                    p0      <= {N_ADDR_W+1{1'b0}};
                    p1      <= {N_ADDR_W+1{1'b0}};
                    l0      <= l0_c;
                    l1      <= l1_c;
                    out_pos <= seg0;
                    v0      <= 1'b0;
                    v1      <= 1'b0;
                end
                // ---- 3) 取元素写目标 RAM（写口为组合，此处仅推进指针） ----
                if (in_seg && any_take) begin
                    out_pos   <= out_pos + 1;
                    // 取走后立即失效该段候选，新值经 pend2 到达后置位
                    if (take0) begin
                        p0 <= p0 + 1;
                        v0 <= 1'b0;
                    end else begin
                        p1 <= p1 + 1;
                        v1 <= 1'b0;
                    end
                end
                // ---- 4) 源 RAM 读请求 ----
                if (issue) begin
                    rd_src_en   <= 1'b1;
                    rd_src_addr <= issue_addr;
                end else begin
                    rd_src_en <= 1'b0;
                end
                // ---- 5) pend2 数据到达（吸收，用 pend_valid2 当拍值） ----
                if (pend_valid2) begin
                    if (pend_target2 == 1'b0) begin
                        cur0 <= rd_src_data;
                        v0   <= 1'b1;
                    end else begin
                        cur1 <= rd_src_data;
                        v1   <= 1'b1;
                    end
                end
                // ---- 6) 流水转移 + 请求在途标志 ----
                pend_valid2  <= pend_valid;
                pend_target2 <= pend_target;
                if (issue) begin
                    pend_valid  <= 1'b1;
                    pend_target <= issue_target;
                end else begin
                    pend_valid <= 1'b0;
                end
            end
            //==============================================================
            S_OUT: begin
                // 接收（含最后一个元素 → done）
                if (out_valid_r && out_ready) begin
                    out_valid_r <= 1'b0;
                    if (o_cnt >= n_reg) begin
                        done  <= 1'b1;
                        busy  <= 1'b0;
                        state <= S_IDLE;
                    end
                end
                // 数据到达（req2 当拍值），输出口可用时吸收
                if (req2 && (out_valid_r == 1'b0 || out_ready)) begin
                    out_valid_r <= 1'b1;
                    out_idx_r   <= rd_src_data[N_ADDR_W-1:0];
                end
                // 请求 + 流水转移
                if (can_issue) begin
                    rd_src_en   <= 1'b1;
                    rd_src_addr <= o_cnt;
                    o_cnt       <= o_cnt + 1;
                    req1        <= 1'b1;
                end else begin
                    rd_src_en <= 1'b0;
                    req1      <= 1'b0;
                end
                req2 <= req1;
            end
            //==============================================================
            default: state <= S_IDLE;
            endcase
        end
    end

    assign out_valid = out_valid_r;
    assign out_idx   = out_idx_r;

endmodule
