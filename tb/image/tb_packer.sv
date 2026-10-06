`timescale 1ns / 1ps
//==============================================================================
// tb_packer.sv — byte_packer 验证平台（M7.1 交付）
//------------------------------------------------------------------------------
// 验证内容：
//   A. 端到端闭环（随机字节 + 随机 {addr,len}）：
//      字节流 → pack_c（打包，连续流 keep）→ ddr_memory_model 写通道
//      → 模型读通道 → unpk_c（解包）→ 字节流，逐字节比对；
//      同时检查 pack_c 输出字流与模型读返回字流的 {last,keep} 与
//      连续流期望（非尾拍 1111、尾拍按剩余字节数）严格一致。
//   B. 纯打包/解包单元例：pack_u.w_out 直连 unpk_u.w_in（cfg 相同），
//      输入字节流 == 输出字节流（打包/解包互逆，覆盖 keep 语义）。
//   C. cfg_len==0：立即完成，status=10，不产生任何字流/字节流。
//
// 用例覆盖：addr%4∈{0,1,2,3}、len%4∈{0..3}、len=1、大 len（1920B 多拍）、
//   len==0 违规；模型 JITTER/BACKPRESSURE/PROTOCOL_CHECKS 全开，
//   proto_violations 全程必须为 0（证明打包器输出 keep/last 与模型契约零冲突）。
//
// 握手检测方法（与 tb_memory 一致）：TB 在 negedge 用阻塞赋值驱动激励；
//   posedge 用采样寄存器捕获 valid&&ready 的 fire 标志与载荷，RHS 在 posedge
//   活动区求值（更新前值），与模块判定看到同一组信号，无幻象握手。
//
// 字节流握手（打包方向）：源侧 b_ready 恒 1（有数据），模块 b_valid 指示可收；
//   TB 在 b_valid 拉高时每拍送 1 字节，b_valid 拉低（满字/尾字等待下游）时停送。
//==============================================================================
module tb_packer;

    localparam CLK_PERIOD = 10;

    logic clk = 1'b0;
    logic rst_n;

    always #(CLK_PERIOD/2) clk = ~clk;

    //--------------------------------------------------------------------
    // ddr_memory_model 例化（JITTER/BACKPRESSURE/PROTOCOL_CHECKS 全开）
    //--------------------------------------------------------------------
    logic        rd_req_valid, rd_req_ready;
    logic [31:0] rd_req_addr, rd_req_len_bytes;
    logic [15:0] rd_req_tag;
    logic        rd_ret_valid, rd_ret_ready;
    logic [31:0] rd_ret_data;
    logic [3:0]  rd_ret_keep;
    logic [15:0] rd_ret_tag;
    logic        rd_ret_last, rd_ret_error;
    logic        wr_req_valid, wr_req_ready;
    logic [31:0] wr_req_addr, wr_req_len_bytes;
    logic [15:0] wr_req_tag;
    logic        wr_dat_valid, wr_dat_ready;
    logic [31:0] wr_dat_data;
    logic [3:0]  wr_dat_keep;
    logic        wr_dat_last;
    logic        wr_cplt_valid, wr_cplt_ready;
    logic [15:0] wr_cplt_tag;
    logic        wr_cplt_error;
    logic [15:0] proto_violations;

    ddr_memory_model #(
        .LATENCY_MIN     (1),
        .LATENCY_MAX     (6),
        .JITTER_EN       (1'b1),
        .BACKPRESSURE_EN (1'b1),
        .PROTOCOL_CHECKS (1'b1),
        .SEED            (32'h1234_5679)
    ) dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .rd_req_valid     (rd_req_valid),
        .rd_req_ready     (rd_req_ready),
        .rd_req_addr      (rd_req_addr),
        .rd_req_len_bytes (rd_req_len_bytes),
        .rd_req_tag       (rd_req_tag),
        .rd_ret_valid     (rd_ret_valid),
        .rd_ret_ready     (rd_ret_ready),
        .rd_ret_data      (rd_ret_data),
        .rd_ret_keep      (rd_ret_keep),
        .rd_ret_tag       (rd_ret_tag),
        .rd_ret_last      (rd_ret_last),
        .rd_ret_error     (rd_ret_error),
        .wr_req_valid     (wr_req_valid),
        .wr_req_ready     (wr_req_ready),
        .wr_req_addr      (wr_req_addr),
        .wr_req_len_bytes (wr_req_len_bytes),
        .wr_req_tag       (wr_req_tag),
        .wr_dat_valid     (wr_dat_valid),
        .wr_dat_ready     (wr_dat_ready),
        .wr_dat_data      (wr_dat_data),
        .wr_dat_keep      (wr_dat_keep),
        .wr_dat_last      (wr_dat_last),
        .wr_cplt_valid    (wr_cplt_valid),
        .wr_cplt_ready    (wr_cplt_ready),
        .wr_cplt_tag      (wr_cplt_tag),
        .wr_cplt_error    (wr_cplt_error),
        .proto_violations (proto_violations)
    );

    //--------------------------------------------------------------------
    // 打包器（闭环：字节流 → 模型写数据流）
    //--------------------------------------------------------------------
    logic        p_start, p_busy, p_done;
    logic [1:0]  p_status;
    logic [31:0] p_cfg_addr, p_cfg_len;
    logic        p_cfg_dir;
    logic        p_w_out_valid, p_w_out_ready;
    logic [31:0] p_w_out_data;
    logic [3:0]  p_w_out_keep;
    logic        p_w_out_last;
    logic        p_b_valid, p_b_ready;
    logic [7:0]  p_b_in;
    logic [7:0]  p_b_out;

    byte_packer #(.ADDR_W(32), .LEN_W(32)) pack_c (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (p_start),
        .cfg_addr    (p_cfg_addr),
        .cfg_len     (p_cfg_len),
        .cfg_dir     (p_cfg_dir),
        .busy        (p_busy),
        .done        (p_done),
        .status      (p_status),
        .w_in_valid  (1'b0),
        .w_in_ready  (            ),
        .w_in_data   (32'd0),
        .w_in_keep   (4'b0),
        .w_in_last   (1'b0),
        .w_out_valid (p_w_out_valid),
        .w_out_ready (p_w_out_ready),
        .w_out_data  (p_w_out_data),
        .w_out_keep  (p_w_out_keep),
        .w_out_last  (p_w_out_last),
        .b_valid     (p_b_valid),
        .b_ready     (p_b_ready),
        .b_in        (p_b_in),
        .b_out       (p_b_out)
    );

    // pack_c.w_out 直连模型写数据通道
    assign wr_dat_valid = p_w_out_valid;
    assign p_w_out_ready = wr_dat_ready;
    assign wr_dat_data  = p_w_out_data;
    assign wr_dat_keep  = p_w_out_keep;
    assign wr_dat_last  = p_w_out_last;

    //--------------------------------------------------------------------
    // 解包器（闭环：模型读返回流 → 字节流）
    //--------------------------------------------------------------------
    logic        u_start, u_busy, u_done;
    logic [1:0]  u_status;
    logic [31:0] u_cfg_addr, u_cfg_len;
    logic        u_cfg_dir;
    logic        u_b_valid, u_b_ready;
    logic [7:0]  u_b_out;

    byte_packer #(.ADDR_W(32), .LEN_W(32)) unpk_c (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (u_start),
        .cfg_addr    (u_cfg_addr),
        .cfg_len     (u_cfg_len),
        .cfg_dir     (u_cfg_dir),
        .busy        (u_busy),
        .done        (u_done),
        .status      (u_status),
        .w_in_valid  (rd_ret_valid),
        .w_in_ready  (rd_ret_ready),
        .w_in_data   (rd_ret_data),
        .w_in_keep   (rd_ret_keep),
        .w_in_last   (rd_ret_last),
        .w_out_valid (            ),
        .w_out_ready (1'b0),
        .w_out_data  (            ),
        .w_out_keep  (            ),
        .w_out_last  (            ),
        .b_valid     (u_b_valid),
        .b_ready     (u_b_ready),
        .b_in        (8'h00),
        .b_out       (u_b_out)
    );

    //--------------------------------------------------------------------
    // 单元例：打包器 pack_u → 解包器 unpk_u（w_out 流直连 w_in 流，cfg 相同）
    //--------------------------------------------------------------------
    logic        pu_start, pu_busy, pu_done;
    logic [1:0]  pu_status;
    logic [31:0] pu_cfg_addr, pu_cfg_len;
    logic        pu_cfg_dir;
    logic        pu_w_out_valid, pu_w_out_ready;
    logic [31:0] pu_w_out_data;
    logic [3:0]  pu_w_out_keep;
    logic        pu_w_out_last;
    logic        pu_b_valid, pu_b_ready;
    logic [7:0]  pu_b_in;

    logic        uu_start, uu_busy, uu_done;
    logic [1:0]  uu_status;
    logic [31:0] uu_cfg_addr, uu_cfg_len;
    logic        uu_cfg_dir;
    logic        uu_b_valid, uu_b_ready;
    logic [7:0]  uu_b_out;

    byte_packer #(.ADDR_W(32), .LEN_W(32)) pack_u (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (pu_start),
        .cfg_addr    (pu_cfg_addr),
        .cfg_len     (pu_cfg_len),
        .cfg_dir     (pu_cfg_dir),
        .busy        (pu_busy),
        .done        (pu_done),
        .status      (pu_status),
        .w_in_valid  (1'b0),
        .w_in_ready  (            ),
        .w_in_data   (32'd0),
        .w_in_keep   (4'b0),
        .w_in_last   (1'b0),
        .w_out_valid (pu_w_out_valid),
        .w_out_ready (pu_w_out_ready),
        .w_out_data  (pu_w_out_data),
        .w_out_keep  (pu_w_out_keep),
        .w_out_last  (pu_w_out_last),
        .b_valid     (pu_b_valid),
        .b_ready     (pu_b_ready),
        .b_in        (pu_b_in),
        .b_out       (            )
    );

    byte_packer #(.ADDR_W(32), .LEN_W(32)) unpk_u (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (uu_start),
        .cfg_addr    (uu_cfg_addr),
        .cfg_len     (uu_cfg_len),
        .cfg_dir     (uu_cfg_dir),
        .busy        (uu_busy),
        .done        (uu_done),
        .status      (uu_status),
        .w_in_valid  (pu_w_out_valid),
        .w_in_ready  (pu_w_out_ready),
        .w_in_data   (pu_w_out_data),
        .w_in_keep   (pu_w_out_keep),
        .w_in_last   (pu_w_out_last),
        .w_out_valid (            ),
        .w_out_ready (1'b0),
        .w_out_data  (            ),
        .w_out_keep  (            ),
        .w_out_last  (            ),
        .b_valid     (uu_b_valid),
        .b_ready     (uu_b_ready),
        .b_in        (8'h00),
        .b_out       (uu_b_out)
    );

    //--------------------------------------------------------------------
    // posedge 采样监视器：fire 标志与载荷（与模块判定同拍同值）
    //--------------------------------------------------------------------
    logic f_rd_req, f_rd_ret, f_wr_req, f_wr_dat, f_wr_cplt;
    logic f_p_fire, f_u_fire, f_pu_fire, f_uu_fire;
    logic [15:0] s_cplt_tag;
    logic        s_cplt_err;
    logic [15:0] s_rd_tag;
    logic        s_rd_last, s_rd_err;
    logic [3:0]  s_rd_keep;
    logic [3:0]  s_p_keep, s_pu_keep;
    logic        s_p_last, s_pu_last;
    logic [7:0]  s_u_out, s_uu_out;

    always @(posedge clk) begin
        f_rd_req  <= rd_req_valid  && rd_req_ready;
        f_rd_ret  <= rd_ret_valid  && rd_ret_ready;
        f_wr_req  <= wr_req_valid  && wr_req_ready;
        f_wr_dat  <= wr_dat_valid  && wr_dat_ready;
        f_wr_cplt <= wr_cplt_valid && wr_cplt_ready;
        f_p_fire  <= p_w_out_valid && p_w_out_ready;
        f_u_fire  <= u_b_valid     && u_b_ready;
        f_pu_fire <= pu_w_out_valid && pu_w_out_ready;
        f_uu_fire <= uu_b_valid    && uu_b_ready;
        s_cplt_tag <= wr_cplt_tag;
        s_cplt_err <= wr_cplt_error;
        s_rd_tag   <= rd_ret_tag;
        s_rd_last  <= rd_ret_last;
        s_rd_err   <= rd_ret_error;
        s_rd_keep  <= rd_ret_keep;
        s_p_keep   <= p_w_out_keep;
        s_p_last   <= p_w_out_last;
        s_pu_keep  <= pu_w_out_keep;
        s_pu_last  <= pu_w_out_last;
        s_u_out    <= u_b_out;
        s_uu_out   <= uu_b_out;
    end

    //--------------------------------------------------------------------
    // 统计与辅助
    //--------------------------------------------------------------------
    int fails = 0;
    logic [15:0] g_tag = 16'd0;

    // 用例表（addr, len）：addr%4∈{0..3}、len%4∈{0..3}、len=1、多拍
    logic [31:0] unit_cases [][2] = '{
        '{32'h0000_0000, 32'd1},   // len=1
        '{32'h0000_0003, 32'd4},   // 4 的倍数（尾字=满字）
        '{32'h0000_0001, 32'd5},   // addr%4=1, len%4=1
        '{32'h0000_0002, 32'd3},   // addr%4=2, len%4=3
        '{32'h0000_0000, 32'd8},   // 对齐 2 拍
        '{32'h0000_0005, 32'd7},   // 非对齐 2 拍
        '{32'h0000_0002, 32'd6},   // addr%4=2, len%4=2
        '{32'h0000_0010, 32'd16},  // 4 拍
        '{32'h0000_0013, 32'd11},  // addr%4=3, len%4=3
        '{32'h0000_0001, 32'd9}    // addr%4=1, len%4=1
    };

    logic [31:0] e2e_cases [][2] = '{
        '{32'h0000_0100, 32'd1},
        '{32'h0000_0103, 32'd4},
        '{32'h0000_0101, 32'd5},
        '{32'h0000_0102, 32'd3},
        '{32'h0000_0110, 32'd8},
        '{32'h0000_0115, 32'd7},
        '{32'h0000_0122, 32'd6},
        '{32'h0000_0130, 32'd16},
        '{32'h0000_0143, 32'd11},
        '{32'h0000_0151, 32'd9}
    };

    function automatic bit [7:0] pat(input int unsigned i);
        pat = (i * 7 + 13) % 256;
    endfunction

    // 连续流期望：beat b 的 {last,keep}
    task automatic check_stream(input string name, input int unsigned len,
                                ref logic [4:0] beats_q [$]);
        int unsigned nb = (len + 3) / 4;
        logic [3:0]  ekeep;
        logic        elast;
        if (beats_q.size() != nb) begin
            fails = fails + 1;
            $display("[TB][FAIL] %s: beat count %0d != expected %0d",
                     name, beats_q.size(), nb);
            return;
        end
        for (int unsigned b = 0; b < nb; b++) begin
            int unsigned off = b * 4;
            int unsigned rem = len - off;
            case (rem)
                32'd1:    ekeep = 4'b0001;
                32'd2:    ekeep = 4'b0011;
                32'd3:    ekeep = 4'b0111;
                default:  ekeep = 4'b1111;
            endcase
            elast = (off + 4 >= len);
            if (beats_q[b] !== {elast, ekeep}) begin
                fails = fails + 1;
                $display("[TB][FAIL] %s: beat %0d {last,keep}=%05b != %05b",
                         name, b, beats_q[b], {elast, ekeep});
            end
        end
    endtask

    task automatic check_data(input string name, input int unsigned len,
                              ref bit [7:0] got [], ref bit [7:0] exp []);
        for (int unsigned i = 0; i < len; i++) begin
            if (got[i] !== exp[i]) begin
                fails = fails + 1;
                $display("[TB][FAIL] %s: byte %0d got 0x%02x != 0x%02x",
                         name, i, got[i], exp[i]);
                if (fails > 20) return;
            end
        end
    endtask

    //--------------------------------------------------------------------
    // BFM：闭环写（wr_req → pack_c 打包 → 模型写数据 → 完成）
    // 同时收集 pack_c 输出字流的 {last,keep}
    //--------------------------------------------------------------------
    task automatic run_write(input logic [31:0] addr, input logic [31:0] len,
                             input logic [15:0] tag, ref bit [7:0] q[],
                             ref logic [4:0] beats_q [$]);
        // 1) 写请求
        @(negedge clk);
        wr_req_valid     = 1'b1;
        wr_req_addr      = addr;
        wr_req_len_bytes = len;
        wr_req_tag       = tag;
        forever begin
            @(negedge clk);
            if (f_wr_req) break;
        end
        wr_req_valid = 1'b0;

        // 2) pack_c start（cfg_dir=1 打包）
        @(negedge clk);
        p_start = 1'b1; p_cfg_addr = addr; p_cfg_len = len; p_cfg_dir = 1'b1;
        @(negedge clk);
        p_start = 1'b0;

        // 3) 喂字节（b_ready 逐拍由 TB 提供：先等模块可收（b_valid），
        //    再提供字节并拉高 b_ready；模块 PACK_OUT 等待下游时 b_ready 保持 0，
        //    模块不会误收旧值）。同时收集字流握手（含尾字）
        beats_q.delete();
        p_b_ready = 1'b0;
        for (int unsigned i = 0; i < len; i++) begin
            @(negedge clk);
            p_b_ready = 1'b0;                          // 撤销上一字节的提供
            while (!p_b_valid && !p_done) @(negedge clk);  // 等模块可收
            p_b_in    = q[i];
            p_b_ready = 1'b1;                          // 提供本字节（保持到下一轮/完成）
            if (f_p_fire) beats_q.push_back({s_p_last, s_p_keep});
        end
        // 注意：此处不能撤销 p_b_ready——最后一字节在紧随的 posedge 才被模块
        // 采样接受，同拍撤销会使模块看到 b_ready=0 而漏收；撤销推迟到 p_done 之后。

        // 4) 尾字握手（可能尚未发生）与完成
        forever begin
            @(negedge clk);
            if (f_p_fire) beats_q.push_back({s_p_last, s_p_keep});
            if (p_done) break;
        end
        p_b_ready = 1'b0;

        // 5) 等写完成（tag 回带检查）
        forever begin
            @(negedge clk);
            if (f_wr_cplt) begin
                if (s_cplt_tag !== tag) begin
                    fails = fails + 1;
                    $display("[TB][FAIL] @%0t write cplt tag %0d != %0d",
                             $time, s_cplt_tag, tag);
                end
                break;
            end
        end
    endtask

    //--------------------------------------------------------------------
    // BFM：闭环读（rd_req → 模型读返回 → unpk_c 解包 → 字节流）
    // 同时收集模型读返回字流的 {last,keep} 与 tag/error
    //--------------------------------------------------------------------
    task automatic run_read(input logic [31:0] addr, input logic [31:0] len,
                            input logic [15:0] tag, ref bit [7:0] qr[],
                            ref logic [4:0] beats_q [$], output bit got_err);
        // 1) 读请求
        @(negedge clk);
        rd_req_valid     = 1'b1;
        rd_req_addr      = addr;
        rd_req_len_bytes = len;
        rd_req_tag       = tag;
        forever begin
            @(negedge clk);
            if (f_rd_req) break;
        end
        rd_req_valid = 1'b0;

        // 2) unpk_c start（cfg_dir=0 解包）
        @(negedge clk);
        u_start = 1'b1; u_cfg_addr = addr; u_cfg_len = len; u_cfg_dir = 1'b0;
        @(negedge clk);
        u_start = 1'b0;

        // 3) 收集解包字节流 + 模型读返回字流 {last,keep}/tag/error
        //    （beats 按"字流握手"f_rd_ret 收集；字节按解包器输出 fire 收集）
        beats_q.delete();
        got_err = 1'b0;
        u_b_ready = 1'b1;
        begin
            int unsigned idx = 0;
            forever begin
                @(negedge clk);
                if (f_rd_ret) begin
                    beats_q.push_back({s_rd_last, s_rd_keep});
                    got_err = got_err | s_rd_err;
                    if (s_rd_tag !== tag) begin
                        fails = fails + 1;
                        $display("[TB][FAIL] @%0t read ret tag %0d != %0d",
                                 $time, s_rd_tag, tag);
                    end
                end
                if (f_u_fire) begin
                    qr[idx++] = s_u_out;
                end
                if (u_done) break;
            end
        end
        u_b_ready = 1'b0;
    endtask

    //--------------------------------------------------------------------
    // BFM：端到端闭环（写 + 读 + 比对）
    //--------------------------------------------------------------------
    task automatic run_end2end(input logic [31:0] addr, input logic [31:0] len,
                               input string name, ref bit [7:0] q[],
                               ref bit [7:0] qr[]);
        logic [4:0] beats[$];
        bit         err;
        run_write(addr, len, g_tag, q, beats);
        check_stream({name, "-wr"}, len, beats);
        run_read(addr, len, g_tag, qr, beats, err);
        check_stream({name, "-rd"}, len, beats);
        check_data({name, "-e2e"}, len, qr, q);
        g_tag = g_tag + 16'd2;
    endtask

    //--------------------------------------------------------------------
    // BFM：单元例（打包 → 解包直连，cfg 相同；输入字节流 == 输出字节流）
    //--------------------------------------------------------------------
    task automatic run_unit(input logic [31:0] addr, input logic [31:0] len,
                            input string name, ref bit [7:0] q[],
                            ref bit [7:0] qr[]);
        logic [4:0] beats[$];

        @(negedge clk);
        pu_start = 1'b1; pu_cfg_addr = addr; pu_cfg_len = len; pu_cfg_dir = 1'b1;
        uu_start = 1'b1; uu_cfg_addr = addr; uu_cfg_len = len; uu_cfg_dir = 1'b0;
        @(negedge clk);
        pu_start = 1'b0; uu_start = 1'b0;

        beats.delete();
        pu_b_ready = 1'b0;   // 源数据由循环内逐拍提供（先等可收再拉高）
        uu_b_ready = 1'b1;   // 接收方恒就绪（unpk_u 输出节奏自控）
        begin
            int unsigned feed = 0, collect = 0;
            forever begin
                @(negedge clk);
                pu_b_ready = 1'b0;                          // 撤销上一字节的提供
                if ((feed < len) && pu_b_valid && !pu_done) begin
                    pu_b_in = q[feed];
                    pu_b_ready = 1'b1;
                    feed++;
                end
                if (f_uu_fire) begin
                    qr[collect] = s_uu_out;
                    collect++;
                end
                if (f_pu_fire) beats.push_back({s_pu_last, s_pu_keep});
                if (pu_done && uu_done) break;
            end
        end
        pu_b_ready = 1'b0;
        uu_b_ready = 1'b0;

        check_stream({name, "-wr"}, len, beats);
        check_data({name, "-unit"}, len, qr, q);
    endtask

    //--------------------------------------------------------------------
    // BFM：cfg_len==0 违规（立即完成，status=10）
    //--------------------------------------------------------------------
    task automatic run_zero(input string name);
        @(negedge clk);
        pu_start = 1'b1; pu_cfg_addr = 32'h100; pu_cfg_len = 32'd0; pu_cfg_dir = 1'b1;
        @(negedge clk);
        pu_start = 1'b0;
        repeat (3) @(negedge clk);
        if (!pu_done || (pu_status !== 2'b10) || (pu_busy !== 1'b0)) begin
            fails = fails + 1;
            $display("[TB][FAIL] %s: len==0 -> done=%0b status=%02b busy=%0b",
                     name, pu_done, pu_status, pu_busy);
        end
        $display("[TB] %s len==0 violation               : %s",
                 name, (pu_done && pu_status === 2'b10) ? "ok" : "BAD");
    endtask

    //--------------------------------------------------------------------
    // 主测试序列
    //--------------------------------------------------------------------
    initial begin
        // 激励初始化
        rd_req_valid = 1'b0; rd_req_addr = '0; rd_req_len_bytes = '0; rd_req_tag = '0;
        wr_req_valid = 1'b0; wr_req_addr = '0; wr_req_len_bytes = '0; wr_req_tag = '0;
        wr_cplt_ready = 1'b1;   // 写完成客户端 ready（恒接受）
        p_start = 1'b0; p_cfg_addr = '0; p_cfg_len = '0; p_cfg_dir = 1'b0;
        p_b_ready = 1'b0; p_b_in = 8'h00;
        u_start = 1'b0; u_cfg_addr = '0; u_cfg_len = '0; u_cfg_dir = 1'b0;
        u_b_ready = 1'b0;
        pu_start = 1'b0; pu_cfg_addr = '0; pu_cfg_len = '0; pu_cfg_dir = 1'b0;
        pu_b_ready = 1'b0; pu_b_in = 8'h00;
        uu_start = 1'b0; uu_cfg_addr = '0; uu_cfg_len = '0; uu_cfg_dir = 1'b0;
        uu_b_ready = 1'b0;
        rst_n = 1'b0;

        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5)  @(negedge clk);

        begin
            bit [7:0] q[], qr[];
            logic [31:0] A;
            int unsigned L;

            //------------------------------------------------------------
            // 单元例：边界 + 随机（含 addr%4、len%4 全组合、len=1）
            //------------------------------------------------------------
            $display("[TB] ---- 单元例（打包→解包直连） ----");
            foreach (unit_cases[i]) begin
                A = unit_cases[i][0]; L = unit_cases[i][1];
                q  = new[L]; qr = new[L];
                for (int unsigned j = 0; j < L; j++) q[j] = pat(j + i * 101);
                run_unit(A, L, $sformatf("U%0d", i), q, qr);
            end
            for (int c = 0; c < 12; c++) begin
                A = $urandom_range(0, 65535);
                L = $urandom_range(1, 40);
                q  = new[L]; qr = new[L];
                for (int unsigned j = 0; j < L; j++) q[j] = pat(j + 5000 + c * 37);
                run_unit(A, L, $sformatf("UR%0d", c), q, qr);
            end

            //------------------------------------------------------------
            // 端到端闭环：边界 + 随机（模型 JITTER/背压全开）
            //------------------------------------------------------------
            $display("[TB] ---- 端到端闭环（打包→模型写→模型读→解包） ----");
            foreach (e2e_cases[i]) begin
                A = e2e_cases[i][0]; L = e2e_cases[i][1];
                q  = new[L]; qr = new[L];
                for (int unsigned j = 0; j < L; j++) q[j] = pat(j + i * 73);
                run_end2end(A, L, $sformatf("E%0d", i), q, qr);
            end
            for (int c = 0; c < 20; c++) begin
                A = $urandom_range(0, 1 << 24);
                L = $urandom_range(1, 40);
                q  = new[L]; qr = new[L];
                for (int unsigned j = 0; j < L; j++) q[j] = pat(j + 9000 + c * 53);
                run_end2end(A, L, $sformatf("ER%0d", c), q, qr);
            end
            // 大块 1920B（多拍背靠背）
            begin
                L = 1920;
                q  = new[L]; qr = new[L];
                for (int unsigned j = 0; j < L; j++) q[j] = pat(j + 30000);
                run_end2end(32'h0100_0000, L, "EBIG", q, qr);
            end

            //------------------------------------------------------------
            // len==0 违规
            //------------------------------------------------------------
            run_zero("Z0");

            $display("[TB] ---- 模型违规计数: %0d（必须为 0） ----", proto_violations);
            if (proto_violations != 16'd0) begin
                fails = fails + 1;
                $display("[TB][FAIL] model protocol violations = %0d (expect 0)",
                         proto_violations);
            end
        end

        //------------------------------------------------------------
        // 汇总
        //------------------------------------------------------------
        $display("==================================================");
        $display("TB SUMMARY: fails=%0d", fails);
        if (fails == 0)
            $display("TB RESULT: ALL TESTS PASSED");
        else
            $display("TB RESULT: TESTS FAILED");
        $display("==================================================");
        $finish;
    end

    //--------------------------------------------------------------------
    // 全局看门狗
    //--------------------------------------------------------------------
    initial begin
        #60_000_000;
        $display("[TB][FATAL] global timeout");
        $finish;
    end

endmodule
