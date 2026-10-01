`timescale 1ns / 1ps
//==============================================================================
// tb_frame_b5x2.sv — M8.1 快速连续帧回归：同一 top 实例连续两帧（grid_order S_DONE 根治验证）
//------------------------------------------------------------------------------
// 目的：M8 的 tb_frame_top 大场景耗时 55 分钟，迭代慢。本 TB 用 board5x8
//   （DEPTH=1，native 26112 像素）在同一 corner_detect_ddr_top 实例上连跑两帧，
//   快速验证"连续帧复用"不再死锁（grid_order_ctrl S_DONE 再武装根治，top 已移除
//   帧级软复位规避）。cfg_resp_dump_en=0（不写响应图，聚焦连续帧）。
// 流程：ext_wr（raster_dma 写）预载 m5_board5x8_gray.bin @0x200000 →
//   process 帧1（dump 关）→ 40 点 vs m6_chain_board5x8.bin → process 帧2（同 cfg）
//   → 40 点 vs 同一权威。判据：两帧 done/40 点 + proto_violations=0。
//==============================================================================
module tb_frame_b5x2;

    localparam integer CLK_PERIOD = 10;
    logic clk = 1'b0;
    logic rst_n;
    always #(CLK_PERIOD/2) clk = ~clk;

    integer fd, code;
    reg [7:0]  fbuf [0:1048575];

    //--------------------------------------------------------------------
    // DDR 模型
    //--------------------------------------------------------------------
    logic        m_rd_req_valid, m_rd_req_ready;
    logic [31:0] m_rd_req_addr,  m_rd_req_len;
    logic [15:0] m_rd_req_tag;
    logic        m_rd_ret_valid, m_rd_ret_ready;
    logic [31:0] m_rd_ret_data;
    logic [3:0]  m_rd_ret_keep;
    logic [15:0] m_rd_ret_tag;
    logic        m_rd_ret_last,  m_rd_ret_error;
    logic        m_wr_req_valid, m_wr_req_ready;
    logic [31:0] m_wr_req_addr,  m_wr_req_len;
    logic [15:0] m_wr_req_tag;
    logic        m_wr_dat_valid, m_wr_dat_ready;
    logic [31:0] m_wr_dat_data;
    logic [3:0]  m_wr_dat_keep;
    logic        m_wr_dat_last;
    logic        m_wr_done_valid, m_wr_done_ready;
    logic [15:0] m_wr_done_tag;
    logic        m_wr_done_error;
    logic [15:0] m_proto_viol;

    ddr_memory_model #(
        .LATENCY_MIN (1), .LATENCY_MAX (6),
        .JITTER_EN (1'b1), .BACKPRESSURE_EN (1'b1),
        .PROTOCOL_CHECKS (1'b1), .SEED (32'h2468_ACE0)
    ) u_ddr (
        .clk (clk), .rst_n (rst_n),
        .rd_req_valid (m_rd_req_valid), .rd_req_ready (m_rd_req_ready),
        .rd_req_addr (m_rd_req_addr), .rd_req_len_bytes (m_rd_req_len), .rd_req_tag (m_rd_req_tag),
        .rd_ret_valid (m_rd_ret_valid), .rd_ret_ready (m_rd_ret_ready),
        .rd_ret_data (m_rd_ret_data), .rd_ret_keep (m_rd_ret_keep),
        .rd_ret_tag (m_rd_ret_tag), .rd_ret_last (m_rd_ret_last), .rd_ret_error (m_rd_ret_error),
        .wr_req_valid (m_wr_req_valid), .wr_req_ready (m_wr_req_ready),
        .wr_req_addr (m_wr_req_addr), .wr_req_len_bytes (m_wr_req_len), .wr_req_tag (m_wr_req_tag),
        .wr_dat_valid (m_wr_dat_valid), .wr_dat_ready (m_wr_dat_ready),
        .wr_dat_data (m_wr_dat_data), .wr_dat_keep (m_wr_dat_keep), .wr_dat_last (m_wr_dat_last),
        .wr_cplt_valid (m_wr_done_valid), .wr_cplt_ready (m_wr_done_ready),
        .wr_cplt_tag (m_wr_done_tag), .wr_cplt_error (m_wr_done_error),
        .proto_violations (m_proto_viol)
    );

    //--------------------------------------------------------------------
    // u_top：corner_detect_ddr_top（272x96 DEPTH=1）
    //--------------------------------------------------------------------
    logic        top_process;
    logic [31:0] top_cfg_gray_base, top_cfg_gray_stride, top_cfg_resp_base;
    logic [15:0] top_cfg_gray_w, top_cfg_gray_h;
    logic [20:0] top_cfg_ram_base;
    logic        top_cfg_resp_dump_en, top_cfg_pyr_en;
    logic        top_busy, top_done;
    logic [1:0]  top_status;
    logic        top_out_valid, top_out_ready, top_out_grid_ok;
    logic [31:0] top_out_x, top_out_y;
    logic [15:0] top_out_total;

    logic        ext_wr_req_valid, ext_wr_req_ready;
    logic [31:0] ext_wr_req_addr,  ext_wr_req_len;
    logic [15:0] ext_wr_req_tag;
    logic        ext_wr_dat_valid, ext_wr_dat_ready;
    logic [31:0] ext_wr_dat_data;
    logic [3:0]  ext_wr_dat_keep;
    logic        ext_wr_dat_last;
    logic        ext_wr_done_valid, ext_wr_done_ready;
    logic [15:0] ext_wr_done_tag;
    logic        ext_wr_done_error;

    corner_detect_ddr_top #(
        .W0 (272), .H0 (96), .DEPTH (1), .GRAY_ADDR_W (21),
        .MAX_W (272), .MAX_H (96), .MAX_DEPTH (4)
    ) u_top (
        .clk (clk), .rst_n (rst_n),
        .process_frame (top_process), .busy (top_busy), .done (top_done), .status (top_status),
        .cfg_gray_base (top_cfg_gray_base), .cfg_gray_stride (top_cfg_gray_stride),
        .cfg_gray_w (top_cfg_gray_w), .cfg_gray_h (top_cfg_gray_h),
        .cfg_ram_base (top_cfg_ram_base),
        .cfg_resp_base (top_cfg_resp_base), .cfg_resp_dump_en (top_cfg_resp_dump_en),
        .cfg_pyr_en (top_cfg_pyr_en),
        .out_valid (top_out_valid), .out_ready (top_out_ready),
        .out_x (top_out_x), .out_y (top_out_y),
        .out_total (top_out_total), .out_grid_ok (top_out_grid_ok),
        .m_rd_req_valid (m_rd_req_valid), .m_rd_req_ready (m_rd_req_ready),
        .m_rd_req_addr (m_rd_req_addr), .m_rd_req_len_bytes (m_rd_req_len), .m_rd_req_tag (m_rd_req_tag),
        .m_rd_ret_valid (m_rd_ret_valid), .m_rd_ret_ready (m_rd_ret_ready),
        .m_rd_ret_data (m_rd_ret_data), .m_rd_ret_keep (m_rd_ret_keep),
        .m_rd_ret_tag (m_rd_ret_tag), .m_rd_ret_last (m_rd_ret_last), .m_rd_ret_error (m_rd_ret_error),
        .m_wr_req_valid (m_wr_req_valid), .m_wr_req_ready (m_wr_req_ready),
        .m_wr_req_addr (m_wr_req_addr), .m_wr_req_len_bytes (m_wr_req_len), .m_wr_req_tag (m_wr_req_tag),
        .m_wr_dat_valid (m_wr_dat_valid), .m_wr_dat_ready (m_wr_dat_ready),
        .m_wr_dat_data (m_wr_dat_data), .m_wr_dat_keep (m_wr_dat_keep), .m_wr_dat_last (m_wr_dat_last),
        .m_wr_cplt_valid (m_wr_done_valid), .m_wr_cplt_ready (m_wr_done_ready),
        .m_wr_cplt_tag (m_wr_done_tag), .m_wr_cplt_error (m_wr_done_error),
        .ext_rd_req_valid (1'b0), .ext_rd_req_ready (), .ext_rd_req_addr (32'd0),
        .ext_rd_req_len (32'd0), .ext_rd_req_tag (16'd0),
        .ext_rd_ret_valid (), .ext_rd_ret_ready (1'b0), .ext_rd_ret_data (), .ext_rd_ret_keep (),
        .ext_rd_ret_tag (), .ext_rd_ret_last (), .ext_rd_ret_error (),
        .ext_wr_req_valid (ext_wr_req_valid), .ext_wr_req_ready (ext_wr_req_ready),
        .ext_wr_req_addr (ext_wr_req_addr), .ext_wr_req_len (ext_wr_req_len), .ext_wr_req_tag (ext_wr_req_tag),
        .ext_wr_dat_valid (ext_wr_dat_valid), .ext_wr_dat_ready (ext_wr_dat_ready),
        .ext_wr_dat_data (ext_wr_dat_data), .ext_wr_dat_keep (ext_wr_dat_keep), .ext_wr_dat_last (ext_wr_dat_last),
        .ext_wr_done_valid (ext_wr_done_valid), .ext_wr_done_ready (ext_wr_done_ready),
        .ext_wr_done_tag (ext_wr_done_tag), .ext_wr_done_error (ext_wr_done_error)
    );

    //--------------------------------------------------------------------
    // ext_wr 预载（raster_dma 写方向，经 top 外部写客户端槽 1）
    //--------------------------------------------------------------------
    logic        pre_start, pre_busy, pre_done;
    logic [1:0]  pre_status;
    logic        pre_in_valid, pre_in_ready;
    logic [7:0]  pre_in_byte;
    reg  [31:0]  pre_cfg_base, pre_cfg_stride, pre_cfg_offset;
    reg  [15:0]  pre_cfg_row_bytes, pre_cfg_rows;

    raster_dma u_pre (
        .clk (clk), .rst_n (rst_n),
        .start (pre_start), .busy (pre_busy), .done (pre_done), .status (pre_status),
        .cfg_base (pre_cfg_base), .cfg_stride (pre_cfg_stride),
        .cfg_row_bytes (pre_cfg_row_bytes), .cfg_rows (pre_cfg_rows),
        .cfg_offset (pre_cfg_offset), .cfg_dir (1'b1),
        .out_valid (), .out_ready (1'b1), .out_byte (),
        .in_valid (pre_in_valid), .in_ready (pre_in_ready), .in_byte (pre_in_byte),
        .rd_req_valid (), .rd_req_ready (1'b0), .rd_req_addr (), .rd_req_len (), .rd_req_tag (),
        .rd_ret_valid (1'b0), .rd_ret_ready (), .rd_ret_data (32'd0), .rd_ret_keep (4'd0),
        .rd_ret_tag (16'd0), .rd_ret_last (1'b0), .rd_ret_error (1'b0),
        .wr_req_valid (ext_wr_req_valid), .wr_req_ready (ext_wr_req_ready),
        .wr_req_addr (ext_wr_req_addr), .wr_req_len (ext_wr_req_len), .wr_req_tag (ext_wr_req_tag),
        .wr_dat_valid (ext_wr_dat_valid), .wr_dat_ready (ext_wr_dat_ready),
        .wr_dat_data (ext_wr_dat_data), .wr_dat_keep (ext_wr_dat_keep), .wr_dat_last (ext_wr_dat_last),
        .wr_done_valid (ext_wr_done_valid), .wr_done_ready (ext_wr_done_ready),
        .wr_done_tag (ext_wr_done_tag), .wr_done_error (ext_wr_done_error)
    );

    //--------------------------------------------------------------------
    // 期望向量与输出采集
    //--------------------------------------------------------------------
    reg [31:0] exp_chain [0:1023];
    integer    exp_N, oi = 0, err_cnt = 0;

    always @(posedge clk) begin
        if (top_out_valid && top_out_ready) begin
            if (oi >= exp_N || top_out_x !== exp_chain[2 + oi*2] ||
                top_out_y !== exp_chain[3 + oi*2]) begin
                err_cnt = err_cnt + 1;
                if (err_cnt <= 10)
                    $display("[FAIL] pt%0d got=(%08x,%08x) exp=(%08x,%08x)",
                             oi, top_out_x, top_out_y, exp_chain[2+oi*2], exp_chain[3+oi*2]);
            end
            oi = oi + 1;
        end
    end
    assign top_out_ready = 1'b1;

    task automatic load_vec(input string path);
        begin
            fd = $fopen(path, "rb");
            if (fd == 0) begin $display("[FATAL] no %s", path); $finish; end
            code = $fread(fbuf, fd);
            $fclose(fd);
        end
    endtask

    function automatic [31:0] le32(input integer i);
        le32 = {fbuf[i+3], fbuf[i+2], fbuf[i+1], fbuf[i]};
    endfunction

    task automatic load_chain(input string path);
        integer i;
        begin
            load_vec(path);
            exp_N = le32(4);
            for (i = 0; i < 2 + exp_N*2; ++i)
                exp_chain[i] = le32(i*4);
        end
    endtask

    //--------------------------------------------------------------------
    // 执行
    //--------------------------------------------------------------------
    integer t, frm;
    initial begin
        string vec_dir = "../tests/build/vectors/";
        rst_n = 1'b0;
        top_process = 1'b0; pre_start = 1'b0;
        pre_in_valid = 1'b0; pre_in_byte = 8'd0;
        pre_cfg_base = 0; pre_cfg_stride = 0; pre_cfg_offset = 0;
        pre_cfg_row_bytes = 0; pre_cfg_rows = 0;
        top_cfg_gray_base = 32'h0020_0000; top_cfg_gray_stride = 272;
        top_cfg_gray_w = 272; top_cfg_gray_h = 96;
        top_cfg_ram_base = 21'd0;
        top_cfg_resp_base = 32'h0040_0000;
        top_cfg_resp_dump_en = 1'b0;      // 聚焦连续帧，不写响应图
        top_cfg_pyr_en = 1'b0;
        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // 预载 board5x8 灰度（经 ext_wr 槽 1 → arbiter → 模型）
        load_vec({vec_dir, "m5_board5x8_gray.bin"});
        pre_cfg_base = 32'h0020_0000; pre_cfg_stride = 272;
        pre_cfg_row_bytes = 272; pre_cfg_rows = 96; pre_cfg_offset = 0;
        @(negedge clk);
        pre_start = 1'b1;
        @(negedge clk);
        pre_start = 1'b0;
        t = 0;
        while (t < 272 * 96) begin
            if (pre_in_ready) begin
                pre_in_valid <= 1'b1;
                pre_in_byte  <= fbuf[t];
                @(negedge clk);
                t = t + 1;
                pre_in_valid <= 1'b0;
            end else begin
                @(negedge clk);
            end
        end
        begin
            reg d_prev;
            d_prev = 1'b0;
            forever begin
                @(negedge clk);
                if (pre_done && !d_prev) break;
                d_prev = pre_done;
            end
        end
        repeat (3) @(negedge clk);
        $display("[b5x2] preload done status=%02b", pre_status);
        if (pre_status !== 2'b01) err_cnt = err_cnt + 1;

        // 连续两帧（同一 top 实例，grid_order S_DONE 再武装验证）
        load_chain({vec_dir, "m6_chain_board5x8.bin"});
        for (frm = 1; frm <= 2; frm = frm + 1) begin
            oi = 0;
            top_process = 1'b1;
            @(negedge clk);
            top_process = 1'b0;
            // 等 done 0→1 沿
            begin
                reg d_prev;
                d_prev = 1'b0;
                forever begin
                    @(negedge clk);
                    if (top_done && !d_prev) break;
                    d_prev = top_done;
                end
            end
            $display("[b5x2] frame%0d done status=%02b total=%0d grid_ok=%b pts=%0d",
                     frm, top_status, top_out_total, top_out_grid_ok, oi);
            if (top_status !== 2'b01 || top_out_grid_ok !== 1'b1 || oi != 40) begin
                err_cnt = err_cnt + 1;
                $display("[FAIL] frame%0d status/points", frm);
            end
            repeat (5) @(negedge clk);
        end

        repeat (20) @(negedge clk);
        $display("==============================================");
        $display("FRAME-B5X2 TB: err=%0d proto_violations=%0d", err_cnt, m_proto_viol);
        if (err_cnt == 0 && m_proto_viol == 0)
            $display("TB RESULT: ALL FRAME-B5X2 TESTS PASSED");
        else
            $display("TB RESULT: FAILED");
        $finish;
    end

    // 看门狗（board5x8 native 26112 像素/帧，两帧 + 预载）
    initial begin
        #60_000_000;
        $display("[FATAL] global timeout err=%0d done=%b", err_cnt, top_done);
        $display("[DBG] det stage=%0d dl=%0d busy=%b done=%b status=%02b",
                 u_top.u_det.stage, u_top.u_det.dl, u_top.u_det.busy,
                 u_top.u_det.done, u_top.u_det.status);
        $display("[DBG] order state=%0d ast=%0d busy=%b done=%b gok=%b",
                 u_top.u_det.g_slot[0].u_order.state, u_top.u_det.g_slot[0].u_order.ast,
                 u_top.u_det.g_slot[0].u_order.busy, u_top.u_det.g_slot[0].u_order.done,
                 u_top.u_det.g_slot[0].u_order.out_grid_ok);
        $display("[DBG] filter busy=%b done=%b state=%0d m5=%b m3=%b near=%b rg_sub=%0d ring_fin=%b spx_sub=%0d",
                 u_top.u_det.g_slot[0].u_filter.busy, u_top.u_det.g_slot[0].u_filter.done,
                 u_top.u_det.g_slot[0].u_filter.state,
                 u_top.u_det.g_slot[0].u_filter.m5_started, u_top.u_det.g_slot[0].u_filter.m3_started,
                 u_top.u_det.g_slot[0].u_filter.near_started, u_top.u_det.g_slot[0].u_filter.rg_sub,
                 u_top.u_det.g_slot[0].u_filter.ring_finish, u_top.u_det.g_slot[0].u_filter.spx_sub);
        $display("[DBG] subpx busy=%b done=%b state=%0d ist=%0d nst=%0d solve_st=%b pt_idx=%0d iter=%0d wst=%0d smpl=%0d",
                 u_top.u_det.g_slot[0].u_filter.u_subpx.busy, u_top.u_det.g_slot[0].u_filter.u_subpx.done,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.state, u_top.u_det.g_slot[0].u_filter.u_subpx.ist,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.nst,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.solve_started,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.pt_idx,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.iter,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.wst,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.smpl);
        $display("[DBG] handshakes: add_sx_v=%b add_sy_v=%b sub_dx_v=%b sub_dy_v=%b bil_in_ready=%b bil_out_valid=%b bil_fire=%b",
                 u_top.u_det.g_slot[0].u_filter.u_subpx.add_sx_v,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.add_sy_v,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.sub_dx_v,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.sub_dy_v,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.bil_in_ready,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.bil_out_valid,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.bil_fire);
        $display("[DBG] ts busy=%b done=%b", u_top.u_det.g_slot[0].u_filter.u_subpx.u_ts.busy,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.u_ts.done);
        $display("[DBG] bil state=%0d out_v=%b sdx_v=%b sdy_v=%b m1_v=%b m5_v=%b at_v=%b ab_v=%b ao_v=%b",
                 u_top.u_det.g_slot[0].u_filter.u_subpx.u_bil.state,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.u_bil.out_valid,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.u_bil.sdx_v,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.u_bil.sdy_v,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.u_bil.m1_v,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.u_bil.m5_v,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.u_bil.at_v,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.u_bil.ab_v,
                 u_top.u_det.g_slot[0].u_filter.u_subpx.u_bil.ao_v);
        $display("[DBG] shi state=%0d done=%b cand=%0d", u_top.u_det.g_slot[0].u_shi.state,
                 u_top.u_det.g_slot[0].u_shi.done, u_top.u_det.g_slot[0].u_shi.cand_total);
        $display("[DBG] fct busy=%b done=%b gf_busy=%b gf_done=%b det_busy=%b det_done=%b w_busy=%b w_done=%b",
                 u_top.u_frame.busy, u_top.u_frame.done, u_top.u_frame.gf_busy, u_top.u_frame.gf_done,
                 u_top.u_frame.det_busy, u_top.u_frame.det_done, u_top.u_frame.w_busy, u_top.u_frame.w_done);
        $finish;
    end

endmodule
