`timescale 1ns / 1ps
//==============================================================================
// tb_detect.sv — Shi-Tomasi (M2) 两遍扫描控制/存储行为验证 TB
//------------------------------------------------------------------------------
// 合成小图（W=8,H=6，PIXELS=48），绕过前端：TB 直接把固定 resp 数组按光栅序
// 喂给 ctrl.resp_valid/resp_data。TB 内以软件模型复刻 C++ 判据（用与 RTL 相同
// 的序位变换做 fp 比较）自算期望：rmax、thr、候选列表（光栅序）。
//
// 场景矩阵：
//   1) 全 0 图            -> rmax=0  -> status=2'b10, cand_total=0
//   2) 全 -1.0 图         -> rmax<0  -> status=2'b10, cand_total=0
//   3) 主场景（含正角点、相等邻居不抑制、负 resp、等于thr 的 >= 边界、
//      图像角部高值被范围内缩抑制、rmax 由角部高值决定）
//   4) 重跑主场景（验证 done/start 复位时序）
//
// 校验：1) pass1 后 rmax 位相等；2) rmax<=0 场景 status/cand_total；
//       3) 候选与期望全等且光栅序一致、cand_total 相等；4) done 时序。
// 全部通过打印 ALL DETECT TESTS PASSED。
//==============================================================================
module tb_detect;

    parameter IMG_W  = 8;
    parameter IMG_H  = 6;
    parameter PIXELS = IMG_W * IMG_H;
    parameter ADDR_W = $clog2(PIXELS);

    reg clk = 1'b0;
    always #5 clk = ~clk;                 // 10ns 周期

    reg                        rst_n;
    reg                        start;
    reg                        resp_valid;
    reg  [31:0]                resp_data;
    wire                       resp_rdy;
    wire                       busy, done;
    wire [1:0]                 status;
    wire                       cand_valid;
    wire [10:0]                cand_x, cand_y;
    wire [15:0]                cand_total;
    reg  [31:0]                thr_to_dut;

    // DUT
    shi_tomasi_ctrl #(
        .IMG_W (IMG_W), .IMG_H (IMG_H),
        .PIXELS (PIXELS), .ADDR_W (ADDR_W)
    ) u_ctrl (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (start),
        .busy      (busy),
        .thr       (thr_to_dut),
        .resp_valid(resp_valid),
        .resp_rdy  (resp_rdy),
        .resp_data (resp_data),
        .mem_addr  (),
        .mem_data  (32'd0),
        .done      (done),
        .status    (status),
        .cand_valid(cand_valid),
        .cand_ready(1'b1),
        .cand_x    (cand_x),
        .cand_y    (cand_y),
        .cand_total(cand_total)
    );

    //--------------------------------------------------------------------
    // 被测帧 + 期望
    //--------------------------------------------------------------------
    reg [31:0] frame[0:PIXELS-1];
    // 期望候选（光栅序）
    reg [10:0] exp_x[0:255];
    reg [10:0] exp_y[0:255];
    integer    exp_n;

    //--------------------------------------------------------------------
    // 序位变换（与 RTL 一致）：a[31]?~a:(a|0x8000_0000)，无符号比较即 IEEE 序
    //--------------------------------------------------------------------
    function automatic [31:0] ord(input [31:0] a);
        ord = a[31] ? ~a : (a | 32'h8000_0000);
    endfunction
    function automatic bit ge(input [31:0] a, b);
        ge = ($unsigned(ord(a)) >= $unsigned(ord(b)));
    endfunction
    function automatic bit gt(input [31:0] a, b);
        gt = ($unsigned(ord(a)) >  $unsigned(ord(b)));
    endfunction

    //--------------------------------------------------------------------
    // 帧 -> rmax 位（软件模型，逐拍等效 max）
    //--------------------------------------------------------------------
    function automatic [31:0] frame_rmax(input reg [31:0] q[0:PIXELS-1]);
        reg [31:0] m;
        integer i;
        begin
            m = 32'hFF80_0000;   // -inf
            for (i = 0; i < PIXELS; i = i + 1)
                if (gt(q[i], m)) m = q[i];
            frame_rmax = m;
        end
    endfunction

    //--------------------------------------------------------------------
    // fp32 乘法（位级 RN，用于 thr=rmax*0.08f 的忠实建模）
    //--------------------------------------------------------------------
    function automatic [31:0] fpmul(input [31:0] a, input [31:0] b);
        reg        sa, sb, sg;
        reg [7:0]  ea, eb;
        reg [23:0] ma, mb;
        reg [47:0] m;
        reg [7:0]  e;
        reg [24:0] manr;
        reg        carry;
        begin
            sa = a[31]; sg = sa ^ b[31];
            ea = a[30:23]; eb = b[30:23];
            ma = (ea == 0) ? {1'b0, a[22:0]} : {1'b1, a[22:0]};
            mb = (eb == 0) ? {1'b0, b[22:0]} : {1'b1, b[22:0]};
            if ((ma==0)||(mb==0)) begin fpmul = {sg,8'h00,23'h0}; end
            else begin
                m = ma * mb;
                e = (ea + eb) - 8'd127;
                if (m[47]) e = e + 1'b1; else m = m << 1;   // 规格化 m[47]=1
                // RN: guard=m[23], round=m[22], sticky=|m[21:0]
                carry = m[23] && ((|m[22:0]) || m[24]);
                manr  = m[47:24] + carry;
                if (manr[24]) begin e = e + 1'b1; manr = manr >> 1; end
                if (e >= 8'd255)      fpmul = {sg,8'hFF,23'h0};          // inf
                else if (e == 8'd0)   fpmul = {sg,8'h00,23'h0};          // 极小 -> 0
                else                  fpmul = {sg, e, manr[22:0]};
            end
        end
    endfunction

    //--------------------------------------------------------------------
    // 软件模型：计算期望候选（光栅序，与 window 窗口流一致）
    //--------------------------------------------------------------------
    task automatic model_candidates(input [31:0] thrb);
        integer x, y, x0, y0;
        reg [31:0] v;
        reg ok;
        begin
            exp_n = 0;
            for (y = 0; y < IMG_H; y = y + 1)
              for (x = 0; x < IMG_W; x = x + 1) begin
                // Pass2 内缩：中心有效范围
                if ((x >= 2) && (x <= IMG_W-3) && (y >= 2) && (y <= IMG_H-3)) begin
                    v = frame[y*IMG_W + x];
                    if (gt(v, 32'h0) && ge(v, thrb)) begin
                        // 8 邻居都不严格大于 v（相等不抑制）
                        ok = 1'b1;
                        for (y0 = -1; y0 <= 1; y0 = y0 + 1)
                          for (x0 = -1; x0 <= 1; x0 = x0 + 1) begin
                            if (!(x0==0 && y0==0)) begin
                                if (gt(frame[(y+y0)*IMG_W + (x+x0)], v)) ok = 1'b0;
                            end
                          end
                        if (ok) begin
                            exp_x[exp_n] = x;
                            exp_y[exp_n] = y;
                            exp_n = exp_n + 1;
                        end
                    end
                end
              end
        end
    endtask

    //--------------------------------------------------------------------
    // 真正驱动 DUT：喂帧 + 启动两遍
    //--------------------------------------------------------------------
    integer errors;
    integer cycle_cnt;

    reg  got_valid;
    integer gi, got_n;
    reg [10:0] got_xq[0:255];
    reg [10:0] got_yq[0:255];
    reg  capture;

    // 候选捕获（passthrough，cand_ready=1）
    always @(posedge clk) begin
        if (capture && cand_valid) begin
            got_xq[got_n] = cand_x;
            got_yq[got_n] = cand_y;
            got_n = got_n + 1;
        end
    end

    // 复位
    task automatic do_reset;
        begin
            rst_n = 1'b0;
            start = 1'b0;
            resp_valid = 1'b0;
            resp_data = 32'd0;
            capture = 1'b0;
            got_n = 0;
            @(negedge clk); @(negedge clk); @(negedge clk);
            rst_n = 1'b1;
            @(negedge clk);
        end
    endtask

    // 运行一帧：喂 frame 数组，期望 thrb 与 exp 已算好
    task automatic run_frame(input string name, input [31:0] thrb,
                             input logic expect_skippass2);
        integer j;
        begin
            capture = 1'b0; got_n = 0;
            // 启动
            @(negedge clk);
            start = 1'b1; @(negedge clk); start = 1'b0;
            // Pass1 喂帧（带少量背压间隙，测握手）
            resp_valid = 1'b1;
            for (j = 0; j < PIXELS; j = j + 1) begin
                resp_data = frame[j];
                // 每 5 拍插入一拍停，验证握手/存储
                if ((j % 7) == 6) begin
                    @(negedge clk);
                    resp_valid = 1'b0;
                    @(negedge clk);
                    resp_valid = 1'b1;
                end else begin
                    @(negedge clk);
                end
            end
            resp_valid = 1'b0;

            // pass1_done 采样 rmax（稳定后一拍）
            do @(posedge clk); while (!u_ctrl.u_store.pass1_done);
            @(posedge clk); @(posedge clk);
            if ($unsigned(u_ctrl.u_store.rmax) !== $unsigned(rmax_exp)) begin
                errors = errors + 1;
                $error("[%0s] rmax 不匹配: dut=%08h exp=%08h", name,
                       u_ctrl.u_store.rmax, rmax_exp);
            end else
                $display("[%0s] rmax OK = %08h", name, u_ctrl.u_store.rmax);

            // 等待整体 done（超时保护）
            cycle_cnt = 0;
            capture = 1'b1;
            while (!done) begin
                @(posedge clk);
                cycle_cnt = cycle_cnt + 1;
                if (cycle_cnt > 100000) begin
                    errors = errors + 1;
                    $error("[%0s] 超时未 done", name);
                    break;
                end
            end
            capture = 1'b0;

            if (expect_skippass2) begin
                if (status !== 2'b10) begin
                    errors = errors + 1;
                    $error("[%0s] 期望 status=10 得到 %02b", name, status);
                end else
                    $display("[%0s] rmax<=0 status=10 OK, cand_total=%0d",
                             name, cand_total);
                if (cand_total != 0) begin
                    errors = errors + 1;
                    $error("[%0s] 无角点场景 cand_total 应=0", name);
                end
            end else begin
                if (status !== 2'b01) begin
                    errors = errors + 1;
                    $error("[%0s] 期望 status=01 得到 %02b", name, status);
                end
                // 候选比对
                if (got_n !== exp_n) begin
                    errors = errors + 1;
                    $error("[%0s] 候选数量不符: dut=%0d exp=%0d", name, got_n, exp_n);
                end
                if (got_n !== 0) begin
                    for (gi = 0; gi < exp_n; gi = gi + 1) begin
                        if ((got_xq[gi] !== exp_x[gi]) || (got_yq[gi] !== exp_y[gi])) begin
                            errors = errors + 1;
                            $error("[%0s] 候选顺序/坐标不符 @%0d: dut=(%0d,%0d) exp=(%0d,%0d)",
                                   name, gi, got_xq[gi], got_yq[gi], exp_x[gi], exp_y[gi]);
                        end
                    end
                end
                if ((got_n === exp_n) && (got_n !== 0))
                    $display("[%0s] 候选 %0d 个全等（光栅序）OK", name, got_n);
                if (cand_total !== exp_n[15:0]) begin
                    errors = errors + 1;
                    $error("[%0s] cand_total 不符: dut=%0d exp=%0d", name,
                           cand_total, exp_n);
                end
                if (!done) begin
                    errors = errors + 1;
                    $error("[%0s] done 未置位", name);
                end
            end
            $display("[%0s] done 于 %0d 周期置位", name, cycle_cnt);
        end
    endtask

    reg [31:0] rmax_exp;

    //--------------------------------------------------------------------
    // 场景
    //--------------------------------------------------------------------
    integer i, y, x;
    reg [31:0] thrb;

    initial begin
        errors = 0;
        do_reset();

        //================ 场景1：全 0 ====================
        for (i = 0; i < PIXELS; i = i + 1) frame[i] = 32'h0000_0000;
        rmax_exp = frame_rmax(frame);
        thrb     = fpmul(rmax_exp, 32'h3DA3_D70A);   // 0.08f
        model_candidates(thrb);
        thr_to_dut = thrb;
        run_frame("ALLZERO", thrb, 1'b1);

        //================ 场景2：全 -1.0 =================
        do_reset();
        for (i = 0; i < PIXELS; i = i + 1) frame[i] = 32'hBF80_0000; // -1.0
        rmax_exp = frame_rmax(frame);
        thrb     = fpmul(rmax_exp, 32'h3DA3_D70A);
        model_candidates(thrb);
        thr_to_dut = thrb;
        run_frame("ALLNEG", thrb, 1'b1);

        //================ 场景3：MAIN（角点 + 相等邻居 + 负 resp + 角部高值）
        // 背景 0.1
        do_reset();
        for (i = 0; i < PIXELS; i = i + 1) frame[i] = 32'h3DCC_CCCD; // 0.1
        frame[0*IMG_W + 0] = 32'h4110_0000; // (0,0)=9.0  角部高值，范围内缩应抑制（但决定 rmax）
        frame[3*IMG_W + 3] = 32'h40A0_0000; // (3,3)=5.0  sharp 角点
        frame[3*IMG_W + 4] = 32'hC120_0000; // (4,3)=-10.0  负 resp（不入选，且抑制其邻居外的判断）
        frame[2*IMG_W + 5] = 32'h4000_0000; // (5,2)=2.0  相等邻居之一
        frame[3*IMG_W + 5] = 32'h4000_0000; // (5,3)=2.0  相等邻居之二（相等不互抑）
        rmax_exp = frame_rmax(frame);        // =9.0
        thrb     = fpmul(rmax_exp, 32'h3DA3_D70A);   // ~0.72
        model_candidates(thrb);
        thr_to_dut = thrb;
        run_frame("MAIN", thrb, 1'b0);
        $display("MAIN 期望候选 %0d 个", exp_n);

        //================ 场景4：THR_EDGE（等于thr 的 >= 边界 + 背景<thr 被滤除）
        // 背景 0.01（< thr），高点放角部（0,0），(3,3) 恰等于 thr
        do_reset();
        for (i = 0; i < PIXELS; i = i + 1) frame[i] = 32'h3C23_D70A; // 0.01
        frame[0*IMG_W + 0] = 32'h3F80_0000;  // (0,0)=1.0 -> rmax=1.0, thr=0.08
        rmax_exp = frame_rmax(frame);         // 1.0
        thrb     = fpmul(rmax_exp, 32'h3DA3_D70A);   // 0.08
        frame[3*IMG_W + 3] = thrb;            // (3,3)=thr（>= 含等号应入选）
        model_candidates(thrb);
        thr_to_dut = thrb;
        run_frame("THR_EDGE", thrb, 1'b0);
        $display("THR_EDGE 期望候选 %0d 个", exp_n);

        //================ 场景5：重跑 MAIN（done/start 复位时序）
        // 重新填充 MAIN 帧
        do_reset();
        for (i = 0; i < PIXELS; i = i + 1) frame[i] = 32'h3DCC_CCCD;
        frame[0*IMG_W + 0] = 32'h4110_0000;
        frame[3*IMG_W + 3] = 32'h40A0_0000;
        frame[3*IMG_W + 4] = 32'hC120_0000;
        frame[2*IMG_W + 5] = 32'h4000_0000;
        frame[3*IMG_W + 5] = 32'h4000_0000;
        rmax_exp = frame_rmax(frame);
        thrb     = fpmul(rmax_exp, 32'h3DA3_D70A);
        model_candidates(thrb);
        thr_to_dut = thrb;
        run_frame("MAIN_RERUN", thrb, 1'b0);

        // 结果
        if (errors == 0)
            $display("ALL DETECT TESTS PASSED");
        else
            $display("FAILED: %0d errors", errors);
        $finish;
    end

endmodule