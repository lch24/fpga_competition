`timescale 1ns / 1ps
//==============================================================================
// byte_packer.v — 通用字节流 ↔ 32 位字流转换原语（M7.1 基础件）
//------------------------------------------------------------------------------
// 用途：把算法侧的字节流打包为 32 位字流（cfg_dir=1），或把上游（DDR 服务层）
//       返回的 32 位字流解包为字节流（cfg_dir=0）。与 DDR 服务层契约
//       （ddr_memory_model.sv / README §3.2）对齐：
//         - 字流为"连续字节流"语义：字节 i 落 data[8*(i%4) +: 8]（字节 0 在
//           data[7:0]，随后低位到高位），与字节地址对齐无关——模型按
//           mem[addr+off+k] 连续字节地址读写，首个写入/返回字节恒在 data[7:0]。
//         - keep[k] 对应 data[8k+7:8k]；非尾拍 keep=4'b1111，尾拍按剩余字节数
//           0001/0011/0111，整事务最后字 last=1。
//         - cfg_addr 仅记录本次事务首字节地址（供上层/调试，本原语不参与字节
//           排列）；真实控制器侧的"字对齐拆拍"由上层完成，本原语不负责。
//
// 字节流双向握手（b_valid 恒由本模块驱动、b_ready 恒由外部驱动，角色随 cfg_dir
// 对调）：
//   - 解包（cfg_dir=0）：b_valid/b_out = 模块输出的有效字节（keep 位命中且还有
//     剩余字节时拉高）；b_ready = 接收方就绪。握手 b_valid&&b_ready 每拍推进 1 字节。
//   - 打包（cfg_dir=1）：b_valid = 模块可接收（PACK_FILL 期间拉高）；b_ready =
//     数据源有数据（外部驱动，源侧在 b_valid 拉低时不得送新字节）；b_in = 输入
//     字节。握手 b_valid&&b_ready 收 1 字节。模块满字/等下游字握手时 b_valid 拉低，
//     天然背压字节源。
//
// 时序与状态：
//   - start 仅在 busy=0（IDLE/DONE）时单拍有效，posedge 锁存 cfg_* 并开始。
//   - 单事务在途：busy=1 期间忽略 start；完成（done=1，电平保持）后仍可 start
//     直接开启下一事务（start 拍清除 done）。
//   - status：01=正常完成，10=长度违规（cfg_len==0，立即完成，不产生任何字流/
//     字节流）。
//   - 解包完成条件：共输出 cfg_len 个字节后 done（多余 keep 位/后续字一律忽略，
//     "只输出 cfg_len 个字节"）；若上游字流短于 cfg_len，模块停留在等字状态
//     （上层协议错误，本原语不产生完成、不死锁）。
//   - 打包完成条件：整事务最后一个字被 w_out_ready 接受后 done。
//
// 复位：rst_n 低有效（同步释放，建议来自 reset_sync）；复位后 IDLE、busy=0。
//==============================================================================
module byte_packer #(
    parameter ADDR_W = 32,
    parameter LEN_W  = 32
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,          // busy=0 时单拍；锁存配置
    input  wire [ADDR_W-1:0]  cfg_addr,       // 本次事务首字节地址（任意起点，仅记录）
    input  wire [LEN_W-1:0]   cfg_len,        // 本次事务字节数（>0；==0 违规）
    input  wire               cfg_dir,        // 0=解包（字→字节流） 1=打包（字节流→字）
    output reg                busy,
    output reg                done,           // 电平保持
    output reg  [1:0]         status,         // 01=完成 10=长度违规（len==0）
    // 解包方向：字流输入（含 keep/last）
    input  wire               w_in_valid,
    output wire               w_in_ready,
    input  wire [31:0]        w_in_data,
    input  wire [3:0]         w_in_keep,
    input  wire               w_in_last,
    // 打包方向：字流输出
    output reg                w_out_valid,
    input  wire               w_out_ready,
    output reg  [31:0]        w_out_data,
    output reg  [3:0]         w_out_keep,
    output reg                w_out_last,
    // 字节流（双向握手，方向由 cfg_dir 决定）
    output reg                b_valid,        // 解包时=输出字节有效 / 打包时=模块可收
    input  wire               b_ready,
    input  wire [7:0]         b_in,           // 打包时输入字节
    output reg  [7:0]         b_out           // 解包时输出字节
);

    //--------------------------------------------------------------------
    // 状态定义
    //--------------------------------------------------------------------
    localparam [2:0] IDLE      = 3'd0,
                     UNP_WORD  = 3'd1,   // 解包：等字输入（w_in_ready=1）
                     UNP_BYTE  = 3'd2,   // 解包：输出字内字节（按 keep 低位先出）
                     PACK_FILL = 3'd3,   // 打包：收字节组装当前字（b_valid=1）
                     PACK_OUT  = 3'd4,   // 打包：当前字输出（w_out_valid=1）
                     DONE      = 3'd5;

    reg [2:0]  state;

    reg [ADDR_W-1:0] cfg_addr_r;         // 锁存的首字节地址（仅记录）
    reg [LEN_W-1:0]  cfg_len_r;          // 剩余字节数（含当前正在收/出的字节）
    reg              cfg_dir_r;

    reg [31:0] cur_word;                 // 解包锁存的输入字 / 打包组装中的字
    reg [3:0]  cur_keep;                 // 解包锁存的输入 keep / 打包已填 keep
    reg [1:0]  k;                        // 解包：字内字节指针（0..3）
    reg [1:0]  pos;                      // 打包：下一个字节要填的字内位置（0 起）
    reg [2:0]  cur_cnt;                  // 打包：当前字已填字节数（1..4）
    reg        out_last;                 // 打包：当前 PACK_OUT 字是否为尾字

    //--------------------------------------------------------------------
    // 解包：字流 ready（仅解包方向、等字状态拉高）
    //--------------------------------------------------------------------
    assign w_in_ready = (cfg_dir_r == 1'b0) && (state == UNP_WORD);

    //--------------------------------------------------------------------
    // 打包：字流输出（组合驱动）
    //--------------------------------------------------------------------
    always @(*) begin
        if ((cfg_dir_r == 1'b1) && (state == PACK_OUT))
            w_out_valid = 1'b1;
        else
            w_out_valid = 1'b0;
    end

    always @(*) begin
        w_out_data = cur_word;
    end

    always @(*) begin
        if ((cfg_dir_r == 1'b1) && (state == PACK_OUT)) begin
            if (out_last) begin
                case (cur_cnt)
                    3'd1:    w_out_keep = 4'b0001;
                    3'd2:    w_out_keep = 4'b0011;
                    3'd3:    w_out_keep = 4'b0111;
                    default: w_out_keep = 4'b1111;   // cur_cnt==4（len%4==0 尾字）
                endcase
            end else begin
                w_out_keep = 4'b1111;                // 满字
            end
        end else begin
            w_out_keep = 4'b1111;
        end
    end

    always @(*) begin
        w_out_last = out_last;
    end

    //--------------------------------------------------------------------
    // 字节流（组合驱动）
    //--------------------------------------------------------------------
    always @(*) begin
        // 解包：输出有效字节（keep 位命中且剩余>0）；打包：可接收
        if (cfg_dir_r == 1'b0) begin
            if ((state == UNP_BYTE) && cur_keep[k] && (cfg_len_r != {LEN_W{1'b0}})) begin
                b_valid = 1'b1;
                b_out   = cur_word[8*k +: 8];
            end else begin
                b_valid = 1'b0;
                b_out   = 8'h00;
            end
        end else begin
            if (state == PACK_FILL)
                b_valid = 1'b1;
            else
                b_valid = 1'b0;
            b_out = 8'h00;
        end
    end

    //--------------------------------------------------------------------
    // 主状态机（含 busy/done/status）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state     <= IDLE;
            busy      <= 1'b0;
            done      <= 1'b0;
            status    <= 2'b00;
            cfg_addr_r <= {ADDR_W{1'b0}};
            cfg_len_r  <= {LEN_W{1'b0}};
            cfg_dir_r  <= 1'b0;
            cur_word   <= 32'h0;
            cur_keep   <= 4'b0;
            k          <= 2'b00;
            pos        <= 2'b00;
            cur_cnt    <= 3'd0;
            out_last   <= 1'b0;
        end else begin
            case (state)
                //--------------------------------------------------------
                IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        cfg_addr_r <= cfg_addr;
                        cfg_dir_r  <= cfg_dir;
                        if (cfg_len == {LEN_W{1'b0}}) begin
                            // 长度违规：立即完成（status=10）
                            status <= 2'b10;
                            done   <= 1'b1;
                            state  <= DONE;
                        end else begin
                            cfg_len_r <= cfg_len;
                            done   <= 1'b0;
                            busy   <= 1'b1;
                            if (cfg_dir == 1'b0) begin
                                k <= 2'b00;
                                state <= UNP_WORD;
                            end else begin
                                pos     <= 2'b00;
                                cur_cnt <= 3'd0;
                                cur_keep <= 4'b0;
                                state   <= PACK_FILL;
                            end
                        end
                    end
                end

                //--------------------------------------------------------
                // 解包：等字输入
                //--------------------------------------------------------
                UNP_WORD: begin
                    if (w_in_valid) begin
                        cur_word <= w_in_data;
                        cur_keep <= w_in_keep;
                        k        <= 2'b00;
                        state    <= UNP_BYTE;
                    end
                end

                //--------------------------------------------------------
                // 解包：输出字内有效字节（keep 低位先出）
                //--------------------------------------------------------
                UNP_BYTE: begin
                    if (!cur_keep[k]) begin
                        // keep 空洞：跳过该位置
                        if (k == 2'b11)
                            state <= UNP_WORD;
                        else
                            k <= k + 2'd1;
                    end else if (b_valid && b_ready) begin
                        // 输出一字节
                        if (cfg_len_r == 1) begin
                            // 本字节恰为事务最后一字节：完成
                            done   <= 1'b1;
                            busy   <= 1'b0;
                            status <= 2'b01;
                            state  <= DONE;
                        end else begin
                            cfg_len_r <= cfg_len_r - 1'b1;
                            if (k == 2'b11)
                                state <= UNP_WORD;
                            else
                                k <= k + 2'd1;
                        end
                    end
                end

                //--------------------------------------------------------
                // 打包：收字节组装当前字
                //--------------------------------------------------------
                PACK_FILL: begin
                    if (b_valid && b_ready) begin
                        cur_word[8*pos +: 8] <= b_in;
                        cur_keep[pos]          <= 1'b1;
                        cur_cnt                <= cur_cnt + 3'd1;
                        if (cfg_len_r == 1) begin
                            // 本字节为事务最后一字节：输出尾字
                            out_last <= 1'b1;
                            state    <= PACK_OUT;
                        end else begin
                            cfg_len_r <= cfg_len_r - 1'b1;
                            if (pos == 2'b11) begin
                                // 当前字已满 4 字节：输出满字
                                out_last <= 1'b0;
                                state    <= PACK_OUT;
                            end else begin
                                pos <= pos + 2'd1;
                            end
                        end
                    end
                end

                //--------------------------------------------------------
                // 打包：输出当前字（等待下游握手）
                //--------------------------------------------------------
                PACK_OUT: begin
                    if (w_out_valid && w_out_ready) begin
                        if (out_last) begin
                            // 尾字被接受：完成
                            done   <= 1'b1;
                            busy   <= 1'b0;
                            status <= 2'b01;
                            state  <= DONE;
                        end else begin
                            // 满字已出：清空组装，继续收字节
                            cur_keep <= 4'b0;
                            cur_cnt  <= 3'd0;
                            pos      <= 2'b00;
                            state    <= PACK_FILL;
                        end
                    end
                end

                //--------------------------------------------------------
                DONE: begin
                    busy <= 1'b0;
                    if (start) begin
                        cfg_addr_r <= cfg_addr;
                        cfg_dir_r  <= cfg_dir;
                        if (cfg_len == {LEN_W{1'b0}}) begin
                            status <= 2'b10;
                            done   <= 1'b1;
                            state  <= DONE;
                        end else begin
                            cfg_len_r <= cfg_len;
                            done   <= 1'b0;
                            busy   <= 1'b1;
                            if (cfg_dir == 1'b0) begin
                                k <= 2'b00;
                                state <= UNP_WORD;
                            end else begin
                                pos     <= 2'b00;
                                cur_cnt <= 3'd0;
                                cur_keep <= 4'b0;
                                state   <= PACK_FILL;
                            end
                        end
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
