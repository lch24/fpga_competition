`include "calib_defs.vh"

/*
配置：统一见 rtl/include/calib_config.vh；下文具体计数例子以默认3张5×8为例，
实际容量、循环边界和端口位宽由PAR_*派生，修改配置后须重新编译/综合。
作用：给定一组内参和三张H，生成一个完整LM初始状态。
每视图计算K^-1H，尺度/深度符号修正，Gram-Schmidt正交化与叉乘得到R。
调用rotation(mode=1)由R求旋转向量；tx/ty保留，tz必须正后取log。
焦距取log，主点除W/H；畸变5槽清零。逐视图失败即该seed无效。
子模块：rotation、fp_operator；3x3点积/叉积顺序控制直接放本模块，不再切碎。
命令时锁存三张H及K；顺序完成view0..2，任一视图失败使整组seed无效。
H整体取负仍可恢复正深度：尺度及前两列正交基使用相同符号，第三列由叉乘生成。
BAD_CONFIG：宽高<2；CALIB_INVALID：非有限、非正焦距、退化基向量或非正深度。
只有成功响应的27项状态有效；失败时整个输出清零，不暴露部分视图的结果。

共同契约：
- 已实现：单事务顺序状态机；FP64 运算核按请求/响应串行复用。
- valid && ready 才传输；背压时 valid 和对应全部载荷保持不变。
- 除另有说明，一次一项任务，rsp 被接收前不接受新 cmd；不假设固定延迟。
- cmd_* 随命令锁存；rsp_* 随响应有效。失败仍须发完成响应，不能无声挂起。
- 非零状态通常使结果载荷无效；本模块明确说明的诊断有效位例外。
- 复位取消在途数据；复位后所有生产端 valid 清零。实现时禁止 real 运算。
*/
module pose_init #(parameter FP_SHARED=0) (
    // Optional shared FP64 service; local mode keeps standalone compatibility.
    output wire [1:0] shared_req_valid,
    input wire [1:0] shared_req_ready,
    output wire [9:0] shared_req_op,
    output wire [127:0] shared_req_a,
    output wire [127:0] shared_req_b,
    output wire [1:0] shared_active,
    input wire [1:0] shared_rsp_valid,
    output wire [1:0] shared_rsp_ready,
    input wire [63:0] shared_rsp_result,
    input wire [4:0] shared_rsp_flags,

    input wire clk, // core_clk，同一时钟域
    input wire rst_n, // 低有效复位，同步释放；取消全部在途事务
    input wire cmd_valid, // 命令有效
    output wire cmd_ready, // 可接受命令
    input wire [15:0] cmd_width, // 图像宽度，>=2
    input wire [15:0] cmd_height, // 图像高度，>=2
    input wire [`PAR_H_ALL_W-1:0] cmd_h_all_fp64, // 三张单应矩阵
    input wire [`PAR_K_W-1:0] cmd_k_fp64, // 内参seed
    output wire rsp_valid, // 完成响应有效；最后一笔输出握手后才置位
    input wire rsp_ready, // 接收完成响应
    output reg [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [`PAR_STATE_W-1:0] rsp_state // 完整PAR_STATE_N项状态
);

    // 工作寄存器映射及各阶段见下方注释；数组不复位，事务内先写后读。
    localparam IDLE=9000, FP_REQ=9001, FP_WAIT=9002, RESPONSE=9003;
    localparam ADD=0,SUB=1,MUL=2,DIV=3,SQRT=4,LOG=9,CONVERT=11;
    localparam [63:0] ZERO=64'b0,ONE=64'h3ff0000000000000;
    reg [63:0] v[0:`PAR_STATE_N+46];
    integer pc,destination,continuation,j;
    reg [4:0] fp_op;
    reg [63:0] fp_a,fp_b;
    wire fp_ready,fp_valid;
    wire [63:0] fp_result;
    wire [4:0] fp_flags;
    // v[0:3]=K, [4:5]=W/H, [10:18]=K^-1 H(列序), [20:28]=正交基列,
    // 临时区[70:73]随PAR_STATE_N后移，避免与增加的输出外参重叠。默认：
    // [30:32]=t, [33:39]=范数/尺度/临时量, [40:66]=27项输出, [70:73]=临时量。
    reg [63:0] hom[0:9*`PAR_VIEWS-1];integer view;
    wire r_ready,r_valid;wire [7:0] r_status;wire [191:0] r_vec;
    rotation #(.FP_SHARED(FP_SHARED)) rot(.shared_req_valid(shared_req_valid[1 +: 1]),.shared_req_ready(shared_req_ready[1 +: 1]),.shared_req_op(shared_req_op[5 +: 5]),.shared_req_a(shared_req_a[64 +: 64]),.shared_req_b(shared_req_b[64 +: 64]),.shared_active(shared_active[1 +: 1]),.shared_rsp_valid(shared_rsp_valid[1 +: 1]),.shared_rsp_ready(shared_rsp_ready[1 +: 1]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(rst_n),.cmd_valid(rst_n && pc==82),.cmd_ready(r_ready),.cmd_mode(1'b1),
        .cmd_rotvec_fp64(192'b0),.cmd_r_fp64({v[28],v[25],v[22],v[27],v[24],v[21],v[26],v[23],v[20]}),
        .rsp_valid(r_valid),.rsp_ready(rst_n && pc==83),.rsp_status(r_status),.rsp_rotvec_fp64(r_vec),.rsp_r_fp64());
    genvar g;generate for(g=0;g<`PAR_STATE_N;g=g+1)begin:pack_state
        assign rsp_state[64*g+:64]=(rsp_status==0)?v[40+g]:64'b0;
    end endgenerate
    assign cmd_ready=rst_n && pc==IDLE;
    assign rsp_valid=rst_n && pc==RESPONSE;
    function finite;
        input [63:0] x;
        begin finite=(x[62:52]!=11'h7ff);end
    endfunction
    // 精确无符号16位整数转FP64，不使用综合不支持的 real。
    function [63:0] u16;
        input [15:0] x;
        integer k,top;
        reg [63:0] shifted;
        reg [10:0] exponent;
        begin top=0;for(k=0;k<16;k=k+1)if(x[k])top=k;
            shifted={48'b0,x} << (52-top);exponent=1023+top;
            u16=(x==0)?64'b0:{1'b0,exponent,shifted[51:0]};end
    endfunction

    generate if(FP_SHARED) begin : g_shared_fp
        assign shared_req_valid[0 +: 1] = rst_n && pc==FP_REQ;
        assign shared_req_op[0 +: 5] = fp_op;
        assign shared_req_a[0 +: 64] = fp_a;
        assign shared_req_b[0 +: 64] = fp_b;
        assign shared_rsp_ready[0 +: 1] = rst_n && pc==FP_WAIT;
        assign shared_active[0] = rst_n;
        assign fp_ready = shared_req_ready[0];
        assign fp_valid = shared_rsp_valid[0];
        assign fp_result = shared_rsp_result;
        assign fp_flags = shared_rsp_flags;
    end else begin : g_local_fp
    fp_operator #(.FP_W(64), .ENABLE_EXP(0), .ENABLE_LOG(1), .ENABLE_SINCOS(0), .ENABLE_ATAN_ACOS(0)) arithmetic(.clk(clk),.rst_n(rst_n),
        .req_valid(rst_n && pc==FP_REQ),.req_ready(fp_ready),.req_op(fp_op),.req_a(fp_a),.req_b(fp_b),
        .rsp_valid(fp_valid),.rsp_ready(rst_n && pc==FP_WAIT),.rsp_result(fp_result),.rsp_flags(fp_flags),
        .rsp_less(),.rsp_equal(),.rsp_unordered());
assign shared_req_valid[0 +: 1] = 0;
assign shared_req_op[0 +: 5] = 0;
assign shared_req_a[0 +: 64] = 0;
assign shared_req_b[0 +: 64] = 0;
assign shared_active[0 +: 1] = 0;
assign shared_rsp_ready[0 +: 1] = 0;
    end endgenerate

    task calculate;
        input [4:0] operation;input [63:0] a,b;input integer target,next_pc;
        begin fp_op<=operation;fp_a<=a;fp_b<=b;destination<=target;continuation<=next_pc;pc<=FP_REQ;end
    endtask
    task fail;
        input [7:0] status;
        begin rsp_status<=status;pc<=RESPONSE;end
    endtask
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pc<=IDLE;rsp_status<=0;fp_op<=0;fp_a<=0;fp_b<=0;destination<=0;continuation<=0;
            view<=0;
        end else case(pc)
            IDLE: if(cmd_valid) begin
                rsp_status<=0;pc<=0;
                view<=0;v[4]<=u16(cmd_width);v[5]<=u16(cmd_height);
                for(j=0;j<9*`PAR_VIEWS;j=j+1)hom[j]<=cmd_h_all_fp64[64*j +:64];
                for(j=0;j<`PAR_STATE_N;j=j+1)v[40+j]<=0;
                for(j=0;j<4;j=j+1)v[j]<=cmd_k_fp64[64*j +:64];
                for(j=0;j<9*`PAR_VIEWS;j=j+1)if(!finite(cmd_h_all_fp64[64*j +:64]))fail(`PAR_CALIB_INVALID);
                for(j=0;j<4;j=j+1)if(!finite(cmd_k_fp64[64*j +:64]))fail(`PAR_CALIB_INVALID);
                if(cmd_width<2 || cmd_height<2)fail(`PAR_BAD_CONFIG);
            end
            FP_REQ: if(fp_ready) pc<=FP_WAIT;
            FP_WAIT: if(fp_valid) begin
                if((|fp_flags[2:0]) || !finite(fp_result)) fail(`PAR_CALIB_INVALID);
                else begin v[destination]<=fp_result;pc<=continuation;end
            end
            RESPONSE: if(rsp_ready) pc<=IDLE;
                // 焦距必须正；编码log(fx/fy)、cx/W、cy/H；畸变槽保持0
                0: begin if(v[0][63] || v[1][63] || v[0][62:0]==0 || v[1][62:0]==0)fail(`PAR_CALIB_INVALID);else pc<=1; end
                1: begin calculate(LOG, v[0], ZERO, 40, 2); end
                2: begin calculate(LOG, v[1], ZERO, 41, 3); end
                3: begin calculate(DIV, v[2], v[4], 42, 4); end
                4: begin calculate(DIV, v[3], v[5], 43, 5); end
                // 逐视图计算 K^-1 H 的三列
                5: begin calculate(MUL, v[2], hom[view*9+6], `PAR_STATE_N+43, 6); end
                6: begin calculate(SUB, hom[view*9+0], v[`PAR_STATE_N+43], `PAR_STATE_N+43, 7); end
                7: begin calculate(DIV, v[`PAR_STATE_N+43], v[0], 10, 8); end
                8: begin calculate(MUL, v[3], hom[view*9+6], `PAR_STATE_N+44, 9); end
                9: begin calculate(SUB, hom[view*9+3], v[`PAR_STATE_N+44], `PAR_STATE_N+44, 10); end
                10: begin calculate(DIV, v[`PAR_STATE_N+44], v[1], 11, 11); end
                11: begin v[12]<=hom[view*9+6];pc<=12; end
                12: begin calculate(MUL, v[2], hom[view*9+7], `PAR_STATE_N+43, 13); end
                13: begin calculate(SUB, hom[view*9+1], v[`PAR_STATE_N+43], `PAR_STATE_N+43, 14); end
                14: begin calculate(DIV, v[`PAR_STATE_N+43], v[0], 13, 15); end
                15: begin calculate(MUL, v[3], hom[view*9+7], `PAR_STATE_N+44, 16); end
                16: begin calculate(SUB, hom[view*9+4], v[`PAR_STATE_N+44], `PAR_STATE_N+44, 17); end
                17: begin calculate(DIV, v[`PAR_STATE_N+44], v[1], 14, 18); end
                18: begin v[15]<=hom[view*9+7];pc<=19; end
                19: begin calculate(MUL, v[2], hom[view*9+8], `PAR_STATE_N+43, 20); end
                20: begin calculate(SUB, hom[view*9+2], v[`PAR_STATE_N+43], `PAR_STATE_N+43, 21); end
                21: begin calculate(DIV, v[`PAR_STATE_N+43], v[0], 16, 22); end
                22: begin calculate(MUL, v[3], hom[view*9+8], `PAR_STATE_N+44, 23); end
                23: begin calculate(SUB, hom[view*9+5], v[`PAR_STATE_N+44], `PAR_STATE_N+44, 24); end
                24: begin calculate(DIV, v[`PAR_STATE_N+44], v[1], 17, 25); end
                25: begin v[18]<=hom[view*9+8];pc<=26; end
                26: begin calculate(MUL, v[10], v[10], `PAR_STATE_N+43, 27); end
                27: begin calculate(MUL, v[11], v[11], `PAR_STATE_N+44, 28); end
                28: begin calculate(ADD, v[`PAR_STATE_N+43], v[`PAR_STATE_N+44], `PAR_STATE_N+43, 29); end
                29: begin calculate(MUL, v[12], v[12], `PAR_STATE_N+44, 30); end
                30: begin calculate(ADD, v[`PAR_STATE_N+43], v[`PAR_STATE_N+44], 33, 31); end
                31: begin calculate(SQRT, v[33], ZERO, 33, 32); end
                32: begin calculate(MUL, v[13], v[13], `PAR_STATE_N+43, 33); end
                33: begin calculate(MUL, v[14], v[14], `PAR_STATE_N+44, 34); end
                34: begin calculate(ADD, v[`PAR_STATE_N+43], v[`PAR_STATE_N+44], `PAR_STATE_N+43, 35); end
                35: begin calculate(MUL, v[15], v[15], `PAR_STATE_N+44, 36); end
                36: begin calculate(ADD, v[`PAR_STATE_N+43], v[`PAR_STATE_N+44], 34, 37); end
                37: begin calculate(SQRT, v[34], ZERO, 34, 38); end
                38: begin if(v[33]<64'h3d719799812dea11 || v[34]<64'h3d719799812dea11)fail(`PAR_CALIB_INVALID);else pc<=39; end
                39: begin calculate(ADD, v[33], v[34], 35, 40); end
                40: begin calculate(DIV, 64'h4000000000000000, v[35], 35, 41); end
                41: begin if(v[18][63] && v[18][62:0]!=0)v[35]<=(v[35] ^ 64'h8000000000000000);pc<=42; end
                42: begin calculate(MUL, v[16], v[35], 30, 43); end
                43: begin calculate(MUL, v[17], v[35], 31, 44); end
                44: begin calculate(MUL, v[18], v[35], 32, 45); end
                45: begin calculate(DIV, ONE, v[33], 36, 46); end
                46: begin if(v[35][63])v[36]<=(v[36] ^ 64'h8000000000000000);pc<=47; end
                47: begin calculate(MUL, v[10], v[36], 20, 48); end
                48: begin calculate(MUL, v[11], v[36], 21, 49); end
                49: begin calculate(MUL, v[12], v[36], 22, 50); end
                // Gram-Schmidt：从第二列减去沿第一列的投影，再归一化
                50: begin calculate(MUL, v[20], v[13], `PAR_STATE_N+43, 51); end
                51: begin calculate(MUL, v[21], v[14], `PAR_STATE_N+44, 52); end
                52: begin calculate(ADD, v[`PAR_STATE_N+43], v[`PAR_STATE_N+44], `PAR_STATE_N+43, 53); end
                53: begin calculate(MUL, v[22], v[15], `PAR_STATE_N+44, 54); end
                54: begin calculate(ADD, v[`PAR_STATE_N+43], v[`PAR_STATE_N+44], 37, 55); end
                55: begin calculate(MUL, v[37], v[20], `PAR_STATE_N+43, 56); end
                56: begin calculate(SUB, v[13], v[`PAR_STATE_N+43], 13, 57); end
                57: begin calculate(MUL, v[37], v[21], `PAR_STATE_N+43, 58); end
                58: begin calculate(SUB, v[14], v[`PAR_STATE_N+43], 14, 59); end
                59: begin calculate(MUL, v[37], v[22], `PAR_STATE_N+43, 60); end
                60: begin calculate(SUB, v[15], v[`PAR_STATE_N+43], 15, 61); end
                61: begin calculate(MUL, v[13], v[13], `PAR_STATE_N+43, 62); end
                62: begin calculate(MUL, v[14], v[14], `PAR_STATE_N+44, 63); end
                63: begin calculate(ADD, v[`PAR_STATE_N+43], v[`PAR_STATE_N+44], `PAR_STATE_N+43, 64); end
                64: begin calculate(MUL, v[15], v[15], `PAR_STATE_N+44, 65); end
                65: begin calculate(ADD, v[`PAR_STATE_N+43], v[`PAR_STATE_N+44], 38, 66); end
                66: begin calculate(SQRT, v[38], ZERO, 38, 67); end
                67: begin if(v[38]<64'h3d719799812dea11 || v[32][63] || v[32][62:0]==0)fail(`PAR_CALIB_INVALID);else pc<=68; end
                68: begin calculate(DIV, ONE, v[38], 39, 69); end
                69: begin if(v[35][63])v[39]<=(v[39] ^ 64'h8000000000000000);pc<=70; end
                70: begin calculate(MUL, v[13], v[39], 23, 71); end
                71: begin calculate(MUL, v[14], v[39], 24, 72); end
                72: begin calculate(MUL, v[15], v[39], 25, 73); end
                73: begin calculate(MUL, v[21], v[25], `PAR_STATE_N+43, 74); end
                74: begin calculate(MUL, v[22], v[24], `PAR_STATE_N+44, 75); end
                75: begin calculate(SUB, v[`PAR_STATE_N+43], v[`PAR_STATE_N+44], 26, 76); end
                76: begin calculate(MUL, v[22], v[23], `PAR_STATE_N+43, 77); end
                77: begin calculate(MUL, v[20], v[25], `PAR_STATE_N+44, 78); end
                78: begin calculate(SUB, v[`PAR_STATE_N+43], v[`PAR_STATE_N+44], 27, 79); end
                79: begin calculate(MUL, v[20], v[24], `PAR_STATE_N+43, 80); end
                80: begin calculate(MUL, v[21], v[23], `PAR_STATE_N+44, 81); end
                81: begin calculate(SUB, v[`PAR_STATE_N+43], v[`PAR_STATE_N+44], 28, 82); end
                // 旋转矩阵转旋转向量，使用稳定四元数分支
                82: begin if(r_ready)pc<=83; end
                83: begin if(r_valid)begin if(r_status!=0)fail(r_status);else begin for(j=0;j<3;j=j+1)v[49+view*6+j]<=r_vec[64*j +:64];v[52+view*6]<=v[30];v[53+view*6]<=v[31];pc<=84;end end end
                84: begin calculate(LOG, v[32], ZERO, 54+view*6, 85); end
                85: begin if(view==(`PAR_VIEWS-1))pc<=RESPONSE;else begin view<=view+1;pc<=5;end end
            default: fail(`PAR_CALIB_INVALID);
        endcase
    end
endmodule
