`include "calib_defs.vh"

/*
作用：稳定的旋转向量/旋转矩阵互转，供初始化、残差与报告使用。
mode=0：Rodrigues rotvec->R，保留小角度分支。
mode=1：R->四元数分支->rotvec，保留接近pi时的稳定分支，禁止简单除sin(theta)。
输入未选中的载荷忽略；成功时两个输出均有效（已知输入透传、未知方向计算）。
本模块只转换旋转，无平移/棋盘尺度；子模块fp_operator。

实现：一个 arithmetic 实例串行复用，所有算术步骤独立舍入。
v[0:2] = rotvec；v[3:11] = 行优先 R；v[12:18] = 标量中间量；
v[40:48] = S；v[49:57] = S*S；v[60:63] = qw,qx,qy,qz。
pc 0..16 求 Rodrigues 系数，17..116 构造矩阵，117 发布正向响应；
118 起计算 trace、选最大对角线分支、统一四元数符号并生成旋转向量。
输入 R 应为旋转矩阵：与 C++ 一致，本模块不做正交性/行列式校验。
任一选中输入或算术中间值非有限时失败，两个结果总线均清零。

共同契约：
- valid && ready 才传输；背压时 valid 和对应全部载荷保持不变。
- 除另有说明，一次一项任务，rsp 被接收前不接受新 cmd；不假设固定延迟。
- cmd_* 随命令锁存；rsp_* 随响应有效。失败仍须发完成响应，不能无声挂起。
- 非零状态通常使结果载荷无效；本模块明确说明的诊断有效位例外。
- 复位取消在途数据；复位后所有生产端 valid 清零。实现时禁止 real 运算。
*/
module rotation #(parameter FP_SHARED=0) (
    // Optional shared FP64 service; local mode keeps standalone compatibility.
    output wire [0:0] shared_req_valid,
    input wire [0:0] shared_req_ready,
    output wire [4:0] shared_req_op,
    output wire [63:0] shared_req_a,
    output wire [63:0] shared_req_b,
    output wire [0:0] shared_active,
    input wire [0:0] shared_rsp_valid,
    output wire [0:0] shared_rsp_ready,
    input wire [63:0] shared_rsp_result,
    input wire [4:0] shared_rsp_flags,

    input wire clk, // core_clk，同一时钟域
    input wire rst_n, // 低有效复位，同步释放；取消全部在途事务
    input wire cmd_valid, // 命令有效
    output wire cmd_ready, // 可接受命令
    input wire cmd_mode, // 0向量到矩阵，1矩阵到向量
    input wire [191:0] cmd_rotvec_fp64, // rx,ry,rz，低位起
    input wire [575:0] cmd_r_fp64, // R行优先
    output wire rsp_valid, // 完成响应有效
    input wire rsp_ready, // 接收完成响应
    output reg [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [191:0] rsp_rotvec_fp64, // 旋转向量
    output wire [575:0] rsp_r_fp64 // 旋转矩阵
);

    // 单事务控制：每条算术指令先请求、再等待；响应背压期间不接新命令。
    // v[] 是工作寄存器，每个使用项在本事务中先写后读；运算顺序对应 C++。
    localparam IDLE=9000, FP_REQ=9001, FP_WAIT=9002, RESPONSE=9003;
    localparam ADD=0, SUB=1, MUL=2, DIV=3, SQRT=4, SIN=5, COS=6,
               ATAN2=7, EXP=8, CONVERT=11;
    localparam [63:0] ZERO=64'b0, ONE=64'h3ff0000000000000, TWO=64'h4000000000000000;
    reg [63:0] v [0:63];
    integer pc, continuation, destination;
    reg [4:0] fp_op;
    reg [63:0] fp_a,fp_b;
    wire fp_ready,fp_valid;
    wire [63:0] fp_result;
    wire [4:0] fp_flags;
    assign cmd_ready=rst_n && pc==IDLE;
    assign rsp_valid=rst_n && pc==RESPONSE;
    function finite;
        input [63:0] x;
        begin finite=(x[62:52]!=11'h7ff); end
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
    fp_operator #(.FP_W(64), .ENABLE_EXP(0), .ENABLE_LOG(0), .ENABLE_SINCOS(1), .ENABLE_ATAN_ACOS(1)) arithmetic(
        .clk(clk),.rst_n(rst_n),.req_valid(rst_n && pc==FP_REQ),
        .req_ready(fp_ready),.req_op(fp_op),.req_a(fp_a),.req_b(fp_b),
        .rsp_valid(fp_valid),.rsp_ready(rst_n && pc==FP_WAIT),
        .rsp_result(fp_result),.rsp_flags(fp_flags),
        .rsp_less(),.rsp_equal(),.rsp_unordered());
assign shared_req_valid[0 +: 1] = 0;
assign shared_req_op[0 +: 5] = 0;
assign shared_req_a[0 +: 64] = 0;
assign shared_req_b[0 +: 64] = 0;
assign shared_active[0 +: 1] = 0;
assign shared_rsp_ready[0 +: 1] = 0;
    end endgenerate

    task calculate;
        input [4:0] operation;
        input [63:0] a,b;
        input integer target,next_pc;
        begin fp_op<=operation;fp_a<=a;fp_b<=b;
            destination<=target;continuation<=next_pc;pc<=FP_REQ;
        end
    endtask
    task fail;
        input [7:0] status;
        begin rsp_status<=status;pc<=RESPONSE;end
    endtask
    assign rsp_rotvec_fp64[0 +: 64]=(rsp_status==0)?v[0]:ZERO;
    assign rsp_rotvec_fp64[64 +: 64]=(rsp_status==0)?v[1]:ZERO;
    assign rsp_rotvec_fp64[128 +: 64]=(rsp_status==0)?v[2]:ZERO;
    assign rsp_r_fp64[0 +: 64]=(rsp_status==0)?v[3]:ZERO;
    assign rsp_r_fp64[64 +: 64]=(rsp_status==0)?v[4]:ZERO;
    assign rsp_r_fp64[128 +: 64]=(rsp_status==0)?v[5]:ZERO;
    assign rsp_r_fp64[192 +: 64]=(rsp_status==0)?v[6]:ZERO;
    assign rsp_r_fp64[256 +: 64]=(rsp_status==0)?v[7]:ZERO;
    assign rsp_r_fp64[320 +: 64]=(rsp_status==0)?v[8]:ZERO;
    assign rsp_r_fp64[384 +: 64]=(rsp_status==0)?v[9]:ZERO;
    assign rsp_r_fp64[448 +: 64]=(rsp_status==0)?v[10]:ZERO;
    assign rsp_r_fp64[512 +: 64]=(rsp_status==0)?v[11]:ZERO;
    localparam INVERSE_START=118;
    // 有限 FP64 的有符号比较；+0 与 -0 视为相等。
    function greater;
        input [63:0] a,b;
        begin
            if(a[62:0]==0 && b[62:0]==0) greater=0;
            else if(a[63]!=b[63]) greater=!a[63];
            else greater=a[63] ? (a[62:0]<b[62:0]) : (a[62:0]>b[62:0]);
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc<=IDLE;rsp_status<=`PAR_OK;
            fp_op<=0;fp_a<=0;fp_b<=0;destination<=0;continuation<=0;

        end else begin
            case(pc)
                IDLE: if(cmd_valid) begin
                    rsp_status<=`PAR_OK;
                    if(cmd_mode) begin v[3]<=cmd_r_fp64[0 +: 64];
                    v[4]<=cmd_r_fp64[64 +: 64];
                    v[5]<=cmd_r_fp64[128 +: 64];
                    v[6]<=cmd_r_fp64[192 +: 64];
                    v[7]<=cmd_r_fp64[256 +: 64];
                    v[8]<=cmd_r_fp64[320 +: 64];
                    v[9]<=cmd_r_fp64[384 +: 64];
                    v[10]<=cmd_r_fp64[448 +: 64];
                    v[11]<=cmd_r_fp64[512 +: 64];pc<=INVERSE_START;end else begin v[0]<=cmd_rotvec_fp64[0 +: 64];
                    v[1]<=cmd_rotvec_fp64[64 +: 64];
                    v[2]<=cmd_rotvec_fp64[128 +: 64];pc<=0;end
                    if(!(finite((cmd_mode ? cmd_r_fp64[0 +: 64] : cmd_rotvec_fp64[0 +: 64])) &&
                        finite((cmd_mode ? cmd_r_fp64[64 +: 64] : cmd_rotvec_fp64[64 +: 64])) &&
                        finite((cmd_mode ? cmd_r_fp64[128 +: 64] : cmd_rotvec_fp64[128 +: 64])) &&
                        finite((cmd_mode ? cmd_r_fp64[192 +: 64] : ZERO)) &&
                        finite((cmd_mode ? cmd_r_fp64[256 +: 64] : ZERO)) &&
                        finite((cmd_mode ? cmd_r_fp64[320 +: 64] : ZERO)) &&
                        finite((cmd_mode ? cmd_r_fp64[384 +: 64] : ZERO)) &&
                        finite((cmd_mode ? cmd_r_fp64[448 +: 64] : ZERO)) &&
                        finite((cmd_mode ? cmd_r_fp64[512 +: 64] : ZERO)))) fail(`PAR_CALIB_INVALID);
                end
                FP_REQ: if(fp_ready) pc<=FP_WAIT;
                FP_WAIT: if(fp_valid) begin
                    // 下溢和非精确允许继续；非法、除零、溢出、非有限值终止。
                    if((|fp_flags[2:0]) || !finite(fp_result)) fail(`PAR_CALIB_INVALID);
                    else begin v[destination]<=fp_result;pc<=continuation;end
                end
                RESPONSE: if(rsp_ready) pc<=IDLE;
                0: begin calculate(MUL, v[0], v[0], 12, 1); end
                1: begin calculate(MUL, v[1], v[1], 13, 2); end
                2: begin calculate(ADD, v[12], v[13], 12, 3); end
                3: begin calculate(MUL, v[2], v[2], 13, 4); end
                // 4: t2=rx²+ry²+rz²
                4: begin calculate(ADD, v[12], v[13], 12, 5); end
                // 5: 小角度 t2<1e-12
                5: begin if(v[12]<64'h3d719799812dea11) pc<=6;else pc<=11; end
                6: begin calculate(DIV, v[12], 64'h4018000000000000, 14, 7); end
                // 7: a=1-t2/6
                7: begin calculate(SUB, ONE, v[14], 14, 8); end
                8: begin calculate(DIV, v[12], 64'h4038000000000000, 15, 9); end
                // 9: b=.5-t2/24
                9: begin calculate(SUB, 64'h3fe0000000000000, v[15], 15, 10); end
                10: begin pc<=17; end
                11: begin calculate(SQRT, v[12], ZERO, 16, 12); end
                12: begin calculate(SIN, v[16], ZERO, 14, 13); end
                13: begin calculate(DIV, v[14], v[16], 14, 14); end
                14: begin calculate(COS, v[16], ZERO, 15, 15); end
                15: begin calculate(SUB, ONE, v[15], 15, 16); end
                16: begin calculate(DIV, v[15], v[12], 15, 17); end
                // 17: 构造反对称矩阵 S
                17: begin v[40]<=ZERO;v[41]<={~v[2][63],v[2][62:0]};v[42]<=v[1];v[43]<=v[2];v[44]<=ZERO;v[45]<={~v[0][63],v[0][62:0]};v[46]<={~v[1][63],v[1][62:0]};v[47]<=v[0];v[48]<=ZERO;pc<=18; end
                // 18: S*S 行 0 列 0
                18: begin v[49]<=ZERO;pc<=19; end
                19: begin calculate(MUL, v[40], v[40], 17, 20); end
                20: begin calculate(ADD, v[49], v[17], 49, 21); end
                21: begin calculate(MUL, v[41], v[43], 17, 22); end
                22: begin calculate(ADD, v[49], v[17], 49, 23); end
                23: begin calculate(MUL, v[42], v[46], 17, 24); end
                24: begin calculate(ADD, v[49], v[17], 49, 25); end
                25: begin calculate(MUL, v[14], v[40], 17, 26); end
                26: begin calculate(ADD, ONE, v[17], 17, 27); end
                27: begin calculate(MUL, v[15], v[49], 18, 28); end
                // 28: R=I+a*S+b*S²
                28: begin calculate(ADD, v[17], v[18], 3, 29); end
                // 29: S*S 行 0 列 1
                29: begin v[50]<=ZERO;pc<=30; end
                30: begin calculate(MUL, v[40], v[41], 17, 31); end
                31: begin calculate(ADD, v[50], v[17], 50, 32); end
                32: begin calculate(MUL, v[41], v[44], 17, 33); end
                33: begin calculate(ADD, v[50], v[17], 50, 34); end
                34: begin calculate(MUL, v[42], v[47], 17, 35); end
                35: begin calculate(ADD, v[50], v[17], 50, 36); end
                36: begin calculate(MUL, v[14], v[41], 17, 37); end
                37: begin calculate(ADD, ZERO, v[17], 17, 38); end
                38: begin calculate(MUL, v[15], v[50], 18, 39); end
                // 39: R=I+a*S+b*S²
                39: begin calculate(ADD, v[17], v[18], 4, 40); end
                // 40: S*S 行 0 列 2
                40: begin v[51]<=ZERO;pc<=41; end
                41: begin calculate(MUL, v[40], v[42], 17, 42); end
                42: begin calculate(ADD, v[51], v[17], 51, 43); end
                43: begin calculate(MUL, v[41], v[45], 17, 44); end
                44: begin calculate(ADD, v[51], v[17], 51, 45); end
                45: begin calculate(MUL, v[42], v[48], 17, 46); end
                46: begin calculate(ADD, v[51], v[17], 51, 47); end
                47: begin calculate(MUL, v[14], v[42], 17, 48); end
                48: begin calculate(ADD, ZERO, v[17], 17, 49); end
                49: begin calculate(MUL, v[15], v[51], 18, 50); end
                // 50: R=I+a*S+b*S²
                50: begin calculate(ADD, v[17], v[18], 5, 51); end
                // 51: S*S 行 1 列 0
                51: begin v[52]<=ZERO;pc<=52; end
                52: begin calculate(MUL, v[43], v[40], 17, 53); end
                53: begin calculate(ADD, v[52], v[17], 52, 54); end
                54: begin calculate(MUL, v[44], v[43], 17, 55); end
                55: begin calculate(ADD, v[52], v[17], 52, 56); end
                56: begin calculate(MUL, v[45], v[46], 17, 57); end
                57: begin calculate(ADD, v[52], v[17], 52, 58); end
                58: begin calculate(MUL, v[14], v[43], 17, 59); end
                59: begin calculate(ADD, ZERO, v[17], 17, 60); end
                60: begin calculate(MUL, v[15], v[52], 18, 61); end
                // 61: R=I+a*S+b*S²
                61: begin calculate(ADD, v[17], v[18], 6, 62); end
                // 62: S*S 行 1 列 1
                62: begin v[53]<=ZERO;pc<=63; end
                63: begin calculate(MUL, v[43], v[41], 17, 64); end
                64: begin calculate(ADD, v[53], v[17], 53, 65); end
                65: begin calculate(MUL, v[44], v[44], 17, 66); end
                66: begin calculate(ADD, v[53], v[17], 53, 67); end
                67: begin calculate(MUL, v[45], v[47], 17, 68); end
                68: begin calculate(ADD, v[53], v[17], 53, 69); end
                69: begin calculate(MUL, v[14], v[44], 17, 70); end
                70: begin calculate(ADD, ONE, v[17], 17, 71); end
                71: begin calculate(MUL, v[15], v[53], 18, 72); end
                // 72: R=I+a*S+b*S²
                72: begin calculate(ADD, v[17], v[18], 7, 73); end
                // 73: S*S 行 1 列 2
                73: begin v[54]<=ZERO;pc<=74; end
                74: begin calculate(MUL, v[43], v[42], 17, 75); end
                75: begin calculate(ADD, v[54], v[17], 54, 76); end
                76: begin calculate(MUL, v[44], v[45], 17, 77); end
                77: begin calculate(ADD, v[54], v[17], 54, 78); end
                78: begin calculate(MUL, v[45], v[48], 17, 79); end
                79: begin calculate(ADD, v[54], v[17], 54, 80); end
                80: begin calculate(MUL, v[14], v[45], 17, 81); end
                81: begin calculate(ADD, ZERO, v[17], 17, 82); end
                82: begin calculate(MUL, v[15], v[54], 18, 83); end
                // 83: R=I+a*S+b*S²
                83: begin calculate(ADD, v[17], v[18], 8, 84); end
                // 84: S*S 行 2 列 0
                84: begin v[55]<=ZERO;pc<=85; end
                85: begin calculate(MUL, v[46], v[40], 17, 86); end
                86: begin calculate(ADD, v[55], v[17], 55, 87); end
                87: begin calculate(MUL, v[47], v[43], 17, 88); end
                88: begin calculate(ADD, v[55], v[17], 55, 89); end
                89: begin calculate(MUL, v[48], v[46], 17, 90); end
                90: begin calculate(ADD, v[55], v[17], 55, 91); end
                91: begin calculate(MUL, v[14], v[46], 17, 92); end
                92: begin calculate(ADD, ZERO, v[17], 17, 93); end
                93: begin calculate(MUL, v[15], v[55], 18, 94); end
                // 94: R=I+a*S+b*S²
                94: begin calculate(ADD, v[17], v[18], 9, 95); end
                // 95: S*S 行 2 列 1
                95: begin v[56]<=ZERO;pc<=96; end
                96: begin calculate(MUL, v[46], v[41], 17, 97); end
                97: begin calculate(ADD, v[56], v[17], 56, 98); end
                98: begin calculate(MUL, v[47], v[44], 17, 99); end
                99: begin calculate(ADD, v[56], v[17], 56, 100); end
                100: begin calculate(MUL, v[48], v[47], 17, 101); end
                101: begin calculate(ADD, v[56], v[17], 56, 102); end
                102: begin calculate(MUL, v[14], v[47], 17, 103); end
                103: begin calculate(ADD, ZERO, v[17], 17, 104); end
                104: begin calculate(MUL, v[15], v[56], 18, 105); end
                // 105: R=I+a*S+b*S²
                105: begin calculate(ADD, v[17], v[18], 10, 106); end
                // 106: S*S 行 2 列 2
                106: begin v[57]<=ZERO;pc<=107; end
                107: begin calculate(MUL, v[46], v[42], 17, 108); end
                108: begin calculate(ADD, v[57], v[17], 57, 109); end
                109: begin calculate(MUL, v[47], v[45], 17, 110); end
                110: begin calculate(ADD, v[57], v[17], 57, 111); end
                111: begin calculate(MUL, v[48], v[48], 17, 112); end
                112: begin calculate(ADD, v[57], v[17], 57, 113); end
                113: begin calculate(MUL, v[14], v[48], 17, 114); end
                114: begin calculate(ADD, ONE, v[17], 17, 115); end
                115: begin calculate(MUL, v[15], v[57], 18, 116); end
                // 116: R=I+a*S+b*S²
                116: begin calculate(ADD, v[17], v[18], 11, 117); end
                // 117: 向量到矩阵结束
                117: begin pc<=RESPONSE; end
                118: begin calculate(ADD, v[3], v[7], 12, 119); end
                // 119: trace
                119: begin calculate(ADD, v[12], v[11], 12, 120); end
                // 120: 选择 trace / 最大对角线四元数分支
                120: begin if(!v[12][63] && v[12][62:0]!=0) pc<=121;else if(greater(v[3],v[7]) && greater(v[3],v[11])) pc<=132;else if(greater(v[7],v[11])) pc<=145;else pc<=158; end
                121: begin calculate(ADD, v[12], ONE, 13, 122); end
                122: begin calculate(SQRT, v[13], ZERO, 13, 123); end
                // 123: s=2*sqrt(...)
                123: begin calculate(MUL, TWO, v[13], 13, 124); end
                124: begin calculate(DIV, v[13], 64'h4010000000000000, 60, 125); end
                125: begin calculate(SUB, v[10], v[8], 61, 126); end
                126: begin calculate(DIV, v[61], v[13], 61, 127); end
                127: begin calculate(SUB, v[5], v[9], 62, 128); end
                128: begin calculate(DIV, v[62], v[13], 62, 129); end
                129: begin calculate(SUB, v[6], v[4], 63, 130); end
                130: begin calculate(DIV, v[63], v[13], 63, 131); end
                131: begin pc<=171; end
                132: begin calculate(ADD, ONE, v[3], 13, 133); end
                133: begin calculate(SUB, v[13], v[7], 13, 134); end
                134: begin calculate(SUB, v[13], v[11], 13, 135); end
                135: begin calculate(SQRT, v[13], ZERO, 13, 136); end
                // 136: s=2*sqrt(...)
                136: begin calculate(MUL, TWO, v[13], 13, 137); end
                137: begin calculate(SUB, v[10], v[8], 60, 138); end
                138: begin calculate(DIV, v[60], v[13], 60, 139); end
                139: begin calculate(DIV, v[13], 64'h4010000000000000, 61, 140); end
                140: begin calculate(ADD, v[4], v[6], 62, 141); end
                141: begin calculate(DIV, v[62], v[13], 62, 142); end
                142: begin calculate(ADD, v[5], v[9], 63, 143); end
                143: begin calculate(DIV, v[63], v[13], 63, 144); end
                144: begin pc<=171; end
                145: begin calculate(ADD, ONE, v[7], 13, 146); end
                146: begin calculate(SUB, v[13], v[3], 13, 147); end
                147: begin calculate(SUB, v[13], v[11], 13, 148); end
                148: begin calculate(SQRT, v[13], ZERO, 13, 149); end
                // 149: s=2*sqrt(...)
                149: begin calculate(MUL, TWO, v[13], 13, 150); end
                150: begin calculate(SUB, v[5], v[9], 60, 151); end
                151: begin calculate(DIV, v[60], v[13], 60, 152); end
                152: begin calculate(ADD, v[4], v[6], 61, 153); end
                153: begin calculate(DIV, v[61], v[13], 61, 154); end
                154: begin calculate(DIV, v[13], 64'h4010000000000000, 62, 155); end
                155: begin calculate(ADD, v[8], v[10], 63, 156); end
                156: begin calculate(DIV, v[63], v[13], 63, 157); end
                157: begin pc<=171; end
                158: begin calculate(ADD, ONE, v[11], 13, 159); end
                159: begin calculate(SUB, v[13], v[3], 13, 160); end
                160: begin calculate(SUB, v[13], v[7], 13, 161); end
                161: begin calculate(SQRT, v[13], ZERO, 13, 162); end
                // 162: s=2*sqrt(...)
                162: begin calculate(MUL, TWO, v[13], 13, 163); end
                163: begin calculate(SUB, v[6], v[4], 60, 164); end
                164: begin calculate(DIV, v[60], v[13], 60, 165); end
                165: begin calculate(ADD, v[5], v[9], 61, 166); end
                166: begin calculate(DIV, v[61], v[13], 61, 167); end
                167: begin calculate(ADD, v[8], v[10], 62, 168); end
                168: begin calculate(DIV, v[62], v[13], 62, 169); end
                169: begin calculate(DIV, v[13], 64'h4010000000000000, 63, 170); end
                170: begin pc<=171; end
                // 171: 统一四元数半球 qw>=0
                171: begin if(v[60][63] && v[60][62:0]!=0) begin v[60]<={~v[60][63],v[60][62:0]};v[61]<={~v[61][63],v[61][62:0]};v[62]<={~v[62][63],v[62][62:0]};v[63]<={~v[63][63],v[63][62:0]};end pc<=172; end
                172: begin calculate(MUL, v[61], v[61], 14, 173); end
                173: begin calculate(MUL, v[62], v[62], 15, 174); end
                174: begin calculate(ADD, v[14], v[15], 14, 175); end
                175: begin calculate(MUL, v[63], v[63], 15, 176); end
                176: begin calculate(ADD, v[14], v[15], 14, 177); end
                // 177: n=norm(qxyz)
                177: begin calculate(SQRT, v[14], ZERO, 14, 178); end
                178: begin if(v[14]>64'h3d719799812dea11) pc<=179;else begin v[15]<=TWO;pc<=182;end end
                179: begin calculate(ATAN2, v[14], v[60], 15, 180); end
                180: begin calculate(MUL, TWO, v[15], 15, 181); end
                181: begin calculate(DIV, v[15], v[14], 15, 182); end
                182: begin calculate(MUL, v[61], v[15], 0, 183); end
                183: begin calculate(MUL, v[62], v[15], 1, 184); end
                184: begin calculate(MUL, v[63], v[15], 2, 185); end
                // 185: 四元数到旋转向量结束
                185: begin pc<=RESPONSE; end
                default: fail(`PAR_CALIB_INVALID);
            endcase
        end
    end
endmodule
