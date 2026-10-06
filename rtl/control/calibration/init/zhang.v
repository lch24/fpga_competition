`include "calib_defs.vh"

/*
配置：统一见 rtl/include/calib_config.vh；下文具体计数例子以默认3张5×8为例，
实际容量、循环边界和端口位宽由PAR_*派生，修改配置后须重新编译/综合。
作用：由三张H估计fx/fy/cx/cy；不求畸变系数和外参。
按C++图像归一化构造v12及v11-v22，累加6x6约束矩阵，调用Jacobi(n=6)。
恢复内参，检查正定性、分母与范围。失败由上层跳过Zhang seed，不终止所有标定。
子模块：jacobi_eigen、fp_operator；内部36项矩阵。
命令时锁存全部27项H；内部先清矩阵、按三视图顺序累加两组约束，再恢复K。
输出按低位起fx,fy,cx,cy打包；计算中的skew只用于求cx，不是输出参数。
BAD_CONFIG：宽高<2；CALIB_INVALID：非有限、非正定、算术异常或内参超范围。
失败K清零；不额外添加软件没有的秩判断，以免改变固定焦距回退的触发规则。

共同契约：
- 已实现：单事务顺序状态机；FP64 运算核按请求/响应串行复用。
- valid && ready 才传输；背压时 valid 和对应全部载荷保持不变。
- 除另有说明，一次一项任务，rsp 被接收前不接受新 cmd；不假设固定延迟。
- cmd_* 随命令锁存；rsp_* 随响应有效。失败仍须发完成响应，不能无声挂起。
- 非零状态通常使结果载荷无效；本模块明确说明的诊断有效位例外。
- 复位取消在途数据；复位后所有生产端 valid 清零。实现时禁止 real 运算。
*/
module zhang #(parameter FP_SHARED=0, SHARE_EIGEN=0) (
    output wire  eigen_rst_n,
    output wire  eigen_cmd_valid,
    input wire  eigen_cmd_ready,
    output wire [3:0] eigen_cmd_n,
    output wire  eigen_matrix_valid,
    input wire  eigen_matrix_ready,
    output wire [63:0] eigen_matrix_fp64,
    output wire  eigen_matrix_last,
    input wire  eigen_rsp_valid,
    output wire  eigen_rsp_ready,
    input wire [7:0] eigen_rsp_status,
    input wire [575:0] eigen_rsp_min_vector_fp64,
    input wire [63:0] eigen_rsp_min_value_fp64,
    input wire [63:0] eigen_rsp_second_value_fp64,
    input wire [63:0] eigen_rsp_max_value_fp64,
    input wire [13:0] eigen_rsp_rotations,
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
    input wire [`PAR_H_ALL_W-1:0] cmd_h_all_fp64, // 三张H，view0在低位
    output wire rsp_valid, // 完成响应有效；最后一笔输出握手后才置位
    input wire rsp_ready, // 接收完成响应
    output reg [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [`PAR_K_W-1:0] rsp_k_fp64 // fx,fy,cx,cy，各FP64
);

    // 工作寄存器映射及各阶段见下方注释；数组不复位，事务内先写后读。
    localparam IDLE=9000, FP_REQ=9001, FP_WAIT=9002, RESPONSE=9003;
    localparam ADD=0,SUB=1,MUL=2,DIV=3,SQRT=4,LOG=9,CONVERT=11;
    localparam [63:0] ZERO=64'b0,ONE=64'h3ff0000000000000;
    reg [63:0] v[0:76];
    integer pc,destination,continuation,j;
    reg [4:0] fp_op;
    reg [63:0] fp_a,fp_b;
    wire fp_ready,fp_valid;
    wire [63:0] fp_result;
    wire [4:0] fp_flags;
    // v[0:1]=W/H,[2:4]=图像归一化参数,[10:18]=归一化H,
    // [20:37]=v12/v11/v22,[50:55]=绝对二次曲线b,[60:68]=内参恢复临时量,[70:73]=K。
    reg [63:0] hom[0:9*`PAR_VIEWS-1];integer view,rr,cc,component;
    assign rsp_k_fp64=(rsp_status==0)?{v[73],v[72],v[71],v[70]}:256'b0;
    reg [63:0] ata[0:35];
    integer mi;
    // Single synchronous read port, prefetched during the outer-product multiply.
    reg [63:0] ata_q;
    wire [5:0] ata_addr=(pc==117)?mi:rr*6+cc;
    wire [5:0] ata_write_addr=(pc==0)?mi:ata_addr;
    always @(posedge clk) begin
        if(rst_n && (pc==0 || pc==70)) ata[ata_write_addr]<=(pc==0)?64'd0:v[74];
        if(pc==68 || pc==117) ata_q<=ata[ata_addr];
    end
    wire e_ready,e_matrix_ready,e_valid;
    wire [7:0] e_status;
    wire [575:0] e_vector;
    wire [63:0] e_second,e_max;
    eigen_endpoint #(.FP_SHARED(FP_SHARED),.EXTERNAL(SHARE_EIGEN)) eigen(.eigen_rst_n(eigen_rst_n),.eigen_cmd_valid(eigen_cmd_valid),.eigen_cmd_ready(eigen_cmd_ready),.eigen_cmd_n(eigen_cmd_n),.eigen_matrix_valid(eigen_matrix_valid),.eigen_matrix_ready(eigen_matrix_ready),.eigen_matrix_fp64(eigen_matrix_fp64),.eigen_matrix_last(eigen_matrix_last),.eigen_rsp_valid(eigen_rsp_valid),.eigen_rsp_ready(eigen_rsp_ready),.eigen_rsp_status(eigen_rsp_status),.eigen_rsp_min_vector_fp64(eigen_rsp_min_vector_fp64),.eigen_rsp_min_value_fp64(eigen_rsp_min_value_fp64),.eigen_rsp_second_value_fp64(eigen_rsp_second_value_fp64),.eigen_rsp_max_value_fp64(eigen_rsp_max_value_fp64),.eigen_rsp_rotations(eigen_rsp_rotations),.shared_req_valid(shared_req_valid[1 +: 1]),.shared_req_ready(shared_req_ready[1 +: 1]),.shared_req_op(shared_req_op[5 +: 5]),.shared_req_a(shared_req_a[64 +: 64]),.shared_req_b(shared_req_b[64 +: 64]),.shared_active(shared_active[1 +: 1]),.shared_rsp_valid(shared_rsp_valid[1 +: 1]),.shared_rsp_ready(shared_rsp_ready[1 +: 1]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(rst_n),.cmd_valid(rst_n && pc==71),.cmd_ready(e_ready),.cmd_n(4'd6),
        .matrix_valid(rst_n && pc==72),.matrix_ready(e_matrix_ready),.matrix_fp64(ata_q),.matrix_last(mi==35),
        .rsp_valid(e_valid),.rsp_ready(rst_n && pc==73),.rsp_status(e_status),.rsp_min_vector_fp64(e_vector),
        .rsp_min_value_fp64(),.rsp_second_value_fp64(e_second),.rsp_max_value_fp64(e_max),.rsp_rotations());

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
    fp_operator #(.FP_W(64), .ENABLE_EXP(0), .ENABLE_LOG(0), .ENABLE_SINCOS(0), .ENABLE_ATAN_ACOS(0)) arithmetic(.clk(clk),.rst_n(rst_n),
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
            view<=0;rr<=0;cc<=0;component<=0;mi<=0;
        end else case(pc)
            IDLE: if(cmd_valid) begin
                rsp_status<=0;pc<=0;
                view<=0;mi<=0;v[0]<=u16(cmd_width);v[1]<=u16(cmd_height);
                for(j=0;j<9*`PAR_VIEWS;j=j+1)begin hom[j]<=cmd_h_all_fp64[64*j +:64];if(!finite(cmd_h_all_fp64[64*j +:64]))fail(`PAR_CALIB_INVALID);end
                if(cmd_width<2 || cmd_height<2)fail(`PAR_BAD_CONFIG);
            end
            FP_REQ: if(fp_ready) pc<=FP_WAIT;
            FP_WAIT: if(fp_valid) begin
                if((|fp_flags[2:0]) || !finite(fp_result)) fail(`PAR_CALIB_INVALID);
                else begin v[destination]<=fp_result;pc<=continuation;end
            end
            RESPONSE: if(rsp_ready) pc<=IDLE;
                0: begin if(mi==35)pc<=1;else mi<=mi+1; end
                1: begin calculate(DIV, ONE, v[0], 2, 2); end
                2: begin calculate(MUL, 64'h4000000000000000, v[0], 3, 3); end
                3: begin calculate(DIV, v[1], v[3], 3, 4); end
                // 按宽度归一化图像坐标，改善约束矩阵数值尺度
                4: begin calculate(MUL, v[2], hom[view*9+0], 74, 5); end
                5: begin calculate(MUL, 64'hbfe0000000000000, hom[view*9+6], 75, 6); end
                6: begin calculate(ADD, v[74], v[75], 10, 7); end
                7: begin calculate(MUL, v[2], hom[view*9+3], 74, 8); end
                8: begin calculate(MUL, (v[3] ^ 64'h8000000000000000), hom[view*9+6], 75, 9); end
                9: begin calculate(ADD, v[74], v[75], 13, 10); end
                10: begin v[16]<=hom[view*9+6];pc<=11; end
                11: begin calculate(MUL, v[2], hom[view*9+1], 74, 12); end
                12: begin calculate(MUL, 64'hbfe0000000000000, hom[view*9+7], 75, 13); end
                13: begin calculate(ADD, v[74], v[75], 11, 14); end
                14: begin calculate(MUL, v[2], hom[view*9+4], 74, 15); end
                15: begin calculate(MUL, (v[3] ^ 64'h8000000000000000), hom[view*9+7], 75, 16); end
                16: begin calculate(ADD, v[74], v[75], 14, 17); end
                17: begin v[17]<=hom[view*9+7];pc<=18; end
                18: begin calculate(MUL, v[2], hom[view*9+2], 74, 19); end
                19: begin calculate(MUL, 64'hbfe0000000000000, hom[view*9+8], 75, 20); end
                20: begin calculate(ADD, v[74], v[75], 12, 21); end
                21: begin calculate(MUL, v[2], hom[view*9+5], 74, 22); end
                22: begin calculate(MUL, (v[3] ^ 64'h8000000000000000), hom[view*9+8], 75, 23); end
                23: begin calculate(ADD, v[74], v[75], 15, 24); end
                24: begin v[18]<=hom[view*9+8];pc<=25; end
                25: begin calculate(MUL, v[10], v[11], 20, 26); end
                26: begin calculate(MUL, v[10], v[14], 74, 27); end
                27: begin calculate(MUL, v[13], v[11], 75, 28); end
                28: begin calculate(ADD, v[74], v[75], 21, 29); end
                29: begin calculate(MUL, v[13], v[14], 22, 30); end
                30: begin calculate(MUL, v[16], v[11], 74, 31); end
                31: begin calculate(MUL, v[10], v[17], 75, 32); end
                32: begin calculate(ADD, v[74], v[75], 23, 33); end
                33: begin calculate(MUL, v[16], v[14], 74, 34); end
                34: begin calculate(MUL, v[13], v[17], 75, 35); end
                35: begin calculate(ADD, v[74], v[75], 24, 36); end
                36: begin calculate(MUL, v[16], v[17], 25, 37); end
                37: begin calculate(MUL, v[10], v[10], 26, 38); end
                38: begin calculate(MUL, v[10], v[13], 74, 39); end
                39: begin calculate(MUL, v[13], v[10], 75, 40); end
                40: begin calculate(ADD, v[74], v[75], 27, 41); end
                41: begin calculate(MUL, v[13], v[13], 28, 42); end
                42: begin calculate(MUL, v[16], v[10], 74, 43); end
                43: begin calculate(MUL, v[10], v[16], 75, 44); end
                44: begin calculate(ADD, v[74], v[75], 29, 45); end
                45: begin calculate(MUL, v[16], v[13], 74, 46); end
                46: begin calculate(MUL, v[13], v[16], 75, 47); end
                47: begin calculate(ADD, v[74], v[75], 30, 48); end
                48: begin calculate(MUL, v[16], v[16], 31, 49); end
                49: begin calculate(MUL, v[11], v[11], 32, 50); end
                50: begin calculate(MUL, v[11], v[14], 74, 51); end
                51: begin calculate(MUL, v[14], v[11], 75, 52); end
                52: begin calculate(ADD, v[74], v[75], 33, 53); end
                53: begin calculate(MUL, v[14], v[14], 34, 54); end
                54: begin calculate(MUL, v[17], v[11], 74, 55); end
                55: begin calculate(MUL, v[11], v[17], 75, 56); end
                56: begin calculate(ADD, v[74], v[75], 35, 57); end
                57: begin calculate(MUL, v[17], v[14], 74, 58); end
                58: begin calculate(MUL, v[14], v[17], 75, 59); end
                59: begin calculate(ADD, v[74], v[75], 36, 60); end
                60: begin calculate(MUL, v[17], v[17], 37, 61); end
                61: begin calculate(SUB, v[26], v[32], 26, 62); end
                62: begin calculate(SUB, v[27], v[33], 27, 63); end
                63: begin calculate(SUB, v[28], v[34], 28, 64); end
                64: begin calculate(SUB, v[29], v[35], 29, 65); end
                65: begin calculate(SUB, v[30], v[36], 30, 66); end
                66: begin calculate(SUB, v[31], v[37], 31, 67); end
                67: begin rr<=0;cc<=0;component<=0;pc<=68; end
                // 每视图累加v12和(v11-v22)的外积，顺序对应软件
                68: begin calculate(MUL, v[20+component*6+rr], v[20+component*6+cc], 74, 69); end
                69: begin calculate(ADD, ata_q, v[74], 74, 70); end
                70: begin if(cc<5)begin cc<=cc+1;pc<=68;end else if(rr<5)begin cc<=0;rr<=rr+1;pc<=68;end else if(component==0)begin component<=1;rr<=0;cc<=0;pc<=68;end else if(view<(`PAR_VIEWS-1))begin view<=view+1;pc<=4;end else pc<=71; end
                // 特征分解：先提交维度，再按行发送完整对称矩阵
                71: begin if(e_ready) begin mi<=0;pc<=117;end end
                117: pc<=72;
                72: begin if(e_matrix_ready) begin if(mi==35) pc<=73;else begin mi<=mi+1;pc<=117;end end end
                73: begin if(e_valid) begin if(e_status!=0) fail(e_status);else begin for(j=0;j<6;j=j+1)v[50+j]<=e_vector[64*j +:64];v[56]<=e_second;v[57]<=e_max;pc<=74;end end end
                74: begin if(v[50][63] && v[50][62:0]!=0)for(j=0;j<6;j=j+1)v[50+j]<=(v[50+j] ^ 64'h8000000000000000);pc<=75; end
                // 恢复cy、lambda、fx/fy、skew、cx；skew仅辅助恢复cx，不进入输出模型
                75: begin calculate(MUL, v[50], v[52], 74, 76); end
                76: begin calculate(MUL, v[51], v[51], 75, 77); end
                77: begin calculate(SUB, v[74], v[75], 60, 78); end
                78: begin if(v[50][63] || v[50][62:0]==0 || v[60][63] || v[60]<=64'h3d06849b86a12b9b)fail(`PAR_CALIB_INVALID);else pc<=79; end
                79: begin calculate(MUL, v[51], v[53], 74, 80); end
                80: begin calculate(MUL, v[50], v[54], 75, 81); end
                81: begin calculate(SUB, v[74], v[75], 61, 82); end
                82: begin calculate(DIV, v[61], v[60], 62, 83); end
                83: begin calculate(MUL, v[53], v[53], 74, 84); end
                84: begin calculate(MUL, v[62], v[61], 75, 85); end
                85: begin calculate(ADD, v[74], v[75], 74, 86); end
                86: begin calculate(DIV, v[74], v[50], 74, 87); end
                87: begin calculate(SUB, v[55], v[74], 63, 88); end
                88: begin if(v[63][63] || v[63][62:0]==0)fail(`PAR_CALIB_INVALID);else pc<=89; end
                89: begin calculate(DIV, v[63], v[50], 64, 90); end
                90: begin calculate(SQRT, v[64], ZERO, 64, 91); end
                91: begin calculate(MUL, v[63], v[50], 65, 92); end
                92: begin calculate(DIV, v[65], v[60], 65, 93); end
                93: begin calculate(SQRT, v[65], ZERO, 65, 94); end
                94: begin calculate(MUL, (v[51] ^ 64'h8000000000000000), v[64], 66, 95); end
                95: begin calculate(MUL, v[66], v[64], 66, 96); end
                96: begin calculate(MUL, v[66], v[65], 66, 97); end
                97: begin calculate(DIV, v[66], v[63], 66, 98); end
                98: begin calculate(MUL, v[66], v[62], 74, 99); end
                99: begin calculate(DIV, v[74], v[65], 74, 100); end
                100: begin calculate(MUL, v[53], v[64], 75, 101); end
                101: begin calculate(MUL, v[75], v[64], 75, 102); end
                102: begin calculate(DIV, v[75], v[63], 75, 103); end
                103: begin calculate(SUB, v[74], v[75], 67, 104); end
                104: begin calculate(MUL, v[64], v[0], 70, 105); end
                105: begin calculate(MUL, v[65], v[0], 71, 106); end
                106: begin calculate(MUL, v[67], v[0], 72, 107); end
                107: begin calculate(MUL, v[0], 64'h3fe0000000000000, 74, 108); end
                108: begin calculate(ADD, v[72], v[74], 72, 109); end
                109: begin calculate(MUL, v[62], v[0], 73, 110); end
                110: begin calculate(MUL, v[1], 64'h3fe0000000000000, 75, 111); end
                111: begin calculate(ADD, v[73], v[75], 73, 112); end
                // 严格执行软件内参范围，主点允许在图像外但离中心不能超过宽/高
                112: begin calculate(MUL, v[0], 64'h3fa999999999999a, 60, 113); end
                113: begin calculate(MUL, v[0], 64'h4034000000000000, 61, 114); end
                114: begin calculate(SUB, v[72], v[74], 62, 115); end
                115: begin calculate(SUB, v[73], v[75], 63, 116); end
                116: begin if(v[70][63] || v[71][63] || v[70]<=v[60] || v[71]<=v[60] || v[70]>=v[61] || v[71]>=v[61] || (v[62] & 64'h7fffffffffffffff)>=v[0] || (v[63] & 64'h7fffffffffffffff)>=v[1])fail(`PAR_CALIB_INVALID);else pc<=RESPONSE; end
            default: fail(`PAR_CALIB_INVALID);
        endcase
    end
endmodule
