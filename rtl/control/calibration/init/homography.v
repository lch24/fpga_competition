`include "calib_defs.vh"

/*
配置：统一见 rtl/include/calib_config.vh；下文具体计数例子以默认3张5×8为例，
实际容量、循环边界和端口位宽由PAR_*派生，修改配置后须重新编译/综合。
作用：一张视图的归一化DLT，输出行优先3x3单应矩阵。
对象点(X,Y)=(col-3.5,row-2)，单位格，中心为原点。
控制：读取40点并缓存FP64、求均值 -> 扫内部缓存求平均距离 -> 构造归一化对应点及9x9 A^T A
-> 调用jacobi_eigen(n=9) -> 取最小特征向量 -> 反归一化 -> H/H[8]。
内部保存40组FP64坐标、均值/尺度/81项矩阵；成功事务恰好40次外部读取，不修改观测值。
验证角点有限且在图内；距离、特征值秩、H[8]等退化条件沿用C++。
子模块：jacobi_eigen、fp_operator。
距离用sqrt(dx*dx+dy*dy)：输入限制为图内FP32，且宽高最大65535，平方不会溢出。
反归一化跳过稀疏矩阵的零项；浮点舍入允许与软件略有差异，不要求逐位相同。
BAD_CONFIG：宽高<2或view>2；MEM_ERROR：固定返回拍缺valid；CALIB_INVALID：数值/退化错误。
失败响应H清零；复位取消父状态机及子核事务，缓存不清零、下次任务重新写满。

角点读接口例外：只读已提交视图，固定1拍返回，无valid/ready背压。
E_n采样使能/地址，数据在E_n后有效并由调用方在E_(n+1)采样；详见corner_store。
首版最多一笔在途；层间读路由不插入寄存器，返回消费后才能切换所有者。

共同契约：
- 已实现：单事务顺序状态机；FP64 运算核按请求/响应串行复用。
- valid && ready 才传输；背压时 valid 和对应全部载荷保持不变。
- 除另有说明，一次一项任务，rsp 被接收前不接受新 cmd；不假设固定延迟。
- cmd_* 随命令锁存；rsp_* 随响应有效。失败仍须发完成响应，不能无声挂起。
- 非零状态通常使结果载荷无效；本模块明确说明的诊断有效位例外。
- 复位取消在途数据；复位后所有生产端 valid 清零。实现时禁止 real 运算。
*/
module homography #(parameter FP_SHARED=0, SHARE_EIGEN=0) (
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
    input wire [`PAR_VIEW_BITS-1:0] cmd_view_id, // 0..PAR_VIEWS-1
    output wire point_rd_en, // 固定1拍角点读使能，无ready，发起前预留接收空间
    output wire [`PAR_VIEW_BITS-1:0] point_rd_view_id, // 0..PAR_VIEWS-1
    output wire [`PAR_POINT_BITS-1:0] point_rd_index, // 图内角点0..PAR_POINTS-1
    input wire point_rd_valid, // 固定1拍返回，必须当拍消费；无返回背压
    input wire [31:0] point_rd_x_fp32, // 原图x
    input wire [31:0] point_rd_y_fp32, // 原图y
    output wire rsp_valid, // 完成响应有效；最后一笔输出握手后才置位
    input wire rsp_ready, // 接收完成响应
    output reg [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [`PAR_H_W-1:0] rsp_h_fp64 // H[8]=1，576位；仅成功有效
);

    // 工作寄存器映射及各阶段见下方注释；数组不复位，事务内先写后读。
    localparam IDLE=9000, FP_REQ=9001, FP_WAIT=9002, RESPONSE=9003;
    localparam ADD=0,SUB=1,MUL=2,DIV=3,SQRT=4,LOG=9,CONVERT=11;
    localparam [63:0] ZERO=64'b0,ONE=64'h3ff0000000000000;
    reg [63:0] v[0:62];
    integer pc,destination,continuation,j;
    reg [4:0] fp_op;
    reg [63:0] fp_a,fp_b;
    wire fp_ready,fp_valid;
    wire [63:0] fp_result;
    wire [4:0] fp_flags;
    // v[0:1]=W/H,[2:3]=均值,[4:5]=图像/物方距离和,[6:7]=si/sw,
    // [8:9]=当前角点,[10:15]=临时量,[16:19]=归一化x/y/u/v,[40:48]=Hn,[54:62]=H。
    reg [63:0] px[0:`PAR_POINTS-1],py[0:`PAR_POINTS-1],row[0:8];
    integer point,rr,cc,component;reg [`PAR_VIEW_BITS-1:0] view;
    assign point_rd_en=rst_n && pc==0;
    assign point_rd_view_id=view;assign point_rd_index=point[`PAR_POINT_BITS-1:0];
    assign rsp_h_fp64=(rsp_status==0)?{v[62],v[61],v[60],v[59],v[58],v[57],v[56],v[55],v[54]}:576'b0;
    `include "calib_geometry.vh"
    reg [63:0] ata[0:80];
    integer mi;
    // One synchronous read / one write port. Read during the preceding
    // multiply; streaming to eigen uses explicit fetch/send states.
    reg [63:0] ata_q;
    wire [6:0] ata_addr=(pc==87)?mi:rr*9+cc;
    wire [6:0] ata_write_addr=(pc==27)?mi:ata_addr;
    always @(posedge clk) begin
        if(rst_n && (pc==27 || pc==41)) ata[ata_write_addr]<=(pc==27)?64'd0:v[10];
        if(pc==39 || pc==87) ata_q<=ata[ata_addr];
    end
    wire e_ready,e_matrix_ready,e_valid;
    wire [7:0] e_status;
    wire [575:0] e_vector;
    wire [63:0] e_second,e_max;
    eigen_endpoint #(.FP_SHARED(FP_SHARED),.EXTERNAL(SHARE_EIGEN)) eigen(.eigen_rst_n(eigen_rst_n),.eigen_cmd_valid(eigen_cmd_valid),.eigen_cmd_ready(eigen_cmd_ready),.eigen_cmd_n(eigen_cmd_n),.eigen_matrix_valid(eigen_matrix_valid),.eigen_matrix_ready(eigen_matrix_ready),.eigen_matrix_fp64(eigen_matrix_fp64),.eigen_matrix_last(eigen_matrix_last),.eigen_rsp_valid(eigen_rsp_valid),.eigen_rsp_ready(eigen_rsp_ready),.eigen_rsp_status(eigen_rsp_status),.eigen_rsp_min_vector_fp64(eigen_rsp_min_vector_fp64),.eigen_rsp_min_value_fp64(eigen_rsp_min_value_fp64),.eigen_rsp_second_value_fp64(eigen_rsp_second_value_fp64),.eigen_rsp_max_value_fp64(eigen_rsp_max_value_fp64),.eigen_rsp_rotations(eigen_rsp_rotations),.shared_req_valid(shared_req_valid[1 +: 1]),.shared_req_ready(shared_req_ready[1 +: 1]),.shared_req_op(shared_req_op[5 +: 5]),.shared_req_a(shared_req_a[64 +: 64]),.shared_req_b(shared_req_b[64 +: 64]),.shared_active(shared_active[1 +: 1]),.shared_rsp_valid(shared_rsp_valid[1 +: 1]),.shared_rsp_ready(shared_rsp_ready[1 +: 1]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(rst_n),.cmd_valid(rst_n && pc==42),.cmd_ready(e_ready),.cmd_n(4'd9),
        .matrix_valid(rst_n && pc==43),.matrix_ready(e_matrix_ready),.matrix_fp64(ata_q),.matrix_last(mi==80),
        .rsp_valid(e_valid),.rsp_ready(rst_n && pc==44),.rsp_status(e_status),.rsp_min_vector_fp64(e_vector),
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
            point<=0;view<=0;rr<=0;cc<=0;component<=0;mi<=0;
        end else case(pc)
            IDLE: if(cmd_valid) begin
                rsp_status<=0;pc<=0;
                view<=cmd_view_id;point<=0;v[0]<=u16(cmd_width);v[1]<=u16(cmd_height);v[2]<=0;v[3]<=0;v[4]<=0;v[5]<=0;
                if(cmd_width<2 || cmd_height<2 || cmd_view_id>=`PAR_VIEWS)fail(`PAR_BAD_CONFIG);
            end
            FP_REQ: if(fp_ready) pc<=FP_WAIT;
            FP_WAIT: if(fp_valid) begin
                if((|fp_flags[2:0]) || !finite(fp_result)) fail(`PAR_CALIB_INVALID);
                else begin v[destination]<=fp_result;pc<=continuation;end
            end
            RESPONSE: if(rsp_ready) pc<=IDLE;
                // 每点读取一次并缓存FP64：发请求后下一沿采样，无额外等待或背压
                0: begin pc<=1; end
                1: begin if(!point_rd_valid)fail(`PAR_MEM_ERROR);else begin v[8]<={32'b0,point_rd_x_fp32};v[9]<={32'b0,point_rd_y_fp32};pc<=2;end end
                2: begin calculate(CONVERT, v[8], ZERO, 8, 3); end
                3: begin calculate(CONVERT, v[9], ZERO, 9, 4); end
                4: begin if((v[8][63] && v[8][62:0]!=0) || (v[9][63] && v[9][62:0]!=0) || (!v[8][63] && v[8]>=v[0]) || (!v[9][63] && v[9]>=v[1]))fail(`PAR_CALIB_INVALID);else begin px[point]<=v[8];py[point]<=v[9];pc<=5;end end
                5: begin calculate(ADD, v[2], v[8], 2, 6); end
                6: begin calculate(ADD, v[3], v[9], 3, 7); end
                7: begin if(point==(`PAR_POINTS-1))begin point<=0;pc<=8;end else begin point<=point+1;pc<=0;end end
                8: begin calculate(DIV, v[2], u16(`PAR_POINTS), 2, 9); end
                9: begin calculate(DIV, v[3], u16(`PAR_POINTS), 3, 10); end
                // 累计欧氏距离，求Hartley归一化尺度；FP32图内输入不会使平方溢出
                10: begin calculate(SUB, px[point], v[2], 10, 11); end
                11: begin calculate(SUB, py[point], v[3], 11, 12); end
                12: begin calculate(MUL, v[10], v[10], 10, 13); end
                13: begin calculate(MUL, v[11], v[11], 11, 14); end
                14: begin calculate(ADD, v[10], v[11], 10, 15); end
                15: begin calculate(SQRT, v[10], ZERO, 10, 16); end
                16: begin calculate(ADD, v[4], v[10], 4, 17); end
                17: begin calculate(MUL, board_x(point), board_x(point), 10, 18); end
                18: begin calculate(MUL, board_y(point), board_y(point), 11, 19); end
                19: begin calculate(ADD, v[10], v[11], 10, 20); end
                20: begin calculate(SQRT, v[10], ZERO, 10, 21); end
                21: begin calculate(ADD, v[5], v[10], 5, 22); end
                22: begin if(point==(`PAR_POINTS-1))begin point<=0;pc<=23;end else begin point<=point+1;pc<=10;end end
                23: begin if(v[4]<64'h3eb0c6f7a0b5ed8d || v[5]<64'h3eb0c6f7a0b5ed8d)fail(`PAR_CALIB_INVALID);else pc<=86; end
                24: begin calculate(DIV, v[53], v[4], 6, 25); end
                25: begin calculate(DIV, v[53], v[5], 7, 26); end
                26: begin mi<=0;pc<=27; end
                27: begin if(mi==80)pc<=28;else mi<=mi+1; end
                // 构造每点两行DLT约束；先u行再v行，逐元素累加外积
                28: begin calculate(MUL, board_x(point), v[7], 16, 29); end
                29: begin calculate(MUL, board_y(point), v[7], 17, 30); end
                30: begin calculate(SUB, px[point], v[2], 18, 31); end
                31: begin calculate(MUL, v[18], v[6], 18, 32); end
                32: begin calculate(SUB, py[point], v[3], 19, 33); end
                33: begin calculate(MUL, v[19], v[6], 19, 34); end
                34: begin component<=0;pc<=35; end
                35: begin for(j=0;j<9;j=j+1)row[j]<=0;row[component*3]<=(v[16] ^ 64'h8000000000000000);row[component*3+1]<=(v[17] ^ 64'h8000000000000000);row[component*3+2]<=64'hbff0000000000000;row[8]<=component?v[19]:v[18];rr<=0;cc<=0;pc<=36; end
                36: begin calculate(MUL, component?v[19]:v[18], v[16], 10, 37); end
                37: begin calculate(MUL, component?v[19]:v[18], v[17], 11, 38); end
                38: begin row[6]<=v[10];row[7]<=v[11];pc<=39; end
                39: begin calculate(MUL, row[rr], row[cc], 10, 40); end
                40: begin calculate(ADD, ata_q, v[10], 10, 41); end
                41: begin if(cc<8)begin cc<=cc+1;pc<=39;end else if(rr<8)begin cc<=0;rr<=rr+1;pc<=39;end else if(component==0)begin component<=1;pc<=35;end else if(point<(`PAR_POINTS-1))begin point<=point+1;pc<=28;end else pc<=42; end
                // 特征分解：先提交维度，再按行发送完整对称矩阵
                42: begin if(e_ready) begin mi<=0;pc<=87;end end
                87: pc<=43;
                43: begin if(e_matrix_ready) begin if(mi==80) pc<=44;else begin mi<=mi+1;pc<=87;end end end
                44: begin if(e_valid) begin if(e_status!=0) fail(e_status);else begin for(j=0;j<9;j=j+1)v[40+j]<=e_vector[64*j +:64];v[49]<=e_second;v[50]<=e_max;pc<=45;end end end
                45: begin calculate(MUL, v[50], 64'h3ddb7cdfd9d7bdbb, 51, 46); end
                46: begin if(v[49][63] || v[49]<v[51])fail(`PAR_CALIB_INVALID);else pc<=47; end
                // H = T_image^-1 * Hn * T_board；稀疏矩阵按非零项计算
                47: begin calculate(DIV, ONE, v[6], 52, 48); end
                48: begin calculate(MUL, v[52], v[40], 10, 49); end
                49: begin calculate(MUL, v[2], v[46], 11, 50); end
                50: begin calculate(ADD, v[10], v[11], 54, 51); end
                51: begin calculate(MUL, v[54], v[7], 54, 52); end
                52: begin calculate(MUL, v[52], v[41], 10, 53); end
                53: begin calculate(MUL, v[2], v[47], 11, 54); end
                54: begin calculate(ADD, v[10], v[11], 55, 55); end
                55: begin calculate(MUL, v[55], v[7], 55, 56); end
                56: begin calculate(MUL, v[52], v[42], 10, 57); end
                57: begin calculate(MUL, v[2], v[48], 11, 58); end
                58: begin calculate(ADD, v[10], v[11], 56, 59); end
                59: begin calculate(MUL, v[52], v[43], 10, 60); end
                60: begin calculate(MUL, v[3], v[46], 11, 61); end
                61: begin calculate(ADD, v[10], v[11], 57, 62); end
                62: begin calculate(MUL, v[57], v[7], 57, 63); end
                63: begin calculate(MUL, v[52], v[44], 10, 64); end
                64: begin calculate(MUL, v[3], v[47], 11, 65); end
                65: begin calculate(ADD, v[10], v[11], 58, 66); end
                66: begin calculate(MUL, v[58], v[7], 58, 67); end
                67: begin calculate(MUL, v[52], v[45], 10, 68); end
                68: begin calculate(MUL, v[3], v[48], 11, 69); end
                69: begin calculate(ADD, v[10], v[11], 59, 70); end
                70: begin v[60]<=v[46];pc<=71; end
                71: begin calculate(MUL, v[60], v[7], 60, 72); end
                72: begin v[61]<=v[47];pc<=73; end
                73: begin calculate(MUL, v[61], v[7], 61, 74); end
                74: begin v[62]<=v[48];pc<=75; end
                75: begin if((v[62] & 64'h7fffffffffffffff)<64'h3d719799812dea11)fail(`PAR_CALIB_INVALID);else pc<=76; end
                76: begin calculate(DIV, v[54], v[62], 54, 77); end
                77: begin calculate(DIV, v[55], v[62], 55, 78); end
                78: begin calculate(DIV, v[56], v[62], 56, 79); end
                79: begin calculate(DIV, v[57], v[62], 57, 80); end
                80: begin calculate(DIV, v[58], v[62], 58, 81); end
                81: begin calculate(DIV, v[59], v[62], 59, 82); end
                82: begin calculate(DIV, v[60], v[62], 60, 83); end
                83: begin calculate(DIV, v[61], v[62], 61, 84); end
                84: begin calculate(DIV, v[62], v[62], 62, 85); end
                85: begin pc<=RESPONSE; end
                86: begin calculate(MUL, 64'h3ff6a09e667f3bcd, u16(`PAR_POINTS), 53, 24); end
            default: fail(`PAR_CALIB_INVALID);
        endcase
    end
endmodule
