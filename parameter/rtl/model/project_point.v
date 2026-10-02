`include "calib_defs.vh"

/*
作用：单个棋盘平面点完成外参变换、透视除法、Brown畸变和内参投影。
(X,Y,0) -> R*[X,Y,0]+t -> nx=Xc/Zc,ny=Yc/Zc -> Brown -> u=fx*xd+cx,v=fy*yd+cy。
R/t已经解码，本模块不重复Rodrigues；Z<=1e-5或非有限则失败。
子模块：brown_distort(FP_W=64)、fp_operator。按单点调用，无图像、RAM或DDR行为。

实现：每个输入在命令握手时锁存，Brown 子核不接触外部实时输入。
v[0:1] = X,Y；v[2:10] = R；v[11:13] = t；v[14:17] = K；
v[18:22] = k1,k2,k3,p1,p2；v[30:31] 为临时值，v[32:34] = Xc,Yc,Zc；
v[35:36] = nx,ny；v[37:38] = xd,yd；v[40:41] = u,v。
pc 0..11 变换坐标，12 检查深度，13..14 透视除法，
15..16 调用 Brown，17..20 应用内参，21 发布响应。
失败返回 PAR_CALIB_INVALID（或子核状态），结果清零。

共同契约：
- valid && ready 才传输；背压时 valid 和对应全部载荷保持不变。
- 除另有说明，一次一项任务，rsp 被接收前不接受新 cmd；不假设固定延迟。
- cmd_* 随命令锁存；rsp_* 随响应有效。失败仍须发完成响应，不能无声挂起。
- 非零状态通常使结果载荷无效；本模块明确说明的诊断有效位例外。
- 复位取消在途数据；复位后所有生产端 valid 清零。实现时禁止 real 运算。
*/
module project_point (
    input wire clk, // core_clk，同一时钟域
    input wire rst_n, // 低有效复位，同步释放；取消全部在途事务
    input wire cmd_valid, // 命令有效
    output wire cmd_ready, // 可接受命令
    input wire [63:0] cmd_x_fp64, // 中心化单位格X
    input wire [63:0] cmd_y_fp64, // 中心化单位格Y
    input wire [575:0] cmd_r_fp64, // 3x3 R，行优先
    input wire [191:0] cmd_t_fp64, // tx,ty,tz，实际tz非log
    input wire [`PAR_K_W-1:0] cmd_k_fp64, // fx,fy,cx,cy
    input wire [319:0] cmd_dist_fp64, // 低位起k1,k2,k3,p1,p2
    output wire rsp_valid, // 完成响应有效
    input wire rsp_ready, // 接收完成响应
    output reg [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [63:0] rsp_u_fp64, // 预测像素u
    output wire [63:0] rsp_v_fp64 // 预测像素v
);

    // 单事务控制：每条算术指令先请求、再等待；响应背压期间不接新命令。
    // v[] 是工作寄存器，每个使用项在本事务中先写后读；运算顺序对应 C++。
    localparam IDLE=9000, FP_REQ=9001, FP_WAIT=9002, RESPONSE=9003;
    localparam ADD=0, SUB=1, MUL=2, DIV=3, SQRT=4, SIN=5, COS=6,
               ATAN2=7, EXP=8, CONVERT=11;
    localparam [63:0] ZERO=64'b0, ONE=64'h3ff0000000000000, TWO=64'h4000000000000000;
    reg [63:0] v [0:41];
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
    fp_operator #(.FP_W(64)) arithmetic(
        .clk(clk),.rst_n(rst_n),.req_valid(rst_n && pc==FP_REQ),
        .req_ready(fp_ready),.req_op(fp_op),.req_a(fp_a),.req_b(fp_b),
        .rsp_valid(fp_valid),.rsp_ready(rst_n && pc==FP_WAIT),
        .rsp_result(fp_result),.rsp_flags(fp_flags),
        .rsp_less(),.rsp_equal(),.rsp_unordered());
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
    assign rsp_u_fp64=(rsp_status==0)?v[40]:ZERO;
    assign rsp_v_fp64=(rsp_status==0)?v[41]:ZERO;
    wire b_ready,b_valid;
    wire [7:0] b_status;
    wire [63:0] b_x,b_y;
    brown_distort #(.FP_W(64)) distortion(
        .clk(clk),.rst_n(rst_n),.cmd_valid(rst_n && pc==15),.cmd_ready(b_ready),
        .cmd_x(v[35]),.cmd_y(v[36]),.cmd_dist({v[22],v[21],v[20],v[19],v[18]}),
        .rsp_valid(b_valid),.rsp_ready(rst_n && pc==16),.rsp_status(b_status),.rsp_xd(b_x),.rsp_yd(b_y));
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc<=IDLE;rsp_status<=`PAR_OK;
            fp_op<=0;fp_a<=0;fp_b<=0;destination<=0;continuation<=0;

        end else begin
            case(pc)
                IDLE: if(cmd_valid) begin
                    rsp_status<=`PAR_OK;
                    v[0]<=cmd_x_fp64;
                    v[1]<=cmd_y_fp64;
                    v[2]<=cmd_r_fp64[0 +: 64];
                    v[3]<=cmd_r_fp64[64 +: 64];
                    v[4]<=cmd_r_fp64[128 +: 64];
                    v[5]<=cmd_r_fp64[192 +: 64];
                    v[6]<=cmd_r_fp64[256 +: 64];
                    v[7]<=cmd_r_fp64[320 +: 64];
                    v[8]<=cmd_r_fp64[384 +: 64];
                    v[9]<=cmd_r_fp64[448 +: 64];
                    v[10]<=cmd_r_fp64[512 +: 64];
                    v[11]<=cmd_t_fp64[0 +: 64];
                    v[12]<=cmd_t_fp64[64 +: 64];
                    v[13]<=cmd_t_fp64[128 +: 64];
                    v[14]<=cmd_k_fp64[0 +: 64];
                    v[15]<=cmd_k_fp64[64 +: 64];
                    v[16]<=cmd_k_fp64[128 +: 64];
                    v[17]<=cmd_k_fp64[192 +: 64];
                    v[18]<=cmd_dist_fp64[0 +: 64];
                    v[19]<=cmd_dist_fp64[64 +: 64];
                    v[20]<=cmd_dist_fp64[128 +: 64];
                    v[21]<=cmd_dist_fp64[192 +: 64];
                    v[22]<=cmd_dist_fp64[256 +: 64];pc<=0;
                    if(!(finite(cmd_x_fp64) &&
                        finite(cmd_y_fp64) &&
                        finite(cmd_r_fp64[0 +: 64]) &&
                        finite(cmd_r_fp64[64 +: 64]) &&
                        finite(cmd_r_fp64[128 +: 64]) &&
                        finite(cmd_r_fp64[192 +: 64]) &&
                        finite(cmd_r_fp64[256 +: 64]) &&
                        finite(cmd_r_fp64[320 +: 64]) &&
                        finite(cmd_r_fp64[384 +: 64]) &&
                        finite(cmd_r_fp64[448 +: 64]) &&
                        finite(cmd_r_fp64[512 +: 64]) &&
                        finite(cmd_t_fp64[0 +: 64]) &&
                        finite(cmd_t_fp64[64 +: 64]) &&
                        finite(cmd_t_fp64[128 +: 64]) &&
                        finite(cmd_k_fp64[0 +: 64]) &&
                        finite(cmd_k_fp64[64 +: 64]) &&
                        finite(cmd_k_fp64[128 +: 64]) &&
                        finite(cmd_k_fp64[192 +: 64]) &&
                        finite(cmd_dist_fp64[0 +: 64]) &&
                        finite(cmd_dist_fp64[64 +: 64]) &&
                        finite(cmd_dist_fp64[128 +: 64]) &&
                        finite(cmd_dist_fp64[192 +: 64]) &&
                        finite(cmd_dist_fp64[256 +: 64]))) fail(`PAR_CALIB_INVALID);
                end
                FP_REQ: if(fp_ready) pc<=FP_WAIT;
                FP_WAIT: if(fp_valid) begin
                    // 下溢和非精确允许继续；非法、除零、溢出、非有限值终止。
                    if((|fp_flags[2:0]) || !finite(fp_result)) fail(`PAR_CALIB_INVALID);
                    else begin v[destination]<=fp_result;pc<=continuation;end
                end
                RESPONSE: if(rsp_ready) pc<=IDLE;
                0: begin calculate(MUL, v[2], v[0], 30, 1); end
                1: begin calculate(MUL, v[3], v[1], 31, 2); end
                2: begin calculate(ADD, v[30], v[31], 30, 3); end
                // 3: 相机坐标行 0
                3: begin calculate(ADD, v[30], v[11], 32, 4); end
                4: begin calculate(MUL, v[5], v[0], 30, 5); end
                5: begin calculate(MUL, v[6], v[1], 31, 6); end
                6: begin calculate(ADD, v[30], v[31], 30, 7); end
                // 7: 相机坐标行 1
                7: begin calculate(ADD, v[30], v[12], 33, 8); end
                8: begin calculate(MUL, v[8], v[0], 30, 9); end
                9: begin calculate(MUL, v[9], v[1], 31, 10); end
                10: begin calculate(ADD, v[30], v[31], 30, 11); end
                // 11: 相机坐标行 2
                11: begin calculate(ADD, v[30], v[13], 34, 12); end
                // 12: Z 必须严格大于 1e-5
                12: begin if(v[34][63] || v[34]<=64'h3ee4f8b588e368f1) fail(`PAR_CALIB_INVALID); else pc<=13; end
                // 13: nx
                13: begin calculate(DIV, v[32], v[34], 35, 14); end
                // 14: ny
                14: begin calculate(DIV, v[33], v[34], 36, 15); end
                // 15: 请求 Brown 畸变
                15: begin if(b_ready) pc<=16; end
                // 16: 接收畸变
                16: begin if(b_valid) begin if(b_status!=0) fail(b_status); else begin v[37]<=b_x;v[38]<=b_y;pc<=17;end end end
                17: begin calculate(MUL, v[14], v[37], 40, 18); end
                // 18: u=fx*xd+cx
                18: begin calculate(ADD, v[40], v[16], 40, 19); end
                19: begin calculate(MUL, v[15], v[38], 41, 20); end
                // 20: v=fy*yd+cy
                20: begin calculate(ADD, v[41], v[17], 41, 21); end
                21: begin pc<=RESPONSE; end
                default: fail(`PAR_CALIB_INVALID);
            endcase
        end
    end
endmodule
