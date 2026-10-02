`include "calib_defs.vh"

/*
作用：公共Brown正向畸变核，参数FP_W仅允许32或64。
r2=x*x+y*y；radial=1+k1*r2+k2*r2^2+k3*r2^3。
xd=x*radial+2*p1*x*y+p2*(r2+2*x*x)；yd=y*radial+p1*(r2+2*y*y)+2*p2*x*y。
输入/输出为归一化坐标，没有内参、访存、插值或反解迭代；有限性异常返回错误。
运算顺序须按对应参考模型冻结；FP32与FP64分别计算，不能仅把结果截断冒充FP32路径。
子模块fp_operator；未来迁入公共目录时接口不变。

实现：串行执行 32 条浮点指令，各步均等待 arithmetic 握手返回；不融合乘加。
v[0:6] = x,y,k1,k2,k3,p1,p2；v[9:11] = x²,y²,r²；
v[12] = radial；v[13:16] = 复用中间量；v[30:31] = xd,yd。
pc 0..11 求半径/径向因子，12..21 求 xd，22..31 求 yd，32 发布响应。
失败返回 PAR_CALIB_INVALID，结果清零；仅 FP_W=32/64 为有效配置。

共同契约：
- valid && ready 才传输；背压时 valid 和对应全部载荷保持不变。
- 除另有说明，一次一项任务，rsp 被接收前不接受新 cmd；不假设固定延迟。
- cmd_* 随命令锁存；rsp_* 随响应有效。失败仍须发完成响应，不能无声挂起。
- 非零状态通常使结果载荷无效；本模块明确说明的诊断有效位例外。
- 复位取消在途数据；复位后所有生产端 valid 清零。实现时禁止 real 运算。
*/
module brown_distort #(
    parameter FP_W = 64
) (
    input wire clk, // core_clk，同一时钟域
    input wire rst_n, // 低有效复位，同步释放；取消全部在途事务
    input wire cmd_valid, // 命令有效
    output wire cmd_ready, // 可接受命令
    input wire [FP_W-1:0] cmd_x, // 归一化x
    input wire [FP_W-1:0] cmd_y, // 归一化y
    input wire [5*FP_W-1:0] cmd_dist, // 低位起k1,k2,k3,p1,p2，各FP_W位
    output wire rsp_valid, // 完成响应有效
    input wire rsp_ready, // 接收完成响应
    output reg [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [FP_W-1:0] rsp_xd, // 畸变归一化x
    output wire [FP_W-1:0] rsp_yd // 畸变归一化y
);

    // 单事务控制：每条算术指令先请求、再等待；响应背压期间不接新命令。
    // v[] 是工作寄存器，每个使用项在本事务中先写后读；运算顺序对应 C++。
    localparam IDLE=9000, FP_REQ=9001, FP_WAIT=9002, RESPONSE=9003;
    localparam ADD=0, SUB=1, MUL=2, DIV=3, SQRT=4, SIN=5, COS=6,
               ATAN2=7, EXP=8, CONVERT=11;
    localparam [FP_W-1:0] ZERO=64'b0, ONE=(FP_W==32 ? 32'h3f800000 : 64'h3ff0000000000000), TWO=(FP_W==32 ? 32'h40000000 : 64'h4000000000000000);
    reg [FP_W-1:0] v [0:31];
    integer pc, continuation, destination;
    reg [4:0] fp_op;
    reg [FP_W-1:0] fp_a,fp_b;
    wire fp_ready,fp_valid;
    wire [FP_W-1:0] fp_result;
    wire [4:0] fp_flags;
    assign cmd_ready=rst_n && pc==IDLE;
    assign rsp_valid=rst_n && pc==RESPONSE;
    function finite;
        input [FP_W-1:0] x;
        begin finite=((FP_W==32 ? ((x >> 23) & 255)!=255 : ((x >> 52) & 2047)!=2047)); end
    endfunction
    fp_operator #(.FP_W(FP_W)) arithmetic(
        .clk(clk),.rst_n(rst_n),.req_valid(rst_n && pc==FP_REQ),
        .req_ready(fp_ready),.req_op(fp_op),.req_a(fp_a),.req_b(fp_b),
        .rsp_valid(fp_valid),.rsp_ready(rst_n && pc==FP_WAIT),
        .rsp_result(fp_result),.rsp_flags(fp_flags),
        .rsp_less(),.rsp_equal(),.rsp_unordered());
    task calculate;
        input [4:0] operation;
        input [FP_W-1:0] a,b;
        input integer target,next_pc;
        begin fp_op<=operation;fp_a<=a;fp_b<=b;
            destination<=target;continuation<=next_pc;pc<=FP_REQ;
        end
    endtask
    task fail;
        input [7:0] status;
        begin rsp_status<=status;pc<=RESPONSE;end
    endtask
    assign rsp_xd=(rsp_status==0)?v[30]:ZERO;
    assign rsp_yd=(rsp_status==0)?v[31]:ZERO;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc<=IDLE;rsp_status<=`PAR_OK;
            fp_op<=0;fp_a<=0;fp_b<=0;destination<=0;continuation<=0;

        end else begin
            case(pc)
                IDLE: if(cmd_valid) begin
                    rsp_status<=`PAR_OK;
                    v[0]<=cmd_x;
                    v[1]<=cmd_y;
                    v[2]<=cmd_dist[0*FP_W +: FP_W];
                    v[3]<=cmd_dist[1*FP_W +: FP_W];
                    v[4]<=cmd_dist[2*FP_W +: FP_W];
                    v[5]<=cmd_dist[3*FP_W +: FP_W];
                    v[6]<=cmd_dist[4*FP_W +: FP_W];pc<=0;
                    if(!(finite(cmd_x) &&
                        finite(cmd_y) &&
                        finite(cmd_dist[0*FP_W +: FP_W]) &&
                        finite(cmd_dist[1*FP_W +: FP_W]) &&
                        finite(cmd_dist[2*FP_W +: FP_W]) &&
                        finite(cmd_dist[3*FP_W +: FP_W]) &&
                        finite(cmd_dist[4*FP_W +: FP_W]))) fail(`PAR_CALIB_INVALID);
                end
                FP_REQ: if(fp_ready) pc<=FP_WAIT;
                FP_WAIT: if(fp_valid) begin
                    // 下溢和非精确允许继续；非法、除零、溢出、非有限值终止。
                    if((|fp_flags[2:0]) || !finite(fp_result)) fail(`PAR_CALIB_INVALID);
                    else begin v[destination]<=fp_result;pc<=continuation;end
                end
                RESPONSE: if(rsp_ready) pc<=IDLE;
                // 0: x*x
                0: begin calculate(MUL, v[0], v[0], 9, 1); end
                // 1: y*y
                1: begin calculate(MUL, v[1], v[1], 10, 2); end
                // 2: r2
                2: begin calculate(ADD, v[9], v[10], 11, 3); end
                3: begin calculate(MUL, v[2], v[11], 12, 4); end
                4: begin calculate(ADD, ONE, v[12], 12, 5); end
                5: begin calculate(MUL, v[3], v[11], 13, 6); end
                6: begin calculate(MUL, v[13], v[11], 13, 7); end
                7: begin calculate(ADD, v[12], v[13], 12, 8); end
                8: begin calculate(MUL, v[4], v[11], 13, 9); end
                9: begin calculate(MUL, v[13], v[11], 13, 10); end
                10: begin calculate(MUL, v[13], v[11], 13, 11); end
                // 11: radial
                11: begin calculate(ADD, v[12], v[13], 12, 12); end
                12: begin calculate(MUL, v[0], v[12], 14, 13); end
                13: begin calculate(MUL, TWO, v[5], 15, 14); end
                14: begin calculate(MUL, v[15], v[0], 15, 15); end
                15: begin calculate(MUL, v[15], v[1], 15, 16); end
                16: begin calculate(ADD, v[14], v[15], 14, 17); end
                17: begin calculate(MUL, TWO, v[0], 16, 18); end
                18: begin calculate(MUL, v[16], v[0], 16, 19); end
                19: begin calculate(ADD, v[11], v[16], 16, 20); end
                20: begin calculate(MUL, v[6], v[16], 16, 21); end
                // 21: xd
                21: begin calculate(ADD, v[14], v[16], 30, 22); end
                22: begin calculate(MUL, v[1], v[12], 14, 23); end
                23: begin calculate(MUL, TWO, v[1], 16, 24); end
                24: begin calculate(MUL, v[16], v[1], 16, 25); end
                25: begin calculate(ADD, v[11], v[16], 16, 26); end
                26: begin calculate(MUL, v[5], v[16], 16, 27); end
                27: begin calculate(ADD, v[14], v[16], 14, 28); end
                28: begin calculate(MUL, TWO, v[6], 15, 29); end
                29: begin calculate(MUL, v[15], v[0], 15, 30); end
                30: begin calculate(MUL, v[15], v[1], 15, 31); end
                // 31: yd
                31: begin calculate(ADD, v[14], v[15], 31, 32); end
                // 32: 结果已寄存
                32: begin pc<=RESPONSE; end
                default: fail(`PAR_CALIB_INVALID);
            endcase
        end
    end
endmodule
