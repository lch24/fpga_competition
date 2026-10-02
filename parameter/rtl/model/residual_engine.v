`include "calib_defs.vh"

/*
配置：统一见 rtl/common/calib_config.vh；下文具体计数例子以默认3张5×8为例，
实际容量、循环边界和端口位宽由PAR_*派生，修改配置后须重新编译/综合。
作用：一个完整状态产生240个重投影残差及cost，是LM反复调用的预测服务。
先检查状态并解码fx/fy/cx/cy；每视图仅算一次Rodrigues R和exp(logtz)。
读取120角点，构造中心化单位格对象点，调用project_point，再减观测值得du/dv。
输出索引t=2*(view*40+point)+component，component=0为du，1为dv；cost=sum(r^2)。
焦距范围、Z>1e-5、有限性按C++检查；早期失败可能已有部分数据，此时rsp失败，
接收者必须丢弃整个向量，不等待补足240拍；成功则全部数据握手后才rsp。
内部算术FP64，FP32观测精确提升FP64。每次命令重新解码状态，不能沿用旧R缓存。
子模块：rotation、project_point、fp_operator；不更新状态和观测值。

角点读接口例外：只读已提交视图，固定1拍返回，无valid/ready背压。
E_n采样使能/地址，数据在E_n后有效并由调用方在E_(n+1)采样；详见corner_store。
首版最多一笔在途；层间读路由不插入寄存器，返回消费后才能切换所有者。

实现：每条命令重新锁存全部状态；不缓存跨命令的 R、tz 或内参。
工作寄存器中，临时区随PAR_STATE_N后移；默认映射为：
v[0:26] = 原始状态；v[27:30] = fx,fy,cx,cy；v[`PAR_STATE_N+4] = 当前视图 tz；
v[32:33] = 提升后的观测；v[40:41] = 预测；v[42:43] = du,dv；
v[44:45] = 平方和临时量；v[`PAR_STATE_N+22] = cost；v[50:51] = 图像宽高 FP64。
pc 0..4 解码内参；5..7 每视图解码外参；8 发读请求，9 收固定一拍返回；
10..11 提升观测精度；12..13 调用投影；14..19 求残差/累加 cost；
20 发送 du/dv，两次握手完成才推进角点或视图。成功响应在最后一次数据握手后。
无角点返回报 PAR_MEM_ERROR；宽高小于 2 报 PAR_BAD_CONFIG；
数值错误报 PAR_CALIB_INVALID，失败 cost 为 +Inf，不补齐已中断的数据流。
只读已提交角点，任务期间上层不得清空/改写 corner_store 或切换读口所有者。

共同契约：
- valid && ready 才传输；背压时 valid 和对应全部载荷保持不变。
- 除另有说明，一次一项任务，rsp 被接收前不接受新 cmd；不假设固定延迟。
- cmd_* 随命令锁存；rsp_* 随响应有效。失败仍须发完成响应，不能无声挂起。
- 非零状态通常使结果载荷无效；本模块明确说明的诊断有效位例外。
- 复位取消在途数据；复位后所有生产端 valid 清零。实现时禁止 real 运算。
*/
module residual_engine (
    input wire clk, // core_clk，同一时钟域
    input wire rst_n, // 低有效复位，同步释放；取消全部在途事务
    input wire cmd_valid, // 命令有效
    output wire cmd_ready, // 可接受命令
    input wire [15:0] cmd_width, // 图像宽度，>=2
    input wire [15:0] cmd_height, // 图像高度，>=2
    input wire [`PAR_STATE_W-1:0] cmd_state, // PAR_STATE_N 项 FP64 状态快照，命令握手时锁存
    output wire point_rd_en, // 固定1拍角点读使能，无ready，发起前预留接收空间
    output wire [`PAR_VIEW_BITS-1:0] point_rd_view_id, // 0..PAR_VIEWS-1
    output wire [`PAR_POINT_BITS-1:0] point_rd_index, // 图内角点0..PAR_POINTS-1
    input wire point_rd_valid, // 固定1拍返回，必须当拍消费；无返回背压
    input wire [31:0] point_rd_x_fp32, // 原图x
    input wire [31:0] point_rd_y_fp32, // 原图y
    output wire data_valid, // 残差流有效
    input wire data_ready, // 下游可接收
    output wire [`PAR_RES_BITS-1:0] data_index, // 0..239
    output wire [63:0] data_fp64, // 预测减观测
    output wire data_last, // 仅239为1
    output wire rsp_valid, // 完成响应有效
    input wire rsp_ready, // 接收完成响应
    output reg [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [63:0] rsp_cost_fp64 // 平方和；失败时+Inf
);

    // 单事务控制：每条算术指令先请求、再等待；响应背压期间不接新命令。
    // v[] 是工作寄存器，每个使用项在本事务中先写后读；运算顺序对应 C++。
    localparam IDLE=9000, FP_REQ=9001, FP_WAIT=9002, RESPONSE=9003;
    localparam ADD=0, SUB=1, MUL=2, DIV=3, SQRT=4, SIN=5, COS=6,
               ATAN2=7, EXP=8, CONVERT=11;
    localparam [63:0] ZERO=64'b0, ONE=64'h3ff0000000000000, TWO=64'h4000000000000000;
    reg [63:0] v [0:`PAR_STATE_N+24];
    integer pc, continuation, destination,j;
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
    reg [`PAR_VIEW_BITS-1:0] view_id;
    reg [`PAR_POINT_BITS-1:0] point_id;
    reg component;
    assign rsp_cost_fp64=(rsp_status==0)?v[`PAR_STATE_N+22]:64'h7ff0000000000000;
    assign point_rd_view_id=view_id;
    assign point_rd_index=point_id;
    assign data_index=(view_id*`PAR_POINTS+point_id)*2+component;
    assign data_last=(data_index==(`PAR_RESIDUALS-1));
    assign data_fp64=component?v[`PAR_STATE_N+16]:v[`PAR_STATE_N+15];
    reg [31:0] observed_x,observed_y;
    reg [575:0] saved_r;
    wire rot_ready,rot_valid,proj_ready,proj_valid;
    wire [7:0] rot_status,proj_status;
    wire [575:0] rot_r;
    wire [63:0] proj_u,proj_v;
    reg [63:0] object_x,object_y;
    assign point_rd_en=rst_n && pc==8;
    assign data_valid=rst_n && pc==20;
    `include "calib_geometry.vh"
    always @* begin object_x=board_x(point_id);object_y=board_y(point_id);end
    function [63:0] uint16_fp;
        input [15:0] value;
        integer i,msb;
        reg [63:0] mantissa;
        reg [10:0] exponent;
        begin
            msb=0;for(i=0;i<16;i=i+1) if(value[i]) msb=i;
            mantissa={48'b0,value} << (52-msb);exponent=1023+msb;
            uint16_fp=(value==0)?64'b0:{1'b0,exponent,mantissa[51:0]};
        end
    endfunction
    rotation rotate(
        .clk(clk),.rst_n(rst_n),.cmd_valid(rst_n && pc==6),.cmd_ready(rot_ready),.cmd_mode(1'b0),
        .cmd_rotvec_fp64({v[11+6*view_id],v[10+6*view_id],v[9+6*view_id]}),.cmd_r_fp64(576'b0),
        .rsp_valid(rot_valid),.rsp_ready(rst_n && pc==7),.rsp_status(rot_status),
        .rsp_rotvec_fp64(),.rsp_r_fp64(rot_r));
    project_point projection(
        .clk(clk),.rst_n(rst_n),.cmd_valid(rst_n && pc==12),.cmd_ready(proj_ready),
        .cmd_x_fp64(object_x),.cmd_y_fp64(object_y),.cmd_r_fp64(saved_r),
        .cmd_t_fp64({v[`PAR_STATE_N+4],v[13+6*view_id],v[12+6*view_id]}),
        .cmd_k_fp64({v[`PAR_STATE_N+3],v[`PAR_STATE_N+2],v[`PAR_STATE_N+1],v[`PAR_STATE_N+0]}),
        .cmd_dist_fp64({v[7],v[6],v[8],v[5],v[4]}),
        .rsp_valid(proj_valid),.rsp_ready(rst_n && pc==13),.rsp_status(proj_status),
        .rsp_u_fp64(proj_u),.rsp_v_fp64(proj_v));
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc<=IDLE;rsp_status<=`PAR_OK;
            fp_op<=0;fp_a<=0;fp_b<=0;destination<=0;continuation<=0;
            view_id<=0;point_id<=0;component<=0;observed_x<=0;observed_y<=0;
        end else begin
            case(pc)
                IDLE: if(cmd_valid) begin
                    rsp_status<=`PAR_OK;
                    for(j=0;j<`PAR_STATE_N;j=j+1)v[j]<=cmd_state[64*j+:64];
                    v[`PAR_STATE_N+23]<=uint16_fp(cmd_width);
                    v[`PAR_STATE_N+24]<=uint16_fp(cmd_height);
                    v[`PAR_STATE_N+22]<=ZERO;view_id<=0;point_id<=0;component<=0;pc<=0;if(cmd_width<2 || cmd_height<2) fail(`PAR_BAD_CONFIG);
                    for(j=0;j<`PAR_STATE_N;j=j+1)if(!finite(cmd_state[64*j+:64]))fail(`PAR_CALIB_INVALID);
                end
                FP_REQ: if(fp_ready) pc<=FP_WAIT;
                FP_WAIT: if(fp_valid) begin
                    // 下溢和非精确允许继续；非法、除零、溢出、非有限值终止。
                    if((|fp_flags[2:0]) || !finite(fp_result)) fail(`PAR_CALIB_INVALID);
                    else begin v[destination]<=fp_result;pc<=continuation;end
                end
                RESPONSE: if(rsp_ready) pc<=IDLE;
                0: begin calculate(EXP, v[0], ZERO, `PAR_STATE_N+0, 1); end
                1: begin calculate(EXP, v[1], ZERO, `PAR_STATE_N+1, 2); end
                // 2: 解码焦距范围检查
                2: begin if(v[`PAR_STATE_N+0][63] || v[`PAR_STATE_N+1][63] || v[`PAR_STATE_N+0]<64'h3f50624dd2f1a9fc || v[`PAR_STATE_N+1]<64'h3f50624dd2f1a9fc || v[`PAR_STATE_N+0]>64'h416312d000000000 || v[`PAR_STATE_N+1]>64'h416312d000000000) fail(`PAR_CALIB_INVALID);else pc<=3; end
                3: begin calculate(MUL, v[2], v[`PAR_STATE_N+23], `PAR_STATE_N+2, 4); end
                // 4: 主点从相对坐标还原
                4: begin calculate(MUL, v[3], v[`PAR_STATE_N+24], `PAR_STATE_N+3, 5); end
                // 5: 每视图解码一次 tz
                5: begin calculate(EXP, v[14+6*view_id], ZERO, `PAR_STATE_N+4, 6); end
                // 6: 每视图计算一次 R
                6: begin if(rot_ready) pc<=7; end
                7: begin if(rot_valid) begin if(rot_status!=0) fail(rot_status);else begin saved_r<=rot_r;pc<=8;end end end
                // 8: 发出同步 RAM 读
                8: begin pc<=9; end
                // 9: 固定一拍返回，当拍锁存
                9: begin if(!point_rd_valid) fail(`PAR_MEM_ERROR);else begin observed_x<=point_rd_x_fp32;observed_y<=point_rd_y_fp32;pc<=10;end end
                10: begin calculate(CONVERT, {32'b0,observed_x}, ZERO, `PAR_STATE_N+5, 11); end
                11: begin calculate(CONVERT, {32'b0,observed_y}, ZERO, `PAR_STATE_N+6, 12); end
                // 12: 单点投影
                12: begin if(proj_ready) pc<=13; end
                13: begin if(proj_valid) begin if(proj_status!=0) fail(proj_status);else begin v[`PAR_STATE_N+13]<=proj_u;v[`PAR_STATE_N+14]<=proj_v;pc<=14;end end end
                // 14: du
                14: begin calculate(SUB, v[`PAR_STATE_N+13], v[`PAR_STATE_N+5], `PAR_STATE_N+15, 15); end
                // 15: dv
                15: begin calculate(SUB, v[`PAR_STATE_N+14], v[`PAR_STATE_N+6], `PAR_STATE_N+16, 16); end
                16: begin calculate(MUL, v[`PAR_STATE_N+15], v[`PAR_STATE_N+15], `PAR_STATE_N+17, 17); end
                17: begin calculate(MUL, v[`PAR_STATE_N+16], v[`PAR_STATE_N+16], `PAR_STATE_N+18, 18); end
                18: begin calculate(ADD, v[`PAR_STATE_N+17], v[`PAR_STATE_N+18], `PAR_STATE_N+17, 19); end
                // 19: cost += du²+dv²；保持 C++ 分组
                19: begin calculate(ADD, v[`PAR_STATE_N+22], v[`PAR_STATE_N+17], `PAR_STATE_N+22, 20); end
                // 20: 先输出 du 再输出 dv，只有握手后推进索引
                20: begin if(data_ready) begin if(!component) component<=1;else begin component<=0;if(point_id==(`PAR_POINTS-1)) begin point_id<=0;if(view_id==(`PAR_VIEWS-1)) pc<=RESPONSE;else begin view_id<=view_id+1'b1;pc<=5;end end else begin point_id<=point_id+1'b1;pc<=8;end end end end
                default: fail(`PAR_CALIB_INVALID);
            endcase
        end
    end
endmodule
