`include "calib_defs.vh"

/*
配置：统一见 rtl/common/calib_config.vh；下文具体计数例子以默认3张5×8为例，
实际容量、循环边界和端口位宽由PAR_*派生，修改配置后须重新编译/综合。
作用：中心差分生成列归一化雅可比和scale，不负责接受/拒绝LM试探。
逐活动参数k计算h=1e-6*(1+abs(p[index]))，依次调用外部残差服务。
正扰动q=p+h；负扰动在q上减2*h，保留C++先加后减的FP64舍入顺序。
缓存两组`PAR_RESIDUALS项残差/差分列，scale=1/max(sqrt(sum(column^2)),1e-12)。
每列完整后才输出缩放J，顺序为列k外层、残差行t内层。
全部输出握手后rsp带26槽scale；stage未使用的高槽置零。
父模块将J流接normal_equation；本模块不保有全量J，也不直接读角点。

共同契约：
- 已实现串行FP64控制；数组不复位，每次使用前完整写入。尚未进行综合/时序验证。
- valid && ready 才传输；背压时 valid 和对应全部载荷保持不变。
- 除另有说明，一次一项任务，rsp 被接收前不接受新 cmd；不假设固定延迟。
- cmd_* 随命令锁存；rsp_* 随响应有效。失败仍须发完成响应，不能无声挂起。
- 非零状态通常使结果载荷无效；本模块明确说明的诊断有效位例外。
- 复位取消在途数据；复位后所有生产端 valid 清零。实现时禁止 real 运算。
*/
module jacobian (
    input wire clk, // core_clk，同一时钟域
    input wire rst_n, // 低有效复位，同步释放；取消全部在途事务
    input wire cmd_valid, // 命令有效
    output wire cmd_ready, // 可接受命令
    input wire [`PAR_STATE_W-1:0] cmd_state, // PAR_STATE_N 项 FP64 状态快照，命令握手时锁存
    input wire [1:0] cmd_stage, // 决定活动列及参数映射
    output wire eval_cmd_valid, // 请求父模块的残差服务
    input wire eval_cmd_ready, // 服务可接收
    output wire [`PAR_STATE_W-1:0] eval_cmd_state, // 正/负扰动状态；尺寸沿用当前LM命令
    input wire eval_data_valid, // 残差流有效
    output wire eval_data_ready, // 可缓存残差
    input wire [`PAR_RES_BITS-1:0] eval_data_index, // 0..(`PAR_RESIDUALS-1)
    input wire [63:0] eval_data_fp64, // 残差值
    input wire eval_data_last, // 仅(`PAR_RESIDUALS-1)为1
    input wire eval_rsp_valid, // 本次残差服务完成，可能提前失败
    output wire eval_rsp_ready, // 可接收完成
    input wire [7:0] eval_rsp_status, // 0成功；非零终止本列，废弃部分数据
    input wire [63:0] eval_rsp_cost_fp64, // 本次cost；本模块主要使用残差
    output wire j_valid, // 缩放后的J元素
    input wire j_ready, // 下游缓存可接收
    output wire [`PAR_RES_BITS-1:0] j_row, // 残差索引0..(`PAR_RESIDUALS-1)
    output wire [`PAR_COL_BITS-1:0] j_col, // 活动列0..n-1
    output wire [63:0] j_fp64, // J[row,col]*scale[col]
    output wire j_last, // 最后列的第(`PAR_RESIDUALS-1)行
    output wire rsp_valid, // 完成响应有效；最后一笔输出握手后才置位
    input wire rsp_ready, // 接收完成响应
    output reg [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [`PAR_SCALE_W-1:0] rsp_scales_fp64 // 26个FP64，按活动列顺序
);



    localparam IDLE=9000, FP_REQ=9001, FP_WAIT=9002, RESPONSE=9003;
    localparam ADD=0,SUB=1,MUL=2,DIV=3,SQRT=4;
    localparam [63:0] ZERO=0,ONE=64'h3ff0000000000000,TWO=64'h4000000000000000;
    integer pc,continuation,destination;
    reg [63:0] v[0:15];
    reg [4:0] fp_op; reg [63:0] fp_a,fp_b;
    wire fp_ready,fp_valid;wire [63:0] fp_result;wire [4:0] fp_flags;
    // 内部取消用寄存器驱动，避免多位状态译码毛刺进入子核异步复位。
    reg child_clear;
    wire local_rst_n=rst_n && !child_clear;
    always @(posedge clk or negedge rst_n)
      if(!rst_n)child_clear<=0;else child_clear<=(pc==RESPONSE);
    function finite;input [63:0] x;begin finite=(x[62:52]!=2047);end endfunction
    function [63:0] magnitude;input [63:0] x;begin magnitude={1'b0,x[62:0]};end endfunction
    `include "calib_lm_layout.vh"
    fp_operator #(.FP_W(64)) arithmetic(.clk(clk),.rst_n(local_rst_n),
      .req_valid(rst_n && pc==FP_REQ),.req_ready(fp_ready),.req_op(fp_op),.req_a(fp_a),.req_b(fp_b),
      .rsp_valid(fp_valid),.rsp_ready(rst_n && pc==FP_WAIT),.rsp_result(fp_result),.rsp_flags(fp_flags),
      .rsp_less(),.rsp_equal(),.rsp_unordered());
    task calculate;input [4:0] op;input [63:0] a,b;input integer target,next_pc;begin
      fp_op<=op;fp_a<=a;fp_b<=b;destination<=target;continuation<=next_pc;pc<=FP_REQ;
    end endtask
    task fail;input [7:0] status;begin rsp_status<=status;pc<=RESPONSE;end endtask
    assign rsp_valid=rst_n && pc==RESPONSE;

    // 每列缓存plus/minus/差分；不保存全量J。收到失败响应立即结束，成功检查`PAR_RESIDUALS项。
    // v0=1+abs(p),v1=h,v2=扰动值,v3=2h,v4=平方和/模长,
    // v5=差分,v6=平方临时量/scale,v7=缩放后的输出元素。
    // pc0..3正扰动，4..5残差请求/接收，6..8负扰动，9..13差分与模长，
    // 16..17求scale，14..15逐项缩放/输出。输出背压时不推进行号。
    reg [`PAR_STATE_W-1:0] original,perturbed;reg [`PAR_SCALE_W-1:0] scales;
    reg [63:0] plus[0:(`PAR_RESIDUALS-1)],minus[0:(`PAR_RESIDUALS-1)],column[0:(`PAR_RESIDUALS-1)];
    integer n,k,t,count;reg negative,bad_stream;
    assign cmd_ready=rst_n && pc==IDLE;
    assign eval_cmd_valid=rst_n && pc==4;
    assign eval_cmd_state=perturbed;
    assign eval_data_ready=rst_n && pc==5;
    assign eval_rsp_ready=rst_n && pc==5;
    assign j_valid=rst_n && pc==15;
    assign j_row=t;assign j_col=k;assign j_fp64=v[7];assign j_last=(k==n-1 && t==(`PAR_RESIDUALS-1));
    assign rsp_scales_fp64=(rsp_status==0)?scales:{`PAR_SCALE_W{1'b0}};

    always @(posedge clk or negedge rst_n) begin
      if(!rst_n)begin pc<=IDLE;rsp_status<=0;original<=0;perturbed<=0;scales<=0;n<=0;k<=0;t<=0;count<=0;negative<=0;bad_stream<=0; end
      else begin
        
        case(pc)
        FP_REQ:if(fp_ready)pc<=FP_WAIT;
        FP_WAIT:if(fp_valid)begin
          if((|fp_flags[2:0]) || !finite(fp_result))begin fail(`PAR_CALIB_INVALID); end
          else begin v[destination]<=fp_result;pc<=continuation;end
        end
        RESPONSE:if(rsp_ready)pc<=IDLE;
        
        IDLE:if(cmd_valid)begin
          original<=cmd_state;perturbed<=cmd_state;scales<=0;n<=columns(cmd_stage);k<=0;rsp_status<=0;
          if(cmd_stage==3)fail(`PAR_BAD_CONFIG);else pc<=0;
        end
        0:calculate(ADD,ONE,magnitude(original[64*active_index(k)+:64]),0,1);
        1:calculate(MUL,64'h3eb0c6f7a0b5ed8d,v[0],1,2);
        2:calculate(ADD,original[64*active_index(k)+:64],v[1],2,3);
        3:begin perturbed<=original;perturbed[64*active_index(k)+:64]<=v[2];negative<=0;pc<=4;end
        4:if(eval_cmd_ready)begin count<=0;bad_stream<=0;pc<=5;end
        5:begin
          if(eval_data_valid)begin
            if(count>=`PAR_RESIDUALS || eval_data_index!=count || eval_data_last!=(count==(`PAR_RESIDUALS-1)) || !finite(eval_data_fp64))bad_stream<=1;
            if(count<`PAR_RESIDUALS)begin if(negative)minus[count]<=eval_data_fp64;else plus[count]<=eval_data_fp64;count<=count+1;end
          end
          // 服务约定响应在最后一笔数据握手之后，不能同拍冒充完整成功。
          if(eval_rsp_valid)begin
            if(eval_rsp_status!=0)fail(eval_rsp_status);
            else if(bad_stream || count!=`PAR_RESIDUALS || eval_data_valid)fail(`PAR_BAD_CONFIG);
            else if(!finite(eval_rsp_cost_fp64))fail(`PAR_CALIB_INVALID);
            else if(!negative)pc<=6;
            else begin t<=0;v[4]<=0;pc<=9;end
          end
        end
        6:calculate(MUL,TWO,v[1],3,7);
        7:calculate(SUB,v[2],v[3],2,8);
        8:begin perturbed[64*active_index(k)+:64]<=v[2];negative<=1;pc<=4;end
        9:calculate(SUB,plus[t],minus[t],5,10);
        10:calculate(DIV,v[5],v[3],5,11);
        11:begin column[t]<=v[5];calculate(MUL,v[5],v[5],6,12);end
        12:calculate(ADD,v[4],v[6],4,13);
        13:if(t==(`PAR_RESIDUALS-1))calculate(SQRT,v[4],ZERO,4,16);else begin t<=t+1;pc<=9;end
        16:calculate(DIV,ONE,(v[4]<64'h3d719799812dea11)?64'h3d719799812dea11:v[4],6,17);
        17:begin scales[64*k+:64]<=v[6];t<=0;pc<=14;end
        14:calculate(MUL,column[t],v[6],7,15);
        15:if(j_ready)begin
          if(t==(`PAR_RESIDUALS-1))begin if(k==n-1)pc<=RESPONSE;else begin k<=k+1;pc<=0;end end
          else begin t<=t+1;pc<=14;end
        end

        default:fail(`PAR_BAD_CONFIG);
        endcase
      end
    end
endmodule
