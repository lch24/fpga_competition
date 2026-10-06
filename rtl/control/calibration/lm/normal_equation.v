`include "calib_defs.vh"

/*
配置：统一见 rtl/include/calib_config.vh；下文具体计数例子以默认3张5×8为例，
实际容量、循环边界和端口位宽由PAR_*派生，修改配置后须重新编译/综合。
作用：拥有240x26的J RAM及`PAR_RESIDUALS项基准残差RAM，计算N=J^TJ和g=J^Tr。
cmd清除本轮载入标志并锁存stage；随后接收r流和列序J流，可交错但各流内部有序。
两流收齐后逐项求N下三角和g，各点积内部按t=0..(`PAR_RESIDUALS-1)累加；上三角由step补齐。
遍历改为输出元素外层、t内层，保持C++每个累加器的顺序，省去完整N缓存。
先输出N的下三角（row外层，col=0..row），再输出g[0..n-1]。
未收齐时若父模块发现雅可比失败，使用abort握手丢弃本轮，返回CALIB_INVALID。
abort接受后停止接收r/J并取消未握手的N/g（取消是背压保持规则的例外）；
输出部分N/g若已发出，下游必须通过abort清理。
正常rsp给max_abs(g)供LM判断，不保存阻尼，不破坏上轮的current状态。

共同契约：
- 已实现串行FP64控制；数组不复位，每次使用前完整写入。尚未进行综合/时序验证。
- valid && ready 才传输；背压时 valid 和对应全部载荷保持不变。
- 除另有说明，一次一项任务，rsp 被接收前不接受新 cmd；不假设固定延迟。
- cmd_* 随命令锁存；rsp_* 随响应有效。失败仍须发完成响应，不能无声挂起。
- 非零状态通常使结果载荷无效；本模块明确说明的诊断有效位例外。
- 复位取消在途数据；复位后所有生产端 valid 清零。实现时禁止 real 运算。
*/
module normal_equation #(parameter FP_SHARED=0) (
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
    input wire [1:0] cmd_stage, // n=22/23/26
    input wire abort_valid, // 取消当前任务，高于输入数据接收
    output wire abort_ready, // 运行期间可接收取消
    input wire r_valid, // 基准残差输入
    output wire r_ready, // 可接收
    input wire [`PAR_RES_BITS-1:0] r_index, // 严格0..(`PAR_RESIDUALS-1)
    input wire [63:0] r_fp64, // 残差
    input wire r_last, // 索引(`PAR_RESIDUALS-1)
    input wire j_valid, // 列序J输入
    output wire j_ready, // 可接收
    input wire [`PAR_RES_BITS-1:0] j_row, // 0..(`PAR_RESIDUALS-1)
    input wire [`PAR_COL_BITS-1:0] j_col, // 0..n-1
    input wire [63:0] j_fp64, // 已缩放J
    input wire j_last, // 最后元素
    output wire ng_valid, // N/g元素
    input wire ng_ready, // 可接收
    output wire ng_kind, // 0=N下三角，1=g
    output wire [`PAR_COL_BITS-1:0] ng_row, // N行或g索引
    output wire [`PAR_COL_BITS-1:0] ng_col, // N列；g时0
    output wire [63:0] ng_fp64, // 数值
    output wire ng_last, // 仅g[n-1]为1
    output wire rsp_valid, // 完成响应有效；最后一笔输出握手后才置位
    input wire rsp_ready, // 接收完成响应
    output reg [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [63:0] rsp_max_gradient_fp64 // max(abs(g))
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

    generate if(FP_SHARED) begin : g_shared_fp
        assign shared_req_valid[0 +: 1] = rst_n && pc==FP_REQ;
        assign shared_req_op[0 +: 5] = fp_op;
        assign shared_req_a[0 +: 64] = fp_a;
        assign shared_req_b[0 +: 64] = fp_b;
        assign shared_rsp_ready[0 +: 1] = rst_n && pc==FP_WAIT;
        assign shared_active[0] = local_rst_n;
        assign fp_ready = shared_req_ready[0];
        assign fp_valid = shared_rsp_valid[0];
        assign fp_result = shared_rsp_result;
        assign fp_flags = shared_rsp_flags;
    end else begin : g_local_fp
    fp_operator #(.FP_W(64), .ENABLE_EXP(0), .ENABLE_LOG(0), .ENABLE_SINCOS(0), .ENABLE_ATAN_ACOS(0)) arithmetic(.clk(clk),.rst_n(local_rst_n),
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

    task calculate;input [4:0] op;input [63:0] a,b;input integer target,next_pc;begin
      fp_op<=op;fp_a<=a;fp_b<=b;destination<=target;continuation<=next_pc;pc<=FP_REQ;
    end endtask
    task fail;input [7:0] status;begin rsp_status<=status;pc<=RESPONSE;end endtask
    assign rsp_valid=rst_n && pc==RESPONSE;

    // J按列存储，点积按残差t递增累加；与C++每个累加器的舍入顺序一致。
    // 首版采用显式读取寄存器，N不再整块保存，算完一个元素即发送。
    // pc0加载两路RAM；1初始化点积；2寄存读数；3乘法；4累加；
    // 5遍历t；6更新梯度诊断；7等待N/g握手。v0=累加器，v1=乘积。
    reg [63:0] jram[0:`PAR_RESIDUALS*`PAR_ACTIVE_N-1],rram[0:(`PAR_RESIDUALS-1)];
    reg [63:0] operand_a,operand_b,max_gradient;
    integer n,rc,jc,jr,a,b,t;reg kind;
    // One physical J read port. Reading both columns in one cycle duplicated
    // this 49 KiB array in PDS. Two read cycles preserve each dot product's
    // accumulation order and add only two cycles per multiply/add pair.
    wire [$clog2(`PAR_RESIDUALS*`PAR_ACTIVE_N)-1:0] j_read_address=
        ((pc==2)?a:b)*`PAR_RESIDUALS+t;
    reg [63:0] j_read_data;
    always @(posedge clk)
        if(pc==2 || pc==8) j_read_data<=jram[j_read_address];
    assign cmd_ready=rst_n && pc==IDLE;
    assign abort_ready=rst_n && pc!=IDLE && pc!=RESPONSE;
    assign r_ready=rst_n && pc==0 && rc<`PAR_RESIDUALS && !abort_valid;
    assign j_ready=rst_n && pc==0 && jc<n && !abort_valid;
    assign ng_valid=rst_n && pc==7 && !abort_valid;
    assign ng_kind=kind;assign ng_row=a;assign ng_col=kind?0:b;
    assign ng_fp64=v[0];assign ng_last=kind && a==n-1;
    assign rsp_max_gradient_fp64=(rsp_status==0)?max_gradient:ZERO;

    always @(posedge clk or negedge rst_n) begin
      if(!rst_n)begin pc<=IDLE;rsp_status<=0;n<=0;rc<=0;jc<=0;jr<=0;a<=0;b<=0;t<=0;kind<=0;max_gradient<=0; end
      else begin
        // abort优先且同时抑制输入/输出握手；响应态复位算术子核，取消在途除法。
        case(pc)
        FP_REQ:if(fp_ready)pc<=FP_WAIT;
        FP_WAIT:if(fp_valid)begin
          if((|fp_flags[2:0]) || !finite(fp_result))begin fail(`PAR_CALIB_INVALID); end
          else begin v[destination]<=fp_result;pc<=continuation;end
        end
        RESPONSE:if(rsp_ready)pc<=IDLE;

        IDLE:if(cmd_valid)begin
          rsp_status<=0;n<=columns(cmd_stage);rc<=0;jc<=0;jr<=0;max_gradient<=0;
          if(cmd_stage==3)fail(`PAR_BAD_CONFIG);else pc<=0;
        end
        0:begin
          if(r_valid && r_ready)begin
            if(r_index!=rc || r_last!=(rc==(`PAR_RESIDUALS-1)))fail(`PAR_BAD_CONFIG);
            else if(!finite(r_fp64))fail(`PAR_CALIB_INVALID);
            else begin rram[rc]<=r_fp64;rc<=rc+1;end
          end
          if(j_valid && j_ready)begin
            if(j_row!=jr || j_col!=jc || j_last!=(jc==n-1 && jr==(`PAR_RESIDUALS-1)))fail(`PAR_BAD_CONFIG);
            else if(!finite(j_fp64))fail(`PAR_CALIB_INVALID);
            else begin jram[jc*`PAR_RESIDUALS+jr]<=j_fp64;if(jr==(`PAR_RESIDUALS-1))begin jr<=0;jc<=jc+1;end else jr<=jr+1;end
          end
          if(rc==`PAR_RESIDUALS && jc==n)begin a<=0;b<=0;kind<=0;pc<=1;end
        end
        1:begin t<=0;v[0]<=0;pc<=2;end
        2:pc<=8;
        8:begin operand_a<=j_read_data;pc<=9;end
        9:begin operand_b<=kind?rram[t]:j_read_data;pc<=3;end
        3:calculate(MUL,operand_a,operand_b,1,4);
        4:calculate(ADD,v[0],v[1],0,5);
        5:if(t==(`PAR_RESIDUALS-1))pc<=6;else begin t<=t+1;pc<=2;end
        6:begin if(kind && magnitude(v[0])>max_gradient)max_gradient<=magnitude(v[0]);pc<=7;end
        7:if(ng_ready && !abort_valid)begin
          if(kind)begin if(a==n-1)pc<=RESPONSE;else begin a<=a+1;pc<=1;end end
          else if(b==a)begin b<=0;if(a==n-1)begin kind<=1;a<=0;end else a<=a+1;pc<=1;end
          else begin b<=b+1;pc<=1;end
        end

        default:fail(`PAR_BAD_CONFIG);
        endcase
        if(abort_valid && abort_ready)fail(`PAR_CALIB_INVALID);
      end
    end
endmodule
