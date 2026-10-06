`include "calib_defs.vh"

/*
配置：统一见 rtl/include/calib_config.vh；下文具体计数例子以默认3张5×8为例，
实际容量、循环边界和端口位宽由PAR_*派生，修改配置后须重新编译/综合。
作用：缓存不变的N/g，每次lambda构造A=N+lambda*I、b=-g并调用gauss_solver。
本模块是重试数据的所有者：load开始一轮N/g载入，load_rsp成功后才接受solve cmd。
一次load可以随后接受最多16次cmd；每次重新构造完整A/b，绝不复用已消元的矩阵。
加载时不允许solve，求解/响应未结束不允许下一load；新load替换上轮N/g。
load_abort清除未完成载入并发load_rsp错误；不存在有效N/g时拒绝solve。
solve成功输出26槽delta（未用置零），由LM乘scale并更新trial；失败仅令LM加阻尼。
子模块：gauss_solver、fp_operator；不是一个新的外迭代控制器。

共同契约：
- 已实现串行FP64控制；数组不复位，每次使用前完整写入。尚未进行综合/时序验证。
- valid && ready 才传输；背压时 valid 和对应全部载荷保持不变。
- 除另有说明，一次一项任务，rsp 被接收前不接受新 cmd；不假设固定延迟。
- cmd_* 随命令锁存；rsp_* 随响应有效。失败仍须发完成响应，不能无声挂起。
- 非零状态通常使结果载荷无效；本模块明确说明的诊断有效位例外。
- 复位取消在途数据；复位后所有生产端 valid 清零。实现时禁止 real 运算。
*/
module damped_step #(parameter FP_SHARED=0) (
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
    input wire load_valid, // 开始新一轮N/g缓存
    output wire load_ready, // 无在途load/solve时可接受
    input wire [1:0] load_stage, // 决定n
    input wire load_abort_valid, // 终止未完成的N/g载入
    output wire load_abort_ready, // 加载期间可接受
    input wire ng_valid, // 接normal_equation的N/g流
    output wire ng_ready, // 缓存可接收
    input wire ng_kind, // 0=N下三角，1=g
    input wire [`PAR_COL_BITS-1:0] ng_row, // 行/索引
    input wire [`PAR_COL_BITS-1:0] ng_col, // 列
    input wire [63:0] ng_fp64, // 数值
    input wire ng_last, // g[n-1]
    output wire load_rsp_valid, // 载入完成
    input wire load_rsp_ready, // 接收载入完成
    output wire [7:0] load_rsp_status, // 0完成或非零失败
    input wire cmd_valid, // 命令有效
    output wire cmd_ready, // 可接受命令
    input wire [63:0] cmd_lambda_fp64, // 当前阻尼，有限且>0
    output wire rsp_valid, // 完成响应有效；最后一笔输出握手后才置位
    input wire rsp_ready, // 接收完成响应
    output reg [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [`PAR_SCALE_W-1:0] rsp_delta_fp64 // n个delta，活动列顺序
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

    localparam LOADING=0,LOAD_RESPONSE=1;
    // 下三角紧凑地址row*(row+1)/2+col；缓存只在load期间写。
    // pc2启动gauss；3读原矩阵/负梯度；4只对角加lambda；7接算术结果；
    // 5发送增广矩阵元素，6等解。每次重试从row=col=0重新完整发送。
    // 算术提前失败时RESPONSE取消仍等待矩阵的gauss，N/g缓存仍可重试。
    // N 与 g 合并到一份单读/单写同步 RAM。梯度位于三角矩阵之后。
    // load 完整提交后才允许读取；重试仅修改 lambda，不改工作 RAM。
    localparam CACHE_BITS=$clog2(`PAR_TRIANGLE_SIZE+`PAR_ACTIVE_N);
    reg loaded,load_kind;integer n,lr,lc,row,col,attempts;
    reg [7:0] load_status;
    reg [`PAR_SCALE_W-1:0] delta;
    reg [63:0] lambda,entry;
    wire gauss_ready,matrix_ready,gauss_valid;wire [7:0] gauss_status;wire [`PAR_SCALE_W-1:0] solution;
    // Bound the address arithmetic by the configured matrix dimension. A
    // signed 32x32 multiply here otherwise consumes several DSP blocks.
    function [$clog2(`PAR_TRIANGLE_SIZE)-1:0] triangle_address;
        input [`PAR_COL_BITS-1:0] r,c;
        reg [2*`PAR_COL_BITS:0] product;
        begin
            product=r*({1'b0,r}+1'b1);
            triangle_address=(product>>1)+c;
        end
    endfunction
    wire [CACHE_BITS-1:0] cache_write_address=load_kind?
        `PAR_TRIANGLE_SIZE+lr:triangle_address(lr,lc);
    wire [CACHE_BITS-1:0] cache_read_address=col==n?
        `PAR_TRIANGLE_SIZE+row:(row>=col?triangle_address(row,col):triangle_address(col,row));
    wire cache_write=rst_n && pc==LOADING && ng_valid && !load_abort_valid &&
        ng_kind==load_kind && ng_row==lr && ng_col==(load_kind?0:lc) &&
        ng_last==(load_kind && lr==n-1) && finite(ng_fp64);
    wire [63:0] cache_read_data;
    work_ram #(.WIDTH(64),.ADDR_BITS(CACHE_BITS)) retry_cache(
        .clk(clk),.wr_en(cache_write),.wr_addr(cache_write_address),.wr_data(ng_fp64),
        .rd_en(rst_n && pc==3),.rd_addr(cache_read_address),.rd_data(cache_read_data));
    assign load_ready=rst_n && pc==IDLE;
    // 同周期load优先；新load无条件作废旧缓存，不能同时接受solve。
    assign cmd_ready=rst_n && pc==IDLE && loaded && attempts<`PAR_LM_MAX_TRIES && !load_valid;
    assign load_abort_ready=rst_n && pc==LOADING;
    assign ng_ready=rst_n && pc==LOADING && !load_abort_valid;
    assign load_rsp_valid=rst_n && pc==LOAD_RESPONSE;
    assign load_rsp_status=load_status;
    assign rsp_delta_fp64=(rsp_status==0)?delta:{`PAR_SCALE_W{1'b0}};
    gauss_solver #(.FP_SHARED(FP_SHARED)) solver(.shared_req_valid(shared_req_valid[1 +: 1]),.shared_req_ready(shared_req_ready[1 +: 1]),.shared_req_op(shared_req_op[5 +: 5]),.shared_req_a(shared_req_a[64 +: 64]),.shared_req_b(shared_req_b[64 +: 64]),.shared_active(shared_active[1 +: 1]),.shared_rsp_valid(shared_rsp_valid[1 +: 1]),.shared_rsp_ready(shared_rsp_ready[1 +: 1]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(local_rst_n),.cmd_valid(rst_n && pc==2),.cmd_ready(gauss_ready),
      .cmd_n(n[`PAR_COL_BITS-1:0]),.matrix_valid(rst_n && pc==5),.matrix_ready(matrix_ready),
      .matrix_fp64(entry),.matrix_last(row==n-1 && col==n),.rsp_valid(gauss_valid),
      .rsp_ready(rst_n && pc==6),.rsp_status(gauss_status),.rsp_solution_fp64(solution));
    task load_fail;input [7:0] status;begin loaded<=0;load_status<=status;pc<=LOAD_RESPONSE;end endtask

    always @(posedge clk or negedge rst_n) begin
      if(!rst_n)begin pc<=IDLE;rsp_status<=0;loaded<=0;load_status<=0;attempts<=0;n<=0;lr<=0;lc<=0;row<=0;col<=0;load_kind<=0;delta<=0;lambda<=0;entry<=0; end
      else begin

        case(pc)
        FP_REQ:if(fp_ready)pc<=FP_WAIT;
        FP_WAIT:if(fp_valid)begin
          if((|fp_flags[2:0]) || !finite(fp_result))begin fail(`PAR_CALIB_INVALID); end
          else begin v[destination]<=fp_result;pc<=continuation;end
        end
        RESPONSE:if(rsp_ready)pc<=IDLE;

        IDLE:begin
          if(load_valid)begin loaded<=0;attempts<=0;n<=columns(load_stage);lr<=0;lc<=0;load_kind<=0;load_status<=0;
            if(load_stage==3)load_fail(`PAR_BAD_CONFIG);else pc<=LOADING;
          end else if(cmd_valid && cmd_ready)begin
            rsp_status<=0;delta<=0;lambda<=cmd_lambda_fp64;attempts<=attempts+1;
            if(!finite(cmd_lambda_fp64) || cmd_lambda_fp64[63] || cmd_lambda_fp64[62:0]==0)fail(`PAR_BAD_CONFIG);
            else pc<=2;
          end
        end
        LOADING:if(load_abort_valid)load_fail(`PAR_CALIB_INVALID);
        else if(ng_valid)begin
          if(ng_kind!=load_kind || ng_row!=lr || ng_col!=(load_kind?0:lc) || ng_last!=(load_kind && lr==n-1))load_fail(`PAR_BAD_CONFIG);
          else if(!finite(ng_fp64))load_fail(`PAR_CALIB_INVALID);
          else if(load_kind)begin
            if(lr==n-1)begin loaded<=1;pc<=LOAD_RESPONSE;end else lr<=lr+1;
          end else begin
            if(lc==lr)begin lc<=0;if(lr==n-1)begin lr<=0;load_kind<=1;end else lr<=lr+1;end
            else lc<=lc+1;
          end
        end
        LOAD_RESPONSE:if(load_rsp_ready)pc<=IDLE;
        2:if(gauss_ready)begin row<=0;col<=0;pc<=3;end
        3:pc<=8; // RAM read request; registered data is available next cycle.
        8:begin
          if(col==n)begin entry<={~cache_read_data[63],cache_read_data[62:0]};pc<=5;end
          else begin entry<=cache_read_data;pc<=4;end
        end
        4:if(row==col)calculate(ADD,entry,lambda,0,7);else pc<=5;
        7:begin entry<=v[0];pc<=5;end
        5:if(matrix_ready)begin
          if(col==n)begin col<=0;if(row==n-1)pc<=6;else begin row<=row+1;pc<=3;end end
          else begin col<=col+1;pc<=3;end
        end
        6:if(gauss_valid)begin delta<=solution;rsp_status<=gauss_status;pc<=RESPONSE;end

        default:fail(`PAR_BAD_CONFIG);
        endcase
      end
    end
endmodule
