`include "calib_defs.vh"

/*
配置：统一见 rtl/include/calib_config.vh；下文具体计数例子以默认3张5×8为例，
实际容量、循环边界和端口位宽由PAR_*派生，修改配置后须重新编译/综合。
作用：对一个seed的一阶段执行LM；父顶层按stage=0/1/2依次调用本模块。
拥有current/trial状态、当前残差`PAR_RESIDUALS项RAM、cost、lambda和迭代/重试计数。
调用residual_engine取得基准r；调用jacobian生成缩放J；送normal_equation生成N/g。
梯度未收敛则调用damped_step(lambda)得到delta；q=p+delta*scale，再求trial残差。
只接受new_cost<cost；接受lambda*=0.3(下限1e-12)，拒绝lambda*=10；每轮最多16次。
每阶段lambda=1e-3，最多150外迭代；重试复用N/g，不重算J。
残差核由本模块独占，手动仲裁基准/雅可比正负扰动/试探调用，任一时刻单请求。
子模块：jacobian、normal_equation、damped_step、residual_engine、fp_operator。
正常未收敛仍rsp_status=OK，输出最后接受状态；converged单独判定。
参数扰动无效导致本阶段未收敛，保留current；硬件/协议错误用非零status。
精确停止条件见README及C++ lm.cpp；不能以达到迭代上限冒充收敛。

角点读接口例外：只读已提交视图，固定1拍返回，无valid/ready背压。
E_n采样使能/地址，数据在E_n后有效并由调用方在E_(n+1)采样；详见corner_store。
首版最多一笔在途；层间读路由不插入寄存器，返回消费后才能切换所有者。

共同契约：
- 已实现串行FP64控制；数组不复位，每次使用前完整写入。尚未进行综合/时序验证。
- valid && ready 才传输；背压时 valid 和对应全部载荷保持不变。
- 除另有说明，一次一项任务，rsp 被接收前不接受新 cmd；不假设固定延迟。
- cmd_* 随命令锁存；rsp_* 随响应有效。失败仍须发完成响应，不能无声挂起。
- 非零状态通常使结果载荷无效；本模块明确说明的诊断有效位例外。
- 复位取消在途数据；复位后所有生产端 valid 清零。实现时禁止 real 运算。
*/
module lm_controller #(parameter FP_SHARED=0, SHARE_RESIDUAL=0) (
    output wire  ext_rst_n,
    output wire  ext_cmd_valid,
    input wire  ext_cmd_ready,
    output wire [15:0] ext_cmd_width,
    output wire [15:0] ext_cmd_height,
    output wire [`PAR_STATE_W-1:0] ext_cmd_state,
    input wire  ext_point_rd_en,
    input wire [`PAR_VIEW_BITS-1:0] ext_point_rd_view_id,
    input wire [`PAR_POINT_BITS-1:0] ext_point_rd_index,
    output wire  ext_point_rd_valid,
    output wire [31:0] ext_point_rd_x_fp32,
    output wire [31:0] ext_point_rd_y_fp32,
    input wire  ext_data_valid,
    output wire  ext_data_ready,
    input wire [`PAR_RES_BITS-1:0] ext_data_index,
    input wire [63:0] ext_data_fp64,
    input wire  ext_data_last,
    input wire  ext_rsp_valid,
    output wire  ext_rsp_ready,
    input wire [7:0] ext_rsp_status,
    input wire [63:0] ext_rsp_cost_fp64,
    // Optional shared FP64 service; local mode keeps standalone compatibility.
    output wire [8:0] shared_req_valid,
    input wire [8:0] shared_req_ready,
    output wire [44:0] shared_req_op,
    output wire [575:0] shared_req_a,
    output wire [575:0] shared_req_b,
    output wire [8:0] shared_active,
    input wire [8:0] shared_rsp_valid,
    output wire [8:0] shared_rsp_ready,
    input wire [63:0] shared_rsp_result,
    input wire [4:0] shared_rsp_flags,

    input wire clk, // core_clk，同一时钟域
    input wire rst_n, // 低有效复位，同步释放；取消全部在途事务
    input wire cmd_valid, // 命令有效
    output wire cmd_ready, // 可接受命令
    input wire [15:0] cmd_width, // 图像宽度，>=2
    input wire [15:0] cmd_height, // 图像高度，>=2
    input wire [`PAR_STATE_W-1:0] cmd_state, // PAR_STATE_N 项 FP64 状态快照，命令握手时锁存
    input wire [1:0] cmd_stage, // 0/1/2 -> 22/23/26个活动参数
    output wire point_rd_en, // 固定1拍角点读使能，无ready，发起前预留接收空间
    output wire [`PAR_VIEW_BITS-1:0] point_rd_view_id, // 0..PAR_VIEWS-1
    output wire [`PAR_POINT_BITS-1:0] point_rd_index, // 图内角点0..PAR_POINTS-1
    input wire point_rd_valid, // 固定1拍返回，必须当拍消费；无返回背压
    input wire [31:0] point_rd_x_fp32, // 原图x
    input wire [31:0] point_rd_y_fp32, // 原图y
    output wire rsp_valid, // 完成响应有效；最后一笔输出握手后才置位
    input wire rsp_ready, // 接收完成响应
    output reg [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [`PAR_STATE_W-1:0] rsp_state, // 最后已接受的current，未收敛也保留
    output wire [63:0] rsp_cost_fp64, // 该状态cost；无有效残差时+Inf
    output wire rsp_converged, // 本阶段算法收敛
    output wire [7:0] rsp_accepted_steps, // 该阶段接受更新数0..150
    output wire [7:0] rsp_outer_iterations // 实际开始的外迭代数，独立于接受数
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

    // v0=cost,v1=lambda,v2=max_gradient,v3=1+sqrt(cost),v4=step_norm,
    // v5=d,v6=1+abs(p),v7=relative_d,v8=new_cost,v9=reduction,v10=threshold。
    // 0/1基准残差；2外迭代入口；3..6构造J；7/8接N/g；9..12梯度；
    // 13/14解阻尼方程；15..21更新trial；22/23试探；24..29提交；
    // 30..33拒绝/重试；40..43取消不完整的normal/damped加载。
    // current/residual只在基准成功或trial严格降cost后写入；trial有独立缓存。
    // 先把基准r送齐再启动jac；jac独占残差核，结束后才允许trial请求。
    // 子模块完成响应分别记账，不能假定normal rsp与damped load_rsp同拍。
    // RESPONSE清理全部子核；外部rsp快照来自本模块寄存器，仍保持到握手。
    reg [`PAR_STATE_W-1:0] current,trial;
    reg [63:0] residual[0:(`PAR_RESIDUALS-1)],trial_residual[0:(`PAR_RESIDUALS-1)];
    reg [`PAR_SCALE_W-1:0] scales,delta;
    reg [15:0] width,height;reg [1:0] stage;
    integer n,k,send_r,receive_count,attempt,copy_index;
    reg stream_error,converged,numeric_retry;
    reg [7:0] accepted,outer,pending_status;
    reg ne_done,ds_done;reg [7:0] saved_ne_status,saved_ds_status;
    wire jac_ready,jac_valid,j_valid,j_ready,eval_cmd_valid,eval_cmd_ready,eval_data_ready,eval_rsp_ready;
    wire [`PAR_STATE_W-1:0] eval_state;wire [`PAR_RES_BITS-1:0] j_row;wire [7:0] jac_status;wire [`PAR_COL_BITS-1:0] j_col;wire [63:0] j_value;wire j_last;
    wire [`PAR_SCALE_W-1:0] jac_scales;
    wire ne_ready,ne_valid,ne_r_ready,ne_abort_ready,ng_valid,ng_ready,ng_kind,ng_last;
    wire [`PAR_COL_BITS-1:0] ng_row,ng_col;wire [63:0] ng_value,max_gradient;wire [7:0] ne_status;
    wire ng_ready_internal;
    wire ds_load_ready,ds_load_valid,ds_abort_ready,ds_ready,ds_valid;
    wire [7:0] ds_load_status,ds_status;wire [`PAR_SCALE_W-1:0] ds_delta;
    wire re_ready,re_valid,re_data_valid,re_data_ready;wire [7:0] re_status;wire [`PAR_RES_BITS-1:0] re_index;
    wire [63:0] re_cost,re_value;wire re_last;
    wire jac_owner=(pc==6);
    wire re_command=(pc==0 || pc==22 || (jac_owner && eval_cmd_valid));
    wire re_response_ready=(pc==1 || pc==23 || (jac_owner && eval_rsp_ready));
    assign cmd_ready=rst_n && pc==IDLE;
    assign rsp_state=current;assign rsp_cost_fp64=v[0];assign rsp_converged=converged;
    assign rsp_accepted_steps=accepted;assign rsp_outer_iterations=outer;
    assign eval_cmd_ready=jac_owner && re_ready;
    assign re_data_ready=(pc==1 || pc==23)?1'b1:(jac_owner && eval_data_ready);
    residual_endpoint #(.FP_SHARED(FP_SHARED),.EXTERNAL(SHARE_RESIDUAL)) residual_service(.ext_rst_n(ext_rst_n),.ext_cmd_valid(ext_cmd_valid),.ext_cmd_ready(ext_cmd_ready),.ext_cmd_width(ext_cmd_width),.ext_cmd_height(ext_cmd_height),.ext_cmd_state(ext_cmd_state),.ext_point_rd_en(ext_point_rd_en),.ext_point_rd_view_id(ext_point_rd_view_id),.ext_point_rd_index(ext_point_rd_index),.ext_point_rd_valid(ext_point_rd_valid),.ext_point_rd_x_fp32(ext_point_rd_x_fp32),.ext_point_rd_y_fp32(ext_point_rd_y_fp32),.ext_data_valid(ext_data_valid),.ext_data_ready(ext_data_ready),.ext_data_index(ext_data_index),.ext_data_fp64(ext_data_fp64),.ext_data_last(ext_data_last),.ext_rsp_valid(ext_rsp_valid),.ext_rsp_ready(ext_rsp_ready),.ext_rsp_status(ext_rsp_status),.ext_rsp_cost_fp64(ext_rsp_cost_fp64),.shared_req_valid(shared_req_valid[1 +: 4]),.shared_req_ready(shared_req_ready[1 +: 4]),.shared_req_op(shared_req_op[5 +: 20]),.shared_req_a(shared_req_a[64 +: 256]),.shared_req_b(shared_req_b[64 +: 256]),.shared_active(shared_active[1 +: 4]),.shared_rsp_valid(shared_rsp_valid[1 +: 4]),.shared_rsp_ready(shared_rsp_ready[1 +: 4]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(local_rst_n),
      .cmd_valid(rst_n && re_command),.cmd_ready(re_ready),.cmd_width(width),.cmd_height(height),
      .cmd_state(jac_owner?eval_state:((pc==22)?trial:current)),
      .point_rd_en(point_rd_en),.point_rd_view_id(point_rd_view_id),.point_rd_index(point_rd_index),
      .point_rd_valid(point_rd_valid),.point_rd_x_fp32(point_rd_x_fp32),.point_rd_y_fp32(point_rd_y_fp32),
      .data_valid(re_data_valid),.data_ready(re_data_ready),.data_index(re_index),.data_fp64(re_value),.data_last(re_last),
      .rsp_valid(re_valid),.rsp_ready(re_response_ready),.rsp_status(re_status),.rsp_cost_fp64(re_cost));
    jacobian #(.FP_SHARED(FP_SHARED)) jac(.shared_req_valid(shared_req_valid[5 +: 1]),.shared_req_ready(shared_req_ready[5 +: 1]),.shared_req_op(shared_req_op[25 +: 5]),.shared_req_a(shared_req_a[320 +: 64]),.shared_req_b(shared_req_b[320 +: 64]),.shared_active(shared_active[5 +: 1]),.shared_rsp_valid(shared_rsp_valid[5 +: 1]),.shared_rsp_ready(shared_rsp_ready[5 +: 1]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(local_rst_n),.cmd_valid(rst_n && pc==5),.cmd_ready(jac_ready),
      .cmd_state(current),.cmd_stage(stage),.eval_cmd_valid(eval_cmd_valid),.eval_cmd_ready(eval_cmd_ready),.eval_cmd_state(eval_state),
      .eval_data_valid(jac_owner && re_data_valid),.eval_data_ready(eval_data_ready),.eval_data_index(re_index),.eval_data_fp64(re_value),.eval_data_last(re_last),
      .eval_rsp_valid(jac_owner && re_valid),.eval_rsp_ready(eval_rsp_ready),.eval_rsp_status(re_status),.eval_rsp_cost_fp64(re_cost),
      .j_valid(j_valid),.j_ready(j_ready),.j_row(j_row),.j_col(j_col),.j_fp64(j_value),.j_last(j_last),
      .rsp_valid(jac_valid),.rsp_ready(pc==6),.rsp_status(jac_status),.rsp_scales_fp64(jac_scales));
    normal_equation #(.FP_SHARED(FP_SHARED)) normal_builder(.shared_req_valid(shared_req_valid[6 +: 1]),.shared_req_ready(shared_req_ready[6 +: 1]),.shared_req_op(shared_req_op[30 +: 5]),.shared_req_a(shared_req_a[384 +: 64]),.shared_req_b(shared_req_b[384 +: 64]),.shared_active(shared_active[6 +: 1]),.shared_rsp_valid(shared_rsp_valid[6 +: 1]),.shared_rsp_ready(shared_rsp_ready[6 +: 1]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(local_rst_n),.cmd_valid(rst_n && pc==3),.cmd_ready(ne_ready),.cmd_stage(stage),
      .abort_valid(pc==40),.abort_ready(ne_abort_ready),
      .r_valid(rst_n && pc==4),.r_ready(ne_r_ready),.r_index(send_r[`PAR_RES_BITS-1:0]),.r_fp64(residual[send_r]),.r_last(send_r==(`PAR_RESIDUALS-1)),
      .j_valid(j_valid),.j_ready(j_ready),.j_row(j_row),.j_col(j_col),.j_fp64(j_value),.j_last(j_last),
      .ng_valid(ng_valid),.ng_ready(ng_ready),.ng_kind(ng_kind),.ng_row(ng_row),.ng_col(ng_col),.ng_fp64(ng_value),.ng_last(ng_last),
      .rsp_valid(ne_valid),.rsp_ready(pc==4 || pc==5 || pc==6 || pc==8 || pc==40 || pc==41),.rsp_status(ne_status),.rsp_max_gradient_fp64(max_gradient));
    damped_step #(.FP_SHARED(FP_SHARED)) step_solver(.shared_req_valid(shared_req_valid[7 +: 2]),.shared_req_ready(shared_req_ready[7 +: 2]),.shared_req_op(shared_req_op[35 +: 10]),.shared_req_a(shared_req_a[448 +: 128]),.shared_req_b(shared_req_b[448 +: 128]),.shared_active(shared_active[7 +: 2]),.shared_rsp_valid(shared_rsp_valid[7 +: 2]),.shared_rsp_ready(shared_rsp_ready[7 +: 2]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(local_rst_n),.load_valid(rst_n && pc==7),.load_ready(ds_load_ready),.load_stage(stage),
      .load_abort_valid(pc==42),.load_abort_ready(ds_abort_ready),
      .ng_valid(ng_valid && pc==8),.ng_ready(ng_ready_internal),.ng_kind(ng_kind),.ng_row(ng_row),.ng_col(ng_col),.ng_fp64(ng_value),.ng_last(ng_last),
      .load_rsp_valid(ds_load_valid),.load_rsp_ready(pc==8 || pc==42 || pc==43),.load_rsp_status(ds_load_status),
      .cmd_valid(rst_n && pc==13),.cmd_ready(ds_ready),.cmd_lambda_fp64(v[1]),
      .rsp_valid(ds_valid),.rsp_ready(pc==14),.rsp_status(ds_status),.rsp_delta_fp64(ds_delta));

    assign ng_ready=(pc==8) && ng_ready_internal;
    task algorithm_stop;begin converged<=0;rsp_status<=0;pc<=RESPONSE;end endtask

    always @(posedge clk or negedge rst_n) begin
      if(!rst_n)begin pc<=IDLE;rsp_status<=0;current<=0;trial<=0;scales<=0;delta<=0;width<=0;height<=0;stage<=0;n<=0;k<=0;send_r<=0;receive_count<=0;attempt<=0;copy_index<=0;stream_error<=0;converged<=0;numeric_retry<=0;accepted<=0;outer<=0;pending_status<=0;ne_done<=0;ds_done<=0;saved_ne_status<=0;saved_ds_status<=0;v[0]<=64'h7ff0000000000000; end
      else begin

        case(pc)
        FP_REQ:if(fp_ready)pc<=FP_WAIT;
        FP_WAIT:if(fp_valid)begin
          if((|fp_flags[2:0]) || !finite(fp_result))begin if(numeric_retry)begin numeric_retry<=0;pc<=30;end else algorithm_stop(); end
          else begin v[destination]<=fp_result;pc<=continuation;end
        end
        RESPONSE:if(rsp_ready)pc<=IDLE;

        IDLE:if(cmd_valid)begin
          width<=cmd_width;height<=cmd_height;stage<=cmd_stage;n<=columns(cmd_stage);current<=cmd_state;
          trial<=cmd_state;accepted<=0;outer<=0;converged<=0;rsp_status<=0;numeric_retry<=0;
          v[0]<=64'h7ff0000000000000;v[1]<=64'h3f50624dd2f1a9fc;
          if(cmd_width<2 || cmd_height<2 || cmd_stage==3)fail(`PAR_BAD_CONFIG);else pc<=0;
        end
        0:if(re_ready)begin receive_count<=0;stream_error<=0;pc<=1;end
        1:begin
          if(re_data_valid)begin
            if(receive_count>=`PAR_RESIDUALS || re_index!=receive_count || re_last!=(receive_count==(`PAR_RESIDUALS-1)) || !finite(re_value))stream_error<=1;
            if(receive_count<`PAR_RESIDUALS)begin residual[receive_count]<=re_value;receive_count<=receive_count+1;end
          end
          if(re_valid)begin
            if(re_status==`PAR_CALIB_INVALID)algorithm_stop();
            else if(re_status!=0)fail(re_status);
            else if(stream_error || receive_count!=`PAR_RESIDUALS || !finite(re_cost) || re_cost[63])fail(`PAR_BAD_CONFIG);
            else begin v[0]<=re_cost;pc<=2;end
          end
        end
        2:begin
          if(v[0]<64'h3c9cd2b297d889bc)begin converged<=1;pc<=RESPONSE;end
          else if(outer==`PAR_LM_MAX_ITERS)algorithm_stop();
          else begin outer<=outer+1;pc<=3;end
        end
        3:if(ne_ready)begin send_r<=0;pc<=4;end
        4:if(ne_r_ready)begin if(send_r==(`PAR_RESIDUALS-1))pc<=5;else send_r<=send_r+1;end
        5:if(jac_ready)pc<=6;
        6:if(jac_valid)begin
          if(jac_status!=0)begin pending_status<=(jac_status==`PAR_CALIB_INVALID)?0:jac_status;pc<=40;end
          else begin scales<=jac_scales;pc<=7;end
        end
        7:if(ds_load_ready)begin ne_done<=0;ds_done<=0;pc<=8;end
        8:begin
          if(ne_valid)begin ne_done<=1;saved_ne_status<=ne_status;v[2]<=max_gradient;end
          if(ds_load_valid)begin ds_done<=1;saved_ds_status<=ds_load_status;end
          if(ne_done && saved_ne_status!=0)begin pending_status<=saved_ne_status;pc<=42;end
          else if(ds_done && saved_ds_status!=0)begin pending_status<=saved_ds_status;pc<=40;end
          else if(ne_done && ds_done)pc<=9;
        end
        9:calculate(SQRT,v[0],ZERO,3,10);
        10:calculate(ADD,ONE,v[3],3,11);
        11:calculate(MUL,64'h3e45798ee2308c3a,v[3],10,12);
        12:if(v[2]<v[10])begin converged<=1;pc<=RESPONSE;end else begin attempt<=0;pc<=13;end
        13:if(ds_ready)pc<=14;
        14:if(ds_valid)begin
          if(ds_status==`PAR_CALIB_INVALID)pc<=30;
          else if(ds_status!=0)fail(ds_status);
          else begin delta<=ds_delta;trial<=current;k<=0;v[4]<=0;numeric_retry<=1;pc<=15;end
        end
        15:calculate(MUL,delta[64*k+:64],scales[64*k+:64],5,16);
        16:calculate(ADD,current[64*active_index(k)+:64],v[5],11,17);
        17:begin trial[64*active_index(k)+:64]<=v[11];calculate(ADD,ONE,magnitude(current[64*active_index(k)+:64]),6,18);end
        18:calculate(DIV,magnitude(v[5]),v[6],7,19);
        19:begin if(v[7]>v[4])v[4]<=v[7];if(k==n-1)begin numeric_retry<=0;pc<=22;end else begin k<=k+1;pc<=15;end end
        22:if(re_ready)begin receive_count<=0;stream_error<=0;pc<=23;end
        23:begin
          if(re_data_valid)begin
            if(receive_count>=`PAR_RESIDUALS || re_index!=receive_count || re_last!=(receive_count==(`PAR_RESIDUALS-1)) || !finite(re_value))stream_error<=1;
            if(receive_count<`PAR_RESIDUALS)begin trial_residual[receive_count]<=re_value;receive_count<=receive_count+1;end
          end
          if(re_valid)begin
            if(re_status==`PAR_CALIB_INVALID)pc<=30;
            else if(re_status!=0)fail(re_status);
            else if(stream_error || receive_count!=`PAR_RESIDUALS || !finite(re_cost) || re_cost[63])fail(`PAR_BAD_CONFIG);
            else if(re_cost<v[0])begin v[8]<=re_cost;pc<=24;end
            else pc<=30;
          end
        end
        24:calculate(SUB,v[0],v[8],9,25);
        25:begin current<=trial;v[0]<=v[8];accepted<=accepted+1;copy_index<=0;calculate(MUL,v[1],64'h3fd3333333333333,1,26);end
        26:begin
          residual[copy_index]<=trial_residual[copy_index];
          if(copy_index==(`PAR_RESIDUALS-1))begin if(v[1]<64'h3d719799812dea11)v[1]<=64'h3d719799812dea11;pc<=27;end else copy_index<=copy_index+1;
        end
        27:calculate(ADD,ONE,v[0],10,28);
        28:calculate(MUL,64'h3da5fd7fe1796495,v[10],10,29);
        29:if(v[4]<64'h3e112e0be826d695 || v[9]<v[10])begin converged<=1;pc<=RESPONSE;end
          else if(outer==`PAR_LM_MAX_ITERS)algorithm_stop();else pc<=2;
        30:begin numeric_retry<=0;calculate(MUL,v[1],64'h4024000000000000,1,31);end
        31:if(attempt==(`PAR_LM_MAX_TRIES-1))pc<=32;else begin attempt<=attempt+1;pc<=13;end
        32:calculate(MUL,64'h3ee4f8b588e368f1,v[3],10,33);
        33:begin converged<=(v[2]<v[10]);pc<=RESPONSE;end
        40:begin
          if(ne_valid)begin rsp_status<=pending_status;pc<=RESPONSE;end
          else if(ne_abort_ready)pc<=41;
          else if(ne_ready)begin rsp_status<=pending_status;pc<=RESPONSE;end
        end
        41:if(ne_valid)begin rsp_status<=pending_status;pc<=RESPONSE;end
        42:begin
          if(ds_load_valid || ds_load_ready)begin rsp_status<=pending_status;pc<=RESPONSE;end
          else if(ds_abort_ready)pc<=43;
        end
        43:if(ds_load_valid)begin rsp_status<=pending_status;pc<=RESPONSE;end

        default:fail(`PAR_BAD_CONFIG);
        endcase
        // 输入期间normal可能因格式/非有限数据提前结束，此时jac仍可能等待j_ready。
        // 优先消费错误并进入终态清理全部子核，不能仅等待jac的完成响应。
        if((pc==4 || pc==5 || pc==6) && ne_valid)
          fail((ne_status==0)?`PAR_BAD_CONFIG:ne_status);
      end
    end
endmodule
