`include "calib_defs.vh"

// Collect committed corner views -> one width-based seed -> full-parameter
// LM (stage 2, k3 fixed) -> validation -> camera/diagnostic publication.
// The initializer fills current state one word per cycle under backpressure.
// Jobs still identify external transactions; reset cancels work and output
// backpressure preserves the result until both consumers acknowledge it.
module calib_top (
    input wire clk, // core_clk，同一时钟域
    input wire rst_n, // 低有效复位，同步释放；取消全部在途事务
    input wire collect_valid, // 开始收集一个任务；这是新增的显式缓存初始化握手
    output wire collect_ready, // 空闲时接受，清除上一次任务有效标记
    input wire [31:0] collect_job_id, // 锁存待收集任务号
    input wire corner_valid, // 角点流有效
    output wire corner_ready, // 可接收角点
    input wire [31:0] corner_job_id, // 必须等于当前收集任务
    input wire [7:0] corner_view_id, // 0..PAR_VIEWS-1，不交织视图；按0、1、2接收
    input wire [7:0] corner_point_index, // 每视图严格0..PAR_POINTS-1
    input wire [31:0] corner_x_fp32, // IEEE754 原图像素坐标 x
    input wire [31:0] corner_y_fp32, // IEEE754 原图像素坐标 y
    input wire corner_last, // 仅 point_index=PAR_POINTS-1 为1
    input wire view_rsp_valid, // 检测模块视图完成响应，独立于角点数据流
    output wire view_rsp_ready, // 可接收检测响应，失败时即使点数不足也必须接收
    input wire [31:0] view_rsp_job_id, // 检测任务号
    input wire [7:0] view_rsp_view_id, // 当前视图0..PAR_VIEWS-1
    input wire [7:0] view_rsp_status, // 0成功；失败禁止本任务标定，保留各图状态
    output wire [`PAR_VIEWS-1:0] dbg_view_done, // bit v：第v图已收到检测结束通知
    output wire [8*`PAR_VIEWS-1:0] dbg_view_status, // 第v图[8*v +: 8]；对应done=1时解释
    output wire [`PAR_POINT_BITS*`PAR_VIEWS-1:0] dbg_view_point_count, // 第v图[PAR_POINT_BITS*v +: PAR_POINT_BITS]，已写入0..PAR_POINTS点
    output wire [`PAR_VIEWS-1:0] dbg_view_format_error, // 每图流格式错误标记；独立于检测状态码
    output wire [`PAR_VIEWS-1:0] dbg_view_usable, // 每图完整且成功；三位全1才允许标定
    input wire cmd_valid, // 命令有效
    output wire cmd_ready, // 可接受命令
    input wire [31:0] cmd_job_id, // 须与collect_job_id匹配
    input wire [15:0] cmd_width, // 图像宽度，>=2
    input wire [15:0] cmd_height, // 图像高度，>=2
    input wire [63:0] cmd_square_size_fp64, // 有限且>0；未知单位时1，仅影响输出平移
    output wire camera_valid, // 相机参数数据流 valid
    input wire camera_ready, // 下游接收参数包
    output wire camera_last, // 单拍参数包，固定为1
    output wire [31:0] camera_calib_id, // 等于job_id
    output wire [15:0] camera_width, // 绑定图像宽度
    output wire [15:0] camera_height, // 绑定图像高度
    output wire camera_usable, // 根README中的camera_valid有效性字段；与流valid区分
    output wire [`PAR_CAMERA_W-1:0] camera_params, // 低位起fx,fy,cx,cy,k1,k2,k3,p1,p2，各FP32
    output wire diag_valid, // 每任务一笔诊断；包括失败任务
    input wire diag_ready, // 必须接收；不用的诊断可始终ready=1
    output wire [31:0] diag_job_id, // 任务号
    output wire [7:0] diag_status, // 任务结果
    output wire [3:0] diag_phase, // 0收集/配置,1初值,2LM,3验证,4发布
    output wire diag_metrics_valid, // RMS/pose等数值字段是否有效
    output wire diag_converged, // 最佳seed最终阶段收敛
    output wire diag_weak_geometry, // 提示位，不单独否定参数
    output wire [63:0] diag_rms_fp64, // sqrt(best_cost/PAR_TOTAL_POINTS)
    output wire [`PAR_VIEW_RMS_W-1:0] diag_view_rms_fp64, // PAR_VIEWS个视图RMS，view0低位

    output wire [63:0] diag_max_error_fp64, // 最大角点欧氏误差
    output wire [`PAR_POSES_W-1:0] diag_poses_fp64, // 三视图R和t；平移恢复首角点原点并乘square_size
    output wire [2:0] diag_seed_id, // 2=width-based seed; 7=no valid result
    output wire [15:0] diag_accepted_steps, // 最佳seed三阶段已接受更新总数
    output wire rsp_valid, // 完成响应有效；最后一笔输出握手后才置位
    input wire rsp_ready, // 接收完成响应
    output wire [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [31:0] rsp_job_id // 完成的任务号
);
    generate if(`PAR_VIEWS<3 || `PAR_VIEWS>16 || `PAR_BOARD_ROWS<2 || `PAR_BOARD_COLS<2 ||
        `PAR_POINTS>256 || `PAR_LM_MAX_ITERS<1 || `PAR_LM_MAX_ITERS>255 ||
        `PAR_LM_MAX_TRIES<1 || `PAR_LM_MAX_TRIES>256) begin : invalid_configuration
        CALIB_CONFIGURATION_OUT_OF_SUPPORTED_RANGE invalid_config();
    end endgenerate
    localparam IDLE=0, CLEAR=1, COLLECT=2, WAIT_CMD=3,
               INIT_CMD=4, INIT_WAIT=5, LOAD_SEED=6, LM_CMD=7, LM_WAIT=8,
               NEXT_SEED=9, CHECK_CMD=10, CHECK_WAIT=11, OUTPUT=12, RESPONSE=13, SEED_FETCH=14, SEED_LATCH=15;
    reg [3:0] pc;
    reg [31:0] job;
    reg [15:0] width,height;
    reg [63:0] square_size;
    reg [7:0] status;
    reg [3:0] phase;
    reg camera_pending,diag_pending,metrics,weak_geometry;
    reg [287:0] camera_q;
    reg [63:0] rms_q,max_error_q;
    reg [`PAR_VIEW_RMS_W-1:0] view_rms_q;
    reg [`PAR_POSES_W-1:0] poses_q;

    // Single seed only.
    // Store one FP64 word per cycle. Upstream holds seed_state under
    // backpressure; publish the seed only after its final word is stored.
    localparam SEED_WORD_BITS=$clog2(`PAR_STATE_N+1);
    reg [SEED_WORD_BITS-1:0] seed_word;
    reg [2:0] seed_ids[0:0];
    reg [2:0] seed_count,seed_index;
    reg [4:0] seed_seen;
    reg [1:0] stage;
    reg [`PAR_STATE_W-1:0] current;
    wire [`PAR_STATE_W-1:0] best_state=current;
    reg [63:0] best_cost;
    reg best_valid,best_converged;
    reg [2:0] best_id;
    reg [15:0] accepted,best_accepted;

    // 用寄存器取消计算子核，不用多位pc的组合译码驱动异步复位。
    // 缓存不接此复位：诊断状态必须保留到下一collect。
    reg child_clear;
    wire compute_rst_n=rst_n && !child_clear;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) child_clear<=0;
        else child_clear<=(pc==OUTPUT || pc==RESPONSE);

    wire store_corner_ready,store_rsp_ready;
    wire init_ready,init_valid,seed_valid;
    wire [7:0] init_status;
    wire [2:0] seed_id,init_count;
    wire [`PAR_STATE_W-1:0] seed_state;
    wire seed_ready=rst_n && pc==INIT_WAIT && seed_word==`PAR_STATE_N-1;
    wire lm_ready,lm_valid,lm_converged;
    wire [7:0] lm_status,lm_accepted;
    wire [`PAR_STATE_W-1:0] lm_state;
    wire [63:0] lm_cost;
    wire check_ready,check_valid,check_usable,check_metrics,check_weak;
    wire [7:0] check_status;
    wire [287:0] check_camera;
    wire [63:0] check_rms,check_max;
    wire [`PAR_VIEW_RMS_W-1:0] check_view_rms;
    wire [`PAR_POSES_W-1:0] check_poses;
    wire init_rd,lm_rd,check_rd,rd_valid;
    wire [`PAR_VIEW_BITS-1:0] init_view,lm_view,check_view;
    wire [`PAR_POINT_BITS-1:0] init_index,lm_index,check_index;
    wire [31:0] rd_x,rd_y;
    wire init_owner=(pc==INIT_CMD || pc==INIT_WAIT);
    wire lm_owner=(pc==LM_CMD || pc==LM_WAIT);
    wire check_owner=(pc==CHECK_CMD || pc==CHECK_WAIT);
    wire read_en=rst_n && ((init_owner && init_rd) || (lm_owner && lm_rd) || (check_owner && check_rd));
    wire [`PAR_VIEW_BITS-1:0] read_view=init_owner?init_view:(lm_owner?lm_view:check_view);
    wire [`PAR_POINT_BITS-1:0] read_index=init_owner?init_index:(lm_owner?lm_index:check_index);

    // 非法job/view和已经完成的视图必须被接收后报错，不能因缓存ready=0永久等待。
    // 对未完成图的乱序/last/非有限坐标检查交给corner_store，保留其错误标记。
    wire bad_corner=(corner_job_id!=job || corner_view_id>=`PAR_VIEWS) ? 1'b1 : dbg_view_done[corner_view_id];
    wire bad_view=(view_rsp_job_id!=job || view_rsp_view_id>=`PAR_VIEWS) ? 1'b1 : dbg_view_done[view_rsp_view_id];
    reg [7:0] detected_failure;
    integer failure_view;
    always @* begin
        detected_failure=0;
        for(failure_view=0;failure_view<`PAR_VIEWS;failure_view=failure_view+1)
            if(detected_failure==0 && dbg_view_done[failure_view] && dbg_view_status[8*failure_view+:8]!=0)
                detected_failure=dbg_view_status[8*failure_view+:8];
    end
    wire stored_failure=(|dbg_view_format_error) || detected_failure!=0;
    wire collecting=rst_n && pc==COLLECT && !stored_failure;
    // 同拍一条流非法时不再把另一条流写进缓存，任务整体失败。
    wire input_fault=(corner_valid && bad_corner) || (view_rsp_valid && bad_view);
    assign corner_ready=collecting && (input_fault ? (corner_valid && bad_corner) : store_corner_ready);
    assign view_rsp_ready=collecting && (input_fault ? (view_rsp_valid && bad_view) : store_rsp_ready);
    assign collect_ready=rst_n && pc==IDLE;
    assign cmd_ready=rst_n && pc==WAIT_CMD;
    assign camera_valid=rst_n && pc==OUTPUT && camera_pending;
    assign diag_valid=rst_n && pc==OUTPUT && diag_pending;
    assign rsp_valid=rst_n && pc==RESPONSE;
    assign camera_last=1'b1;
    assign camera_calib_id=job;
    assign camera_width=width;
    assign camera_height=height;
    assign camera_usable=(status==`PAR_OK && metrics);
    assign camera_params=camera_q;
    assign diag_job_id=job;
    assign diag_status=status;
    assign diag_phase=phase;
    assign diag_metrics_valid=metrics;
    assign diag_converged=best_valid && best_converged;
    assign diag_weak_geometry=weak_geometry;
    assign diag_rms_fp64=rms_q;
    assign diag_view_rms_fp64=view_rms_q;
    assign diag_max_error_fp64=max_error_q;
    assign diag_poses_fp64=poses_q;
    assign diag_seed_id=best_valid?best_id:3'd7;
    assign diag_accepted_steps=best_valid?best_accepted:16'd0;
    assign rsp_status=status;
    assign rsp_job_id=job;

     wire [25:0] shared_req_valid;
     wire [25:0] shared_req_ready;
     wire [129:0] shared_req_op;
     wire [1663:0] shared_req_a;
     wire [1663:0] shared_req_b;
     wire [25:0] shared_active;
     wire [25:0] shared_rsp_valid;
     wire [25:0] shared_rsp_ready;
    wire [63:0] shared_rsp_result;
    wire [4:0] shared_rsp_flags;

    // LM and validation are mutually exclusive; share the complete residual datapath.
    wire  lm_ext_rst_n,check_ext_rst_n,service_rst_n;
    assign service_rst_n=check_owner?check_ext_rst_n:lm_ext_rst_n;
    wire  lm_ext_cmd_valid,check_ext_cmd_valid,service_cmd_valid;
    assign service_cmd_valid=check_owner?check_ext_cmd_valid:lm_ext_cmd_valid;
    wire  lm_ext_cmd_ready,check_ext_cmd_ready,service_cmd_ready;
    assign lm_ext_cmd_ready=lm_owner && service_cmd_ready;
    assign check_ext_cmd_ready=check_owner && service_cmd_ready;
    wire [15:0] lm_ext_cmd_width,check_ext_cmd_width,service_cmd_width;
    assign service_cmd_width=check_owner?check_ext_cmd_width:lm_ext_cmd_width;
    wire [15:0] lm_ext_cmd_height,check_ext_cmd_height,service_cmd_height;
    assign service_cmd_height=check_owner?check_ext_cmd_height:lm_ext_cmd_height;
    wire [`PAR_STATE_W-1:0] lm_ext_cmd_state,check_ext_cmd_state,service_cmd_state;
    assign service_cmd_state=check_owner?check_ext_cmd_state:lm_ext_cmd_state;
    wire  lm_ext_point_rd_en,check_ext_point_rd_en,service_point_rd_en;
    assign lm_ext_point_rd_en=lm_owner && service_point_rd_en;
    assign check_ext_point_rd_en=check_owner && service_point_rd_en;
    wire [`PAR_VIEW_BITS-1:0] lm_ext_point_rd_view_id,check_ext_point_rd_view_id,service_point_rd_view_id;
    assign lm_ext_point_rd_view_id=service_point_rd_view_id;
    assign check_ext_point_rd_view_id=service_point_rd_view_id;
    wire [`PAR_POINT_BITS-1:0] lm_ext_point_rd_index,check_ext_point_rd_index,service_point_rd_index;
    assign lm_ext_point_rd_index=service_point_rd_index;
    assign check_ext_point_rd_index=service_point_rd_index;
    wire  lm_ext_point_rd_valid,check_ext_point_rd_valid,service_point_rd_valid;
    assign service_point_rd_valid=check_owner?check_ext_point_rd_valid:lm_ext_point_rd_valid;
    wire [31:0] lm_ext_point_rd_x_fp32,check_ext_point_rd_x_fp32,service_point_rd_x_fp32;
    assign service_point_rd_x_fp32=check_owner?check_ext_point_rd_x_fp32:lm_ext_point_rd_x_fp32;
    wire [31:0] lm_ext_point_rd_y_fp32,check_ext_point_rd_y_fp32,service_point_rd_y_fp32;
    assign service_point_rd_y_fp32=check_owner?check_ext_point_rd_y_fp32:lm_ext_point_rd_y_fp32;
    wire  lm_ext_data_valid,check_ext_data_valid,service_data_valid;
    assign lm_ext_data_valid=lm_owner && service_data_valid;
    assign check_ext_data_valid=check_owner && service_data_valid;
    wire  lm_ext_data_ready,check_ext_data_ready,service_data_ready;
    assign service_data_ready=check_owner?check_ext_data_ready:lm_ext_data_ready;
    wire [`PAR_RES_BITS-1:0] lm_ext_data_index,check_ext_data_index,service_data_index;
    assign lm_ext_data_index=service_data_index;
    assign check_ext_data_index=service_data_index;
    wire [63:0] lm_ext_data_fp64,check_ext_data_fp64,service_data_fp64;
    assign lm_ext_data_fp64=service_data_fp64;
    assign check_ext_data_fp64=service_data_fp64;
    wire  lm_ext_data_last,check_ext_data_last,service_data_last;
    assign lm_ext_data_last=service_data_last;
    assign check_ext_data_last=service_data_last;
    wire  lm_ext_rsp_valid,check_ext_rsp_valid,service_rsp_valid;
    assign lm_ext_rsp_valid=lm_owner && service_rsp_valid;
    assign check_ext_rsp_valid=check_owner && service_rsp_valid;
    wire  lm_ext_rsp_ready,check_ext_rsp_ready,service_rsp_ready;
    assign service_rsp_ready=check_owner?check_ext_rsp_ready:lm_ext_rsp_ready;
    wire [7:0] lm_ext_rsp_status,check_ext_rsp_status,service_rsp_status;
    assign lm_ext_rsp_status=service_rsp_status;
    assign check_ext_rsp_status=service_rsp_status;
    wire [63:0] lm_ext_rsp_cost_fp64,check_ext_rsp_cost_fp64,service_rsp_cost_fp64;
    assign lm_ext_rsp_cost_fp64=service_rsp_cost_fp64;
    assign check_ext_rsp_cost_fp64=service_rsp_cost_fp64;
    residual_engine #(.FP_SHARED(1)) shared_residual(.clk(clk),.rst_n(service_rst_n),.cmd_valid(service_cmd_valid),.cmd_ready(service_cmd_ready),.cmd_width(service_cmd_width),.cmd_height(service_cmd_height),.cmd_state(service_cmd_state),.point_rd_en(service_point_rd_en),.point_rd_view_id(service_point_rd_view_id),.point_rd_index(service_point_rd_index),.point_rd_valid(service_point_rd_valid),.point_rd_x_fp32(service_point_rd_x_fp32),.point_rd_y_fp32(service_point_rd_y_fp32),.data_valid(service_data_valid),.data_ready(service_data_ready),.data_index(service_data_index),.data_fp64(service_data_fp64),.data_last(service_data_last),.rsp_valid(service_rsp_valid),.rsp_ready(service_rsp_ready),.rsp_status(service_rsp_status),.rsp_cost_fp64(service_rsp_cost_fp64),
      .shared_req_valid(shared_req_valid[22+:4]),.shared_req_ready(shared_req_ready[22+:4]),.shared_req_op(shared_req_op[110+:20]),.shared_req_a(shared_req_a[1408+:256]),.shared_req_b(shared_req_b[1408+:256]),.shared_active(shared_active[22+:4]),.shared_rsp_valid(shared_rsp_valid[22+:4]),.shared_rsp_ready(shared_rsp_ready[22+:4]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags));
    fp_calibration_pool #(.CLIENTS(26)) arithmetic_pool(
        .clk(clk),.rst_n(compute_rst_n),
.c_req_valid(shared_req_valid),.c_req_ready(shared_req_ready),.c_req_op(shared_req_op),.c_req_a(shared_req_a),.c_req_b(shared_req_b),.c_active(shared_active),.c_rsp_valid(shared_rsp_valid),.c_rsp_ready(shared_rsp_ready),
        .result(shared_rsp_result),.flags(shared_rsp_flags));
    corner_store store(.clk(clk),.rst_n(rst_n),.clear(pc==CLEAR),
        .corner_valid(collecting && !input_fault && corner_valid),.corner_ready(store_corner_ready),
        .corner_view_id(corner_view_id),.corner_point_index(corner_point_index),
        .corner_x_fp32(corner_x_fp32),.corner_y_fp32(corner_y_fp32),.corner_last(corner_last),
        .view_rsp_valid(collecting && !input_fault && view_rsp_valid),.view_rsp_ready(store_rsp_ready),
        .view_rsp_view_id(view_rsp_view_id),.view_rsp_status(view_rsp_status),
        .view_done(dbg_view_done),.view_status(dbg_view_status),.view_point_count(dbg_view_point_count),
        .view_format_error(dbg_view_format_error),.view_usable(dbg_view_usable),
        .rd_en(read_en),.rd_view_id(read_view),.rd_point_index(read_index),
        .rd_valid(rd_valid),.rd_x_fp32(rd_x),.rd_y_fp32(rd_y));
    init_controller #(.FP_SHARED(1)) initializer(.shared_req_valid(shared_req_valid[0 +: 7]),.shared_req_ready(shared_req_ready[0 +: 7]),.shared_req_op(shared_req_op[0 +: 35]),.shared_req_a(shared_req_a[0 +: 448]),.shared_req_b(shared_req_b[0 +: 448]),.shared_active(shared_active[0 +: 7]),.shared_rsp_valid(shared_rsp_valid[0 +: 7]),.shared_rsp_ready(shared_rsp_ready[0 +: 7]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(compute_rst_n),
        .cmd_valid(rst_n && pc==INIT_CMD),.cmd_ready(init_ready),.cmd_width(width),.cmd_height(height),
        .point_rd_en(init_rd),.point_rd_view_id(init_view),.point_rd_index(init_index),
        .point_rd_valid(init_owner && rd_valid),.point_rd_x_fp32(rd_x),.point_rd_y_fp32(rd_y),
        .seed_valid(seed_valid),.seed_ready(seed_ready),.seed_id(seed_id),.seed_state(seed_state),
        .rsp_valid(init_valid),.rsp_ready(rst_n && pc==INIT_WAIT),.rsp_status(init_status),.rsp_seed_count(init_count));
    lm_controller #(.FP_SHARED(1),.SHARE_RESIDUAL(1)) optimizer(.ext_rst_n(lm_ext_rst_n),.ext_cmd_valid(lm_ext_cmd_valid),.ext_cmd_ready(lm_ext_cmd_ready),.ext_cmd_width(lm_ext_cmd_width),.ext_cmd_height(lm_ext_cmd_height),.ext_cmd_state(lm_ext_cmd_state),.ext_point_rd_en(lm_ext_point_rd_en),.ext_point_rd_view_id(lm_ext_point_rd_view_id),.ext_point_rd_index(lm_ext_point_rd_index),.ext_point_rd_valid(lm_ext_point_rd_valid),.ext_point_rd_x_fp32(lm_ext_point_rd_x_fp32),.ext_point_rd_y_fp32(lm_ext_point_rd_y_fp32),.ext_data_valid(lm_ext_data_valid),.ext_data_ready(lm_ext_data_ready),.ext_data_index(lm_ext_data_index),.ext_data_fp64(lm_ext_data_fp64),.ext_data_last(lm_ext_data_last),.ext_rsp_valid(lm_ext_rsp_valid),.ext_rsp_ready(lm_ext_rsp_ready),.ext_rsp_status(lm_ext_rsp_status),.ext_rsp_cost_fp64(lm_ext_rsp_cost_fp64),.shared_req_valid(shared_req_valid[7 +: 9]),.shared_req_ready(shared_req_ready[7 +: 9]),.shared_req_op(shared_req_op[35 +: 45]),.shared_req_a(shared_req_a[448 +: 576]),.shared_req_b(shared_req_b[448 +: 576]),.shared_active(shared_active[7 +: 9]),.shared_rsp_valid(shared_rsp_valid[7 +: 9]),.shared_rsp_ready(shared_rsp_ready[7 +: 9]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(compute_rst_n),
        .cmd_valid(rst_n && pc==LM_CMD),.cmd_ready(lm_ready),.cmd_width(width),.cmd_height(height),
        .cmd_state(current),.cmd_stage(stage),
        .point_rd_en(lm_rd),.point_rd_view_id(lm_view),.point_rd_index(lm_index),
        .point_rd_valid(lm_owner && rd_valid),.point_rd_x_fp32(rd_x),.point_rd_y_fp32(rd_y),
        .rsp_valid(lm_valid),.rsp_ready(rst_n && pc==LM_WAIT),.rsp_status(lm_status),
        .rsp_state(lm_state),.rsp_cost_fp64(lm_cost),.rsp_converged(lm_converged),
        .rsp_accepted_steps(lm_accepted),.rsp_outer_iterations());
    validate_result #(.FP_SHARED(1),.SHARE_RESIDUAL(1)) validator(.ext_rst_n(check_ext_rst_n),.ext_cmd_valid(check_ext_cmd_valid),.ext_cmd_ready(check_ext_cmd_ready),.ext_cmd_width(check_ext_cmd_width),.ext_cmd_height(check_ext_cmd_height),.ext_cmd_state(check_ext_cmd_state),.ext_point_rd_en(check_ext_point_rd_en),.ext_point_rd_view_id(check_ext_point_rd_view_id),.ext_point_rd_index(check_ext_point_rd_index),.ext_point_rd_valid(check_ext_point_rd_valid),.ext_point_rd_x_fp32(check_ext_point_rd_x_fp32),.ext_point_rd_y_fp32(check_ext_point_rd_y_fp32),.ext_data_valid(check_ext_data_valid),.ext_data_ready(check_ext_data_ready),.ext_data_index(check_ext_data_index),.ext_data_fp64(check_ext_data_fp64),.ext_data_last(check_ext_data_last),.ext_rsp_valid(check_ext_rsp_valid),.ext_rsp_ready(check_ext_rsp_ready),.ext_rsp_status(check_ext_rsp_status),.ext_rsp_cost_fp64(check_ext_rsp_cost_fp64),.shared_req_valid(shared_req_valid[16 +: 6]),.shared_req_ready(shared_req_ready[16 +: 6]),.shared_req_op(shared_req_op[80 +: 30]),.shared_req_a(shared_req_a[1024 +: 384]),.shared_req_b(shared_req_b[1024 +: 384]),.shared_active(shared_active[16 +: 6]),.shared_rsp_valid(shared_rsp_valid[16 +: 6]),.shared_rsp_ready(shared_rsp_ready[16 +: 6]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(compute_rst_n),
        .cmd_valid(rst_n && pc==CHECK_CMD),.cmd_ready(check_ready),.cmd_width(width),.cmd_height(height),
        .cmd_state(best_state),.cmd_square_size_fp64(square_size),.cmd_best_cost_fp64(best_cost),.cmd_converged(best_converged),
        .point_rd_en(check_rd),.point_rd_view_id(check_view),.point_rd_index(check_index),
        .point_rd_valid(check_owner && rd_valid),.point_rd_x_fp32(rd_x),.point_rd_y_fp32(rd_y),
        .rsp_valid(check_valid),.rsp_ready(rst_n && pc==CHECK_WAIT),.rsp_status(check_status),
        .rsp_camera_usable(check_usable),.rsp_camera_params(check_camera),.rsp_metrics_valid(check_metrics),
        .rsp_weak_geometry(check_weak),.rsp_rms_fp64(check_rms),.rsp_view_rms_fp64(check_view_rms),
        .rsp_max_error_fp64(check_max),.rsp_poses_fp64(check_poses));

    task fail;
        input [7:0] code;
        input [3:0] where;
        begin
            status<=code;phase<=where;metrics<=0;weak_geometry<=0;
            camera_q<=0;rms_q<=0;view_rms_q<=0;max_error_q<=0;poses_q<=0;
            camera_pending<=0;diag_pending<=1;pc<=OUTPUT;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pc<=IDLE;job<=0;width<=0;height<=0;square_size<=0;status<=0;phase<=0;
            camera_pending<=0;diag_pending<=0;metrics<=0;weak_geometry<=0;
            camera_q<=0;rms_q<=0;view_rms_q<=0;max_error_q<=0;poses_q<=0;
            seed_count<=0;seed_index<=0;seed_seen<=0;seed_word<=0;stage<=2;current<=0;
            best_cost<=64'h7ff0000000000000;best_valid<=0;
            best_converged<=0;best_id<=7;accepted<=0;best_accepted<=0;
        end else case(pc)
            IDLE: if(collect_valid) begin
                job<=collect_job_id;pc<=CLEAR;width<=0;height<=0;square_size<=0;
                status<=0;phase<=0;metrics<=0;weak_geometry<=0;
                camera_pending<=0;diag_pending<=0;camera_q<=0;
                rms_q<=0;view_rms_q<=0;max_error_q<=0;poses_q<=0;
                seed_count<=0;seed_index<=0;seed_seen<=0;seed_word<=0;stage<=2;accepted<=0;
                best_valid<=0;best_converged<=0;best_id<=7;best_accepted<=0;
                best_cost<=64'h7ff0000000000000;
            end
            CLEAR: pc<=COLLECT;
            COLLECT: begin
                if(stored_failure) begin
                    if(|dbg_view_format_error) fail(`PAR_BAD_CONFIG,0);
                    else fail(detected_failure,0);
                end else if(input_fault) fail(`PAR_BAD_CONFIG,0);
                else if(&dbg_view_usable) pc<=WAIT_CMD;
            end
            WAIT_CMD: if(cmd_valid) begin
                width<=cmd_width;height<=cmd_height;square_size<=cmd_square_size_fp64;
                if(cmd_job_id!=job || cmd_width<2 || cmd_height<2 ||
                   cmd_square_size_fp64[63] || cmd_square_size_fp64[62:0]==0 ||
                   cmd_square_size_fp64[62:52]==11'h7ff) fail(`PAR_BAD_CONFIG,0);
                else begin phase<=1;pc<=INIT_CMD;end
            end
            INIT_CMD: if(init_ready) pc<=INIT_WAIT;
            INIT_WAIT: begin
                // 正常协议的rsp在最后一份seed之后；数量/ID异常直接终止，不让数组越界。
                if(seed_valid) begin
                    if(seed_count!=0 || seed_id!=2) fail(`PAR_BAD_CONFIG,1);

                    else begin
                        current[64*seed_word+:64]<=seed_state[64*seed_word+:64];
                        if(seed_ready) begin
                            seed_ids[0]<=seed_id;
                            seed_seen[seed_id]<=1;seed_count<=seed_count+1'b1;seed_word<=0;
                        end else seed_word<=seed_word+1'b1;
                    end
                end
                if(init_valid) begin
                    if(init_status!=0) fail(init_status,1);
                    else if(seed_valid || init_count!=seed_count) fail(`PAR_BAD_CONFIG,1);
                    else if(seed_count==0) fail(`PAR_CALIB_INVALID,1);
                    else begin seed_index<=0;phase<=2;pc<=LM_CMD;end
                end
            end
            LM_CMD: if(lm_ready) pc<=LM_WAIT;
            LM_WAIT: if(lm_valid) begin
                if(lm_status!=0) fail(lm_status,2);
                else begin
                    current<=lm_state;accepted<=accepted+{8'b0,lm_accepted};
                    begin
                        // 对非负有限FP64，去掉符号后的无符号位序就是数值序。
                        // +Inf/NaN无资格成为best；-0按+0处理。相等cost保留较早seed。
                        if((!lm_cost[63] || lm_cost[62:0]==0) && lm_cost[62:52]!=11'h7ff &&
                           (!best_valid || {1'b0,lm_cost[62:0]}<best_cost)) begin
                            best_cost<={1'b0,lm_cost[62:0]};best_valid<=1;
                            best_converged<=lm_converged;best_id<=seed_ids[0];
                            best_accepted<=accepted+{8'b0,lm_accepted};
                        end
                        pc<=NEXT_SEED;
                    end
                end
            end
            NEXT_SEED: begin
                if(!best_valid) fail(`PAR_CALIB_INVALID,2);
                else begin phase<=3;pc<=CHECK_CMD;end
            end
            CHECK_CMD: if(check_ready) pc<=CHECK_WAIT;
            CHECK_WAIT: if(check_valid) begin
                // 成功状态必须同时有usable与metrics；不能发布不完整的参数包。
                if(check_status==0 && (!check_usable || !check_metrics)) fail(`PAR_BAD_CONFIG,3);
                else begin
                    status<=check_status;phase<=(check_status==0)?4:3;
                    metrics<=check_metrics;weak_geometry<=check_metrics && check_weak;
                    rms_q<=check_metrics?check_rms:64'b0;
                    view_rms_q<=check_metrics?check_view_rms:{`PAR_VIEW_RMS_W{1'b0}};
                    max_error_q<=check_metrics?check_max:64'b0;
                    poses_q<=check_metrics?check_poses:{`PAR_POSES_W{1'b0}};
                    camera_q<=(check_status==0)?check_camera:288'b0;
                    camera_pending<=(check_status==0);diag_pending<=1;pc<=OUTPUT;
                end
            end
            OUTPUT: begin
                if(camera_pending && camera_ready) camera_pending<=0;
                if(diag_pending && diag_ready) diag_pending<=0;
                if((!camera_pending || camera_ready) && (!diag_pending || diag_ready)) pc<=RESPONSE;
            end
            RESPONSE: if(rsp_ready) pc<=IDLE;
            default: fail(`PAR_BAD_CONFIG,phase);
        endcase
    end
endmodule
