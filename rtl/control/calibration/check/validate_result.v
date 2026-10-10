`include "calibration_program_defs.vh"
`include "calib_defs.vh"
// Final report/validation adapter for the shared calibration microprogram.
module validate_result #(parameter FP_SHARED=0, SHARE_RESIDUAL=0, ENGINE_SHARED=0) (
    output wire [127:0] execution_req, input wire [95:0] execution_rsp,
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
    output wire [5:0] shared_req_valid,
    input wire [5:0] shared_req_ready,
    output wire [29:0] shared_req_op,
    output wire [383:0] shared_req_a,
    output wire [383:0] shared_req_b,
    output wire [5:0] shared_active,
    input wire [5:0] shared_rsp_valid,
    output wire [5:0] shared_rsp_ready,
    input wire [63:0] shared_rsp_result,
    input wire [4:0] shared_rsp_flags,

    input wire clk, // core_clk，同一时钟域
    input wire rst_n, // 低有效复位，同步释放；取消全部在途事务
    input wire cmd_valid, // 命令有效
    output wire cmd_ready, // 可接受命令
    input wire [15:0] cmd_width, // 图像宽度，>=2
    input wire [15:0] cmd_height, // 图像高度，>=2
    input wire [`PAR_STATE_W-1:0] cmd_state, // PAR_STATE_N 项 FP64 状态快照，命令握手时锁存
    input wire [63:0] cmd_square_size_fp64, // 正有限格长
    input wire [63:0] cmd_best_cost_fp64, // best最终cost
    input wire cmd_converged, // best最终阶段返回值
    output wire point_rd_en, // 固定1拍角点读使能，无ready，发起前预留接收空间
    output wire [`PAR_VIEW_BITS-1:0] point_rd_view_id, // 0..PAR_VIEWS-1
    output wire [`PAR_POINT_BITS-1:0] point_rd_index, // 图内角点0..PAR_POINTS-1
    input wire point_rd_valid, // 固定1拍返回，必须当拍消费；无返回背压
    input wire [31:0] point_rd_x_fp32, // 原图x
    input wire [31:0] point_rd_y_fp32, // 原图y
    output wire rsp_valid, // 完成响应有效；最后一笔输出握手后才置位
    input wire rsp_ready, // 接收完成响应
    output reg [7:0] rsp_status, // PAR_*；失败诊断是否有效由rsp_metrics_valid单独判断
    output wire rsp_camera_usable, // 所有检查通过
    output wire [`PAR_CAMERA_W-1:0] rsp_camera_params, // FP32参数；仅usable时发布给校正模块
    output wire rsp_metrics_valid, // 下面诊断数值可用
    output wire rsp_weak_geometry, // 弱几何提示
    output wire [63:0] rsp_rms_fp64, // 总RMS
    output wire [`PAR_VIEW_RMS_W-1:0] rsp_view_rms_fp64, // 各视图RMS
    output wire [63:0] rsp_max_error_fp64, // 最大角点误差
    output wire [`PAR_POSES_W-1:0] rsp_poses_fp64 // 3份R(9项)+物理t(3项)
);
    `include "lm_program_defs.vh"
    `include "validate_program_defs.vh"
    // Protocol adapter for report/check microprogram; all numeric work is instructions.
    `include "calib_workspace_layout.vh"
    localparam IDLE=0,INITIALIZE=1,INPUT_STATE=2,START=3,RUN=4,
      META_REQ=5,META_WAIT=6,STATE_REQ=7,STATE_WAIT=8,RES_CMD=9,RES_WAIT=10,
      STORE_STATUS=11,STORE_COST=12,RESUME=13,OUT_REQ=14,OUT_WAIT=15,RESPONSE=16;
    reg [4:0] state;
    reg child_clear;
    wire local_rst_n=rst_n && !child_clear;
    always @(posedge clk or negedge rst_n)
      if(!rst_n)child_clear<=0;else child_clear<=state==RESPONSE;
    reg [31:0] index,destination,received;
    reg [15:0] width,height;
    reg [63:0] square_q,best_q;reg input_converged;
    reg [287:0] camera_q;reg [`PAR_POSES_W-1:0] poses_q;reg [`PAR_VIEW_RMS_W-1:0] view_rms_q;
    reg [63:0] rms_q,maxerr_q;reg metrics_q,usable_q,weak_q;
    localparam OUT_POSES=9,OUT_VIEWS=9+12*`PAR_VIEWS,OUT_SCALARS=9+13*`PAR_VIEWS;
    reg [`PAR_STATE_W-1:0] payload;
    reg [63:0] service_cost;
    reg [7:0] service_status;
    reg stream_error;
    wire engine_ready,engine_valid,svc_valid;
    wire [7:0] engine_status,svc_id;
    wire host_ready,host_valid,host_error;
    wire [63:0] host_result;
    reg host_en,host_write;
    reg [31:0] host_addr;
    reg [63:0] host_data,initial_data;
    wire imem_en;wire [15:0] imem_addr;reg [31:0] imem_data;
    reg [31:0] program_memory[0:2047];integer rom_i,rom_j;
    initial begin
      for(rom_j=0;rom_j<2;rom_j=rom_j+1)
        for(rom_i=0;rom_i<1024;rom_i=rom_i+1)program_memory[rom_j*1024+rom_i]=32'h040000e1;
      `include "validate_program_init.vh"
    end
    always @(posedge clk)if(imem_en)imem_data<=program_memory[imem_addr[10:0]];
    `include "calib_geometry.vh"
    function [63:0] u16;input [15:0] x;integer k,top;reg [63:0] shifted;reg [10:0] exponent;
      begin top=0;for(k=0;k<16;k=k+1)if(x[k])top=k;shifted={48'd0,x}<<(52-top);exponent=1023+top;
      u16=x==0?64'd0:{1'b0,exponent,shifted[51:0]};end endfunction
    always @* begin
      initial_data=0;
      case(index)
        `LM_N:initial_data=`PAR_ACTIVE_N;
        `LM_CAMERA:initial_data=PLUS;
        `LM_POSES:initial_data=JAC;
        `LM_VIEWRMS:initial_data=SCALES;
        `LM_VIEWS:initial_data=`PAR_VIEWS;
        `LM_POINTS:initial_data=`PAR_POINTS;
        `LM_FP_POINTS:initial_data=u16(`PAR_POINTS);
        `LM_FP_TOTAL:initial_data=u16(`PAR_TOTAL_POINTS);
        `LM_FWIDTH:initial_data=u16(width);
        `LM_FHEIGHT:initial_data=u16(height);
        `LM_HALFCOLS:initial_data=board_half(`PAR_BOARD_COLS-1);
        `LM_HALFROWS:initial_data=board_half(`PAR_BOARD_ROWS-1);
        `LM_SQUARE:initial_data=square_q;
        `LM_BESTCOST:initial_data=best_q;
        `LM_IN_CONVERGED:initial_data={63'd0,input_converged};
        `LM_NR:initial_data=`PAR_RESIDUALS;
        `LM_NS:initial_data=`PAR_STATE_N;
        `LM_MAXITER:initial_data=`PAR_LM_MAX_ITERS;
        `LM_MAXTRIES:initial_data=`PAR_LM_MAX_TRIES;
        `LM_DESC:initial_data=DESC;
        `LM_CURRENT:initial_data=CURRENT;
        `LM_TRIAL:initial_data=TRIAL;
        `LM_BASELINE:initial_data=BASELINE;
        `LM_PLUS:initial_data=PLUS;
        `LM_MINUS:initial_data=MINUS;
        `LM_COLUMN:initial_data=COLUMN;
        `LM_JAC:initial_data=JAC;
        `LM_NORMAL:initial_data=NORMAL;
        `LM_AUGMENTED:initial_data=AUGMENTED;
        `LM_SCALES:initial_data=SCALES;
        `LM_DELTA:initial_data=DELTA;
        `LM_GRADIENT:initial_data=GRADIENT;
        `include "lm_constants.vh"
        default:initial_data=0;
      endcase
    end
    wire re_ready,re_valid,re_data_valid,re_last;
    wire [`PAR_RES_BITS-1:0] re_index;
    wire [7:0] re_status;
    wire [63:0] re_value,re_cost;
    wire re_data_ready=state==RES_WAIT && host_ready;
    always @* begin
      host_en=0;host_write=0;host_addr=0;host_data=0;
      case(state)
        INITIALIZE:begin host_en=1;host_write=1;host_addr=index;host_data=initial_data;end
        INPUT_STATE:begin host_en=1;host_write=1;host_addr=CURRENT+index;host_data=payload[63:0];end
        META_REQ:begin host_en=1;host_addr=`LM_SVC_DST;end
        STATE_REQ:begin host_en=1;host_addr=TRIAL+index;end
        RES_WAIT:begin host_en=re_data_valid && received<`PAR_RESIDUALS;host_write=1;
          host_addr=destination+received;host_data=re_value;end
        STORE_STATUS:begin host_en=1;host_write=1;host_addr=`LM_SVC_STATUS;host_data={56'd0,service_status};end
        STORE_COST:begin host_en=1;host_write=1;host_addr=`LM_SVC_COST;host_data=service_cost;end
        OUT_REQ:begin host_en=1;
          if(index<OUT_POSES)host_addr=PLUS+index;
          else if(index<OUT_VIEWS)host_addr=JAC+index-OUT_POSES;
          else if(index<OUT_SCALARS)host_addr=SCALES+index-OUT_VIEWS;
          else case(index-OUT_SCALARS)
            0:host_addr=`LM_Q23;1:host_addr=`LM_MAXERR;2:host_addr=`LM_USABLE;
            3:host_addr=`LM_METRICS;4:host_addr=`LM_WEAK;default:host_addr=`LM_STATUS;
          endcase
        end
        default:begin end
      endcase
    end
    wire fv,fr,sv,sr;wire [4:0] fo,ff;wire [63:0] fa,fb,fd;
    calib_execution_port #(.EXTERNAL(ENGINE_SHARED),.PROGRAM_BASE(`VALIDATE_ENGINE_PC),.RAM_WORDS(WORDS),.CONST_BASE(WORDS),
      .HOST_CALLS(1)) engine(
      .execution_req(execution_req),.execution_rsp(execution_rsp),
      .clk(clk),.rst_n(local_rst_n),.start_valid(state==START),.start_ready(engine_ready),
      .start_pc(16'd0),.program_words(16'd`VALIDATE_PROGRAM_WORDS),
      .imem_en(imem_en),.imem_addr(imem_addr),.imem_data(imem_data),
      .host_en(host_en),.host_write(host_write),.host_addr(host_addr),.host_data(host_data),
      .host_ready(host_ready),.host_valid(host_valid),.host_error(host_error),.host_result(host_result),
      .busy(),.rsp_valid(engine_valid),.rsp_ready(state==RUN),.rsp_status(engine_status),
      .debug_pc(),.cycle_count(),.instruction_count(),
      .svc_valid(svc_valid),.svc_ready(state==RESUME),.svc_id(svc_id),
      .shared_req_valid(fv),.shared_req_ready(fr),.shared_req_op(fo),.shared_req_a(fa),.shared_req_b(fb),
      .shared_rsp_valid(sv),.shared_rsp_ready(sr),.shared_rsp_result(fd),.shared_rsp_flags(ff));
    generate if(FP_SHARED)begin : g_shared
      assign shared_req_valid[0]=fv;assign shared_req_op[4:0]=fo;
      assign shared_req_a[63:0]=fa;assign shared_req_b[63:0]=fb;
      assign shared_active[0]=local_rst_n;assign shared_rsp_ready[0]=sr;
      assign fr=shared_req_ready[0];assign sv=shared_rsp_valid[0];
      assign fd=shared_rsp_result;assign ff=shared_rsp_flags;
    end else begin : g_local
      fp_operator #(.FP_W(64),.ENABLE_LOG(0)) arithmetic(.clk(clk),.rst_n(local_rst_n),.req_valid(fv),.req_ready(fr),
        .req_op(fo),.req_a(fa),.req_b(fb),.rsp_valid(sv),.rsp_ready(sr),
        .rsp_result(fd),.rsp_flags(ff),.rsp_less(),.rsp_equal(),.rsp_unordered());
      assign shared_req_valid[0]=0;assign shared_req_op[4:0]=0;
      assign shared_req_a[63:0]=0;assign shared_req_b[63:0]=0;
      assign shared_active[0]=0;assign shared_rsp_ready[0]=0;
    end endgenerate
    assign shared_req_valid[1]=0;assign shared_req_op[9:5]=0;
    assign shared_req_a[127:64]=0;assign shared_req_b[127:64]=0;
    assign shared_active[1]=0;assign shared_rsp_ready[1]=0;
    residual_endpoint #(.FP_SHARED(FP_SHARED),.EXTERNAL(SHARE_RESIDUAL)) residual_service(.ext_rst_n(ext_rst_n),.ext_cmd_valid(ext_cmd_valid),.ext_cmd_ready(ext_cmd_ready),.ext_cmd_width(ext_cmd_width),.ext_cmd_height(ext_cmd_height),.ext_cmd_state(ext_cmd_state),.ext_point_rd_en(ext_point_rd_en),.ext_point_rd_view_id(ext_point_rd_view_id),.ext_point_rd_index(ext_point_rd_index),.ext_point_rd_valid(ext_point_rd_valid),.ext_point_rd_x_fp32(ext_point_rd_x_fp32),.ext_point_rd_y_fp32(ext_point_rd_y_fp32),.ext_data_valid(ext_data_valid),.ext_data_ready(ext_data_ready),.ext_data_index(ext_data_index),.ext_data_fp64(ext_data_fp64),.ext_data_last(ext_data_last),.ext_rsp_valid(ext_rsp_valid),.ext_rsp_ready(ext_rsp_ready),.ext_rsp_status(ext_rsp_status),.ext_rsp_cost_fp64(ext_rsp_cost_fp64),.shared_req_valid(shared_req_valid[2 +: 4]),.shared_req_ready(shared_req_ready[2 +: 4]),.shared_req_op(shared_req_op[10 +: 20]),.shared_req_a(shared_req_a[128 +: 256]),.shared_req_b(shared_req_b[128 +: 256]),.shared_active(shared_active[2 +: 4]),.shared_rsp_valid(shared_rsp_valid[2 +: 4]),.shared_rsp_ready(shared_rsp_ready[2 +: 4]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(local_rst_n),.cmd_valid(rst_n && state==RES_CMD),.cmd_ready(re_ready),
      .cmd_width(width),.cmd_height(height),.cmd_state(payload),
      .point_rd_en(point_rd_en),.point_rd_view_id(point_rd_view_id),.point_rd_index(point_rd_index),
      .point_rd_valid(point_rd_valid),.point_rd_x_fp32(point_rd_x_fp32),.point_rd_y_fp32(point_rd_y_fp32),
      .data_valid(re_data_valid),.data_ready(re_data_ready),.data_index(re_index),.data_fp64(re_value),.data_last(re_last),
      .rsp_valid(re_valid),.rsp_ready(state==RES_WAIT),.rsp_status(re_status),.rsp_cost_fp64(re_cost));
    assign cmd_ready=rst_n && state==IDLE;
    assign rsp_valid=rst_n && state==RESPONSE;
    assign rsp_camera_usable=usable_q;assign rsp_camera_params=usable_q?camera_q:288'd0;
    assign rsp_metrics_valid=metrics_q;assign rsp_weak_geometry=metrics_q && weak_q;
    assign rsp_rms_fp64=metrics_q?rms_q:64'd0;assign rsp_max_error_fp64=metrics_q?maxerr_q:64'd0;
    assign rsp_view_rms_fp64=metrics_q?view_rms_q:{`PAR_VIEW_RMS_W{1'b0}};
    assign rsp_poses_fp64=metrics_q?poses_q:{`PAR_POSES_W{1'b0}};
    always @(posedge clk or negedge rst_n)begin
      if(!rst_n)begin
        state<=IDLE;index<=0;destination<=0;received<=0;width<=0;height<=0;square_q<=0;best_q<=0;input_converged<=0;
        camera_q<=0;poses_q<=0;view_rms_q<=0;rms_q<=0;maxerr_q<=0;metrics_q<=0;usable_q<=0;weak_q<=0;
        payload<=0;service_cost<=0;service_status<=0;
        rsp_status<=0;stream_error<=0;
      end else case(state)
        IDLE:if(cmd_valid)begin
          payload<=cmd_state;width<=cmd_width;height<=cmd_height;square_q<=cmd_square_size_fp64;best_q<=cmd_best_cost_fp64;input_converged<=cmd_converged;
          metrics_q<=0;usable_q<=0;weak_q<=0;index<=0;
          rsp_status<=0;
          if(cmd_width<2 || cmd_height<2)begin rsp_status<=`PAR_BAD_CONFIG;state<=RESPONSE;end
          else state<=INITIALIZE;
        end
        INITIALIZE:if(host_ready)begin
          if(index==287)begin index<=0;state<=INPUT_STATE;end else index<=index+1;
        end
        INPUT_STATE:if(host_ready)begin
          payload<={64'd0,payload[`PAR_STATE_W-1:64]};
          if(index==`PAR_STATE_N-1)state<=START;else index<=index+1;
        end
        START:if(engine_ready)state<=RUN;
        RUN:begin
          if(svc_valid)begin
            if(svc_id==0)state<=META_REQ;
            else begin rsp_status<=`PAR_BAD_CONFIG;state<=RESPONSE;end
          end else if(engine_valid)begin
            if(engine_status!=0)begin rsp_status<=`PAR_BAD_CONFIG;state<=RESPONSE;end
            else begin index<=0;state<=OUT_REQ;end
          end
        end
        META_REQ:if(host_ready)state<=META_WAIT;
        META_WAIT:if(host_valid)begin destination<=host_result[31:0];index<=0;state<=STATE_REQ;end
        STATE_REQ:if(host_ready)state<=STATE_WAIT;
        STATE_WAIT:if(host_valid)begin
          payload<={host_result,payload[`PAR_STATE_W-1:64]};
          if(index==`PAR_STATE_N-1)state<=RES_CMD;else begin index<=index+1;state<=STATE_REQ;end
        end
        RES_CMD:if(re_ready)begin received<=0;stream_error<=0;state<=RES_WAIT;end
        RES_WAIT:begin
          if(re_data_valid && re_data_ready)begin
            if(received>=`PAR_RESIDUALS || re_index!=received || re_last!=(received==`PAR_RESIDUALS-1) || re_value[62:52]==2047)stream_error<=1;
            if(received<`PAR_RESIDUALS)received<=received+1;
          end
          if(re_valid)begin
            service_cost<=re_cost;service_status<=re_status;
            if(re_status==0)begin
              if(stream_error || received!=`PAR_RESIDUALS || re_data_valid)service_status<=`PAR_BAD_CONFIG;
              else if(re_cost[62:52]==2047)service_status<=`PAR_CALIB_INVALID;
              else if(re_cost[63] && |re_cost[62:0])service_status<=`PAR_BAD_CONFIG;
            end
            state<=STORE_STATUS;
          end
        end
        STORE_STATUS:if(host_ready)state<=STORE_COST;
        STORE_COST:if(host_ready)state<=RESUME;
        RESUME:state<=RUN;
        OUT_REQ:if(host_ready)state<=OUT_WAIT;
        OUT_WAIT:if(host_valid)begin
          if(index<OUT_POSES)camera_q<={host_result[31:0],camera_q[287:32]};
          else if(index<OUT_VIEWS)poses_q<={host_result,poses_q[`PAR_POSES_W-1:64]};
          else if(index<OUT_SCALARS)view_rms_q<={host_result,view_rms_q[`PAR_VIEW_RMS_W-1:64]};
          else case(index-OUT_SCALARS)
            0:rms_q<=host_result;1:maxerr_q<=host_result;2:usable_q<=host_result[0];
            3:metrics_q<=host_result[0];4:weak_q<=host_result[0];5:rsp_status<=host_result[7:0];
          endcase
          if(index==OUT_SCALARS+5)state<=RESPONSE;else begin index<=index+1;state<=OUT_REQ;end
        end
        RESPONSE:if(rsp_ready)state<=IDLE;
        default:begin rsp_status<=`PAR_BAD_CONFIG;state<=RESPONSE;end
      endcase
    end
endmodule
