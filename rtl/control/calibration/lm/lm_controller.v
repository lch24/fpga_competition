`include "calibration_program_defs.vh"
`include "calib_defs.vh"
// One microprogram/workspace owns the entire LM stage. See build_lm.py.
module lm_controller #(parameter FP_SHARED=0, SHARE_RESIDUAL=0, ENGINE_SHARED=0) (
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
    output wire [127:0] execution_req, input wire [95:0] execution_rsp,
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
    `include "lm_program_defs.vh"
    // Only protocol adaptation is hardwired. LM, differences, scaling, J'J,
    // damping, pivoted elimination, acceptance and termination live in ROM.
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
    reg [1:0] stage;
    reg [`PAR_STATE_W-1:0] payload;
    reg [63:0] cost_q,service_cost;
    reg [7:0] service_status,accepted_q,outer_q;
    reg converged_q,stream_error;
    wire engine_ready,engine_valid,svc_valid;
    wire [7:0] engine_status,svc_id;
    wire host_ready,host_valid,host_error;
    wire [63:0] host_result;
    reg host_en,host_write;
    reg [31:0] host_addr;
    reg [63:0] host_data,initial_data;
    wire imem_en;wire [15:0] imem_addr;reg [31:0] imem_data;
    reg [31:0] program_memory[0:1023];integer rom_i,rom_j;
    initial begin
      for(rom_i=0;rom_i<1024;rom_i=rom_i+1)program_memory[rom_i]=32'h040000e1;
      `include "lm_program_init.vh"
    end
    always @(posedge clk)if(imem_en)imem_data<=program_memory[imem_addr[9:0]];
    `include "calib_lm_layout.vh"
    // Descriptors are populated once, not decoded on every matrix access.
    // Local pose column axis shares a bank across disjoint view row ranges.
    integer descriptor_col,descriptor_field,vi,axis,first_row,last_row,packed_col;
    always @* begin
      descriptor_col=(index-DESC)>>2;descriptor_field=(index-DESC)&3;
      first_row=0;last_row=`PAR_RESIDUALS;packed_col=descriptor_col-6*`PAR_VIEWS;
      if(descriptor_col<4)packed_col=descriptor_col;
      for(vi=0;vi<`PAR_VIEWS;vi=vi+1)
        for(axis=0;axis<6;axis=axis+1)
          if(descriptor_col==4+6*vi+axis)begin
            first_row=vi*(2*`PAR_POINTS);last_row=(vi+1)*(2*`PAR_POINTS);packed_col=8+axis;
          end
      initial_data=0;
      if(index>=DESC)case(descriptor_field)
        0:initial_data=active_index(descriptor_col);
        1:initial_data=first_row;
        2:initial_data=last_row;
        3:initial_data=JAC+packed_col*`PAR_RESIDUALS;
      endcase
      else case(index)
        `LM_N:initial_data=columns(stage);
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
          if(index<`PAR_STATE_N)host_addr=CURRENT+index;
          else case(index-`PAR_STATE_N)
            0:host_addr=`LM_COST;1:host_addr=`LM_ACCEPTED;2:host_addr=`LM_OUTER;
            3:host_addr=`LM_CONVERGED;default:host_addr=`LM_STATUS;
          endcase
        end
        default:begin end
      endcase
    end
    wire fv,fr,sv,sr;wire [4:0] fo,ff;wire [63:0] fa,fb,fd;
    calib_execution_port #(.EXTERNAL(ENGINE_SHARED),.PROGRAM_BASE(`LM_ENGINE_PC),.RAM_WORDS(WORDS),.CONST_BASE(WORDS),
      .HOST_CALLS(1)) engine(
      .execution_req(execution_req),.execution_rsp(execution_rsp),
      .clk(clk),.rst_n(local_rst_n),.start_valid(state==START),.start_ready(engine_ready),
      .start_pc(16'd0),.program_words(16'd`LM_PROGRAM_WORDS),
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
      calib_alu arithmetic(.ce(1'b1),.clk(clk),.rst_n(local_rst_n),.req_valid(fv),.req_ready(fr),
        .req_op(fo),.req_a(fa),.req_b(fb),.rsp_valid(sv),.rsp_ready(sr),
        .rsp_result(fd),.rsp_flags(ff),.rsp_less(),.rsp_equal(),.rsp_unordered());
      assign shared_req_valid[0]=0;assign shared_req_op[4:0]=0;
      assign shared_req_a[63:0]=0;assign shared_req_b[63:0]=0;
      assign shared_active[0]=0;assign shared_rsp_ready[0]=0;
    end endgenerate
    // Retain parent lane numbering during staged integration. All removed
    // Jacobian/normal/damped/solver clients are constants and synthesize away.
    assign shared_req_valid[8:5]=0;assign shared_req_op[44:25]=0;
    assign shared_req_a[575:320]=0;assign shared_req_b[575:320]=0;
    assign shared_active[8:5]=0;assign shared_rsp_ready[8:5]=0;
    residual_endpoint #(.FP_SHARED(FP_SHARED),.EXTERNAL(SHARE_RESIDUAL)) residual_service(.ext_rst_n(ext_rst_n),.ext_cmd_valid(ext_cmd_valid),.ext_cmd_ready(ext_cmd_ready),.ext_cmd_width(ext_cmd_width),.ext_cmd_height(ext_cmd_height),.ext_cmd_state(ext_cmd_state),.ext_point_rd_en(ext_point_rd_en),.ext_point_rd_view_id(ext_point_rd_view_id),.ext_point_rd_index(ext_point_rd_index),.ext_point_rd_valid(ext_point_rd_valid),.ext_point_rd_x_fp32(ext_point_rd_x_fp32),.ext_point_rd_y_fp32(ext_point_rd_y_fp32),.ext_data_valid(ext_data_valid),.ext_data_ready(ext_data_ready),.ext_data_index(ext_data_index),.ext_data_fp64(ext_data_fp64),.ext_data_last(ext_data_last),.ext_rsp_valid(ext_rsp_valid),.ext_rsp_ready(ext_rsp_ready),.ext_rsp_status(ext_rsp_status),.ext_rsp_cost_fp64(ext_rsp_cost_fp64),.shared_req_valid(shared_req_valid[1 +: 4]),.shared_req_ready(shared_req_ready[1 +: 4]),.shared_req_op(shared_req_op[5 +: 20]),.shared_req_a(shared_req_a[64 +: 256]),.shared_req_b(shared_req_b[64 +: 256]),.shared_active(shared_active[1 +: 4]),.shared_rsp_valid(shared_rsp_valid[1 +: 4]),.shared_rsp_ready(shared_rsp_ready[1 +: 4]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(local_rst_n),
      .cmd_valid(rst_n && state==RES_CMD),.cmd_ready(re_ready),.cmd_width(width),.cmd_height(height),
      .cmd_state(payload),
      .point_rd_en(point_rd_en),.point_rd_view_id(point_rd_view_id),.point_rd_index(point_rd_index),
      .point_rd_valid(point_rd_valid),.point_rd_x_fp32(point_rd_x_fp32),.point_rd_y_fp32(point_rd_y_fp32),
      .data_valid(re_data_valid),.data_ready(re_data_ready),.data_index(re_index),.data_fp64(re_value),.data_last(re_last),
      .rsp_valid(re_valid),.rsp_ready(state==RES_WAIT),.rsp_status(re_status),.rsp_cost_fp64(re_cost));
    assign cmd_ready=rst_n && state==IDLE;
    assign rsp_valid=rst_n && state==RESPONSE;
    assign rsp_state=payload;assign rsp_cost_fp64=cost_q;
    assign rsp_converged=converged_q;assign rsp_accepted_steps=accepted_q;
    assign rsp_outer_iterations=outer_q;
    always @(posedge clk or negedge rst_n)begin
      if(!rst_n)begin
        state<=IDLE;index<=0;destination<=0;received<=0;width<=0;height<=0;stage<=0;
        payload<=0;cost_q<=64'h7ff0000000000000;service_cost<=0;service_status<=0;
        rsp_status<=0;accepted_q<=0;outer_q<=0;converged_q<=0;stream_error<=0;
      end else case(state)
        IDLE:if(cmd_valid)begin
          payload<=cmd_state;width<=cmd_width;height<=cmd_height;stage<=cmd_stage;index<=0;
          cost_q<=64'h7ff0000000000000;accepted_q<=0;outer_q<=0;converged_q<=0;rsp_status<=0;
          if(cmd_width<2 || cmd_height<2 || cmd_stage==3)begin rsp_status<=`PAR_BAD_CONFIG;state<=RESPONSE;end
          else state<=INITIALIZE;
        end
        INITIALIZE:if(host_ready)begin
          if(index==CURRENT-1)begin index<=0;state<=INPUT_STATE;end else index<=index+1;
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
          if(index<`PAR_STATE_N)payload<={host_result,payload[`PAR_STATE_W-1:64]};
          else case(index-`PAR_STATE_N)
            0:cost_q<=host_result;1:accepted_q<=host_result[7:0];2:outer_q<=host_result[7:0];
            3:converged_q<=host_result[0];4:rsp_status<=host_result[7:0];
          endcase
          if(index==`PAR_STATE_N+4)state<=RESPONSE;else begin index<=index+1;state<=OUT_REQ;end
        end
        RESPONSE:if(rsp_ready)state<=IDLE;
        default:begin rsp_status<=`PAR_BAD_CONFIG;state<=RESPONSE;end
      endcase
    end
endmodule
