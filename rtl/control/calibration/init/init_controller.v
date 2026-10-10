`include "calibration_program_defs.vh"
`include "calib_defs.vh"
// Full initialization microprogram; host only transports corner/state words.
// Board ENGINE_SHARED=1: load workspace -> START (local PC 0, INIT_ENGINE_PC
// added by calib_execution_port) -> RUN -> read status/seed -> publish response.
// This adapter does not execute the matrix algorithm. See
// docs/INSTRUCTION_CONTROL_GUIDE.md for program sources and the full call path.
module init_controller #(parameter FP_SHARED=0, ENGINE_SHARED=0) (
    output wire [127:0] execution_req, input wire [95:0] execution_rsp,
    // Optional shared FP64 service; local mode keeps standalone compatibility.
    output wire [6:0] shared_req_valid,
    input wire [6:0] shared_req_ready,
    output wire [34:0] shared_req_op,
    output wire [447:0] shared_req_a,
    output wire [447:0] shared_req_b,
    output wire [6:0] shared_active,
    input wire [6:0] shared_rsp_valid,
    output wire [6:0] shared_rsp_ready,
    input wire [63:0] shared_rsp_result,
    input wire [4:0] shared_rsp_flags,

    input wire clk, // core_clk，同一时钟域
    input wire rst_n, // 低有效复位，同步释放；取消全部在途事务
    input wire cmd_valid, // 命令有效
    output wire cmd_ready, // 可接受命令
    input wire [15:0] cmd_width, // 图像宽度，>=2
    input wire [15:0] cmd_height, // 图像高度，>=2
    output wire point_rd_en, // 固定1拍角点读使能，无ready，发起前预留接收空间
    output wire [`PAR_VIEW_BITS-1:0] point_rd_view_id, // 0..PAR_VIEWS-1
    output wire [`PAR_POINT_BITS-1:0] point_rd_index, // 图内角点0..PAR_POINTS-1
    input wire point_rd_valid, // 固定1拍返回，必须当拍消费；无返回背压
    input wire [31:0] point_rd_x_fp32, // 原图x
    input wire [31:0] point_rd_y_fp32, // 原图y
    output wire seed_valid, // 有效seed状态
    input wire seed_ready, // 父模块可缓存
    output wire [2:0] seed_id, // fixed diagnostic id 2
    output wire [`PAR_STATE_W-1:0] seed_state, // 内部PAR_STATE_N项FP64
    output wire rsp_valid, // 完成响应有效；最后一笔输出握手后才置位
    input wire rsp_ready, // 接收完成响应
    output reg [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [2:0] rsp_seed_count // 成功输出的seed数，0..1
);
    `include "lm_program_defs.vh"
    `include "init_program_defs.vh"
    `include "calib_workspace_layout.vh"
    `include "calib_geometry.vh"
    localparam RAW=1024,PX=RAW+`PAR_TOTAL_POINTS,PY=PX+`PAR_POINTS,
      BX=PY+`PAR_POINTS,BY=BX+`PAR_POINTS,SEED_BASE=BY+`PAR_POINTS;
    localparam IDLE=0,INITIALIZE=1,GRID_X=2,GRID_Y=3,READ_REQ=4,READ_WAIT=5,
      START=6,RUN=7,STATUS_REQ=8,STATUS_WAIT=9,OUT_REQ=10,OUT_WAIT=11,SEED=12,RESPONSE=13;
    reg [3:0] state;
    reg [31:0] index,raw_index;
    reg [`PAR_VIEW_BITS-1:0] view;
    reg [`PAR_POINT_BITS-1:0] point;
    reg [15:0] width,height;
    reg [`PAR_STATE_W-1:0] payload;
    reg [2:0] count;
    wire engine_ready,engine_valid;wire [7:0] engine_status;
    wire host_ready,host_valid,host_error;wire [63:0] host_result;
    reg host_en,host_write;reg [31:0] host_addr;reg [63:0] host_data,initial_data;
    wire imem_en;wire [15:0] imem_addr;reg [31:0] imem_data;
    reg [31:0] program_memory[0:4095];integer rom_i,rom_j;
    initial begin
      for(rom_j=0;rom_j<4;rom_j=rom_j+1)
        for(rom_i=0;rom_i<1024;rom_i=rom_i+1)program_memory[rom_j*1024+rom_i]=32'h040000e1;
      `include "init_program_init.vh"
    end
    always @(posedge clk)if(imem_en)imem_data<=program_memory[imem_addr[11:0]];
    function [63:0] u16;input [15:0] x;integer k,top;reg [63:0] shifted;reg [10:0] exponent;
      begin top=0;for(k=0;k<16;k=k+1)if(x[k])top=k;shifted={48'd0,x}<<(52-top);exponent=1023+top;
      u16=x==0?64'd0:{1'b0,exponent,shifted[51:0]};end endfunction
    always @* begin
      initial_data=0;
      case(index)
        `LM_VIEWS:initial_data=`PAR_VIEWS;
        `LM_POINTS:initial_data=`PAR_POINTS;
        `LM_FP_POINTS:initial_data=u16(`PAR_POINTS);
        `LM_FWIDTH:initial_data=u16(width);
        `LM_FHEIGHT:initial_data=u16(height);
        `LM_INIT_RAW:initial_data=RAW;
        `LM_INIT_PX:initial_data=PX;
        `LM_INIT_PY:initial_data=PY;
        `LM_INIT_BX:initial_data=BX;
        `LM_INIT_BY:initial_data=BY;
        `LM_INIT_SEED:initial_data=SEED_BASE;
        `include "lm_constants.vh"
        default:initial_data=0;
      endcase
    end
    always @* begin
      host_en=0;host_write=0;host_addr=0;host_data=0;
      case(state)
        INITIALIZE:begin host_en=1;host_write=1;host_addr=index;host_data=initial_data;end
        GRID_X:begin host_en=1;host_write=1;host_addr=BX+index;host_data=board_x(index);end
        GRID_Y:begin host_en=1;host_write=1;host_addr=BY+index;host_data=board_y(index);end
        READ_WAIT:begin host_en=point_rd_valid;host_write=1;host_addr=RAW+raw_index;host_data={point_rd_y_fp32,point_rd_x_fp32};end
        STATUS_REQ:begin host_en=1;host_addr=`LM_STATUS;end
        OUT_REQ:begin host_en=1;host_addr=SEED_BASE+index;end
        default:begin end
      endcase
    end
    wire fv,fr,sv,sr;wire [4:0] fo,ff;wire [63:0] fa,fb,fd;
    calib_execution_port #(.EXTERNAL(ENGINE_SHARED),.PROGRAM_BASE(`INIT_ENGINE_PC),.RAM_WORDS(WORDS),.CONST_BASE(WORDS),.HOST_CALLS(1)) engine(
      .execution_req(execution_req),.execution_rsp(execution_rsp),
      .clk(clk),.rst_n(rst_n),.start_valid(state==START),.start_ready(engine_ready),
      .start_pc(16'd0),.program_words(16'd`INIT_PROGRAM_WORDS),
      .imem_en(imem_en),.imem_addr(imem_addr),.imem_data(imem_data),
      .host_en(host_en),.host_write(host_write),.host_addr(host_addr),.host_data(host_data),
      .host_ready(host_ready),.host_valid(host_valid),.host_error(host_error),.host_result(host_result),
      .busy(),.rsp_valid(engine_valid),.rsp_ready(state==RUN),.rsp_status(engine_status),
      .debug_pc(),.cycle_count(),.instruction_count(),.svc_valid(),.svc_ready(1'b0),.svc_id(),
      .shared_req_valid(fv),.shared_req_ready(fr),.shared_req_op(fo),.shared_req_a(fa),.shared_req_b(fb),
      .shared_rsp_valid(sv),.shared_rsp_ready(sr),.shared_rsp_result(fd),.shared_rsp_flags(ff));
    generate if(FP_SHARED)begin : g_shared
      assign shared_req_valid[0]=fv;assign shared_req_op[4:0]=fo;
      assign shared_req_a[63:0]=fa;assign shared_req_b[63:0]=fb;
      assign shared_active[0]=rst_n;assign shared_rsp_ready[0]=sr;
      assign fr=shared_req_ready[0];assign sv=shared_rsp_valid[0];
      assign fd=shared_rsp_result;assign ff=shared_rsp_flags;
    end else begin : g_local
      fp_operator #(.FP_W(64),.ENABLE_EXP(0)) arithmetic(.clk(clk),.rst_n(rst_n),.req_valid(fv),.req_ready(fr),
        .req_op(fo),.req_a(fa),.req_b(fb),.rsp_valid(sv),.rsp_ready(sr),
        .rsp_result(fd),.rsp_flags(ff),.rsp_less(),.rsp_equal(),.rsp_unordered());
      assign shared_req_valid[0]=0;assign shared_req_op[4:0]=0;
      assign shared_req_a[63:0]=0;assign shared_req_b[63:0]=0;
      assign shared_active[0]=0;assign shared_rsp_ready[0]=0;
    end endgenerate
    assign shared_req_valid[6:1]=0;assign shared_req_op[34:5]=0;
    assign shared_req_a[447:64]=0;assign shared_req_b[447:64]=0;
    assign shared_active[6:1]=0;assign shared_rsp_ready[6:1]=0;
    assign cmd_ready=rst_n && state==IDLE;
    assign rsp_valid=rst_n && state==RESPONSE;
    assign seed_valid=rst_n && state==SEED;
    assign seed_state=payload;assign seed_id=3'd2;assign rsp_seed_count=count;
    assign point_rd_en=rst_n && state==READ_REQ;
    assign point_rd_view_id=view;assign point_rd_index=point;
    always @(posedge clk or negedge rst_n)begin
      if(!rst_n)begin state<=IDLE;index<=0;raw_index<=0;view<=0;point<=0;width<=0;height<=0;
        payload<=0;count<=0;rsp_status<=0;end
      else case(state)
        IDLE:if(cmd_valid)begin
          width<=cmd_width;height<=cmd_height;index<=0;count<=0;rsp_status<=0;
          if(cmd_width<2 || cmd_height<2)begin rsp_status<=`PAR_BAD_CONFIG;state<=RESPONSE;end
          else state<=INITIALIZE;
        end
        INITIALIZE:if(host_ready)begin if(index==287)begin index<=0;state<=GRID_X;end else index<=index+1;end
        GRID_X:if(host_ready)state<=GRID_Y;
        GRID_Y:if(host_ready)begin
          if(index==`PAR_POINTS-1)begin raw_index<=0;view<=0;point<=0;state<=READ_REQ;end
          else begin index<=index+1;state<=GRID_X;end
        end
        READ_REQ:state<=READ_WAIT;
        READ_WAIT:begin
          if(!point_rd_valid || !host_ready)begin rsp_status<=`PAR_MEM_ERROR;state<=RESPONSE;end
          else if(raw_index==`PAR_TOTAL_POINTS-1)state<=START;
          else begin raw_index<=raw_index+1;
            if(point==`PAR_POINTS-1)begin point<=0;view<=view+1'b1;end else point<=point+1'b1;
            state<=READ_REQ;
          end
        end
        START:if(engine_ready)state<=RUN;
        RUN:if(engine_valid)begin
          if(engine_status!=0)begin rsp_status<=`PAR_CALIB_INVALID;state<=RESPONSE;end
          else state<=STATUS_REQ;
        end
        STATUS_REQ:if(host_ready)state<=STATUS_WAIT;
        STATUS_WAIT:if(host_valid)begin
          rsp_status<=host_result[7:0];index<=0;
          if(host_result!=0)state<=RESPONSE;else state<=OUT_REQ;
        end
        OUT_REQ:if(host_ready)state<=OUT_WAIT;
        OUT_WAIT:if(host_valid)begin
          payload<={host_result,payload[`PAR_STATE_W-1:64]};
          if(index==`PAR_STATE_N-1)state<=SEED;else begin index<=index+1;state<=OUT_REQ;end
        end
        SEED:if(seed_ready)begin count<=1;state<=RESPONSE;end
        RESPONSE:if(rsp_ready)state<=IDLE;
        default:begin rsp_status<=`PAR_BAD_CONFIG;state<=RESPONSE;end
      endcase
    end
endmodule
