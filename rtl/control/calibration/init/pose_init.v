`include "calib_defs.vh"
// Pose initialization: microprogram + block RAM, shared FP; standalone reference module.
// Interface/precision are unchanged; each view reuses the same work area.
module pose_init #(parameter FP_SHARED=0) (
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
    input wire cmd_valid, // 命令有效
    output wire cmd_ready, // 可接受命令
    input wire [15:0] cmd_width, // 图像宽度，>=2
    input wire [15:0] cmd_height, // 图像高度，>=2
    input wire [`PAR_H_ALL_W-1:0] cmd_h_all_fp64, // 三张单应矩阵
    input wire [`PAR_K_W-1:0] cmd_k_fp64, // 内参seed
    output wire rsp_valid, // 完成响应有效；最后一笔输出握手后才置位
    input wire rsp_ready, // 接收完成响应
    output reg [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [`PAR_STATE_W-1:0] rsp_state // 完整PAR_STATE_N项状态
);
    // One view at a time: 128x64 workspace + 616x32 program replaces the
    // former large multi-read working register array and arithmetic FSM.
    // The same program also performs stable quaternion-based R -> rotvec.
    `include "pose_program_defs.vh"
    localparam IDLE=0,LOAD_COMMON=1,START=2,WAIT_ENGINE=3,READ_GLOBAL=4,
      WAIT_GLOBAL=5,PAD=6,LOAD_VIEW=7,READ_VIEW=8,WAIT_VIEW=9,
      RESPONSE=10;
    reg [3:0] state,index;
    reg [`PAR_VIEW_BITS-1:0] view;
    reg common_phase;
    reg [`PAR_H_ALL_W-1:0] h_shift;
    reg [255:0] k_shift;
    reg [63:0] width_fp,height_fp;
    reg [`PAR_STATE_W-1:0] output_shift;




    wire engine_ready,engine_valid;wire [7:0] engine_status;
    wire host_ready,host_valid,host_error;wire [63:0] host_result;
    reg [31:0] host_addr;reg [63:0] host_data;
    wire host_en=state==LOAD_COMMON || state==LOAD_VIEW || state==READ_GLOBAL || state==READ_VIEW;
    wire host_write=state==LOAD_COMMON || state==LOAD_VIEW;
    wire imem_en;wire [15:0] imem_addr;reg [31:0] imem_data;
    reg [31:0] program_memory[0:1023];integer rom_index;
    initial begin
      for(rom_index=0;rom_index<1024;rom_index=rom_index+1)program_memory[rom_index]=32'h040000d2;
      `include "pose_program_init.vh"
    end
    always @(posedge clk)if(imem_en)imem_data<=program_memory[imem_addr[9:0]];
    wire fp_request,fp_ready,fp_response,fp_response_ready;
    wire [4:0] fp_op,fp_flags;wire [63:0] fp_a,fp_b,fp_result;
    assign cmd_ready=rst_n && state==IDLE;
    assign rsp_valid=rst_n && state==RESPONSE;
    assign rsp_state=rsp_status==`PAR_OK ? output_shift : {`PAR_STATE_W{1'b0}};
    calib_datapath #(.FP_SHARED(1),.ENABLE_KERNELS(0),.RAM_WORDS(128),.CONST_BASE(92),.TRAP_FP(1)) engine(
      .svc_valid(),.svc_ready(1'b0),.svc_id(),
      .clk(clk),.rst_n(rst_n),.start_valid(state==START),.start_ready(engine_ready),
      .start_pc(common_phase?16'd`POSE_COMMON_PC:16'd`POSE_VIEW_PC),.program_words(16'd`POSE_PROGRAM_WORDS),
      .imem_en(imem_en),.imem_addr(imem_addr),.imem_data(imem_data),
      .host_en(host_en),.host_write(host_write),.host_addr(host_addr),.host_data(host_data),
      .host_ready(host_ready),.host_valid(host_valid),.host_error(host_error),.host_result(host_result),
      .busy(),.rsp_valid(engine_valid),.rsp_ready(state==WAIT_ENGINE),.rsp_status(engine_status),
      .debug_pc(),.cycle_count(),.instruction_count(),
      .shared_req_valid(fp_request),.shared_req_ready(fp_ready),.shared_req_op(fp_op),.shared_req_a(fp_a),.shared_req_b(fp_b),
      .shared_rsp_ready(fp_response_ready),.shared_rsp_valid(fp_response),.shared_rsp_result(fp_result),.shared_rsp_flags(fp_flags));
    generate if(FP_SHARED)begin : g_shared_fp
      assign shared_req_valid[0]=fp_request;assign shared_req_op[4:0]=fp_op;
      assign shared_req_a[63:0]=fp_a;assign shared_req_b[63:0]=fp_b;
      assign shared_active[0]=rst_n;assign shared_rsp_ready[0]=fp_response_ready;
      assign fp_ready=shared_req_ready[0];assign fp_response=shared_rsp_valid[0];
      assign fp_result=shared_rsp_result;assign fp_flags=shared_rsp_flags;
    end else begin : g_local_fp
      fp_operator #(.FP_W(64),.ENABLE_EXP(0),.ENABLE_LOG(1),.ENABLE_SINCOS(0),.ENABLE_ATAN_ACOS(1)) arithmetic(
        .clk(clk),.rst_n(rst_n),.req_valid(fp_request),.req_ready(fp_ready),.req_op(fp_op),.req_a(fp_a),.req_b(fp_b),
        .rsp_valid(fp_response),.rsp_ready(fp_response_ready),.rsp_result(fp_result),.rsp_flags(fp_flags),
        .rsp_less(),.rsp_equal(),.rsp_unordered());
      assign shared_req_valid[0]=0;assign shared_req_op[4:0]=0;assign shared_req_a[63:0]=0;assign shared_req_b[63:0]=0;
      assign shared_active[0]=0;assign shared_rsp_ready[0]=0;
    end endgenerate
    // The old rotation client is removed; its legacy slot remains tied off
    // until the parent bus is consolidated with later stage migrations.
    assign shared_req_valid[1]=0;assign shared_req_op[9:5]=0;
    assign shared_req_a[127:64]=0;assign shared_req_b[127:64]=0;
    assign shared_active[1]=0;assign shared_rsp_ready[1]=0;
    function [63:0] u16;
      input [15:0] x;integer k,top;reg [63:0] shifted;reg [10:0] exponent;
      begin top=0;for(k=0;k<16;k=k+1)if(x[k])top=k;
        shifted={48'd0,x}<<(52-top);exponent=1023+top;
        u16=x==0?64'd0:{1'b0,exponent,shifted[51:0]};end
    endfunction
    always @* begin
      host_addr=0;host_data=0;
      case(state)
        LOAD_COMMON:begin
          host_addr=index<6?{28'd0,index}:32'd86+{28'd0,index};
          case(index)
            0,1,2,3:host_data=k_shift[63:0];
            4:host_data=width_fp;
            5:host_data=height_fp;
            6:host_data=64'd0;
            7:host_data=64'h3ff0000000000000;
            8:host_data=64'h4000000000000000;
            9:host_data=64'h3d719799812dea11;
            10:host_data=64'h4010000000000000;
          endcase
        end
        LOAD_VIEW:begin host_addr=32'd80+{28'd0,index};host_data=h_shift[63:0];end
        READ_GLOBAL:host_addr=32'd40+{28'd0,index};
        READ_VIEW:case(index)
          0:host_addr=55;1:host_addr=56;2:host_addr=57;
          3:host_addr=30;4:host_addr=31;5:host_addr=54;
        endcase
      endcase
    end
    task fail;
      input [7:0] status;
      begin rsp_status<=status;state<=RESPONSE;end
    endtask
    integer j;
    always @(posedge clk or negedge rst_n)begin
      if(!rst_n)begin state<=IDLE;index<=0;view<=0;rsp_status<=0;common_phase<=1;end
      else case(state)
        IDLE:if(cmd_valid)begin
          rsp_status<=`PAR_OK;index<=0;view<=0;common_phase<=1;state<=LOAD_COMMON;
          h_shift<=cmd_h_all_fp64;k_shift<=cmd_k_fp64;output_shift<=0;
          width_fp<=u16(cmd_width);height_fp<=u16(cmd_height);
          for(j=0;j<9*`PAR_VIEWS;j=j+1)if(cmd_h_all_fp64[64*j+52+:11]==2047)fail(`PAR_CALIB_INVALID);
          for(j=0;j<4;j=j+1)if(cmd_k_fp64[64*j+52+:11]==2047)fail(`PAR_CALIB_INVALID);
          if(cmd_k_fp64[63] || cmd_k_fp64[127] || cmd_k_fp64[62:0]==0 || cmd_k_fp64[126:64]==0)fail(`PAR_CALIB_INVALID);
          if(cmd_width<2 || cmd_height<2)fail(`PAR_BAD_CONFIG);
        end
        LOAD_COMMON:if(host_ready)begin
          if(index<4)k_shift<=k_shift>>64;
          if(index==10)state<=START;else index<=index+1'b1;
        end
        START:if(engine_ready)state<=WAIT_ENGINE;
        WAIT_ENGINE:if(engine_valid)begin
          if(engine_status!=0)fail(`PAR_CALIB_INVALID);
          else begin index<=0;state<=common_phase?READ_GLOBAL:READ_VIEW;end
        end
        READ_GLOBAL:if(host_ready)state<=WAIT_GLOBAL;
        WAIT_GLOBAL:if(host_valid)begin
          if(host_error)fail(`PAR_CALIB_INVALID);
          else begin
            output_shift<={host_result,output_shift[`PAR_STATE_W-1:64]};
            if(index==3)begin index<=0;state<=PAD;end
            else begin index<=index+1'b1;state<=READ_GLOBAL;end
          end
        end
        PAD:begin
          output_shift<={64'd0,output_shift[`PAR_STATE_W-1:64]};
          if(index==4)begin index<=0;common_phase<=0;state<=LOAD_VIEW;end else index<=index+1'b1;
        end
        LOAD_VIEW:if(host_ready)begin
          h_shift<=h_shift>>64;
          if(index==8)state<=START;else index<=index+1'b1;
        end
        READ_VIEW:if(host_ready)state<=WAIT_VIEW;
        WAIT_VIEW:if(host_valid)begin
          if(host_error)fail(`PAR_CALIB_INVALID);
          else begin
            output_shift<={host_result,output_shift[`PAR_STATE_W-1:64]};
            if(index==5)begin
              if(view==`PAR_VIEWS-1)state<=RESPONSE;
              else begin view<=view+1'b1;index<=0;state<=LOAD_VIEW;end
            end else begin index<=index+1'b1;state<=READ_VIEW;end
          end
        end
        RESPONSE:if(rsp_ready)state<=IDLE;
        default:fail(`PAR_CALIB_INVALID);
      endcase
    end
endmodule
