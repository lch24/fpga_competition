`include "calib_defs.vh"

/*
配置：统一见 rtl/include/calib_config.vh；下文具体计数例子以默认3张5×8为例，
实际容量、循环边界和端口位宽由PAR_*派生，修改配置后须重新编译/综合。
作用：最终报告和有效性检查；不再优化参数，也不改选次优seed。
先将best_state解码成9个FP32相机参数；重新调用residual_engine统计每视图RMS和最大误差。
总RMS=sqrt(best_cost/120)。输出R与平移：从中心化单位格恢复首内角点原点，再乘square_size。
检查最终阶段converged、焦距(0.05W,20W)、主点在图内、RMS<3、最大法向夹角>0.01。
按C++对已量化FP32参数提升到FP64，在33x25点检查畸变映射Jacobian；a>0,d>0,ad-b^2>1e-4。
weak_geometry=(max_angle<0.17 || 视图数<5)，不单独否定camera。
正常完成但不可靠时status=CALIB_INVALID且metrics_valid仍可为1；计算失败metrics_valid=0。
子模块：residual_engine、rotation、fp_operator；验证Jacobian公式在本模块实现。

实现说明：单事务串行FP64状态机；量化后的相机参数与原始状态分开保存。
残差成功收齐240项后，按二维hypot统计角点误差，避免直接平方造成额外溢出。
每图RMS按sum(hypot(du,dv)^2)/40开方，保持C++统计定义；总RMS沿用输入best_cost。
该接口信任best_cost属于cmd_state，不额外校验它与重算cost的一致性。
DECODE/QUANTIZE解码并量化；RES_CMD/WAIT缓存残差；HYPOT/ERROR_ACC统计误差；
ROT/TRANSLATION恢复报告姿态；ANGLE遍历所有不重复的视图对；MAP遍历825个映射采样点。
计算/协议失败清除metrics_valid并将所有载荷屏蔽为0；检查不通过仍保留完整诊断。
相机载荷仅usable=1时输出；weak_geometry按全部视图法向夹角和实际视图数计算。
宽高/格长/cost非法报BAD_CONFIG；state非有限或算术失败报CALIB_INVALID；读缺失报MEM_ERROR。
FP32转换溢出属于计算失败；映射检查阶段的非有限结果属于不可用映射，保留诊断。
内部取消信号经过寄存器，再复位子核；没有用多位状态组合译码驱动异步复位。

角点读接口例外：只读已提交视图，固定1拍返回，无valid/ready背压。
E_n采样使能/地址，数据在E_n后有效并由调用方在E_(n+1)采样；详见corner_store。
首版最多一笔在途；层间读路由不插入寄存器，返回消费后才能切换所有者。

共同契约：
- valid && ready 才传输；背压时 valid 和对应全部载荷保持不变。
- 除另有说明，一次一项任务，rsp 被接收前不接受新 cmd；不假设固定延迟。
- cmd_* 随命令锁存；rsp_* 随响应有效。失败仍须发完成响应，不能无声挂起。
- 非零状态通常使结果载荷无效；本模块明确说明的诊断有效位例外。
- 复位取消在途数据；复位后所有生产端 valid 清零。实现时禁止 real 运算。
*/
module validate_result #(parameter FP_SHARED=0, SHARE_RESIDUAL=0) (
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

    // 单事务串行运算；无全数组复位。27项p保留原FP64状态供残差/姿态使用。
    // v0..8=量化后的相机参数；v9..19=映射/临时量；v20..22=角点误差统计；
    // v23=总RMS，v24=tz，v26=最大法向夹角，v30/31=W/H，v32/33=焦距上下界。
    localparam IDLE=9000,FP_REQ=9001,FP_WAIT=9002,RESPONSE=9003;
    localparam ADD=0,SUB=1,MUL=2,DIV=3,SQRT=4,EXP=8,ACOS=10,TO64=11,TO32=12;
    localparam [63:0] ZERO=0,ONE=64'h3ff0000000000000;
    localparam DECODE_0=0;
    localparam DECODE_1=1;
    localparam DECODE_2=2;
    localparam DECODE_3=3;
    localparam COPY_DISTORTION=4;
    localparam QUANTIZE=5;
    localparam SAVE_CAMERA=6;
    localparam NEXT_CAMERA=7;
    localparam TOTAL_0=8;
    localparam TOTAL_1=9;
    localparam TOTAL_2=10;
    localparam TOTAL_3=11;
    localparam RES_CMD=12;
    localparam RES_WAIT=13;
    localparam HYPOT_START=14;
    localparam HYPOT_CHECK=15;
    localparam HYPOT_0=16;
    localparam HYPOT_1=17;
    localparam HYPOT_2=18;
    localparam HYPOT_3=19;
    localparam HYPOT_4=20;
    localparam ERROR_ACC_0=21;
    localparam ERROR_ACC_1=22;
    localparam NEXT_POINT=23;
    localparam VIEW_RMS_0=24;
    localparam VIEW_RMS_1=25;
    localparam SAVE_VIEW_RMS=26;
    localparam ROT_CMD=27;
    localparam ROT_WAIT=28;
    localparam TZ=29;
    localparam TRANSLATION_START=30;
    localparam TRANSLATION_0=31;
    localparam TRANSLATION_1=32;
    localparam TRANSLATION_2=33;
    localparam TRANSLATION_3=34;
    localparam TRANSLATION_4=35;
    localparam SAVE_TRANSLATION=36;
    localparam ANGLE_0=37;
    localparam ANGLE_1=38;
    localparam ANGLE_2=39;
    localparam ANGLE_3=40;
    localparam ANGLE_4=41;
    localparam ANGLE_5=42;
    localparam NEXT_PAIR=43;
    localparam CHECK_REPORT=44;
    localparam MAP_0=45;
    localparam MAP_1=46;
    localparam MAP_2=47;
    localparam MAP_3=48;
    localparam MAP_4=49;
    localparam MAP_5=50;
    localparam MAP_6=51;
    localparam MAP_7=52;
    localparam MAP_8=53;
    localparam MAP_9=54;
    localparam MAP_10=55;
    localparam MAP_11=56;
    localparam MAP_12=57;
    localparam MAP_13=58;
    localparam MAP_14=59;
    localparam MAP_15=60;
    localparam MAP_16=61;
    localparam MAP_17=62;
    localparam MAP_18=63;
    localparam MAP_19=64;
    localparam MAP_20=65;
    localparam MAP_21=66;
    localparam MAP_22=67;
    localparam MAP_23=68;
    localparam MAP_24=69;
    localparam MAP_25=70;
    localparam MAP_26=71;
    localparam MAP_27=72;
    localparam MAP_28=73;
    localparam MAP_29=74;
    localparam MAP_30=75;
    localparam MAP_31=76;
    localparam MAP_32=77;
    localparam MAP_33=78;
    localparam MAP_34=79;
    localparam MAP_35=80;
    localparam MAP_36=81;
    localparam MAP_37=82;
    localparam MAP_38=83;
    localparam MAP_39=84;
    localparam MAP_40=85;
    localparam MAP_41=86;
    localparam MAP_42=87;
    localparam MAP_43=88;
    localparam MAP_44=89;
    localparam MAP_45=90;
    localparam MAP_46=91;
    localparam MAP_47=92;
    localparam MAP_48=93;
    localparam MAP_49=94;
    localparam MAP_50=95;
    localparam MAP_51=96;
    localparam MAP_52=97;
    localparam MAP_53=98;
    localparam MAP_54=99;
    localparam MAP_55=100;
    localparam MAP_56=101;
    localparam MAP_57=102;
    localparam MAP_58=103;
    localparam MAP_CHECK=104;
    localparam SAVE_ROTATION=105;
    reg [575:0] rotation_q;
    reg [3:0] rotation_index;
    integer pc,continuation,destination,j,index,count,point,view,row,pair_a,pair_b;
    reg [15:0] width,height,ix,iy;
    reg [`PAR_STATE_W-1:0] state_q;reg [63:0] v[0:33],residual[0:`PAR_RESIDUALS-1],pose[0:12*`PAR_VIEWS-1];
    reg [63:0] square_size,best_cost,max_error;reg [`PAR_VIEW_RMS_W-1:0] view_rms;reg [287:0] camera;
    reg converged,metrics_valid,usable,map_phase,stream_bad,child_clear;
    wire child_rst_n=rst_n && !child_clear;
    always @(posedge clk or negedge rst_n)if(!rst_n)child_clear<=0;else child_clear<=(pc==RESPONSE);
    assign cmd_ready=rst_n && pc==IDLE;assign rsp_valid=rst_n && pc==RESPONSE;
    assign rsp_camera_usable=usable;assign rsp_camera_params=usable?camera:288'b0;
    assign rsp_metrics_valid=metrics_valid;assign rsp_weak_geometry=metrics_valid && (`PAR_VIEWS<5 || v[26]<64'h3fc5c28f5c28f5c3); // max_angle<0.17
    assign rsp_rms_fp64=metrics_valid?v[23]:ZERO;
    assign rsp_view_rms_fp64=metrics_valid?view_rms:{`PAR_VIEW_RMS_W{1'b0}};
    assign rsp_max_error_fp64=metrics_valid?max_error:ZERO;
    wire [63:0] p[0:`PAR_STATE_N-1];
    genvar parameter_index;
    generate for(parameter_index=0;parameter_index<`PAR_STATE_N;parameter_index=parameter_index+1)begin: unpack_state
        assign p[parameter_index]=state_q[64*parameter_index+:64];
    end endgenerate
    genvar g;generate for(g=0;g<12*`PAR_VIEWS;g=g+1)begin: pack_pose
      assign rsp_poses_fp64[64*g+:64]=metrics_valid?pose[g]:ZERO;
    end endgenerate
    `include "calib_geometry.vh"
    function finite;input [63:0] x;begin finite=x[62:52]!=2047;end endfunction
    function [63:0] magnitude;input [63:0] x;begin magnitude={1'b0,x[62:0]};end endfunction
    function [63:0] u16;input [15:0] x;integer k,top;reg [63:0] shifted;reg [10:0] exponent;
      begin top=0;for(k=0;k<16;k=k+1)if(x[k])top=k;shifted={48'b0,x}<<(52-top);exponent=1023+top;
      u16=x==0?64'b0:{1'b0,exponent,shifted[51:0]};end endfunction
    reg [4:0] fp_op;reg [63:0] fp_a,fp_b;
    wire fp_ready,fp_valid;wire [63:0] fp_result;wire [4:0] fp_flags;

    generate if(FP_SHARED) begin : g_shared_fp
        assign shared_req_valid[0 +: 1] = rst_n && pc==FP_REQ;
        assign shared_req_op[0 +: 5] = fp_op;
        assign shared_req_a[0 +: 64] = fp_a;
        assign shared_req_b[0 +: 64] = fp_b;
        assign shared_rsp_ready[0 +: 1] = rst_n && pc==FP_WAIT;
        assign shared_active[0] = child_rst_n;
        assign fp_ready = shared_req_ready[0];
        assign fp_valid = shared_rsp_valid[0];
        assign fp_result = shared_rsp_result;
        assign fp_flags = shared_rsp_flags;
    end else begin : g_local_fp
    fp_operator #(.FP_W(64), .ENABLE_EXP(1), .ENABLE_LOG(0), .ENABLE_SINCOS(0), .ENABLE_ATAN_ACOS(1)) arithmetic(.clk(clk),.rst_n(child_rst_n),.req_valid(rst_n && pc==FP_REQ),.req_ready(fp_ready),
      .req_op(fp_op),.req_a(fp_a),.req_b(fp_b),.rsp_valid(fp_valid),.rsp_ready(rst_n && pc==FP_WAIT),
      .rsp_result(fp_result),.rsp_flags(fp_flags),.rsp_less(),.rsp_equal(),.rsp_unordered());
assign shared_req_valid[0 +: 1] = 0;
assign shared_req_op[0 +: 5] = 0;
assign shared_req_a[0 +: 64] = 0;
assign shared_req_b[0 +: 64] = 0;
assign shared_active[0 +: 1] = 0;
assign shared_rsp_ready[0 +: 1] = 0;
    end endgenerate

    task calculate;input [4:0] op;input [63:0] a,b;input integer target,next_pc;
      begin fp_op<=op;fp_a<=a;fp_b<=b;destination<=target;continuation<=next_pc;pc<=FP_REQ;end endtask
    task fail;input [7:0] status;begin rsp_status<=status;metrics_valid<=0;usable<=0;pc<=RESPONSE;end endtask
    task reject;begin rsp_status<=`PAR_CALIB_INVALID;usable<=0;pc<=RESPONSE;end endtask
    wire res_ready,res_valid,res_data_valid,res_last;wire [7:0] res_status;wire [`PAR_RES_BITS-1:0] res_index;wire [63:0] res_value,res_cost;
    residual_endpoint #(.FP_SHARED(FP_SHARED),.EXTERNAL(SHARE_RESIDUAL)) residual_service(.ext_rst_n(ext_rst_n),.ext_cmd_valid(ext_cmd_valid),.ext_cmd_ready(ext_cmd_ready),.ext_cmd_width(ext_cmd_width),.ext_cmd_height(ext_cmd_height),.ext_cmd_state(ext_cmd_state),.ext_point_rd_en(ext_point_rd_en),.ext_point_rd_view_id(ext_point_rd_view_id),.ext_point_rd_index(ext_point_rd_index),.ext_point_rd_valid(ext_point_rd_valid),.ext_point_rd_x_fp32(ext_point_rd_x_fp32),.ext_point_rd_y_fp32(ext_point_rd_y_fp32),.ext_data_valid(ext_data_valid),.ext_data_ready(ext_data_ready),.ext_data_index(ext_data_index),.ext_data_fp64(ext_data_fp64),.ext_data_last(ext_data_last),.ext_rsp_valid(ext_rsp_valid),.ext_rsp_ready(ext_rsp_ready),.ext_rsp_status(ext_rsp_status),.ext_rsp_cost_fp64(ext_rsp_cost_fp64),.shared_req_valid(shared_req_valid[2 +: 4]),.shared_req_ready(shared_req_ready[2 +: 4]),.shared_req_op(shared_req_op[10 +: 20]),.shared_req_a(shared_req_a[128 +: 256]),.shared_req_b(shared_req_b[128 +: 256]),.shared_active(shared_active[2 +: 4]),.shared_rsp_valid(shared_rsp_valid[2 +: 4]),.shared_rsp_ready(shared_rsp_ready[2 +: 4]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(child_rst_n),.cmd_valid(rst_n && pc==RES_CMD),.cmd_ready(res_ready),
      .cmd_width(width),.cmd_height(height),.cmd_state(state_q),
      .point_rd_en(point_rd_en),.point_rd_view_id(point_rd_view_id),.point_rd_index(point_rd_index),
      .point_rd_valid(point_rd_valid),.point_rd_x_fp32(point_rd_x_fp32),.point_rd_y_fp32(point_rd_y_fp32),
      .data_valid(res_data_valid),.data_ready(rst_n && pc==RES_WAIT),.data_index(res_index),.data_fp64(res_value),.data_last(res_last),
      .rsp_valid(res_valid),.rsp_ready(rst_n && pc==RES_WAIT),.rsp_status(res_status),.rsp_cost_fp64(res_cost));
    wire rot_ready,rot_valid;wire [7:0] rot_status;wire [575:0] rot_matrix;
    rotation #(.FP_SHARED(FP_SHARED)) rot(.shared_req_valid(shared_req_valid[1 +: 1]),.shared_req_ready(shared_req_ready[1 +: 1]),.shared_req_op(shared_req_op[5 +: 5]),.shared_req_a(shared_req_a[64 +: 64]),.shared_req_b(shared_req_b[64 +: 64]),.shared_active(shared_active[1 +: 1]),.shared_rsp_valid(shared_rsp_valid[1 +: 1]),.shared_rsp_ready(shared_rsp_ready[1 +: 1]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(child_rst_n),.cmd_valid(rst_n && pc==ROT_CMD),.cmd_ready(rot_ready),.cmd_mode(1'b0),
      .cmd_rotvec_fp64({p[11+6*view],p[10+6*view],p[9+6*view]}),.cmd_r_fp64(576'b0),
      .rsp_valid(rot_valid),.rsp_ready(rst_n && pc==ROT_WAIT),.rsp_status(rot_status),.rsp_rotvec_fp64(),.rsp_r_fp64(rot_matrix));
    always @(posedge clk or negedge rst_n)begin
      if(!rst_n)begin pc<=IDLE;rsp_status<=0;usable<=0;metrics_valid<=0;map_phase<=0;stream_bad<=0;
        rotation_q<=0;rotation_index<=0;
        width<=0;height<=0;ix<=0;iy<=0;state_q<=0;square_size<=0;best_cost<=0;converged<=0;
        camera<=0;view_rms<=0;max_error<=0;index<=0;count<=0;point<=0;view<=0;row<=0;pair_a<=0;pair_b<=0;
        fp_op<=0;fp_a<=0;fp_b<=0;destination<=0;continuation<=0;
      end else case(pc)
        IDLE:if(cmd_valid)begin
          state_q<=cmd_state;width<=cmd_width;height<=cmd_height;converged<=cmd_converged;
          square_size<=cmd_square_size_fp64;best_cost<=(cmd_best_cost_fp64[62:0]==0)?ZERO:cmd_best_cost_fp64;
          usable<=0;metrics_valid<=0;map_phase<=0;rsp_status<=0;camera<=0;view_rms<=0;max_error<=0;
          v[30]<=u16(cmd_width);v[31]<=u16(cmd_height);pc<=DECODE_0;
          for(j=0;j<`PAR_STATE_N;j=j+1)begin if(!finite(cmd_state[64*j+:64]))fail(`PAR_CALIB_INVALID);end
          if(cmd_width<2 || cmd_height<2 || !finite(cmd_square_size_fp64) || cmd_square_size_fp64[63] || cmd_square_size_fp64[62:0]==0 ||
            !finite(cmd_best_cost_fp64) || (cmd_best_cost_fp64[63] && cmd_best_cost_fp64[62:0]!=0))fail(`PAR_BAD_CONFIG);
        end
        FP_REQ:if(fp_ready)pc<=FP_WAIT;
        FP_WAIT:if(fp_valid)begin
          if((|fp_flags[2:0]) || !finite(fp_result))begin if(map_phase)reject();else fail(`PAR_CALIB_INVALID);end
          else begin v[destination]<=fp_result;pc<=continuation;end
        end
        RESPONSE:if(rsp_ready)pc<=IDLE;
        DECODE_0: begin calculate(EXP,p[0],ZERO,0,DECODE_1); end
        DECODE_1: begin calculate(EXP,p[1],ZERO,1,DECODE_2); end
        DECODE_2: begin calculate(MUL,p[2],v[30],2,DECODE_3); end
        DECODE_3: begin calculate(MUL,p[3],v[31],3,COPY_DISTORTION); end
        COPY_DISTORTION: begin v[4]<=p[4];v[5]<=p[5];v[6]<=p[8];v[7]<=p[6];v[8]<=p[7];index<=0;pc<=QUANTIZE; end
        QUANTIZE: begin calculate(TO32,v[index],ZERO,20,SAVE_CAMERA); end
        SAVE_CAMERA: begin camera[32*index+:32]<=v[20][31:0];calculate(TO64,v[20],ZERO, index, NEXT_CAMERA); end
        NEXT_CAMERA: begin if(index==8)pc<=TOTAL_0;else begin index<=index+1;pc<=QUANTIZE;end end
        TOTAL_0: begin calculate(DIV,best_cost,u16(`PAR_TOTAL_POINTS),23,TOTAL_1); end
        TOTAL_1: begin calculate(SQRT,v[23],ZERO,23,TOTAL_2); end
        TOTAL_2: begin calculate(MUL,64'h3fa999999999999a,v[30],32,TOTAL_3); end
        TOTAL_3: begin calculate(MUL,64'h4034000000000000,v[30],33,RES_CMD); end
        RES_CMD: begin if(res_ready)begin count<=0;stream_bad<=0;pc<=RES_WAIT;end end
        RES_WAIT: begin if(res_data_valid)begin
          if(count>=`PAR_RESIDUALS || res_index!=count || res_last!=(count==(`PAR_RESIDUALS-1)) || !finite(res_value))stream_bad<=1;
          if(count<`PAR_RESIDUALS)begin residual[count]<=res_value;count<=count+1;end
        end
        if(res_valid)begin
          if(res_status!=0)fail(res_status);
          else if(stream_bad || count!=`PAR_RESIDUALS || res_data_valid)fail(`PAR_BAD_CONFIG);
          else if(!finite(res_cost))fail(`PAR_CALIB_INVALID);
          else begin point<=0;view<=0;v[22]<=0;max_error<=0;pc<=HYPOT_START;end
        end end
        HYPOT_START: begin // 缩放hypot，避免直接du*du+dv*dv产生不必要的溢出/下溢。
        if(magnitude(residual[2*point])>=magnitude(residual[2*point+1]))begin v[18]<=magnitude(residual[2*point]);v[19]<=magnitude(residual[2*point+1]);end
        else begin v[18]<=magnitude(residual[2*point+1]);v[19]<=magnitude(residual[2*point]);end
        pc<=HYPOT_CHECK; end
        HYPOT_CHECK: begin if(v[18]==0)begin v[21]<=0;pc<=ERROR_ACC_0;end else pc<=HYPOT_0; end
        HYPOT_0: begin calculate(DIV,v[19],v[18],20,HYPOT_1); end
        HYPOT_1: begin calculate(MUL,v[20],v[20],20,HYPOT_2); end
        HYPOT_2: begin calculate(ADD,ONE,v[20],20,HYPOT_3); end
        HYPOT_3: begin calculate(SQRT,v[20],ZERO,20,HYPOT_4); end
        HYPOT_4: begin calculate(MUL,v[18],v[20],21,ERROR_ACC_0); end
        ERROR_ACC_0: begin calculate(MUL,v[21],v[21],20,ERROR_ACC_1); end
        ERROR_ACC_1: begin calculate(ADD,v[22],v[20],22,NEXT_POINT); end
        NEXT_POINT: begin if(v[21]>max_error)max_error<=v[21];
        if(point==((view+1)*`PAR_POINTS-1))pc<=VIEW_RMS_0;
        else begin point<=point+1;pc<=HYPOT_START;end end
        VIEW_RMS_0: begin calculate(DIV,v[22],u16(`PAR_POINTS),20,VIEW_RMS_1); end
        VIEW_RMS_1: begin calculate(SQRT,v[20],ZERO,20,SAVE_VIEW_RMS); end
        SAVE_VIEW_RMS: begin view_rms[64*view+:64]<=v[20];
        if(view==(`PAR_VIEWS-1))begin view<=0;pc<=ROT_CMD;end
        else begin view<=view+1;point<=point+1;v[22]<=0;pc<=HYPOT_START;end end
        ROT_CMD: begin if(rot_ready)pc<=ROT_WAIT; end
        ROT_WAIT: begin if(rot_valid)begin
        if(rot_status!=0)fail(rot_status);
        else begin rotation_q<=rot_matrix;rotation_index<=0;pc<=SAVE_ROTATION;end end end
        // Capture on handshake, then use one pose write per cycle (nine cycles/view).
        SAVE_ROTATION: begin
          pose[12*view+rotation_index]<=rotation_q[64*rotation_index+:64];
          if(rotation_index==8)pc<=TZ;
          else rotation_index<=rotation_index+1'b1;
        end
        TZ: begin calculate(EXP,p[14+6*view],ZERO,24,TRANSLATION_START); end
        TRANSLATION_START: begin row<=0;pc<=TRANSLATION_0; end
        TRANSLATION_0: begin calculate(MUL,pose[12*view+3*row],board_half(`PAR_BOARD_COLS-1),18,TRANSLATION_1); end
        TRANSLATION_1: begin calculate(SUB,(row==2)?v[24]:p[12+6*view+row],v[18],18,TRANSLATION_2); end
        TRANSLATION_2: begin calculate(MUL,pose[12*view+3*row+1],board_half(`PAR_BOARD_ROWS-1),19,TRANSLATION_3); end
        TRANSLATION_3: begin calculate(SUB,v[18],v[19],18,TRANSLATION_4); end
        TRANSLATION_4: begin calculate(MUL,v[18],square_size,18,SAVE_TRANSLATION); end
        SAVE_TRANSLATION: begin pose[12*view+9+row]<=v[18];
        if(row<2)begin row<=row+1;pc<=TRANSLATION_0;end
        else if(view<(`PAR_VIEWS-1))begin view<=view+1;pc<=ROT_CMD;end
        else begin pair_a<=1;pair_b<=0;v[26]<=0;pc<=ANGLE_0;end end
        ANGLE_0: begin calculate(MUL,pose[12*pair_a+2],pose[12*pair_b+2],18,ANGLE_1); end
        ANGLE_1: begin calculate(MUL,pose[12*pair_a+5],pose[12*pair_b+5],19,ANGLE_2); end
        ANGLE_2: begin calculate(ADD,v[18],v[19],18,ANGLE_3); end
        ANGLE_3: begin calculate(MUL,pose[12*pair_a+8],pose[12*pair_b+8],19,ANGLE_4); end
        ANGLE_4: begin calculate(ADD,v[18],v[19],18,ANGLE_5); end
        ANGLE_5: begin calculate(ACOS,(magnitude(v[18])>ONE)?ONE:magnitude(v[18]),ZERO,18,NEXT_PAIR); end
        NEXT_PAIR: begin if(v[18]>v[26])v[26]<=v[18];
        if(pair_b<pair_a-1)begin pair_b<=pair_b+1;pc<=ANGLE_0;end
        else if(pair_a<`PAR_VIEWS-1)begin pair_a<=pair_a+1;pair_b<=0;pc<=ANGLE_0;end else pc<=CHECK_REPORT; end
        CHECK_REPORT: begin metrics_valid<=1;
        if(!converged || v[0][63] || v[1][63] || v[0]<=v[32] || v[1]<=v[32] || v[0]>=v[33] || v[1]>=v[33] ||
          (v[2][63] && magnitude(v[2])!=0) || (v[3][63] && magnitude(v[3])!=0) || v[2]>=v[30] && !v[2][63] || v[3]>=v[31] && !v[3][63] ||
          v[23]>=64'h4008000000000000 || v[26]<=64'h3f847ae147ae147b)reject();
        else begin ix<=0;iy<=0;map_phase<=1;pc<=MAP_0;end end
        MAP_0: begin calculate(MUL,u16(width-16'd1),u16(ix),18,MAP_1); end
        MAP_1: begin calculate(DIV,v[18],64'h4040000000000000,18,MAP_2); end
        MAP_2: begin calculate(SUB,v[18],v[2],18,MAP_3); end
        MAP_3: begin calculate(DIV,v[18],v[0],9,MAP_4); end
        MAP_4: begin calculate(MUL,u16(height-16'd1),u16(iy),18,MAP_5); end
        MAP_5: begin calculate(DIV,v[18],64'h4038000000000000,18,MAP_6); end
        MAP_6: begin calculate(SUB,v[18],v[3],18,MAP_7); end
        MAP_7: begin calculate(DIV,v[18],v[1],10,MAP_8); end
        MAP_8: begin calculate(MUL,v[9],v[9],18,MAP_9); end
        MAP_9: begin calculate(MUL,v[10],v[10],19,MAP_10); end
        MAP_10: begin calculate(ADD,v[18],v[19],11,MAP_11); end
        MAP_11: begin calculate(MUL,v[4],v[11],18,MAP_12); end
        MAP_12: begin calculate(ADD,ONE,v[18],12,MAP_13); end
        MAP_13: begin calculate(MUL,v[5],v[11],18,MAP_14); end
        MAP_14: begin calculate(MUL,v[18],v[11],18,MAP_15); end
        MAP_15: begin calculate(ADD,v[12],v[18],12,MAP_16); end
        MAP_16: begin calculate(MUL,v[6],v[11],18,MAP_17); end
        MAP_17: begin calculate(MUL,v[18],v[11],18,MAP_18); end
        MAP_18: begin calculate(MUL,v[18],v[11],18,MAP_19); end
        MAP_19: begin calculate(ADD,v[12],v[18],12,MAP_20); end
        MAP_20: begin calculate(MUL,64'h4000000000000000,v[5],18,MAP_21); end
        MAP_21: begin calculate(MUL,v[18],v[11],18,MAP_22); end
        MAP_22: begin calculate(ADD,v[4],v[18],13,MAP_23); end
        MAP_23: begin calculate(MUL,64'h4008000000000000,v[6],18,MAP_24); end
        MAP_24: begin calculate(MUL,v[18],v[11],18,MAP_25); end
        MAP_25: begin calculate(MUL,v[18],v[11],18,MAP_26); end
        MAP_26: begin calculate(ADD,v[13],v[18],13,MAP_27); end
        MAP_27: begin calculate(MUL,64'h4000000000000000,v[9],18,MAP_28); end
        MAP_28: begin calculate(MUL,v[18],v[9],18,MAP_29); end
        MAP_29: begin calculate(MUL,v[18],v[13],18,MAP_30); end
        MAP_30: begin calculate(ADD,v[12],v[18],14,MAP_31); end
        MAP_31: begin calculate(MUL,64'h4000000000000000,v[7],18,MAP_32); end
        MAP_32: begin calculate(MUL,v[18],v[10],18,MAP_33); end
        MAP_33: begin calculate(ADD,v[14],v[18],14,MAP_34); end
        MAP_34: begin calculate(MUL,64'h4018000000000000,v[8],18,MAP_35); end
        MAP_35: begin calculate(MUL,v[18],v[9],18,MAP_36); end
        MAP_36: begin calculate(ADD,v[14],v[18],14,MAP_37); end
        MAP_37: begin calculate(MUL,64'h4000000000000000,v[9],18,MAP_38); end
        MAP_38: begin calculate(MUL,v[18],v[10],18,MAP_39); end
        MAP_39: begin calculate(MUL,v[18],v[13],15,MAP_40); end
        MAP_40: begin calculate(MUL,64'h4000000000000000,v[7],18,MAP_41); end
        MAP_41: begin calculate(MUL,v[18],v[9],18,MAP_42); end
        MAP_42: begin calculate(ADD,v[15],v[18],15,MAP_43); end
        MAP_43: begin calculate(MUL,64'h4000000000000000,v[8],18,MAP_44); end
        MAP_44: begin calculate(MUL,v[18],v[10],18,MAP_45); end
        MAP_45: begin calculate(ADD,v[15],v[18],15,MAP_46); end
        MAP_46: begin calculate(MUL,64'h4000000000000000,v[10],18,MAP_47); end
        MAP_47: begin calculate(MUL,v[18],v[10],18,MAP_48); end
        MAP_48: begin calculate(MUL,v[18],v[13],18,MAP_49); end
        MAP_49: begin calculate(ADD,v[12],v[18],16,MAP_50); end
        MAP_50: begin calculate(MUL,64'h4018000000000000,v[7],18,MAP_51); end
        MAP_51: begin calculate(MUL,v[18],v[10],18,MAP_52); end
        MAP_52: begin calculate(ADD,v[16],v[18],16,MAP_53); end
        MAP_53: begin calculate(MUL,64'h4000000000000000,v[8],18,MAP_54); end
        MAP_54: begin calculate(MUL,v[18],v[9],18,MAP_55); end
        MAP_55: begin calculate(ADD,v[16],v[18],16,MAP_56); end
        MAP_56: begin calculate(MUL,v[14],v[16],18,MAP_57); end
        MAP_57: begin calculate(MUL,v[15],v[15],19,MAP_58); end
        MAP_58: begin calculate(SUB,v[18],v[19],17,MAP_CHECK); end
        MAP_CHECK: begin if(v[14][63] || magnitude(v[14])==0 || v[16][63] || magnitude(v[16])==0 || v[17][63] || v[17]<=64'h3f1a36e2eb1c432d)reject();
        else if(ix<32)begin ix<=ix+1;pc<=MAP_0;end
        else if(iy<24)begin ix<=0;iy<=iy+1;pc<=MAP_0;end
        else begin usable<=1;rsp_status<=0;pc<=RESPONSE;end end
        default:fail(`PAR_BAD_CONFIG);
      endcase
    end
endmodule
