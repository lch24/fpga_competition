`include "calib_defs.vh"

/*
配置：统一见 rtl/include/calib_config.vh；下文具体计数例子以默认3张5×8为例，
实际容量、循环边界和端口位宽由PAR_*派生，修改配置后须重新编译/综合。
作用：初值阶段的控制器，保留三张H，产生最多5份可优化的状态。
依次调用homography三次；检查重复视图；调用zhang，然后尝试4个固定焦距种子。
固定fx=fy=W*{0.6,1,1.8,3}，主点((W-1)/2,(H-1)/2)，畸变初值全0。
每个K调用pose_init得到全部视图R/t与27项状态；pose失败仅跳过该seed。
Zhang失败继续固定种子；DLT失败/重复视图/全部seed失败才使本阶段失败。
seed逐笔输出（没有last）；父模块缓存，每笔握手后推进；rsp表示枚举结束。
子模块：homography、zhang、pose_init、fp_operator；本层算术核计算视图差异及固定K。
重复判断沿用C++：sum(abs(H1[0:7]-H0[0:7]))+sum(abs(H2[0:7]-H0[0:7]))<1e-6。
因此两张重复、第三张不同不会被本层直接拒绝；是否可标定还需后续优化和验证。
seed_id允许跳号；seed_count仅在seed握手时加1，所有seed处理完毕才发rsp。
初始化成功只代表至少一组初值可用，不代表最终标定有效或LM已收敛。

角点读接口例外：只读已提交视图，固定1拍返回，无valid/ready背压。
E_n采样使能/地址，数据在E_n后有效并由调用方在E_(n+1)采样；详见corner_store。
首版最多一笔在途；层间读路由不插入寄存器，返回消费后才能切换所有者。

共同契约：
- 已实现：单事务顺序状态机；FP64 运算核按请求/响应串行复用。
- valid && ready 才传输；背压时 valid 和对应全部载荷保持不变。
- 除另有说明，一次一项任务，rsp 被接收前不接受新 cmd；不假设固定延迟。
- cmd_* 随命令锁存；rsp_* 随响应有效。失败仍须发完成响应，不能无声挂起。
- 非零状态通常使结果载荷无效；本模块明确说明的诊断有效位例外。
- 复位取消在途数据；复位后所有生产端 valid 清零。实现时禁止 real 运算。
*/
module init_controller #(parameter FP_SHARED=0) (
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
    output wire [2:0] seed_id, // 0=Zhang；1..4依次为四个固定焦距
    output wire [`PAR_STATE_W-1:0] seed_state, // 内部PAR_STATE_N项FP64
    output wire rsp_valid, // 完成响应有效；最后一笔输出握手后才置位
    input wire rsp_ready, // 接收完成响应
    output reg [7:0] rsp_status, // PAR_* 状态码；非零时结果载荷无效
    output wire [2:0] rsp_seed_count // 成功输出的seed数，0..5
);

    // 工作寄存器映射及各阶段见下方注释；数组不复位，事务内先写后读。
    localparam IDLE=9000, FP_REQ=9001, FP_WAIT=9002, RESPONSE=9003;
    localparam ADD=0,SUB=1,MUL=2,DIV=3,SQRT=4,LOG=9,CONVERT=11;
    localparam [63:0] ZERO=64'b0,ONE=64'h3ff0000000000000;
    reg [63:0] v[0:11];
    integer pc,destination,continuation,j;
    reg [4:0] fp_op;
    reg [63:0] fp_a,fp_b;
    wire fp_ready,fp_valid;
    wire [63:0] fp_result;
    wire [4:0] fp_flags;
    // v[0:1]=W/H,[2]=视图差异累计,[3]=临时量,[4:7]=当前K。
    reg [15:0] width,height;reg [`PAR_VIEW_BITS-1:0] view;reg [2:0] seed,count;
    reg [`PAR_H_ALL_W-1:0] hom_all;reg [`PAR_STATE_W-1:0] state_q;integer element,other_view;
    wire h_ready,h_valid,z_ready,z_valid,p_ready,p_valid;
    wire [7:0] h_status,z_status,p_status;
    wire [575:0] h_result;wire [255:0] z_result;wire [`PAR_STATE_W-1:0] p_result;
    assign seed_valid=rst_n && pc==16;
    assign seed_id=seed;assign seed_state=state_q;assign rsp_seed_count=count;

    // Reuse the entire Jacobi matrix store and controller between H and Zhang.
    wire z_eigen_owner=(pc==6 || pc==7);
    wire  h_eigen_rst_n,z_eigen_rst_n,common_eigen_rst_n;
    assign common_eigen_rst_n=z_eigen_owner?z_eigen_rst_n:h_eigen_rst_n;
    wire  h_eigen_cmd_valid,z_eigen_cmd_valid,common_eigen_cmd_valid;
    assign common_eigen_cmd_valid=z_eigen_owner?z_eigen_cmd_valid:h_eigen_cmd_valid;
    wire  h_eigen_cmd_ready,z_eigen_cmd_ready,common_eigen_cmd_ready;
    assign h_eigen_cmd_ready=(!z_eigen_owner) && common_eigen_cmd_ready;
    assign z_eigen_cmd_ready=(z_eigen_owner) && common_eigen_cmd_ready;
    wire [3:0] h_eigen_cmd_n,z_eigen_cmd_n,common_eigen_cmd_n;
    assign common_eigen_cmd_n=z_eigen_owner?z_eigen_cmd_n:h_eigen_cmd_n;
    wire  h_eigen_matrix_valid,z_eigen_matrix_valid,common_eigen_matrix_valid;
    assign common_eigen_matrix_valid=z_eigen_owner?z_eigen_matrix_valid:h_eigen_matrix_valid;
    wire  h_eigen_matrix_ready,z_eigen_matrix_ready,common_eigen_matrix_ready;
    assign h_eigen_matrix_ready=(!z_eigen_owner) && common_eigen_matrix_ready;
    assign z_eigen_matrix_ready=(z_eigen_owner) && common_eigen_matrix_ready;
    wire [63:0] h_eigen_matrix_fp64,z_eigen_matrix_fp64,common_eigen_matrix_fp64;
    assign common_eigen_matrix_fp64=z_eigen_owner?z_eigen_matrix_fp64:h_eigen_matrix_fp64;
    wire  h_eigen_matrix_last,z_eigen_matrix_last,common_eigen_matrix_last;
    assign common_eigen_matrix_last=z_eigen_owner?z_eigen_matrix_last:h_eigen_matrix_last;
    wire  h_eigen_rsp_valid,z_eigen_rsp_valid,common_eigen_rsp_valid;
    assign h_eigen_rsp_valid=(!z_eigen_owner) && common_eigen_rsp_valid;
    assign z_eigen_rsp_valid=(z_eigen_owner) && common_eigen_rsp_valid;
    wire  h_eigen_rsp_ready,z_eigen_rsp_ready,common_eigen_rsp_ready;
    assign common_eigen_rsp_ready=z_eigen_owner?z_eigen_rsp_ready:h_eigen_rsp_ready;
    wire [7:0] h_eigen_rsp_status,z_eigen_rsp_status,common_eigen_rsp_status;
    assign h_eigen_rsp_status=common_eigen_rsp_status;
    assign z_eigen_rsp_status=common_eigen_rsp_status;
    wire [575:0] h_eigen_rsp_min_vector_fp64,z_eigen_rsp_min_vector_fp64,common_eigen_rsp_min_vector_fp64;
    assign h_eigen_rsp_min_vector_fp64=common_eigen_rsp_min_vector_fp64;
    assign z_eigen_rsp_min_vector_fp64=common_eigen_rsp_min_vector_fp64;
    wire [63:0] h_eigen_rsp_min_value_fp64,z_eigen_rsp_min_value_fp64,common_eigen_rsp_min_value_fp64;
    assign h_eigen_rsp_min_value_fp64=common_eigen_rsp_min_value_fp64;
    assign z_eigen_rsp_min_value_fp64=common_eigen_rsp_min_value_fp64;
    wire [63:0] h_eigen_rsp_second_value_fp64,z_eigen_rsp_second_value_fp64,common_eigen_rsp_second_value_fp64;
    assign h_eigen_rsp_second_value_fp64=common_eigen_rsp_second_value_fp64;
    assign z_eigen_rsp_second_value_fp64=common_eigen_rsp_second_value_fp64;
    wire [63:0] h_eigen_rsp_max_value_fp64,z_eigen_rsp_max_value_fp64,common_eigen_rsp_max_value_fp64;
    assign h_eigen_rsp_max_value_fp64=common_eigen_rsp_max_value_fp64;
    assign z_eigen_rsp_max_value_fp64=common_eigen_rsp_max_value_fp64;
    wire [13:0] h_eigen_rsp_rotations,z_eigen_rsp_rotations,common_eigen_rsp_rotations;
    assign h_eigen_rsp_rotations=common_eigen_rsp_rotations;
    assign z_eigen_rsp_rotations=common_eigen_rsp_rotations;
    wire [1:0] h_fp_req_valid;
    assign shared_req_valid[1 +: 1]=h_fp_req_valid[0 +: 1];
    wire [1:0] h_fp_req_ready;
    assign h_fp_req_ready={1'd0,shared_req_ready[1 +: 1]};
    wire [9:0] h_fp_req_op;
    assign shared_req_op[5 +: 5]=h_fp_req_op[0 +: 5];
    wire [127:0] h_fp_req_a;
    assign shared_req_a[64 +: 64]=h_fp_req_a[0 +: 64];
    wire [127:0] h_fp_req_b;
    assign shared_req_b[64 +: 64]=h_fp_req_b[0 +: 64];
    wire [1:0] h_fp_active;
    assign shared_active[1 +: 1]=h_fp_active[0 +: 1];
    wire [1:0] h_fp_rsp_valid;
    assign h_fp_rsp_valid={1'd0,shared_rsp_valid[1 +: 1]};
    wire [1:0] h_fp_rsp_ready;
    assign shared_rsp_ready[1 +: 1]=h_fp_rsp_ready[0 +: 1];
    jacobi_eigen #(.FP_SHARED(FP_SHARED)) common_eigen(.clk(clk),.rst_n(common_eigen_rst_n),.cmd_valid(common_eigen_cmd_valid),.cmd_ready(common_eigen_cmd_ready),.cmd_n(common_eigen_cmd_n),.matrix_valid(common_eigen_matrix_valid),.matrix_ready(common_eigen_matrix_ready),.matrix_fp64(common_eigen_matrix_fp64),.matrix_last(common_eigen_matrix_last),.rsp_valid(common_eigen_rsp_valid),.rsp_ready(common_eigen_rsp_ready),.rsp_status(common_eigen_rsp_status),.rsp_min_vector_fp64(common_eigen_rsp_min_vector_fp64),.rsp_min_value_fp64(common_eigen_rsp_min_value_fp64),.rsp_second_value_fp64(common_eigen_rsp_second_value_fp64),.rsp_max_value_fp64(common_eigen_rsp_max_value_fp64),.rsp_rotations(common_eigen_rsp_rotations),
.shared_req_valid(shared_req_valid[2 +: 1]),.shared_req_ready(shared_req_ready[2 +: 1]),.shared_req_op(shared_req_op[10 +: 5]),.shared_req_a(shared_req_a[128 +: 64]),.shared_req_b(shared_req_b[128 +: 64]),.shared_active(shared_active[2 +: 1]),.shared_rsp_valid(shared_rsp_valid[2 +: 1]),.shared_rsp_ready(shared_rsp_ready[2 +: 1]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags));
    homography #(.FP_SHARED(FP_SHARED),.SHARE_EIGEN(1)) h_core(.eigen_rst_n(h_eigen_rst_n),.eigen_cmd_valid(h_eigen_cmd_valid),.eigen_cmd_ready(h_eigen_cmd_ready),.eigen_cmd_n(h_eigen_cmd_n),.eigen_matrix_valid(h_eigen_matrix_valid),.eigen_matrix_ready(h_eigen_matrix_ready),.eigen_matrix_fp64(h_eigen_matrix_fp64),.eigen_matrix_last(h_eigen_matrix_last),.eigen_rsp_valid(h_eigen_rsp_valid),.eigen_rsp_ready(h_eigen_rsp_ready),.eigen_rsp_status(h_eigen_rsp_status),.eigen_rsp_min_vector_fp64(h_eigen_rsp_min_vector_fp64),.eigen_rsp_min_value_fp64(h_eigen_rsp_min_value_fp64),.eigen_rsp_second_value_fp64(h_eigen_rsp_second_value_fp64),.eigen_rsp_max_value_fp64(h_eigen_rsp_max_value_fp64),.eigen_rsp_rotations(h_eigen_rsp_rotations),.shared_req_valid(h_fp_req_valid),.shared_req_ready(h_fp_req_ready),.shared_req_op(h_fp_req_op),.shared_req_a(h_fp_req_a),.shared_req_b(h_fp_req_b),.shared_active(h_fp_active),.shared_rsp_valid(h_fp_rsp_valid),.shared_rsp_ready(h_fp_rsp_ready),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(rst_n),.cmd_valid(rst_n && pc==0),.cmd_ready(h_ready),
        .cmd_width(width),.cmd_height(height),.cmd_view_id(view),.point_rd_en(point_rd_en),.point_rd_view_id(point_rd_view_id),
        .point_rd_index(point_rd_index),.point_rd_valid(point_rd_valid),.point_rd_x_fp32(point_rd_x_fp32),.point_rd_y_fp32(point_rd_y_fp32),
        .rsp_valid(h_valid),.rsp_ready(rst_n && pc==1),.rsp_status(h_status),.rsp_h_fp64(h_result));
    zhang #(.FP_SHARED(FP_SHARED),.SHARE_EIGEN(1)) z_core(.eigen_rst_n(z_eigen_rst_n),.eigen_cmd_valid(z_eigen_cmd_valid),.eigen_cmd_ready(z_eigen_cmd_ready),.eigen_cmd_n(z_eigen_cmd_n),.eigen_matrix_valid(z_eigen_matrix_valid),.eigen_matrix_ready(z_eigen_matrix_ready),.eigen_matrix_fp64(z_eigen_matrix_fp64),.eigen_matrix_last(z_eigen_matrix_last),.eigen_rsp_valid(z_eigen_rsp_valid),.eigen_rsp_ready(z_eigen_rsp_ready),.eigen_rsp_status(z_eigen_rsp_status),.eigen_rsp_min_vector_fp64(z_eigen_rsp_min_vector_fp64),.eigen_rsp_min_value_fp64(z_eigen_rsp_min_value_fp64),.eigen_rsp_second_value_fp64(z_eigen_rsp_second_value_fp64),.eigen_rsp_max_value_fp64(z_eigen_rsp_max_value_fp64),.eigen_rsp_rotations(z_eigen_rsp_rotations),.shared_req_valid(shared_req_valid[3 +: 2]),.shared_req_ready(shared_req_ready[3 +: 2]),.shared_req_op(shared_req_op[15 +: 10]),.shared_req_a(shared_req_a[192 +: 128]),.shared_req_b(shared_req_b[192 +: 128]),.shared_active(shared_active[3 +: 2]),.shared_rsp_valid(shared_rsp_valid[3 +: 2]),.shared_rsp_ready(shared_rsp_ready[3 +: 2]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(rst_n),.cmd_valid(rst_n && pc==6),.cmd_ready(z_ready),
        .cmd_width(width),.cmd_height(height),.cmd_h_all_fp64(hom_all),
        .rsp_valid(z_valid),.rsp_ready(rst_n && pc==7),.rsp_status(z_status),.rsp_k_fp64(z_result));
    pose_init #(.FP_SHARED(FP_SHARED)) p_core(.shared_req_valid(shared_req_valid[5 +: 2]),.shared_req_ready(shared_req_ready[5 +: 2]),.shared_req_op(shared_req_op[25 +: 10]),.shared_req_a(shared_req_a[320 +: 128]),.shared_req_b(shared_req_b[320 +: 128]),.shared_active(shared_active[5 +: 2]),.shared_rsp_valid(shared_rsp_valid[5 +: 2]),.shared_rsp_ready(shared_rsp_ready[5 +: 2]),.shared_rsp_result(shared_rsp_result),.shared_rsp_flags(shared_rsp_flags),.clk(clk),.rst_n(rst_n),.cmd_valid(rst_n && pc==14),.cmd_ready(p_ready),
        .cmd_width(width),.cmd_height(height),.cmd_h_all_fp64(hom_all),.cmd_k_fp64({v[7],v[6],v[5],v[4]}),
        .rsp_valid(p_valid),.rsp_ready(rst_n && pc==15),.rsp_status(p_status),.rsp_state(p_result));
    function [63:0] factor;input [2:0] id;begin case(id)
        1:factor=64'h3fe3333333333333;2:factor=64'h3ff0000000000000;3:factor=64'h3ffccccccccccccd;default:factor=64'h4008000000000000;endcase end endfunction
    assign cmd_ready=rst_n && pc==IDLE;
    assign rsp_valid=rst_n && pc==RESPONSE;
    function finite;
        input [63:0] x;
        begin finite=(x[62:52]!=11'h7ff);end
    endfunction
    // 精确无符号16位整数转FP64，不使用综合不支持的 real。
    function [63:0] u16;
        input [15:0] x;
        integer k,top;
        reg [63:0] shifted;
        reg [10:0] exponent;
        begin top=0;for(k=0;k<16;k=k+1)if(x[k])top=k;
            shifted={48'b0,x} << (52-top);exponent=1023+top;
            u16=(x==0)?64'b0:{1'b0,exponent,shifted[51:0]};end
    endfunction

    generate if(FP_SHARED) begin : g_shared_fp
        assign shared_req_valid[0 +: 1] = rst_n && pc==FP_REQ;
        assign shared_req_op[0 +: 5] = fp_op;
        assign shared_req_a[0 +: 64] = fp_a;
        assign shared_req_b[0 +: 64] = fp_b;
        assign shared_rsp_ready[0 +: 1] = rst_n && pc==FP_WAIT;
        assign shared_active[0] = rst_n;
        assign fp_ready = shared_req_ready[0];
        assign fp_valid = shared_rsp_valid[0];
        assign fp_result = shared_rsp_result;
        assign fp_flags = shared_rsp_flags;
    end else begin : g_local_fp
    fp_operator #(.FP_W(64), .ENABLE_EXP(0), .ENABLE_LOG(0), .ENABLE_SINCOS(0), .ENABLE_ATAN_ACOS(0)) arithmetic(.clk(clk),.rst_n(rst_n),
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

    task calculate;
        input [4:0] operation;input [63:0] a,b;input integer target,next_pc;
        begin fp_op<=operation;fp_a<=a;fp_b<=b;destination<=target;continuation<=next_pc;pc<=FP_REQ;end
    endtask
    task fail;
        input [7:0] status;
        begin rsp_status<=status;pc<=RESPONSE;end
    endtask
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pc<=IDLE;rsp_status<=0;fp_op<=0;fp_a<=0;fp_b<=0;destination<=0;continuation<=0;
            width<=0;height<=0;view<=0;seed<=0;count<=0;state_q<=0;element<=0;other_view<=1;
        end else case(pc)
            IDLE: if(cmd_valid) begin
                rsp_status<=0;pc<=0;
                width<=cmd_width;height<=cmd_height;v[0]<=u16(cmd_width);v[1]<=u16(cmd_height);v[2]<=0;view<=0;seed<=0;count<=0;element<=0;other_view<=1;if(cmd_width<2 || cmd_height<2)fail(`PAR_BAD_CONFIG);
            end
            FP_REQ: if(fp_ready) pc<=FP_WAIT;
            FP_WAIT: if(fp_valid) begin
                if((|fp_flags[2:0]) || !finite(fp_result)) fail(`PAR_CALIB_INVALID);
                else begin v[destination]<=fp_result;pc<=continuation;end
            end
            RESPONSE: if(rsp_ready) pc<=IDLE;
                // 逐图求H，所有H成功后才能进入种子枚举
                0: begin if(h_ready)pc<=1; end
                1: begin if(h_valid)begin if(h_status!=0)fail(h_status);else begin hom_all[576*view +:576]<=h_result;if(view==(`PAR_VIEWS-1))pc<=2;else begin view<=view+1;pc<=0;end end end end
                // 与C++一致：累加H1/H2相对H0的前8项绝对差；仅全部重复时拒绝
                2: begin calculate(SUB, hom_all[(other_view*9+element)*64 +:64], hom_all[element*64 +:64], 3, 3); end
                3: begin calculate(ADD, v[2], (v[3] & 64'h7fffffffffffffff), 2, 4); end
                4: begin if(element<7)begin element<=element+1;pc<=2;end else if(other_view<(`PAR_VIEWS-1))begin element<=0;other_view<=other_view+1;pc<=2;end else pc<=5; end
                5: begin if(v[2]<64'h3eb0c6f7a0b5ed8d)fail(`PAR_CALIB_INVALID);else pc<=6; end
                6: begin if(z_ready)pc<=7; end
                7: begin if(z_valid)begin if(z_status==0)begin for(j=0;j<4;j=j+1)v[4+j]<=z_result[64*j +:64];pc<=14;end else begin seed<=1;pc<=8;end end end
                // Zhang失败仍尝试四个固定焦距；每组pose失败仅跳过该组
                8: begin calculate(MUL, v[0], factor(seed), 4, 9); end
                9: begin v[5]<=v[4];pc<=10; end
                10: begin calculate(SUB, v[0], ONE, 6, 11); end
                11: begin calculate(MUL, v[6], 64'h3fe0000000000000, 6, 12); end
                12: begin calculate(SUB, v[1], ONE, 7, 13); end
                13: begin calculate(MUL, v[7], 64'h3fe0000000000000, 7, 14); end
                14: begin if(p_ready)pc<=15; end
                15: begin if(p_valid)begin if(p_status!=0)pc<=17;else begin state_q<=p_result;pc<=16;end end end
                // 输出背压时state/id保持；只统计真正被父模块接收的seed
                16: begin if(seed_ready)begin count<=count+1'b1;pc<=17;end end
                17: begin if(seed==4)begin if(count==0)fail(`PAR_CALIB_INVALID);else pc<=RESPONSE;end else begin seed<=seed+1'b1;pc<=8;end end
            default: fail(`PAR_CALIB_INVALID);
        endcase
    end
endmodule
