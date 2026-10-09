`include "calib_defs.vh"

// Sparse bundle normal equations. Input remains the ordered column stream,
// but off-view pose derivatives must be zero and are never stored. Camera
// columns span all views; pose columns span one view. Cross-view pose blocks
// are emitted as zero without invoking the FP64 engine. One synchronous J
// read port avoids duplicated block RAM; dot products retain row order.
// Abort cancels partial loading/output; a successful load initializes all
// locations that will be read. RAM contents are never globally reset.
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
    reg [63:0] jram[0:`PAR_RESIDUALS*14-1],rram[0:(`PAR_RESIDUALS-1)];
    reg [63:0] operand_a,operand_b,max_gradient;
    integer n,rc,jc,jr,a,b,t,limit;reg kind;
    // Structural sparsity: eight camera columns plus six local pose columns.
    // A pose column is stored only in its owning view's rows. Thus storage is
    // 14*residuals, independent of the number of poses in the state vector.
    // Constant lookup decoding avoids variable divide/modulo in address logic.
    function integer pose_view;
      input integer col; integer view_id,axis;
      begin
        pose_view=-1;
        for(view_id=0;view_id<`PAR_VIEWS;view_id=view_id+1)
          for(axis=0;axis<6;axis=axis+1)
            if(col==4+6*view_id+axis)pose_view=view_id;
      end
    endfunction
    function integer packed_column;
      input integer col; integer view_id,axis;
      begin
        packed_column=col-6*`PAR_VIEWS;
        if(col<4)packed_column=col;
        for(view_id=0;view_id<`PAR_VIEWS;view_id=view_id+1)
          for(axis=0;axis<6;axis=axis+1)
            if(col==4+6*view_id+axis)packed_column=8+axis;
      end
    endfunction
    wire write_local=pose_view(jc)>=0;
    wire write_owned=!write_local ||
      (jr>=pose_view(jc)*(2*`PAR_POINTS) && jr<(pose_view(jc)+1)*(2*`PAR_POINTS));
    wire [$clog2(`PAR_RESIDUALS*14)-1:0] j_read_address=
        packed_column((pc==2)?a:b)*`PAR_RESIDUALS+t;
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
            else if(!write_owned && j_fp64[62:0]!=0)fail(`PAR_BAD_CONFIG);
            else begin if(write_owned)jram[packed_column(jc)*`PAR_RESIDUALS+jr]<=j_fp64;if(jr==(`PAR_RESIDUALS-1))begin jr<=0;jc<=jc+1;end else jr<=jr+1;end
          end
          if(rc==`PAR_RESIDUALS && jc==n)begin a<=0;b<=0;kind<=0;pc<=1;end
        end
        1:begin
          v[0]<=0;
          if(!kind && pose_view(a)>=0 && pose_view(b)>=0 && pose_view(a)!=pose_view(b))pc<=6;
          else begin
            if(pose_view(a)>=0)begin t<=pose_view(a)*(2*`PAR_POINTS);limit<=(pose_view(a)+1)*(2*`PAR_POINTS)-1;end
            else if(!kind && pose_view(b)>=0)begin t<=pose_view(b)*(2*`PAR_POINTS);limit<=(pose_view(b)+1)*(2*`PAR_POINTS)-1;end
            else begin t<=0;limit<=`PAR_RESIDUALS-1;end
            pc<=2;
          end
        end
        2:pc<=8;
        8:begin operand_a<=j_read_data;pc<=9;end
        9:begin operand_b<=kind?rram[t]:j_read_data;pc<=3;end
        3:calculate(MUL,operand_a,operand_b,1,4);
        4:calculate(ADD,v[0],v[1],0,5);
        5:if(t==limit)pc<=6;else begin t<=t+1;pc<=2;end
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
