`include "calib_defs.vh"
/*
6/9阶实对称矩阵Jacobi特征分解。完整载入n*n个FP64后开始，末元素last=1。
A与V各81x64位，固定9槽行跨度，同步双读单写；V在载入时写为单位阵。
先拒绝NaN/Inf及不对称输入（+0/-0视为相等），不静默取某一三角替代输入。
每轮按行扫描上三角，取最大绝对非对角项；相等保留首次位置。
largest<=1e-14*max(max_abs_diag,1e-30)停止。默认最多100*n*n轮搜索；
最后允许的一轮旋转后直接失败，不额外增加一次收敛检查，与C++循环边界一致。
MAX_SEARCH_ROUNDS=0采用默认值；非零用于限制迭代或测试退出路径，应在1..16383。
角度0.5*atan2(2*apq,aqq-app)，分别计算sin/cos，所有乘加分开舍入。
A非对角项和V列成对读取旧值，旋转后逐项写回；A两侧保持对称。
收敛后对特征值索引稳定排序，仅输出最小向量及最小/次小/最大值；
相同特征值保留原列顺序，退化特征子空间中的基不保证与其他软件一致。
错误码：尺寸/last/不对称=BAD_CONFIG，非有限/算术异常/不收敛=CALIB_INVALID。
输入含非有限值仍收完整矩阵；提前last终止输入，不支持残余旧流跨任务。
一次一任务；响应背压时保持；失败数值输出0，rotations仍保留已完成旋转数。
复位取消任务和算术请求，不清RAM。没有输入等待超时，调用方须提供完整输入。
可综合RTL功能版本；未验证目标器件资源和时序。
*/
module jacobi_eigen #(parameter FP_SHARED=0, parameter MAX_SEARCH_ROUNDS=0) (
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

    input wire clk,
    input wire rst_n,
    input wire cmd_valid,
    output wire cmd_ready,
    input wire [3:0] cmd_n,
    input wire matrix_valid,
    output wire matrix_ready,
    input wire [63:0] matrix_fp64,
    input wire matrix_last,
    output wire rsp_valid,
    input wire rsp_ready,
    output reg [7:0] rsp_status,
    output reg [575:0] rsp_min_vector_fp64,
    output reg [63:0] rsp_min_value_fp64,
    output reg [63:0] rsp_second_value_fp64,
    output reg [63:0] rsp_max_value_fp64,
    output reg [13:0] rsp_rotations
);
    localparam [63:0] ONE=64'h3ff0000000000000,TWO=64'h4000000000000000,
        HALF=64'h3fe0000000000000,TINY=64'h39b4484bfeebc2a0,TOL=64'h3d06849b86a12b9b;
    localparam IDLE=0,LOAD=1,SYM_START=2,SYM_CHECK=3,LOOP_START=4,SCAN=5,
        THRESHOLD=6,THRESH_DONE=7,PIV_FETCH=8,PIV_CAPTURE=9,OFF_CAPTURE=10,
        OFF_NEXT=11,PAIR_CAPTURE=12,OFF_W1=13,OFF_W2=14,OFF_W3=15,OFF_W4=16,
        DIAG_W1=17,DIAG_W2=18,DIAG_W3=19,DIAG_W4=20,V_NEXT=21,V_W1=22,V_W2=23,
        SORT_START=24,SORT=25,OUTPUT_START=26,OUTPUT_CAPTURE=27,
        MEM_READ=28,MEM_WAIT=29,PROGRAM=30,FP_REQ=31,FP_WAIT=32,RESPONSE=33;
    localparam ANGLE=0,DIAGONAL=1,PAIR=2,BOUND=3;
    reg [5:0] state,mem_return,program_return;
    reg [1:0] program_kind;
    reg [3:0] step,dest;
    reg [3:0] n,r,c,p,q,k,sort_pass,sort_pos;
    reg [13:0] search_limit;
    reg bad_input,read_bank;
    reg [6:0] read_addr_a,read_addr_b;
    reg [63:0] read_a,read_b;
    reg [63:0] a_mem[0:80],v_mem[0:80];
    reg [63:0] scalar[0:15];
    reg [63:0] eigenvalue[0:8];
    reg [3:0] order[0:8];
    reg [63:0] largest,diagonal,app,aqq,apq;
    reg [4:0] fp_op;
    reg [63:0] fp_a,fp_b;
    wire fp_req_ready,fp_rsp_valid;
    wire [63:0] fp_result;
    wire [4:0] fp_flags;
    reg a_we,v_we;
    reg [6:0] write_addr;
    reg [63:0] a_write,v_write;
    integer i;

    function [6:0] address;
        input [3:0] row,col;
        begin address={3'd0,row}*7'd9+{3'd0,col};end
    endfunction
    function finite;
        input [63:0] value;
        begin finite=(value[62:52]!=11'h7ff);end
    endfunction
    function less;
        input [63:0] a,b;
        begin
            if(a[62:0]==0 && b[62:0]==0)less=0;
            else if(a[63]!=b[63])less=a[63];
            else less=a[63]?(a>b):(a<b);
        end
    endfunction
    task fail;
        input [7:0] status;
        begin
            rsp_status<=status;rsp_min_vector_fp64<=0;rsp_min_value_fp64<=0;
            rsp_second_value_fp64<=0;rsp_max_value_fp64<=0;state<=RESPONSE;
        end
    endtask
    task fetch;
        input bank;
        input [6:0] a,b;
        input [5:0] next_state;
        begin
            read_bank<=bank;read_addr_a<=a;read_addr_b<=b;mem_return<=next_state;state<=MEM_READ;
        end
    endtask
    task run_program;
        input [1:0] kind;
        input [5:0] next_state;
        begin program_kind<=kind;step<=0;program_return<=next_state;state<=PROGRAM;end
    endtask

    assign cmd_ready=rst_n && state==IDLE;
    assign matrix_ready=rst_n && state==LOAD;
    assign rsp_valid=rst_n && state==RESPONSE;

    generate if(FP_SHARED) begin : g_shared_fp
        assign shared_req_valid[0 +: 1] = rst_n && state==FP_REQ;
        assign shared_req_op[0 +: 5] = fp_op;
        assign shared_req_a[0 +: 64] = fp_a;
        assign shared_req_b[0 +: 64] = fp_b;
        assign shared_rsp_ready[0 +: 1] = rst_n && state==FP_WAIT;
        assign shared_active[0] = rst_n;
        assign fp_req_ready = shared_req_ready[0];
        assign fp_rsp_valid = shared_rsp_valid[0];
        assign fp_result = shared_rsp_result;
        assign fp_flags = shared_rsp_flags;
    end else begin : g_local_fp
    fp_operator #(.FP_W(64), .ENABLE_EXP(0), .ENABLE_LOG(0), .ENABLE_SINCOS(1), .ENABLE_ATAN_ACOS(1)) arithmetic(
        .clk(clk),.rst_n(rst_n),.req_valid(rst_n && state==FP_REQ),.req_ready(fp_req_ready),
        .req_op(fp_op),.req_a(fp_a),.req_b(fp_b),.rsp_valid(fp_rsp_valid),
        .rsp_ready(rst_n && state==FP_WAIT),.rsp_result(fp_result),.rsp_flags(fp_flags),
        .rsp_less(),.rsp_equal(),.rsp_unordered()
    );
assign shared_req_valid[0 +: 1] = 0;
assign shared_req_op[0 +: 5] = 0;
assign shared_req_a[0 +: 64] = 0;
assign shared_req_b[0 +: 64] = 0;
assign shared_active[0 +: 1] = 0;
assign shared_rsp_ready[0 +: 1] = 0;
    end endgenerate


    // 微步骤表：只连接共享算术单元，所有浮点运算均等待握手。
    always @* begin
        fp_op=`PAR_FP_MUL;fp_a=0;fp_b=0;dest=0;
        case(program_kind)
        ANGLE: case(step)
            0:begin fp_a=TWO;fp_b=apq;dest=3;end
            1:begin fp_op=`PAR_FP_SUB;fp_a=aqq;fp_b=app;dest=4;end
            2:begin fp_op=`PAR_FP_ATAN2;fp_a=scalar[3];fp_b=scalar[4];dest=5;end
            3:begin fp_a=HALF;fp_b=scalar[5];dest=5;end
            4:begin fp_op=`PAR_FP_COS;fp_a=scalar[5];dest=6;end
            5:begin fp_op=`PAR_FP_SIN;fp_a=scalar[5];dest=7;end
            default:begin end
        endcase
        DIAGONAL: case(step)
            0:begin fp_a=scalar[6];fp_b=scalar[6];dest=8;end
            1:begin fp_a=scalar[7];fp_b=scalar[7];dest=9;end
            2:begin fp_a=TWO;fp_b=scalar[7];dest=10;end
            3:begin fp_a=scalar[10];fp_b=scalar[6];dest=10;end
            4:begin fp_a=scalar[10];fp_b=apq;dest=10;end
            5:begin fp_a=scalar[8];fp_b=app;dest=11;end
            6:begin fp_a=scalar[9];fp_b=aqq;dest=12;end
            7:begin fp_op=`PAR_FP_SUB;fp_a=scalar[11];fp_b=scalar[10];dest=13;end
            8:begin fp_op=`PAR_FP_ADD;fp_a=scalar[13];fp_b=scalar[12];dest=14;end
            9:begin fp_a=scalar[9];fp_b=app;dest=11;end
            10:begin fp_a=scalar[8];fp_b=aqq;dest=12;end
            11:begin fp_op=`PAR_FP_ADD;fp_a=scalar[11];fp_b=scalar[10];dest=13;end
            12:begin fp_op=`PAR_FP_ADD;fp_a=scalar[13];fp_b=scalar[12];dest=15;end
            default:begin end
        endcase
        PAIR: case(step)
            0:begin fp_a=scalar[6];fp_b=scalar[0];dest=8;end
            1:begin fp_a=scalar[7];fp_b=scalar[1];dest=9;end
            2:begin fp_op=`PAR_FP_SUB;fp_a=scalar[8];fp_b=scalar[9];dest=10;end
            3:begin fp_a=scalar[7];fp_b=scalar[0];dest=8;end
            4:begin fp_a=scalar[6];fp_b=scalar[1];dest=9;end
            5:begin fp_op=`PAR_FP_ADD;fp_a=scalar[8];fp_b=scalar[9];dest=11;end
            default:begin end
        endcase
        BOUND:begin fp_a=(diagonal>TINY)?diagonal:TINY;fp_b=TOL;dest=0;end
        endcase
    end

    always @* begin
        a_we=0;v_we=0;write_addr=0;a_write=0;v_write=0;
        if(rst_n)case(state)
        LOAD:if(matrix_valid && matrix_ready)begin
            a_we=1;v_we=1;write_addr=address(r,c);a_write=matrix_fp64;v_write=(r==c)?ONE:64'd0;
        end
        OFF_W1:begin a_we=1;write_addr=address(k,p);a_write=scalar[10];end
        OFF_W2:begin a_we=1;write_addr=address(p,k);a_write=scalar[10];end
        OFF_W3:begin a_we=1;write_addr=address(k,q);a_write=scalar[11];end
        OFF_W4:begin a_we=1;write_addr=address(q,k);a_write=scalar[11];end
        DIAG_W1:begin a_we=1;write_addr=address(p,p);a_write=scalar[14];end
        DIAG_W2:begin a_we=1;write_addr=address(q,q);a_write=scalar[15];end
        DIAG_W3:begin a_we=1;write_addr=address(p,q);a_write=0;end
        DIAG_W4:begin a_we=1;write_addr=address(q,p);a_write=0;end
        V_W1:begin v_we=1;write_addr=address(k,p);v_write=scalar[10];end
        V_W2:begin v_we=1;write_addr=address(k,q);v_write=scalar[11];end
        default:begin end
        endcase
    end
    always @(posedge clk)begin
        if(a_we)a_mem[write_addr]<=a_write;
        if(v_we)v_mem[write_addr]<=v_write;
        if(rst_n && state==MEM_READ)begin
            read_a<=read_bank?v_mem[read_addr_a]:a_mem[read_addr_a];
            read_b<=read_bank?v_mem[read_addr_b]:a_mem[read_addr_b];
        end
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;mem_return<=IDLE;program_return<=IDLE;program_kind<=0;step<=0;
            n<=0;r<=0;c<=0;p<=0;q<=1;k<=0;sort_pass<=0;sort_pos<=0;search_limit<=0;
            bad_input<=0;read_bank<=0;read_addr_a<=0;read_addr_b<=0;
            largest<=0;diagonal<=0;app<=0;aqq<=0;apq<=0;
            rsp_status<=`PAR_OK;rsp_min_vector_fp64<=0;rsp_min_value_fp64<=0;
            rsp_second_value_fp64<=0;rsp_max_value_fp64<=0;rsp_rotations<=0;
            for(i=0;i<16;i=i+1)scalar[i]<=0;
            for(i=0;i<9;i=i+1)begin eigenvalue[i]<=0;order[i]<=i;end
        end else case(state)
        IDLE:if(cmd_valid && cmd_ready)begin
            rsp_status<=`PAR_OK;rsp_min_vector_fp64<=0;rsp_min_value_fp64<=0;
            rsp_second_value_fp64<=0;rsp_max_value_fp64<=0;rsp_rotations<=0;
            for(i=0;i<9;i=i+1)order[i]<=i;
            if((cmd_n!=6 && cmd_n!=9) || MAX_SEARCH_ROUNDS<0 || MAX_SEARCH_ROUNDS>16383)
                fail(`PAR_BAD_CONFIG);
            else begin
                n<=cmd_n;r<=0;c<=0;bad_input<=0;
                search_limit<=(MAX_SEARCH_ROUNDS!=0)?MAX_SEARCH_ROUNDS:((cmd_n==6)?3600:8100);
                state<=LOAD;
            end
        end
        LOAD:if(matrix_valid && matrix_ready)begin
            if(!finite(matrix_fp64))bad_input<=1;
            if(matrix_last != ((r==n-1'b1)&&(c==n-1'b1)))fail(`PAR_BAD_CONFIG);
            else if(r==n-1'b1 && c==n-1'b1)begin
                if(bad_input || !finite(matrix_fp64))fail(`PAR_CALIB_INVALID);
                else begin r<=0;c<=1;state<=SYM_START;end
            end else if(c==n-1'b1)begin c<=0;r<=r+1'b1;end
            else c<=c+1'b1;
        end
        SYM_START:fetch(0,address(r,c),address(c,r),SYM_CHECK);
        SYM_CHECK:begin
            if(read_a!=read_b && !(read_a[62:0]==0 && read_b[62:0]==0))fail(`PAR_BAD_CONFIG);
            else if(r==n-2 && c==n-1'b1)state<=LOOP_START;
            else begin
                if(c==n-1'b1)begin r<=r+1'b1;c<=r+2;end else c<=c+1'b1;
                state<=SYM_START;
            end
        end
        LOOP_START:begin
            if(rsp_rotations>=search_limit)fail(`PAR_CALIB_INVALID);
            else begin
                largest<=0;diagonal<=0;p<=0;q<=1;r<=0;c<=0;
                fetch(0,address(0,0),address(0,0),SCAN);
            end
        end
        SCAN:begin
            if(r==c)begin
                eigenvalue[r]<=read_a;
                if({1'b0,read_a[62:0]}>diagonal)diagonal<={1'b0,read_a[62:0]};
            end else if({1'b0,read_a[62:0]}>largest)begin
                largest<={1'b0,read_a[62:0]};p<=r;q<=c;
            end
            if(r==n-1'b1 && c==n-1'b1)state<=THRESHOLD;
            else if(c==n-1'b1)begin
                r<=r+1'b1;c<=r+1'b1;fetch(0,address(r+1'b1,r+1'b1),address(r+1'b1,r+1'b1),SCAN);
            end else begin c<=c+1'b1;fetch(0,address(r,c+1'b1),address(r,c+1'b1),SCAN);end
        end
        THRESHOLD:run_program(BOUND,THRESH_DONE);
        THRESH_DONE:begin
            if(largest<=scalar[0])state<=SORT_START;
            else state<=PIV_FETCH;
        end
        PIV_FETCH:fetch(0,address(p,p),address(q,q),PIV_CAPTURE);
        PIV_CAPTURE:begin app<=read_a;aqq<=read_b;fetch(0,address(p,q),address(p,q),OFF_CAPTURE);end
        OFF_CAPTURE:begin apq<=read_a;k<=0;run_program(ANGLE,OFF_NEXT);end
        OFF_NEXT:begin
            if(k==n)run_program(DIAGONAL,DIAG_W1);
            else if(k==p || k==q)k<=k+1'b1;
            else fetch(0,address(k,p),address(k,q),PAIR_CAPTURE);
        end
        PAIR_CAPTURE:begin
            scalar[0]<=read_a;scalar[1]<=read_b;
            run_program(PAIR,read_bank?V_W1:OFF_W1);
        end
        OFF_W1:state<=OFF_W2;
        OFF_W2:state<=OFF_W3;
        OFF_W3:state<=OFF_W4;
        OFF_W4:begin k<=k+1'b1;state<=OFF_NEXT;end
        DIAG_W1:state<=DIAG_W2;
        DIAG_W2:state<=DIAG_W3;
        DIAG_W3:state<=DIAG_W4;
        DIAG_W4:begin k<=0;state<=V_NEXT;end
        V_NEXT:begin
            if(k==n)begin rsp_rotations<=rsp_rotations+1'b1;state<=LOOP_START;end
            else fetch(1,address(k,p),address(k,q),PAIR_CAPTURE);
        end
        V_W1:state<=V_W2;
        V_W2:begin k<=k+1'b1;state<=V_NEXT;end
        SORT_START:begin sort_pass<=0;sort_pos<=0;state<=SORT;end
        SORT:begin
            if(less(eigenvalue[order[sort_pos+1'b1]],eigenvalue[order[sort_pos]]))begin
                order[sort_pos]<=order[sort_pos+1'b1];order[sort_pos+1'b1]<=order[sort_pos];
            end
            if(sort_pos==n-2-sort_pass)begin
                sort_pos<=0;
                if(sort_pass==n-2)state<=OUTPUT_START;else sort_pass<=sort_pass+1'b1;
            end else sort_pos<=sort_pos+1'b1;
        end
        OUTPUT_START:begin
            rsp_min_value_fp64<=eigenvalue[order[0]];
            rsp_second_value_fp64<=eigenvalue[order[1]];
            rsp_max_value_fp64<=eigenvalue[order[n-1'b1]];
            k<=0;fetch(1,address(0,order[0]),address(0,order[0]),OUTPUT_CAPTURE);
        end
        OUTPUT_CAPTURE:begin
            rsp_min_vector_fp64[64*k +:64]<=read_a;
            if(k==n-1'b1)begin rsp_status<=`PAR_OK;state<=RESPONSE;end
            else begin
                k<=k+1'b1;fetch(1,address(k+1'b1,order[0]),address(k+1'b1,order[0]),OUTPUT_CAPTURE);
            end
        end
        MEM_READ:state<=MEM_WAIT;
        MEM_WAIT:state<=mem_return;
        PROGRAM:state<=FP_REQ;
        FP_REQ:if(fp_req_ready)state<=FP_WAIT;
        FP_WAIT:if(fp_rsp_valid)begin
            if(!finite(fp_result)||(|fp_flags[2:0]))fail(`PAR_CALIB_INVALID);
            else begin
                scalar[dest]<=fp_result;
                if((program_kind==BOUND) ||
                   ((program_kind==ANGLE || program_kind==PAIR)&&step==5) ||
                   (program_kind==DIAGONAL && step==12))
                    state<=program_return;
                else begin step<=step+1'b1;state<=PROGRAM;end
            end
        end
        RESPONSE:if(rsp_ready)state<=IDLE;
        default:fail(`PAR_BAD_CONFIG);
        endcase
    end
endmodule
