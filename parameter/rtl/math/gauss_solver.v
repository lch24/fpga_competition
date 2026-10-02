`include "calib_defs.vh"
/*
配置：统一见 rtl/common/calib_config.vh；下文具体计数例子以默认3张5×8为例，
实际容量、循环边界和端口位宽由PAR_*派生，修改配置后须重新编译/综合。
1..26元FP64高斯消元，部分主元选择，求解A*x=b。
输入按行发送n*(n+1)个元素，每行A[0..n-1]后接b；仅最后一个元素last=1。
非有限输入仍接收至本矩阵结束，再返回CALIB_INVALID；last不匹配返回BAD_CONFIG。
上游必须发送完整矩阵；本模块不对输入空拍设超时。早期last终止后上游须停止旧流。
内部使用固定27槽行跨度的702x64工作RAM（只复制输入，不向外写回），同步双读单写。
全部算术复用一个fp_operator(FP_W=64)，独立等待请求/结果握手。
按C++ matrix.cpp：最大绝对主元（相等取最早行）；行尺度仅统计col..n-1，不含b；
row_scale<1e-30或pivot<row_scale*1e-14失败；还检查abs(pivot)<1e-30。
abs(消元元素)<1e-30时跳过该行；交换从当前列到b；乘法和减法分别舍入，不融合。
中间NaN/Inf或invalid/divide_by_zero/overflow失败；underflow/inexact不直接判失败。
成功返回26槽解，未用高槽0；失败解全0。响应背压时保持全部字段。
复位取消当前矩阵和浮点事务，不清RAM内容；下一任务必须重新完整载入。
目前未完成目标器件综合/时序验证。
*/
module gauss_solver (
    input wire clk,
    input wire rst_n,
    input wire cmd_valid,
    output wire cmd_ready,
    input wire [`PAR_COL_BITS-1:0] cmd_n,
    input wire matrix_valid,
    output wire matrix_ready,
    input wire [63:0] matrix_fp64,
    input wire matrix_last,
    output wire rsp_valid,
    input wire rsp_ready,
    output reg [7:0] rsp_status,
    output reg [`PAR_SCALE_W-1:0] rsp_solution_fp64
);
    localparam [63:0] TINY=64'h39b4484bfeebc2a0;
    localparam [63:0] REL_TOL=64'h3d06849b86a12b9b;
    localparam IDLE=0, LOAD=1, COL_START=2, PIV_SCAN=3, SCALE_START=4, SCALE_SCAN=5,
        SCALE_CHECK=6, REL_CHECK=7, SWAP_FETCH=8, SWAP_WRITE_A=9, SWAP_WRITE_B=10,
        PIV_FETCH=11, PIV_CHECK=12, ROW_FETCH=13, FACTOR_CHECK=14, FACTOR_DONE=15,
        ELIM_FETCH=16, ELIM_MUL=17, ELIM_SUB=18, ELIM_WRITE=19, NEXT_ROW=20,
        BACK_FETCH=21, BACK_BEGIN=22, BACK_NEXT=23, BACK_MUL=24, BACK_SUB=25,
        BACK_ACC=26, BACK_STORE=27, MEM_READ=28, MEM_WAIT=29, FP_REQ=30,
        FP_WAIT=31, RESPONSE=32;
    reg [5:0] state, mem_return, fp_return;
    reg [`PAR_COL_BITS-1:0] n, load_row, load_col, col, row, entry_col, pivot_row, back_row;
    reg bad_input;
    reg [63:0] max_pivot,row_scale,pivot,factor,old_entry,accumulator,fp_value;
    reg [63:0] ram [0:`PAR_GAUSS_SIZE-1];
    reg [`PAR_GAUSS_ADDR_BITS-1:0] read_addr_a,read_addr_b;
    reg [63:0] read_a,read_b;
    reg write_enable;
    reg [`PAR_GAUSS_ADDR_BITS-1:0] write_addr;
    reg [63:0] write_value;
    reg [4:0] fp_op;
    reg [63:0] fp_a,fp_b;
    wire fp_req_ready,fp_rsp_valid;
    wire [63:0] fp_result;
    wire [4:0] fp_flags;

    function [`PAR_GAUSS_ADDR_BITS-1:0] address;
        input [`PAR_COL_BITS-1:0] r,c;
        begin address=r*(`PAR_ACTIVE_N+1)+c; end
    endfunction
    function finite;
        input [63:0] value;
        begin finite=(value[62:52]!=11'h7ff); end
    endfunction

    task fail;
        input [7:0] status;
        begin rsp_status<=status; rsp_solution_fp64<=0; state<=RESPONSE; end
    endtask
    task fetch;
        input [`PAR_GAUSS_ADDR_BITS-1:0] a,b;
        input [5:0] next_state;
        begin read_addr_a<=a; read_addr_b<=b; mem_return<=next_state; state<=MEM_READ; end
    endtask
    task calculate;
        input [4:0] operation;
        input [63:0] a,b;
        input [5:0] next_state;
        begin fp_op<=operation; fp_a<=a; fp_b<=b; fp_return<=next_state; state<=FP_REQ; end
    endtask

    assign cmd_ready=rst_n && (state==IDLE);
    assign matrix_ready=rst_n && (state==LOAD);
    assign rsp_valid=rst_n && (state==RESPONSE);
    fp_operator #(.FP_W(64)) arithmetic (
        .clk(clk),.rst_n(rst_n),.req_valid(rst_n && state==FP_REQ),.req_ready(fp_req_ready),
        .req_op(fp_op),.req_a(fp_a),.req_b(fp_b),.rsp_valid(fp_rsp_valid),
        .rsp_ready(rst_n && state==FP_WAIT),.rsp_result(fp_result),.rsp_flags(fp_flags),
        .rsp_less(),.rsp_equal(),.rsp_unordered()
    );

    // 独立RAM访问块，无数组复位；交换两行分两拍写，避免多写口。
    always @* begin
        write_enable=0; write_addr=0; write_value=0;
        if (rst_n) case(state)
        LOAD: if (matrix_valid && matrix_ready) begin
            write_enable=1; write_addr=address(load_row,load_col); write_value=matrix_fp64;
        end
        SWAP_WRITE_A: begin
            write_enable=1; write_addr=address(col,entry_col); write_value=read_b;
        end
        SWAP_WRITE_B: begin
            write_enable=1; write_addr=address(pivot_row,entry_col); write_value=read_a;
        end
        ELIM_WRITE: begin
            write_enable=1; write_addr=address(row,entry_col); write_value=fp_value;
        end
        default: begin end
        endcase
    end
    always @(posedge clk) begin
        if (write_enable) ram[write_addr]<=write_value;
        if (rst_n && state==MEM_READ) begin
            read_a<=ram[read_addr_a]; read_b<=ram[read_addr_b];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE;mem_return<=IDLE;fp_return<=IDLE;
            n<=0;load_row<=0;load_col<=0;col<=0;row<=0;entry_col<=0;pivot_row<=0;back_row<=0;
            bad_input<=0;max_pivot<=0;row_scale<=0;pivot<=0;factor<=0;old_entry<=0;accumulator<=0;
            fp_value<=0;read_addr_a<=0;read_addr_b<=0;fp_op<=0;fp_a<=0;fp_b<=0;
            rsp_status<=`PAR_OK;rsp_solution_fp64<=0;
        end else case(state)
        IDLE: if(cmd_valid && cmd_ready) begin
            rsp_solution_fp64<=0;rsp_status<=`PAR_OK;
            if(cmd_n==0 || cmd_n>`PAR_ACTIVE_N) fail(`PAR_BAD_CONFIG);
            else begin n<=cmd_n;load_row<=0;load_col<=0;bad_input<=0;state<=LOAD;end
        end
        LOAD: if(matrix_valid && matrix_ready) begin
            if(!finite(matrix_fp64)) bad_input<=1;
            if(matrix_last != ((load_row==n-1'b1)&&(load_col==n)))
                fail(`PAR_BAD_CONFIG);
            else if(load_row==n-1'b1 && load_col==n) begin
                if(bad_input || !finite(matrix_fp64)) fail(`PAR_CALIB_INVALID);
                else begin col<=0;state<=COL_START;end
            end else if(load_col==n) begin load_col<=0;load_row<=load_row+1'b1;end
            else load_col<=load_col+1'b1;
        end
        COL_START: begin
            if(col==n) begin back_row<=n-1'b1;state<=BACK_FETCH;end
            else begin
                row<=col;pivot_row<=col;max_pivot<=0;
                fetch(address(col,col),address(col,col),PIV_SCAN);
            end
        end
        PIV_SCAN: begin
            if({1'b0,read_a[62:0]}>max_pivot)begin max_pivot<={1'b0,read_a[62:0]};pivot_row<=row;end
            if(row==n-1'b1) state<=SCALE_START;
            else begin row<=row+1'b1;fetch(address(row+1'b1,col),address(row+1'b1,col),PIV_SCAN);end
        end
        SCALE_START: begin
            row_scale<=0;entry_col<=col;
            fetch(address(pivot_row,col),address(pivot_row,col),SCALE_SCAN);
        end
        SCALE_SCAN: begin
            if({1'b0,read_a[62:0]}>row_scale)row_scale<={1'b0,read_a[62:0]};
            if(entry_col==n-1'b1)state<=SCALE_CHECK;
            else begin
                entry_col<=entry_col+1'b1;
                fetch(address(pivot_row,entry_col+1'b1),address(pivot_row,entry_col+1'b1),SCALE_SCAN);
            end
        end
        SCALE_CHECK: begin
            if(row_scale<TINY)fail(`PAR_CALIB_INVALID);
            else calculate(`PAR_FP_MUL,row_scale,REL_TOL,REL_CHECK);
        end
        REL_CHECK: begin
            if(max_pivot<fp_value)fail(`PAR_CALIB_INVALID);
            else if(pivot_row!=col)begin entry_col<=col;state<=SWAP_FETCH;end
            else state<=PIV_FETCH;
        end
        SWAP_FETCH: fetch(address(col,entry_col),address(pivot_row,entry_col),SWAP_WRITE_A);
        SWAP_WRITE_A: state<=SWAP_WRITE_B;
        SWAP_WRITE_B: begin
            if(entry_col==n)state<=PIV_FETCH;
            else begin entry_col<=entry_col+1'b1;state<=SWAP_FETCH;end
        end
        PIV_FETCH: fetch(address(col,col),address(col,col),PIV_CHECK);
        PIV_CHECK: begin
            if(!finite(read_a) || {1'b0,read_a[62:0]}<TINY)fail(`PAR_CALIB_INVALID);
            else begin pivot<=read_a;row<=col+1'b1;state<=ROW_FETCH;end
        end
        ROW_FETCH: begin
            if(row==n)begin col<=col+1'b1;state<=COL_START;end
            else fetch(address(row,col),address(row,col),FACTOR_CHECK);
        end
        FACTOR_CHECK: begin
            if({1'b0,read_a[62:0]}<TINY)state<=NEXT_ROW;
            else calculate(`PAR_FP_DIV,read_a,pivot,FACTOR_DONE);
        end
        FACTOR_DONE: begin factor<=fp_value;entry_col<=col;state<=ELIM_FETCH;end
        ELIM_FETCH: fetch(address(row,entry_col),address(col,entry_col),ELIM_MUL);
        ELIM_MUL: begin old_entry<=read_a;calculate(`PAR_FP_MUL,factor,read_b,ELIM_SUB);end
        ELIM_SUB: calculate(`PAR_FP_SUB,old_entry,fp_value,ELIM_WRITE);
        ELIM_WRITE: begin
            if(entry_col==n)state<=NEXT_ROW;
            else begin entry_col<=entry_col+1'b1;state<=ELIM_FETCH;end
        end
        NEXT_ROW: begin row<=row+1'b1;state<=ROW_FETCH;end
        BACK_FETCH: fetch(address(back_row,back_row),address(back_row,n),BACK_BEGIN);
        BACK_BEGIN: begin
            if(!finite(read_a) || {1'b0,read_a[62:0]}<TINY)fail(`PAR_CALIB_INVALID);
            else begin
                pivot<=read_a;accumulator<=read_b;entry_col<=back_row+1'b1;state<=BACK_NEXT;
            end
        end
        BACK_NEXT: begin
            if(entry_col==n)calculate(`PAR_FP_DIV,accumulator,pivot,BACK_STORE);
            else fetch(address(back_row,entry_col),address(back_row,entry_col),BACK_MUL);
        end
        BACK_MUL: calculate(`PAR_FP_MUL,read_a,rsp_solution_fp64[64*entry_col +: 64],BACK_SUB);
        BACK_SUB: calculate(`PAR_FP_SUB,accumulator,fp_value,BACK_ACC);
        BACK_ACC: begin accumulator<=fp_value;entry_col<=entry_col+1'b1;state<=BACK_NEXT;end
        BACK_STORE: begin
            rsp_solution_fp64[64*back_row +: 64]<=fp_value;
            if(back_row==0)begin rsp_status<=`PAR_OK;state<=RESPONSE;end
            else begin back_row<=back_row-1'b1;state<=BACK_FETCH;end
        end
        MEM_READ: state<=MEM_WAIT;
        MEM_WAIT: state<=mem_return;
        FP_REQ: if(fp_req_ready)state<=FP_WAIT;
        FP_WAIT: if(fp_rsp_valid)begin
            if(!finite(fp_result) || (|fp_flags[2:0]))fail(`PAR_CALIB_INVALID);
            else begin fp_value<=fp_result;state<=fp_return;end
        end
        RESPONSE: if(rsp_ready)state<=IDLE;
        default: fail(`PAR_BAD_CONFIG);
        endcase
    end
endmodule
