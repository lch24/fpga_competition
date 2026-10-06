`include "calib_defs.vh"
// 几何指令执行器：投影（含 Brown）或独立 Brown，编译时选择程序。
// 两者使用相同执行逻辑，投影内不再实例化独立 Brown 控制器/工作寄存器。
// 每条指令分拍读 A/B，单次提交共享浮点池，返回后写回同一 RAM。
// PROJECTION=1 仅用于 FP64；Brown 可用 FP32/64。逐条 IEEE 舍入，不融合乘加。
// 命令的并行载荷先锁存，随后移位写 RAM；调用者握手后可立即改变输入。
// RAM 不复位，每项读数据均由本次载荷或前序指令产生。
// 地址：投影输入 0..22，临时/结果 30..49；Brown 输入 0..6，临时 9..16，
// 结果 30..31；常数 60=1、61=2。指令表见 geometry_program.vh。
module geometry_engine #(
    parameter FP_W=64, FP_SHARED=0, PROJECTION=1,
    parameter INPUT_WORDS=PROJECTION?23:7
)(
    input wire clk,rst_n,cmd_valid, output wire cmd_ready,
    input wire [INPUT_WORDS*FP_W-1:0] cmd_payload,
    output wire rsp_valid, input wire rsp_ready,
    output reg [7:0] rsp_status,
    output wire [FP_W-1:0] rsp_x,rsp_y,
    output wire shared_req_valid, input wire shared_req_ready,
    output wire [4:0] shared_req_op,
    output wire [63:0] shared_req_a,shared_req_b,
    output wire shared_active,
    input wire shared_rsp_valid, output wire shared_rsp_ready,
    input wire [63:0] shared_rsp_result,
    input wire [4:0] shared_rsp_flags
);
    localparam IDLE=0,LOAD=1,FETCH=2,READ_A=3,TAKE_A=4,TAKE_B=5,
               REQUEST=6,WAIT_RESULT=7,RESPONSE=8;
    localparam [FP_W-1:0] ONE=FP_W==32?32'h3f800000:64'h3ff0000000000000;
    localparam [FP_W-1:0] TWO=FP_W==32?32'h40000000:64'h4000000000000000;
    localparam LAST_INSTRUCTION=PROJECTION?50:31;
    localparam OUT_X=PROJECTION?40:30, OUT_Y=PROJECTION?41:31;
    reg [3:0] state;
    reg [5:0] pc,load_index;
    reg [22:0] instruction;
    reg [INPUT_WORDS*FP_W-1:0] payload;
    reg [FP_W-1:0] operand_a,operand_b,result_x,result_y;
    wire [4:0] operation=instruction[22:18];
    wire [5:0] source_a=instruction[17:12],source_b=instruction[11:6],destination=instruction[5:0];
    wire fp_ready,fp_valid;
    wire [FP_W-1:0] fp_result;
    wire [4:0] fp_flags;
    function finite;
        input [FP_W-1:0] x;
        begin finite=FP_W==32?((x>>23)&255)!=255:((x>>52)&2047)!=2047; end
    endfunction
    `include "geometry_program.vh"
    wire fp_ok=!(|fp_flags[2:0]) && finite(fp_result);
    wire loading=state==LOAD;
    wire writeback=state==WAIT_RESULT && fp_valid && fp_ok;
    wire [5:0] load_address=load_index<INPUT_WORDS?load_index:
                                    load_index==INPUT_WORDS?6'd60:6'd61;
    wire [FP_W-1:0] load_data=load_index<INPUT_WORDS?payload[0 +: FP_W]:
                                    load_index==INPUT_WORDS?ONE:TWO;
    wire [FP_W-1:0] read_data;
    work_ram #(.WIDTH(FP_W),.ADDR_BITS(6)) scratch(
        .clk(clk),.wr_en(rst_n && (loading || writeback)),
        .wr_addr(loading?load_address:destination),.wr_data(loading?load_data:fp_result),
        .rd_en(rst_n && (state==READ_A || state==TAKE_A)),
        .rd_addr(state==READ_A?source_a:source_b),.rd_data(read_data));
    assign cmd_ready=rst_n && state==IDLE;
    assign rsp_valid=rst_n && state==RESPONSE;
    assign rsp_x=rsp_status==0?result_x:{FP_W{1'b0}};
    assign rsp_y=rsp_status==0?result_y:{FP_W{1'b0}};
    generate if(FP_SHARED) begin:g_shared
        assign shared_req_valid=rst_n && state==REQUEST;
        assign shared_req_op=operation;
        assign shared_req_a=operand_a;
        assign shared_req_b=operand_b;
        assign shared_active=rst_n;
        assign shared_rsp_ready=rst_n && state==WAIT_RESULT;
        assign fp_ready=shared_req_ready;
        assign fp_valid=shared_rsp_valid;
        assign fp_result=shared_rsp_result[0 +: FP_W];
        assign fp_flags=shared_rsp_flags;
    end else begin:g_local
        assign shared_req_valid=0;assign shared_req_op=0;
        assign shared_req_a=0;assign shared_req_b=0;
        assign shared_active=0;assign shared_rsp_ready=0;
        fp_operator #(.FP_W(FP_W),.ENABLE_EXP(0),.ENABLE_LOG(0),
                      .ENABLE_SINCOS(0),.ENABLE_ATAN_ACOS(0)) arithmetic(
            .clk(clk),.rst_n(rst_n),.req_valid(rst_n && state==REQUEST),.req_ready(fp_ready),
            .req_op(operation),.req_a(operand_a),.req_b(operand_b),
            .rsp_valid(fp_valid),.rsp_ready(rst_n && state==WAIT_RESULT),
            .rsp_result(fp_result),.rsp_flags(fp_flags),
            .rsp_less(),.rsp_equal(),.rsp_unordered());
    end endgenerate
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;pc<=0;load_index<=0;rsp_status<=0;
            result_x<=0;result_y<=0;
        end else case(state)
            IDLE: if(cmd_valid) begin
                payload<=cmd_payload;load_index<=0;pc<=0;
                rsp_status<=0;result_x<=0;result_y<=0;state<=LOAD;
            end
            LOAD: begin
                payload<=payload>>FP_W;
                if(!finite(load_data)) begin rsp_status<=`PAR_CALIB_INVALID;state<=RESPONSE;end
                else if(load_index==INPUT_WORDS+1) state<=FETCH;
                else load_index<=load_index+1'b1;
            end
            FETCH: begin instruction<=program_word(pc);state<=READ_A;end
            READ_A: state<=TAKE_A;
            TAKE_A: begin operand_a<=read_data;state<=TAKE_B;end
            TAKE_B: begin
                operand_b<=read_data;
                if(operation==31) begin
                    // Original project_point check: Z must be finite and > 1e-5.
                    if(operand_a[FP_W-1] || operand_a<=64'h3ee4f8b588e368f1) begin
                        rsp_status<=`PAR_CALIB_INVALID;state<=RESPONSE;
                    end else begin pc<=pc+1'b1;state<=FETCH;end
                end else state<=REQUEST;
            end
            REQUEST: if(fp_ready) state<=WAIT_RESULT;
            WAIT_RESULT: if(fp_valid) begin
                if(!fp_ok) begin rsp_status<=`PAR_CALIB_INVALID;state<=RESPONSE;end
                else begin
                    if(destination==OUT_X) result_x<=fp_result;
                    if(destination==OUT_Y) result_y<=fp_result;
                    if(pc==LAST_INSTRUCTION) state<=RESPONSE;
                    else begin pc<=pc+1'b1;state<=FETCH;end
                end
            end
            RESPONSE: if(rsp_ready) state<=IDLE;
            default: begin rsp_status<=`PAR_CALIB_INVALID;state<=RESPONSE;end
        endcase
    end
endmodule
