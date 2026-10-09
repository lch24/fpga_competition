`timescale 1ns/1ps
module grid_validate #(parameter USE_CE=0,
    parameter ROWS     = 5,
    parameter COLS     = 8,
    parameter N_ADDR_W = 6
) (
    // Global synchronous stall for variable-latency backing memory.
    input wire ce,

    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    output reg                busy,
    output reg                done,
    input  wire [15:0]        n_in,
    output reg                rd_en,
    output reg  [N_ADDR_W-1:0] rd_addr,
    input  wire [31:0]        rd_x,
    input  wire [31:0]        rd_y,
    output reg                valid_out,
    output reg  [31:0]        cost_out
);
    // A single serial FP32 operator evaluates squared gates and rational cost.
    // No hypot/sqrt/log datapath; each request consumes registered operands.
    // The operator runs independently of DDR ce; request/response handshakes
    // are gated by ce, so a completed operation waits safely through DDR stalls.
    localparam IDLE=0,SELECT=1,READ=2,LATCH=3,NEXT=4,FINISH=5,FAIL=6,
               FP_REQ=60,FP_WAIT=61;
    localparam ADD=0,SUB=1,MUL=2,DIV=3;
    localparam [31:0] ONE=32'h3f800000,INVALID=32'h7149f2ca;
    reg [5:0] state,continuation;
    reg [3:0] destination;
    reg [4:0] op;
    reg [31:0] operand_a,operand_b,fp_value;
    reg [31:0] v[0:15],px[0:2],py[0:2],cost;
    reg [1:0] mode,corner,point;
    integer row,col,index;
    reg sign_valid,sign_bit;
    wire enabled=!USE_CE || ce;
    wire fp_ready,fp_valid;wire [31:0] fp_result;wire [4:0] fp_flags;
    wire any_request=state==FP_REQ && enabled && fp_ready;
    wire stage_done=state==FP_WAIT && enabled && fp_valid;
    fp_operator #(.FP_W(32),.ENABLE_EXP(0),.ENABLE_LOG(0),
        .ENABLE_SINCOS(0),.ENABLE_ATAN_ACOS(0)) arithmetic(
        .clk(clk),.rst_n(rst_n),.req_valid(state==FP_REQ && enabled),
        .req_ready(fp_ready),.req_op(op),.req_a(operand_a),.req_b(operand_b),
        .rsp_valid(fp_valid),.rsp_ready(state==FP_WAIT && enabled),
        .rsp_result(fp_result),.rsp_flags(fp_flags),
        .rsp_less(),.rsp_equal(),.rsp_unordered());
    task calc(input [4:0] operation,input [31:0] a,b,input [3:0] target,input [5:0] next_state);
        begin op<=operation;operand_a<=a;operand_b<=b;
            destination<=target;continuation<=next_state;state<=FP_REQ;end
    endtask
    function integer cell_address(input [1:0] k);
        begin case(k)
          0:cell_address=index;1:cell_address=index+1;
          2:cell_address=index+COLS+1;3:cell_address=index+COLS;
        endcase end
    endfunction
    always @* begin
        rd_en=rst_n && state==READ;
        if(mode==2)rd_addr=cell_address(corner+point);
        else rd_addr=index+point*(mode==0?1:COLS);
    end
    always @(posedge clk) begin
        if(!rst_n)begin state<=IDLE;busy<=0;done<=0;valid_out<=0;cost_out<=0;end
        else if(enabled)case(state)
            IDLE:if(start)begin
                done<=0;valid_out<=0;busy<=1;cost<=0;mode<=0;corner<=0;
                row<=0;col<=0;index<=0;sign_valid<=0;sign_bit<=0;
                if(n_in!=ROWS*COLS || ROWS<2 || COLS<2)state<=FAIL;else state<=SELECT;
            end
            SELECT:begin
                point<=0;
                if((mode==0 && col+2>=COLS)||(mode==1 && row+2>=ROWS))begin mode<=mode+1'b1;end
                else if(mode==2 && (col+1>=COLS || row+1>=ROWS))state<=NEXT;
                else state<=READ;
            end
            READ:state<=LATCH;
            LATCH:begin px[point]<=rd_x;py[point]<=rd_y;
                if(rd_x[30:23]==255 || rd_y[30:23]==255)state<=FAIL;
                else if(point==2)state<=10;
                else begin point<=point+1'b1;state<=READ;end
            end
            10:calc(SUB,px[1],px[0],0,11);
            11:calc(SUB,py[1],py[0],1,12);
            12:calc(SUB,px[2],px[1],2,13);
            13:calc(SUB,py[2],py[1],3,14);
            14:calc(MUL,v[0],v[0],4,15);
            15:calc(MUL,v[1],v[1],5,16);
            16:calc(ADD,v[4],v[5],4,17);
            17:calc(MUL,v[2],v[2],6,18);
            18:calc(MUL,v[3],v[3],7,19);
            19:calc(ADD,v[6],v[7],6,20);
            20:calc(MUL,v[4],v[6],8,21);
            21:if(mode!=2 && (v[4]<32'h41800000 || v[6]<32'h41800000))state<=FAIL;
                else if(mode==2 && v[8]<32'h43800000)state<=FAIL;
                else state<=22;
            22:calc(MUL,v[0],mode==2?v[3]:v[2],9,23);
            23:calc(MUL,v[1],mode==2?v[2]:v[3],10,24);
            24:calc(mode==2?SUB:ADD,v[9],v[10],9,25);
            25:calc(MUL,v[9],v[9],11,26);
            26:calc(MUL,v[8],mode==2?32'h3d23d70a:32'h3f4f5c29,12,27);
            27:if(v[11]<v[12] || (mode!=2 && v[9][31]))state<=FAIL;
                else if(mode==2)begin
                    if(v[9][30:0]==0 || (sign_valid && sign_bit!=v[9][31]))state<=FAIL;
                    else begin sign_valid<=1;sign_bit<=v[9][31];
                        if(corner==3)state<=NEXT;else begin corner<=corner+1'b1;state<=SELECT;end
                    end
                end else state<=28;
            28:calc(MUL,v[4],32'h3e9ae148,12,29);
            29:if(v[6]<v[12])state<=FAIL;else state<=30;
            30:calc(MUL,v[4],32'h404f5c29,12,31);
            31:if(v[6]>v[12])state<=FAIL;else state<=32;
            32:calc(DIV,v[11],v[8],13,33);
            33:calc(SUB,ONE,v[13],13,34);
            34:calc(SUB,v[6],v[4],14,35);
            35:calc(MUL,v[14],v[14],14,36);
            36:calc(MUL,v[14],32'h40800000,14,37);
            37:calc(ADD,v[4],v[6],15,38);
            38:calc(MUL,v[15],v[15],15,39);
            39:calc(DIV,v[14],v[15],14,40);
            40:calc(ADD,v[13],v[14],13,41);
            41:calc(ADD,cost,v[13],13,42);
            42:begin cost<=v[13];mode<=mode+1'b1;state<=SELECT;end
            NEXT:begin
                mode<=0;corner<=0;index<=index+1;
                if(index==ROWS*COLS-1)state<=FINISH;
                else begin if(col==COLS-1)begin col<=0;row<=row+1;end else col<=col+1;state<=SELECT;end
            end
            FP_REQ:if(fp_ready)state<=FP_WAIT;
            FP_WAIT:if(fp_valid)begin
                if((|fp_flags[2:0]) || fp_result[30:23]==255)state<=FAIL;
                else begin v[destination]<=fp_result;state<=continuation;end
            end
            FINISH:begin cost_out<=cost;busy<=0;done<=1;valid_out<=1;state<=IDLE;end
            FAIL:begin cost_out<=INVALID;busy<=0;done<=1;valid_out<=1;state<=IDLE;end
            default:state<=FAIL;
        endcase
    end
endmodule
