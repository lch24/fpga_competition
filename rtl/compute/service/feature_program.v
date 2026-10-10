// ROM-driven detection scalar service. Tensor solving, FP32 coordinate update
// and FP64 convergence share ONE IEEE ALU. Pixel/window engines remain separate.
// One job at a time; start only when !busy. Inputs latch on acceptance; done is
// held until the next accepted start. CE freezes RAM/sequencer and a local ALU;
// with FP_SHARED=1 it gates this client's handshakes, not the external FP pool.
// Board entry: subpixel_ctrl.ts_start_p; program always begins at PC 0.
// Source: scripts/compute/microcode/build_feature.py (not calibration ISA).
// See docs/INSTRUCTION_CONTROL_GUIDE.md for the board sharing path.
// Instructions: [31:28] kind; FP: op[27:23],dst[22:19],a[18:14],b[13:9];
// OUT: a[18:14],slot[2:0]; BR: target[13:8],condition[2:0]; END: ok[0].
module feature_program #(parameter USE_CE=0,FP_SHARED=0)(
 output wire math_req_valid,input wire math_req_ready,output wire [4:0] math_req_op,
 output wire [63:0] math_req_a,math_req_b,output wire math_active,
 input wire math_rsp_valid,output wire math_rsp_ready,input wire [63:0] math_result,
 input wire [4:0] math_flags,
 input wire clk,rst_n,ce,start,refine,
 input wire [63:0] in_a,in_b,in_c,in_bx,in_by,
 input wire [31:0] in_x,in_y,
 output reg busy,done,out_ok,output reg [63:0] out_dx,out_dy,out_convergence,
 output reg [31:0] out_x,out_y
);
 localparam IDLE=0,LOAD=1,FETCH=2,READ=3,EXEC=4,ISSUE=5,WAIT_RESULT=6;
 reg [2:0] state,load_index;
 reg [5:0] pc;
 reg [31:0] instruction,program_rom[0:63];
 reg [63:0] registers[0:31],a,b;
 reg [447:0] inputs_q;
 reg refine_q,less,equal;
 wire enabled=!USE_CE || ce;
 wire [3:0] kind=instruction[31:28],destination=instruction[22:19];
 wire [4:0] operation=instruction[27:23],ra=instruction[18:14],rb=instruction[13:9];
 wire alu_ready,alu_valid,alu_less,alu_equal;
 wire [63:0] alu_result;
 integer i;
 initial begin
  for(i=0;i<32;i=i+1)registers[i]=0;
  registers[17]=64'h3ee4f8b588e368f1;registers[18]=64'h3e45798ee2308c3a;
  for(i=0;i<64;i=i+1)program_rom[i]=32'h30000000;
  `include "feature_program_init.vh"
 end
 wire ram_write=state==LOAD || (state==WAIT_RESULT && alu_valid && operation!=13);
 wire [4:0] ram_address=state==LOAD?{2'd0,load_index}:ram_write?{1'b0,destination}:ra;
 always @(posedge clk)if(rst_n && enabled)begin
  if(state==READ || ram_write)begin
   if(ram_write)registers[ram_address]<=state==LOAD?inputs_q[63:0]:alu_result;
   a<=registers[ram_address];
  end
  if(state==FETCH)instruction<=program_rom[pc];
  if(state==READ)b<=registers[rb];
 end
 generate if(FP_SHARED)begin : g_shared
  assign math_req_valid=rst_n && enabled && state==ISSUE;
  assign math_req_op=operation;assign math_req_a=a;assign math_req_b=b;assign math_active=rst_n;
  assign math_rsp_ready=rst_n && enabled && state==WAIT_RESULT;
  assign alu_ready=math_req_ready;assign alu_valid=math_rsp_valid;assign alu_result=math_result;
  wire na=(&a[62:52])&&(|a[51:0]),nb=(&b[62:52])&&(|b[51:0]);
  wire zz=!(|a[62:0]) && !(|b[62:0]);
  assign alu_equal=!(na||nb) && (a==b || zz);
  assign alu_less=!(na||nb) && !zz && ((a[63]!=b[63])?a[63]:(a[63]?(a>b):(a<b)));
 end else begin : g_local
 assign math_req_valid=0;assign math_req_op=0;assign math_req_a=0;assign math_req_b=0;
 assign math_active=0;assign math_rsp_ready=0;
 calib_alu #(.USE_CE(USE_CE)) arithmetic(.ce(ce),.clk(clk),.rst_n(rst_n),
  .req_valid(state==ISSUE),.req_ready(alu_ready),.req_op(operation),.req_a(a),.req_b(b),
  .rsp_valid(alu_valid),.rsp_ready(state==WAIT_RESULT),.rsp_result(alu_result),
  .rsp_flags(),.rsp_less(alu_less),.rsp_equal(alu_equal),.rsp_unordered());
 end endgenerate
 task advance;
  begin pc<=pc+1'b1;state<=FETCH;end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;load_index<=0;pc<=0;inputs_q<=0;refine_q<=0;less<=0;equal<=0;
   busy<=0;done<=0;out_ok<=0;out_dx<=0;out_dy<=0;out_x<=0;out_y<=0;out_convergence<=0;
  end else if(enabled)case(state)
   IDLE:if(start)begin
    inputs_q<={32'd0,in_y,32'd0,in_x,in_by,in_bx,in_c,in_b,in_a};
    refine_q<=refine;busy<=1;done<=0;load_index<=0;pc<=0;state<=LOAD;
   end
   LOAD:begin
    inputs_q<=inputs_q>>64;
    if(load_index==6)state<=FETCH;else load_index<=load_index+1'b1;
   end
   FETCH:state<=READ;
   READ:state<=EXEC;
   EXEC:case(kind)
    0:state<=ISSUE;
    1:begin
     case(instruction[2:0])
      0:out_dx<=a;1:out_dy<=a;2:out_x<=a[31:0];3:out_y<=a[31:0];4:out_convergence<=a;
     endcase
     advance();
    end
    2:begin
     if((instruction[2:0]==1 && less) || (instruction[2:0]==2 && (less||equal)) ||
        (instruction[2:0]==3 && !refine_q))begin pc<=instruction[13:8];state<=FETCH;end
     else advance();
    end
    default:begin busy<=0;done<=1;out_ok<=instruction[0];state<=IDLE;end
   endcase
   ISSUE:if(alu_ready)state<=WAIT_RESULT;
   WAIT_RESULT:if(alu_valid)begin less<=alu_less;equal<=alu_equal;advance();end
   default:state<=IDLE;
  endcase
 end
endmodule
