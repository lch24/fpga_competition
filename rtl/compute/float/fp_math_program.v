`include "calib_defs.vh"
// FP64 backend: one scalar IEEE ALU and a ROM-driven Q128 math machine.
// The 32-bit program owns a small two-read/one-write synchronous register RAM.
// There are no separate EXP/LOG/CORDIC state datapaths. Q128 operations retain
// the old operation order and accuracy; ALU requests block instruction fetch.
// External interface accepts one transaction, holds its result under backpressure,
// and reset cancels every in-flight operation. See scripts/compute/microcode.
// Board entry: fp_calibration_pool -> fp_operator(FP_W=64) -> this module.
// DECODE selects a generated MP_* entry from req_op; callers never send a PC.
// Source: scripts/compute/microcode/build_math.py (not calibration ISA).
// See docs/INSTRUCTION_CONTROL_GUIDE.md for nested execution and backpressure.
module fp_math_program #(
 parameter ENABLE_EXP=1,ENABLE_LOG=1,ENABLE_SINCOS=1,ENABLE_ATAN_ACOS=1
)(
 input wire clk,rst_n,req_valid,output wire req_ready,
 input wire [4:0] req_op,input wire [63:0] req_a,req_b,
 output reg rsp_valid,input wire rsp_ready,output reg [63:0] rsp_result,
 output reg [4:0] rsp_flags,output reg rsp_less,rsp_equal,rsp_unordered
);
 `include "fp_constants.vh"
 `include "math_program_defs.vh"
 localparam IDLE=0,DECODE=1,LOAD_A=2,LOAD_B=3,FETCH=4,READ=5,EXEC=6,WRITE=7,
  BASIC_REQ=8,BASIC_WAIT=9,FP_REQ=10,FP_WAIT=11,SHIFT=12,SHIFT_DONE=13,
  UNORM=14,MULTIPLY=15,MCARRY=16,MDONE=17,DIVIDE=18,DDONE=19,
  RANGE_LOAD=20,RANGE_MUL=21,RANGE_CARRY=22,RANGE_SHIFT=23,
  PACK_START=24,PACK_SHIFT=25,PACK_ROUND=26;
 reg [4:0] state;
 reg [4:0] operation;
 reg [63:0] saved_a,saved_b;
 reg [7:0] pc;
 reg [31:0] instruction,program_rom[0:255];
 reg [143:0] registers[0:255];
 reg signed [143:0] operand_a,operand_b,value_q;
 reg [3:0] destination;
 wire [5:0] opcode=instruction[31:26];
 wire [3:0] rd=instruction[25:22];
 wire [7:0] ra=instruction[21:14],rb=instruction[13:6];
 wire [5:0] mode=instruction[5:0];
 reg [7:0] iteration,iteration_limit,iteration_step;
 reg predicate,negative_x,negative_y;
 reg signed [15:0] scale;
 reg [4:0] last_flags;
 integer n;
 initial begin
  for(n=0;n<256;n=n+1)begin registers[n]=0;program_rom[n]=0;end
  registers[17]=FX_ONE;registers[18]=FX_LN2;registers[19]=FX_HALF_PI;
  registers[20]=FX_PI;registers[21]=FX_GAIN;registers[22]=FX_LN2>>>1;
  registers[23]=64'h3ff0000000000000;registers[24]=64'h4000000000000000;
  registers[25]=64'h4008000000000000;registers[26]=144'd32;
  for(n=0;n<128;n=n+1)registers[128+n]=cordic_angle(n);
  `include "math_program_init.vh"
 end
 // Keep all writes in one clocked process so PDS infers memory, not FF banks.
 wire ram_write=state==WRITE || state==LOAD_A || state==LOAD_B;
 wire [7:0] ram_address=state==WRITE?{4'd0,destination}:state==LOAD_A?8'd0:state==LOAD_B?8'd1:
                       opcode==MP_ANGLE?(8'd128+iteration):ra;
 wire [143:0] ram_data=state==LOAD_A?{80'd0,saved_a}:state==LOAD_B?{80'd0,saved_b}:value_q;
 always @(posedge clk)begin
  if(rst_n)begin
   if(state==FETCH)instruction<=program_rom[pc];
   if(state==READ || ram_write)begin
    if(ram_write)registers[ram_address]<=ram_data;
    operand_a<=registers[ram_address];
   end
   if(state==READ)operand_b<=registers[rb];
  end
 end
 wire subtract=(opcode==MP_SUB || opcode==MP_NEG ||
                 (opcode==MP_SUBC && predicate) || (opcode==MP_ADDC && !predicate));
 wire [143:0] adder_a=opcode==MP_NEG ? 144'd0:operand_a;
 wire [143:0] adder_b=opcode==MP_NEG ? operand_a:operand_b;
 wire [143:0] add_result=adder_a+(adder_b^{144{subtract}})+{{143{1'b0}},subtract};
 wire [143:0] abs_a=operand_a[143]?-operand_a:operand_a;
 wire [143:0] abs_b=operand_b[143]?-operand_b:operand_b;
 reg [7:0] shift_count;
 reg shift_left,shift_negative;
 reg [52:0] normal_mantissa;
 reg signed [15:0] normal_exponent;

 // One 64x32 multiplier shared by exact limb products and full-range 2/pi.
 reg [319:0] mul_acc;
 reg [159:0] mul_a,mul_b;
 reg [31:0] mul_carry;
 reg [2:0] mul_row,mul_column;
 reg mul_negative,mul_scaled;
 reg [63:0] range_mantissa,range_carry;
 reg [5:0] range_word,range_base;
 reg [4:0] range_shift;
 reg [1:0] quadrant;
 reg [191:0] range_window;
 reg [31:0] range_constant;
 reg [31:0] range_rom[0:39];
 initial for(n=0;n<40;n=n+1)range_rom[n]=TWO_OVER_PI>>(32*n);
 always @(posedge clk)if(state==RANGE_LOAD)range_constant<=range_rom[range_word];
 wire [63:0] multiply_a=state==MULTIPLY?{32'd0,mul_a[31:0]}:range_mantissa;
 wire [31:0] multiply_b=state==MULTIPLY?mul_b[31:0]:range_constant;
 wire [95:0] product=multiply_a*multiply_b;
 wire [95:0] range_total=product+{32'd0,range_carry};
 wire [63:0] word_sum=product[63:0]+{32'd0,mul_acc[31:0]}+{32'd0,mul_carry};
 wire [319:0] carry_acc={mul_acc[319:32],mul_carry};
 wire [319:0] ordered_product={mul_acc[159:0],mul_acc[319:160]};
 wire [319:0] signed_product=mul_negative?-ordered_product:ordered_product;
 integer lane;

 reg [271:0] quotient;
 reg [143:0] denominator,remainder_q;
 reg [8:0] divide_count;
 reg divide_negative,divide_small;
 wire [144:0] trial={remainder_q,quotient[271]};
 wire take=trial>={1'b0,denominator};
 wire [144:0] difference=trial-{1'b0,denominator};
 function [11:0] divide_four;
  input [7:0] rem;input [3:0] digit;input [6:0] divisor;
  reg [7:0] work;reg [3:0] q;integer k;
  begin
   work=rem;q=0;
   for(k=3;k>=0;k=k-1)begin
    work={work[6:0],digit[k]};
    if(work>={1'b0,divisor})begin work=work-{1'b0,divisor};q[k]=1;end
   end
   divide_four={work,q};
  end
 endfunction
 wire [11:0] four=divide_four(remainder_q[7:0],quotient[271:268],denominator[6:0]);

 reg [143:0] pack_magnitude;
 reg pack_sign;
 reg signed [15:0] pack_exponent;
 wire [53:0] rounded={1'b0,pack_magnitude[55:3]}+
  {{53{1'b0}},pack_magnitude[2]&&(pack_magnitude[1]||pack_magnitude[0]||pack_magnitude[3])};
 wire [53:0] packed_mantissa=rounded[53]?(rounded>>1):rounded;
 wire signed [15:0] final_exponent=pack_exponent+(rounded[53]?16'sd1:16'sd0);
 wire [15:0] biased_exponent=final_exponent+16'sd1023;
 wire normal=packed_mantissa[52];
 wire tiny=!normal && (|pack_magnitude[2:0] || operation==`PAR_FP_EXP);

 wire alu_ready,alu_valid;
 wire [63:0] alu_result;
 wire [4:0] alu_flags;
 wire alu_less,alu_equal,alu_unordered;
 calib_alu alu(.ce(1'b1),.clk(clk),.rst_n(rst_n),
  .req_valid(state==BASIC_REQ || state==FP_REQ),.req_ready(alu_ready),
  .req_op(state==BASIC_REQ?operation:mode[4:0]),
  .req_a(state==BASIC_REQ?saved_a:operand_a[63:0]),
  .req_b(state==BASIC_REQ?saved_b:operand_b[63:0]),
  .rsp_valid(alu_valid),.rsp_ready(state==BASIC_WAIT || state==FP_WAIT),
  .rsp_result(alu_result),.rsp_flags(alu_flags),.rsp_less(alu_less),
  .rsp_equal(alu_equal),.rsp_unordered(alu_unordered));
 assign req_ready=rst_n && state==IDLE && !rsp_valid;
 wire nan_a=(&saved_a[62:52]) && |saved_a[51:0];
 wire nan_b=(&saved_b[62:52]) && |saved_b[51:0];
 wire inf_a=saved_a[62:0]==63'h7ff0000000000000;
 wire zero_a=saved_a[62:0]==0;
 wire disabled=(!ENABLE_EXP && operation==`PAR_FP_EXP)||(!ENABLE_LOG && operation==`PAR_FP_LOG)||
  (!ENABLE_SINCOS && (operation==`PAR_FP_SIN || operation==`PAR_FP_COS))||
  (!ENABLE_ATAN_ACOS && (operation==`PAR_FP_ATAN2 || operation==`PAR_FP_ACOS));
 reg [63:0] atan_a,atan_b;
 reg [143:0] special_angle,selected_value;
 reg selected_negative;
 integer distance,exponent;
 task finish;
  input [63:0] result;input [4:0] flags;
  begin rsp_valid<=1;rsp_result<=result;rsp_flags<=flags;state<=IDLE;end
 endtask
 task advance;
  begin pc<=pc+1'b1;state<=FETCH;end
 endtask
 task write_result;
  input [143:0] result;
  begin value_q<=result;destination<=rd;state<=WRITE;end
 endtask
 task pack_angle;
  input [143:0] angle;
  begin value_q<=angle;scale<=0;state<=PACK_START;end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;rsp_valid<=0;rsp_result<=0;rsp_flags<=0;
   rsp_less<=0;rsp_equal<=0;rsp_unordered<=0;operation<=0;saved_a<=0;saved_b<=0;
   pc<=0;destination<=0;value_q<=0;iteration<=0;iteration_limit<=0;iteration_step<=0;
   predicate<=0;negative_x<=0;negative_y<=0;scale<=0;last_flags<=0;
   shift_count<=0;shift_left<=0;shift_negative<=0;normal_mantissa<=0;normal_exponent<=0;
   mul_acc<=0;mul_a<=0;mul_b<=0;mul_carry<=0;mul_row<=0;mul_column<=0;mul_negative<=0;mul_scaled<=0;
   range_mantissa<=0;range_carry<=0;range_word<=0;range_base<=0;range_shift<=0;quadrant<=0;range_window<=0;
   quotient<=0;denominator<=0;remainder_q<=0;divide_count<=0;divide_negative<=0;divide_small<=0;
   pack_magnitude<=0;pack_sign<=0;pack_exponent<=0;
  end else begin
   if(rsp_valid && rsp_ready)rsp_valid<=0;
   case(state)
    IDLE:if(req_valid && req_ready)begin
     saved_a<=req_a;saved_b<=req_b;operation<=req_op;state<=DECODE;
     rsp_less<=0;rsp_equal<=0;rsp_unordered<=0;last_flags<=0;scale<=0;
    end
    DECODE:begin
     state<=LOAD_A;
     if(disabled)finish(64'h7ff8000000000000,1);
     else if(operation<=4 || operation>=11)state<=BASIC_REQ;
     else if(nan_a || (operation==7 && nan_b))
      finish(64'h7ff8000000000000,{4'd0,(nan_a&&!saved_a[51])||(operation==7&&nan_b&&!saved_b[51])});
     else case(operation)
      `PAR_FP_EXP:begin
       pc<=MP_EXP;
       if(inf_a)finish(saved_a[63]?64'd0:64'h7ff0000000000000,0);
       else if(zero_a)finish(64'h3ff0000000000000,0);
       else if(saved_a[62:52]>1034)finish(saved_a[63]?64'd0:64'h7ff0000000000000,saved_a[63]?5'b11000:5'b10100);
      end
      `PAR_FP_LOG:begin
       pc<=MP_LOG;
       if(zero_a)finish(64'hfff0000000000000,2);
       else if(saved_a[63])finish(64'h7ff8000000000000,1);
       else if(inf_a)finish(64'h7ff0000000000000,0);
       else if(saved_a==64'h3ff0000000000000)finish(0,0);
       else if(saved_a>64'h3ff0000000000000 ?
          saved_a-64'h3ff0000000000000<64'h100000000 :
          64'h3ff0000000000000-saved_a<64'h100000000)pc<=MP_LOG_NEAR;
      end
      `PAR_FP_SIN,`PAR_FP_COS:begin
       pc<=MP_TRIG;
       if(inf_a)finish(64'h7ff8000000000000,1);
       else if(zero_a)finish(operation==5?saved_a:64'h3ff0000000000000,0);
       else if(saved_a[62:52]<993)finish(operation==5?saved_a:64'h3ff0000000000000,
         (operation==5 && saved_a[62:52]==0)?5'b11000:5'b10000);
      end
      `PAR_FP_ATAN2:pc<=MP_ATAN;
      `PAR_FP_ACOS:begin
       pc<=MP_ACOS;
       if(saved_a[62:0]>63'h3ff0000000000000)finish(64'h7ff8000000000000,1);
       else if(saved_a==64'h3ff0000000000000)finish(0,0);
       else if(saved_a==64'hbff0000000000000)pack_angle(FX_PI);
      end
      default:finish(64'h7ff8000000000000,1);
     endcase
    end
    LOAD_A:state<=LOAD_B;
    LOAD_B:state<=FETCH;
    FETCH:state<=READ;
    READ:state<=EXEC;
    WRITE:advance();
    EXEC:case(opcode)
     MP_RET:if(mode==1)begin value_q<=operand_a;state<=PACK_START;end
       else finish(operand_a[63:0],last_flags|5'b10000|
        ((mode==2 && operand_a[62:52]==0)?5'b01000:5'd0));
     MP_MOV,MP_ANGLE:write_result(operand_a);
     MP_ADD,MP_SUB,MP_NEG,MP_ADDC,MP_SUBC:write_result(add_result);
     MP_SHL:write_result(operand_a<<1);
     MP_SAR:begin
      destination<=rd;value_q<=operand_a;shift_left<=0;shift_negative<=0;
      shift_count<=mode==1?iteration:(operand_b>=144?8'd144:operand_b[7:0]);state<=SHIFT;
     end
     MP_FROMFP:begin
      destination<=rd;value_q<={91'd0,|operand_a[62:52],operand_a[51:0]};
      exponent=(operand_a[62:52]==0?-1022:operand_a[62:52]-1023)+76;
      shift_left<=exponent>=0;shift_negative<=operand_a[63];
      distance=exponent>=0?exponent:-exponent;
      shift_count<=distance>=144?8'd144:distance;state<=SHIFT;
     end
     MP_UNPACK:begin
      destination<=rd;normal_mantissa<={|operand_a[62:52],operand_a[51:0]};
      normal_exponent<=operand_a[62:52]==0?-16'sd1022:$signed({5'd0,operand_a[62:52]})-16'sd1023;
      state<=UNORM;
     end
     MP_SETSCALE:begin scale<=operand_a[15:0];advance();end
     MP_GETSCALE:write_result({{128{scale[15]}},scale});
     MP_SETI:begin iteration<=instruction[7:0];iteration_limit<=instruction[15:8];iteration_step<={2'd0,instruction[21:16]};advance();end
     MP_GETI:write_result({136'd0,iteration});
     MP_LOOP:begin
      if(iteration<iteration_limit)begin iteration<=iteration+iteration_step;pc<=instruction[7:0];state<=FETCH;end
      else advance();
     end
     MP_BR:begin
      if(rd==0 || (rd==1 && operand_a[143]) || (rd==2 && operand_a==0) ||
         (rd==7 && negative_x) || (rd==8 && !negative_x) || (rd==9 && !negative_y))begin pc<=instruction[7:0];state<=FETCH;end
      else advance();
     end
     MP_TEST:begin predicate<=mode==0?!operand_a[143]:(!operand_a[143] && operand_a!=0);advance();end
     MP_FP:begin destination<=rd;state<=FP_REQ;end
     MP_MULQ,MP_MULI:begin
      destination<=rd;mul_a<={16'd0,abs_a};mul_b<={16'd0,abs_b};
      mul_acc<=0;mul_carry<=0;mul_row<=0;mul_column<=0;
      mul_negative<=operand_a[143]^operand_b[143];mul_scaled<=opcode==MP_MULQ;state<=MULTIPLY;
     end
     MP_DIVQ,MP_DIVI:begin
      destination<=rd;quotient<=opcode==MP_DIVQ?{abs_a,128'd0}:{128'd0,abs_a};
      denominator<=abs_b;remainder_q<=0;divide_negative<=operand_a[143]^operand_b[143];
      divide_small<=abs_b[143:7]==0 && abs_b[6:0]!=0;
      divide_count<=(abs_b[143:7]==0 && abs_b[6:0]!=0)?67:271;state<=DIVIDE;
     end
     MP_RANGE:begin
      destination<=rd;range_mantissa<={11'd0,1'b1,operand_a[51:0]};
      distance=2227-operand_a[62:52];range_base<=distance[10:5];range_shift<=distance[4:0];
      range_carry<=0;range_word<=0;range_window<=0;state<=RANGE_LOAD;
     end
     MP_SELECT:begin
      if(operation==5)begin
       selected_value=quadrant[0]?operand_a:operand_b;
       selected_negative=quadrant[1]^saved_a[63];
      end else begin
       selected_value=quadrant[0]?operand_b:operand_a;
       selected_negative=quadrant[0]^quadrant[1];
      end
      write_result(selected_negative?-selected_value:selected_value);
     end
     MP_ATEST:begin
      atan_a=operand_a[63:0];atan_b=operand_b[63:0];
      negative_y<=atan_a[63];negative_x<=atan_b[63];
      if(atan_a[62:0]==0 || atan_b[62:0]==0 || atan_a[62:0]==63'h7ff0000000000000 || atan_b[62:0]==63'h7ff0000000000000)begin
       special_angle=0;
       if(atan_a[62:0]==63'h7ff0000000000000 && atan_b[62:0]==63'h7ff0000000000000)
        special_angle=atan_b[63]?(FX_PI-(FX_PI>>>2)):(FX_PI>>>2);
       else if(atan_a[62:0]==63'h7ff0000000000000 || (atan_b[62:0]==0 && atan_a[62:0]!=0))special_angle=FX_HALF_PI;
       else if(atan_b[63])special_angle=FX_PI;
       if(special_angle==0)finish({atan_a[63],63'd0},0);
       else pack_angle(atan_a[63]?-special_angle:special_angle);
      end else advance();
     end
     default:finish(64'h7ff8000000000000,1);
    endcase
    SHIFT:begin
     if(shift_count>=8)begin value_q<=shift_left?(value_q<<<8):(value_q>>>8);shift_count<=shift_count-8;end
     else if(shift_count!=0)begin value_q<=shift_left?(value_q<<<1):(value_q>>>1);shift_count<=shift_count-1'b1;end
     else state<=SHIFT_DONE;
    end
    SHIFT_DONE:begin if(shift_negative)value_q<=-value_q;state<=WRITE;end
    UNORM:if(normal_mantissa!=0 && !normal_mantissa[52])begin normal_mantissa<=normal_mantissa<<1;normal_exponent<=normal_exponent-1'b1;end
      else begin value_q<={15'd0,normal_mantissa,76'd0};scale<=normal_exponent;state<=WRITE;end
    MULTIPLY:begin
     mul_acc<={word_sum[31:0],mul_acc[319:32]};mul_carry<=word_sum[63:32];
     mul_a<={mul_a[31:0],mul_a[159:32]};
     if(mul_column==4)state<=MCARRY;else mul_column<=mul_column+1'b1;
    end
    MCARRY:begin
     mul_acc<={carry_acc[191:0],carry_acc[319:192]};mul_carry<=0;mul_column<=0;mul_b<=mul_b>>32;
     if(mul_row==4)state<=MDONE;else begin mul_row<=mul_row+1'b1;state<=MULTIPLY;end
    end
    MDONE:begin value_q<=mul_scaled?signed_product[271:128]:signed_product[143:0];state<=WRITE;end
    DIVIDE:begin
     if(divide_small)begin remainder_q<={136'd0,four[11:4]};quotient<={quotient[267:0],four[3:0]};end
     else begin remainder_q<=take?difference[143:0]:trial[143:0];quotient<={quotient[270:0],take};end
     if(divide_count==0)state<=DDONE;else divide_count<=divide_count-1'b1;
    end
    DDONE:begin value_q<=divide_negative?-quotient[143:0]:quotient[143:0];state<=WRITE;end
    RANGE_LOAD:state<=RANGE_MUL;
    RANGE_MUL:begin
     for(lane=0;lane<6;lane=lane+1)if(range_word==range_base+lane)range_window[32*lane+:32]<=range_total[31:0];
     range_carry<=range_total[95:32];range_word<=range_word+1'b1;
     state<=range_word==39?RANGE_CARRY:RANGE_LOAD;
    end
    RANGE_CARRY:begin
     for(lane=0;lane<6;lane=lane+1)if(range_word==range_base+lane)range_window[32*lane+:32]<=range_carry[31:0];
     range_carry<=range_carry>>32;range_word<=range_word+1'b1;
     if(range_word==41)state<=RANGE_SHIFT;
    end
    RANGE_SHIFT:begin
     if(range_shift>=8)begin range_window<=range_window>>8;range_shift<=range_shift-8;end
     else if(range_shift!=0)begin range_window<=range_window>>1;range_shift<=range_shift-1'b1;end
     else begin
      value_q<={{16{range_window[127]}},range_window[127:0]};
      quadrant<=range_window[129:128]+{1'b0,range_window[127]};state<=WRITE;
     end
    end
    PACK_START:begin pack_magnitude<=value_q[143]?-value_q:value_q;pack_sign<=value_q[143];pack_exponent<=scale-16'sd73;state<=PACK_SHIFT;end
    PACK_SHIFT:begin
     if(pack_magnitude==0)state<=PACK_ROUND;
     else if(pack_exponent< -16'sd1166)begin pack_magnitude<=1;pack_exponent<=-16'sd1022;state<=PACK_ROUND;end
     else if(pack_exponent< -16'sd1022 || |pack_magnitude[143:56])begin
      pack_magnitude<={1'b0,pack_magnitude[143:2],pack_magnitude[1]|pack_magnitude[0]};pack_exponent<=pack_exponent+1'b1;
     end else if(!pack_magnitude[55] && pack_exponent> -16'sd1022)begin pack_magnitude<=pack_magnitude<<1;pack_exponent<=pack_exponent-1'b1;end
     else state<=PACK_ROUND;
    end
    PACK_ROUND:if(final_exponent>1023)finish({pack_sign,11'h7ff,52'd0},5'b10100);
      else finish({pack_sign,(normal?biased_exponent[10:0]:11'd0),packed_mantissa[51:0]},{1'b1,tiny,3'd0});
    BASIC_REQ:if(alu_ready)state<=BASIC_WAIT;
    BASIC_WAIT:if(alu_valid)begin
     rsp_less<=alu_less;rsp_equal<=alu_equal;rsp_unordered<=alu_unordered;finish(alu_result,alu_flags);
    end
    FP_REQ:if(alu_ready)state<=FP_WAIT;
    FP_WAIT:if(alu_valid)begin value_q<={80'd0,alu_result};last_flags<=alu_flags;state<=WRITE;end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
