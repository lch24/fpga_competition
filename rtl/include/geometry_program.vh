// 23-bit instruction: {opcode[4:0], source A[5:0], source B[5:0], destination[5:0]}.
// Brown instructions preserve the previous staged IEEE rounding order.
function [22:0] program_word;
 input [5:0] address;
 begin
  program_word=0;
  if(PROJECTION) begin
   case(address)
    6'd0: program_word={5'd2,6'd2,6'd0,6'd30};
    6'd1: program_word={5'd2,6'd3,6'd1,6'd31};
    6'd2: program_word={5'd0,6'd30,6'd31,6'd30};
    6'd3: program_word={5'd0,6'd30,6'd11,6'd32};
    6'd4: program_word={5'd2,6'd5,6'd0,6'd30};
    6'd5: program_word={5'd2,6'd6,6'd1,6'd31};
    6'd6: program_word={5'd0,6'd30,6'd31,6'd30};
    6'd7: program_word={5'd0,6'd30,6'd12,6'd33};
    6'd8: program_word={5'd2,6'd8,6'd0,6'd30};
    6'd9: program_word={5'd2,6'd9,6'd1,6'd31};
    6'd10: program_word={5'd0,6'd30,6'd31,6'd30};
    6'd11: program_word={5'd0,6'd30,6'd13,6'd34};
    6'd12: program_word={5'd31,6'd34,6'd34,6'd0};
    6'd13: program_word={5'd3,6'd32,6'd34,6'd35};
    6'd14: program_word={5'd3,6'd33,6'd34,6'd36};
    6'd15: program_word={5'd2,6'd35,6'd35,6'd42};
    6'd16: program_word={5'd2,6'd36,6'd36,6'd43};
    6'd17: program_word={5'd0,6'd42,6'd43,6'd44};
    6'd18: program_word={5'd2,6'd18,6'd44,6'd45};
    6'd19: program_word={5'd0,6'd60,6'd45,6'd45};
    6'd20: program_word={5'd2,6'd19,6'd44,6'd46};
    6'd21: program_word={5'd2,6'd46,6'd44,6'd46};
    6'd22: program_word={5'd0,6'd45,6'd46,6'd45};
    6'd23: program_word={5'd2,6'd20,6'd44,6'd46};
    6'd24: program_word={5'd2,6'd46,6'd44,6'd46};
    6'd25: program_word={5'd2,6'd46,6'd44,6'd46};
    6'd26: program_word={5'd0,6'd45,6'd46,6'd45};
    6'd27: program_word={5'd2,6'd35,6'd45,6'd47};
    6'd28: program_word={5'd2,6'd61,6'd21,6'd48};
    6'd29: program_word={5'd2,6'd48,6'd35,6'd48};
    6'd30: program_word={5'd2,6'd48,6'd36,6'd48};
    6'd31: program_word={5'd0,6'd47,6'd48,6'd47};
    6'd32: program_word={5'd2,6'd61,6'd35,6'd49};
    6'd33: program_word={5'd2,6'd49,6'd35,6'd49};
    6'd34: program_word={5'd0,6'd44,6'd49,6'd49};
    6'd35: program_word={5'd2,6'd22,6'd49,6'd49};
    6'd36: program_word={5'd0,6'd47,6'd49,6'd37};
    6'd37: program_word={5'd2,6'd36,6'd45,6'd47};
    6'd38: program_word={5'd2,6'd61,6'd36,6'd49};
    6'd39: program_word={5'd2,6'd49,6'd36,6'd49};
    6'd40: program_word={5'd0,6'd44,6'd49,6'd49};
    6'd41: program_word={5'd2,6'd21,6'd49,6'd49};
    6'd42: program_word={5'd0,6'd47,6'd49,6'd47};
    6'd43: program_word={5'd2,6'd61,6'd22,6'd48};
    6'd44: program_word={5'd2,6'd48,6'd35,6'd48};
    6'd45: program_word={5'd2,6'd48,6'd36,6'd48};
    6'd46: program_word={5'd0,6'd47,6'd48,6'd38};
    6'd47: program_word={5'd2,6'd14,6'd37,6'd40};
    6'd48: program_word={5'd0,6'd40,6'd16,6'd40};
    6'd49: program_word={5'd2,6'd15,6'd38,6'd41};
    6'd50: program_word={5'd0,6'd41,6'd17,6'd41};
    default: program_word=0;
   endcase
  end else begin
   case(address)
    6'd0: program_word={5'd2,6'd0,6'd0,6'd9};
    6'd1: program_word={5'd2,6'd1,6'd1,6'd10};
    6'd2: program_word={5'd0,6'd9,6'd10,6'd11};
    6'd3: program_word={5'd2,6'd2,6'd11,6'd12};
    6'd4: program_word={5'd0,6'd60,6'd12,6'd12};
    6'd5: program_word={5'd2,6'd3,6'd11,6'd13};
    6'd6: program_word={5'd2,6'd13,6'd11,6'd13};
    6'd7: program_word={5'd0,6'd12,6'd13,6'd12};
    6'd8: program_word={5'd2,6'd4,6'd11,6'd13};
    6'd9: program_word={5'd2,6'd13,6'd11,6'd13};
    6'd10: program_word={5'd2,6'd13,6'd11,6'd13};
    6'd11: program_word={5'd0,6'd12,6'd13,6'd12};
    6'd12: program_word={5'd2,6'd0,6'd12,6'd14};
    6'd13: program_word={5'd2,6'd61,6'd5,6'd15};
    6'd14: program_word={5'd2,6'd15,6'd0,6'd15};
    6'd15: program_word={5'd2,6'd15,6'd1,6'd15};
    6'd16: program_word={5'd0,6'd14,6'd15,6'd14};
    6'd17: program_word={5'd2,6'd61,6'd0,6'd16};
    6'd18: program_word={5'd2,6'd16,6'd0,6'd16};
    6'd19: program_word={5'd0,6'd11,6'd16,6'd16};
    6'd20: program_word={5'd2,6'd6,6'd16,6'd16};
    6'd21: program_word={5'd0,6'd14,6'd16,6'd30};
    6'd22: program_word={5'd2,6'd1,6'd12,6'd14};
    6'd23: program_word={5'd2,6'd61,6'd1,6'd16};
    6'd24: program_word={5'd2,6'd16,6'd1,6'd16};
    6'd25: program_word={5'd0,6'd11,6'd16,6'd16};
    6'd26: program_word={5'd2,6'd5,6'd16,6'd16};
    6'd27: program_word={5'd0,6'd14,6'd16,6'd14};
    6'd28: program_word={5'd2,6'd61,6'd6,6'd15};
    6'd29: program_word={5'd2,6'd15,6'd0,6'd15};
    6'd30: program_word={5'd2,6'd15,6'd1,6'd15};
    6'd31: program_word={5'd0,6'd14,6'd15,6'd31};
    default: program_word=0;
   endcase
  end
 end
endfunction
