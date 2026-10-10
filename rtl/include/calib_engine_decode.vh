// Generated from isa.json.
function ce_legal;
 input [31:0] instruction;
 begin
  case(instruction[31:26])
  `CE_NOP:ce_legal=((instruction & 32'h03ffffff)==0) && (instruction[25:23]==3'd0);
  `CE_END:ce_legal=((instruction & 32'h03ffff00)==0) && (instruction[25:23]==3'd0);
  `CE_MOVI:ce_legal=((instruction & 32'h038fc000)==0) && (instruction[25:23]==3'd0);
  `CE_ADDI:ce_legal=((instruction & 32'h0381c000)==0) && (instruction[25:23]==3'd0);
  `CE_IADD:ce_legal=((instruction & 32'h03803fff)==0) && (instruction[25:23]==3'd0);
  `CE_ISUB:ce_legal=((instruction & 32'h03803fff)==0) && (instruction[25:23]==3'd0);
  `CE_ICMP:ce_legal=((instruction & 32'h00703fff)==0) && (instruction[25:23]==3'd0 || instruction[25:23]==3'd1);
  `CE_BR:ce_legal=((instruction & 32'h007fc000)==0) && (instruction[25:23]==3'd0 || instruction[25:23]==3'd1 || instruction[25:23]==3'd2 || instruction[25:23]==3'd3 || instruction[25:23]==3'd4 || instruction[25:23]==3'd5 || instruction[25:23]==3'd6 || instruction[25:23]==3'd7);
  `CE_CALL:ce_legal=((instruction & 32'h03ffc000)==0) && (instruction[25:23]==3'd0);
  `CE_RET:ce_legal=((instruction & 32'h03ffffff)==0) && (instruction[25:23]==3'd0);
  `CE_DBNZ:ce_legal=((instruction & 32'h038fc000)==0) && (instruction[25:23]==3'd0);
  `CE_LD:ce_legal=((instruction & 32'h0381c000)==0) && (instruction[25:23]==3'd0);
  `CE_ST:ce_legal=((instruction & 32'h0381c000)==0) && (instruction[25:23]==3'd0);
  `CE_FMOV:ce_legal=((instruction & 32'h0381ffff)==0) && (instruction[25:23]==3'd0);
  `CE_LDI32:ce_legal=((instruction & 32'h0381c000)==0) && (instruction[25:23]==3'd0);
  `CE_STI32:ce_legal=((instruction & 32'h0381c000)==0) && (instruction[25:23]==3'd0);
  `CE_FGET32:ce_legal=((instruction & 32'h0001ffff)==0) && (instruction[25:23]==3'd0 || instruction[25:23]==3'd1);
  `CE_FADD:ce_legal=((instruction & 32'h03803fff)==0) && (instruction[25:23]==3'd0);
  `CE_FSUB:ce_legal=((instruction & 32'h03803fff)==0) && (instruction[25:23]==3'd0);
  `CE_FMUL:ce_legal=((instruction & 32'h03803fff)==0) && (instruction[25:23]==3'd0);
  `CE_FDIV:ce_legal=((instruction & 32'h03803fff)==0) && (instruction[25:23]==3'd0);
  `CE_FSQRT:ce_legal=((instruction & 32'h0381ffff)==0) && (instruction[25:23]==3'd0);
  `CE_FCMP:ce_legal=((instruction & 32'h03f03fff)==0) && (instruction[25:23]==3'd0);
  `CE_FABS:ce_legal=((instruction & 32'h0381ffff)==0) && (instruction[25:23]==3'd0);
  `CE_FNEG:ce_legal=((instruction & 32'h0381ffff)==0) && (instruction[25:23]==3'd0);
  `CE_FCVT:ce_legal=((instruction & 32'h0001ffff)==0) && (instruction[25:23]==3'd0 || instruction[25:23]==3'd1);
  `CE_CLASS:ce_legal=((instruction & 32'h0001ffff)==0) && (instruction[25:23]==3'd0 || instruction[25:23]==3'd1);
  `CE_FEXP:ce_legal=((instruction & 32'h0381ffff)==0) && (instruction[25:23]==3'd0);
  `CE_FLOG:ce_legal=((instruction & 32'h0381ffff)==0) && (instruction[25:23]==3'd0);
  `CE_FSIN:ce_legal=((instruction & 32'h0381ffff)==0) && (instruction[25:23]==3'd0);
  `CE_FCOS:ce_legal=((instruction & 32'h0381ffff)==0) && (instruction[25:23]==3'd0);
  `CE_FATAN2:ce_legal=((instruction & 32'h03803fff)==0) && (instruction[25:23]==3'd0);
  `CE_FACOS:ce_legal=((instruction & 32'h0381ffff)==0) && (instruction[25:23]==3'd0);
  `CE_KEXEC:ce_legal=((instruction & 32'h03f1fff8)==0) && (instruction[25:23]==3'd0) && instruction[13:0]<=14'd4;
  `CE_STATUS:ce_legal=((instruction & 32'h000fffff)==0) && (instruction[25:23]==3'd0 || instruction[25:23]==3'd1);
  `CE_CLRFLAGS:ce_legal=((instruction & 32'h03ffffff)==0) && (instruction[25:23]==3'd0);
  `CE_HOST:ce_legal=((instruction & 32'h03ffff00)==0) && (instruction[25:23]==3'd0);
  `CE_GUARD:ce_legal=((instruction & 32'h03ffc000)==0) && (instruction[25:23]==3'd0);
  default:ce_legal=0;
  endcase
 end
endfunction
