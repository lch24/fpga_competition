`include "calib_engine_defs.vh"
`include "calib_defs.vh"
// Blocking 32-bit microsequencer. Eight A32 + eight F64 registers, four return
// addresses. No instruction prefetch or out-of-order execution. ROM supplies
// synchronous data one cycle after imem_en and holds it until the next fetch.
// RAM/ALU/kernel request and response are independent valid/ready channels.
// A memory write must return an acknowledgement, so errors cannot be lost.
// Reading map: IDLE accepts start_pc -> FETCH reads synchronous ROM -> EXEC
// decodes -> MREQ/MWAIT or FREQ/FWAIT waits for a service -> FETCH next PC.
// HOST enters HWAIT without releasing phase ownership; END enters DONE.
// ISA source: data/programs/calibration/isa.json. Reader guide:
// docs/INSTRUCTION_CONTROL_GUIDE.md (math/feature use different encodings).
module calib_sequencer #(parameter RAM_WORDS=4096, CONST_BASE=3584, TRAP_FP=0, HOST_CALLS=0)(
 input wire clk,rst_n,start_valid,output wire start_ready,
 input wire [15:0] start_pc,program_words,
 output wire imem_en,output wire [15:0] imem_addr,input wire [31:0] imem_data,
 output wire mem_req_valid,input wire mem_req_ready,output wire mem_req_write,
 output wire [31:0] mem_req_addr,output wire [63:0] mem_req_data,
 input wire mem_rsp_valid,output wire mem_rsp_ready,input wire [63:0] mem_rsp_data,input wire mem_rsp_error,
 output wire alu_req_valid,input wire alu_req_ready,output wire [4:0] alu_req_op,
 output wire [63:0] alu_req_a,alu_req_b,
 input wire alu_rsp_valid,output wire alu_rsp_ready,input wire [63:0] alu_rsp_data,
 input wire [4:0] alu_rsp_flags,input wire alu_less,alu_equal,alu_unordered,
 output wire kernel_req_valid,input wire kernel_req_ready,output wire [2:0] kernel_id,
 output wire [31:0] kernel_descriptor,
 input wire kernel_rsp_valid,output wire kernel_rsp_ready,input wire [7:0] kernel_status,
 input wire [4:0] kernel_flags,
 output wire busy,output wire rsp_valid,input wire rsp_ready,output reg [7:0] rsp_status,
 output wire [15:0] debug_pc,output reg [31:0] cycle_count,instruction_count,
 output wire svc_valid,input wire svc_ready,output wire [7:0] svc_id
);
 `include "calib_engine_decode.vh"
 localparam IDLE=0,FETCH=1,EXEC=2,MREQ=3,MWAIT=4,FREQ=5,FWAIT=6,KREQ=7,KWAIT=8,DONE=9,HWAIT=10;
 reg guard_valid;reg [15:0] guard_pc;reg [2:0] guard_sp;
 reg [3:0] state;
 reg [15:0] pc,limit;
 reg [31:0] ar[0:7];reg [63:0] fr[0:7];
 reg [15:0] stack[0:3];reg [2:0] sp;
 reg less,equal,unordered;
 reg [4:0] fp_flags;reg [7:0] last_kernel;
 wire [5:0] opcode=imem_data[31:26];wire [2:0] mode=imem_data[25:23];
 wire [2:0] rd=imem_data[22:20],ra=imem_data[19:17],rb=imem_data[16:14];
 wire [13:0] imm=imem_data[13:0];
 wire [33:0] address={2'b0,ar[ra]}+{{20{imm[13]}},imm};
 wire [17:0] sequential={2'b0,pc}+18'd1;
 wire [17:0] relative=sequential+{{4{imm[13]}},imm};
 wire branch=(mode==0) || (mode==1 && equal) || (mode==2 && !equal) ||
    (mode==3 && less && !unordered) || (mode==4 && (less||equal) && !unordered) ||
    (mode==5 && !less && !equal && !unordered) || (mode==6 && !less && !unordered) ||
    (mode==7 && unordered);
 wire narrow=mode==1;
 wire [10:0] class_exp=narrow?{3'd0,fr[ra][30:23]}:fr[ra][62:52];
 wire class_frac=narrow?(|fr[ra][22:0]):(|fr[ra][51:0]);
 wire class_max=narrow?(class_exp==255):(class_exp==2047);
 wire [4:0] classification={class_max&&class_frac,class_max&&!class_frac,
     class_exp!=0&&!class_max,class_exp==0&&class_frac,class_exp==0&&!class_frac};
 integer i;
 assign busy=state!=IDLE && state!=DONE;
 assign start_ready=rst_n && state==IDLE;
 assign rsp_valid=rst_n && state==DONE;
 assign debug_pc=pc;
 // A host call quiesces both execution ports while preserving PC/registers.
 // The adapter may access workspace RAM until it acknowledges completion.
 assign svc_valid=HOST_CALLS && rst_n && state==HWAIT;
 assign svc_id=imm[7:0];
 assign imem_en=rst_n && state==FETCH;assign imem_addr=pc;
 assign mem_req_valid=rst_n && state==MREQ;
 assign mem_req_write=opcode==`CE_ST || opcode==`CE_STI32;
 assign mem_req_addr=address[31:0];
 assign mem_req_data=opcode==`CE_STI32 ? {32'd0,ar[rd]} : fr[rd];
 assign mem_rsp_ready=rst_n && state==MWAIT;
 assign alu_req_valid=rst_n && state==FREQ;
 assign alu_req_op=opcode==`CE_FCMP ? `PAR_FP_COMPARE :
     opcode==`CE_FLOG ? `PAR_FP_LOG :
     opcode==`CE_FEXP ? `PAR_FP_EXP :
     opcode==`CE_FSIN ? `PAR_FP_SIN :
     opcode==`CE_FCOS ? `PAR_FP_COS :
     opcode==`CE_FACOS ? `PAR_FP_ACOS :
     opcode==`CE_FATAN2 ? `PAR_FP_ATAN2 :
     opcode==`CE_FCVT ? (mode==0 ? `PAR_FP_F32_TO_F64 : `PAR_FP_F64_TO_F32) : {1'b0,opcode[3:0]};
 assign alu_req_a=fr[ra];assign alu_req_b=fr[rb];
 assign alu_rsp_ready=rst_n && state==FWAIT;
 assign kernel_req_valid=rst_n && state==KREQ;
 assign kernel_id=imm[2:0];assign kernel_descriptor=ar[ra];
 assign kernel_rsp_ready=rst_n && state==KWAIT;
 task stop;
  input [7:0] code;
  begin rsp_status<=code;state<=DONE;end
 endtask
 task advance;
  input [17:0] target;
  begin
   if(target>={2'b0,limit})stop(8'he2);
   else begin pc<=target[15:0];state<=FETCH;end
  end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;pc<=0;limit<=0;sp<=0;less<=0;equal<=0;unordered<=0;
   fp_flags<=0;last_kernel<=0;rsp_status<=0;cycle_count<=0;instruction_count<=0;
   guard_valid<=0;guard_pc<=0;guard_sp<=0;
   for(i=0;i<8;i=i+1)begin ar[i]<=0;fr[i]<=0;end
   for(i=0;i<4;i=i+1)stack[i]<=0;
  end else begin
   if(busy)cycle_count<=cycle_count+1'b1;
   case(state)
    IDLE:if(start_valid)begin
     limit<=program_words;pc<=start_pc;sp<=0;less<=0;equal<=0;unordered<=0;guard_valid<=0;
     fp_flags<=0;last_kernel<=0;cycle_count<=0;instruction_count<=0;rsp_status<=0;
     for(i=0;i<8;i=i+1)begin ar[i]<=0;fr[i]<=0;end
     if(program_words==0 || start_pc>=program_words)stop(8'he2);else state<=FETCH;
    end
    FETCH:state<=EXEC;
    EXEC:begin
     instruction_count<=instruction_count+1'b1;
     if(!ce_legal(imem_data))stop(8'he1);
     else case(opcode)
      `CE_NOP:advance(sequential);
      `CE_END:stop(imm[7:0]);
      `CE_MOVI:begin ar[rd]<={18'd0,imm};advance(sequential);end
      `CE_ADDI:begin ar[rd]<=ar[ra]+{{18{imm[13]}},imm};advance(sequential);end
      `CE_IADD:begin ar[rd]<=ar[ra]+ar[rb];advance(sequential);end
      `CE_ISUB:begin ar[rd]<=ar[ra]-ar[rb];advance(sequential);end
      `CE_ICMP:begin
       less<=mode==1?($signed(ar[ra])<$signed(ar[rb])):(ar[ra]<ar[rb]);
       equal<=ar[ra]==ar[rb];unordered<=0;advance(sequential);
      end
      `CE_BR:advance(branch?relative:sequential);
      `CE_CALL:begin
       if(sp==4)stop(8'he3);
       else if(sequential>={2'b0,limit})stop(8'he2);
       else begin stack[sp[1:0]]<=sequential[15:0];sp<=sp+1'b1;advance(relative);end
      end
      `CE_RET:if(sp==0)stop(8'he3);else begin sp<=sp-1'b1;advance({2'b0,stack[(sp-1'b1)&3]});end
      `CE_DBNZ:if(ar[rd]==0)stop(8'he7);else begin ar[rd]<=ar[rd]-1'b1;advance(ar[rd]==1?sequential:relative);end
      `CE_LD,`CE_ST,`CE_LDI32,`CE_STI32:begin
       if(address>=RAM_WORDS || (mem_req_write && address>=CONST_BASE))stop(8'he4);
       else state<=MREQ;
      end
      `CE_FMOV:begin fr[rd]<=fr[ra];advance(sequential);end
      `CE_FGET32:begin fr[rd]<={32'd0,(mode==0?fr[ra][31:0]:fr[ra][63:32])};advance(sequential);end
      `CE_FABS:begin fr[rd]<={1'b0,fr[ra][62:0]};advance(sequential);end
      `CE_FNEG:begin fr[rd]<={~fr[ra][63],fr[ra][62:0]};advance(sequential);end
      `CE_CLASS:begin ar[rd]<={27'd0,classification};advance(sequential);end
      `CE_STATUS:begin ar[rd]<=mode==0?{27'd0,fp_flags}:{24'd0,last_kernel};advance(sequential);end
      `CE_CLRFLAGS:begin fp_flags<=0;advance(sequential);end
      `CE_HOST:if(HOST_CALLS)state<=HWAIT;else stop(8'he1);
      `CE_GUARD:if(!HOST_CALLS)stop(8'he1);else if(relative>={2'b0,limit})stop(8'he2);
       else begin guard_valid<=1;guard_pc<=relative[15:0];guard_sp<=sp;advance(sequential);end
      `CE_FADD,`CE_FSUB,`CE_FMUL,`CE_FDIV,`CE_FSQRT,`CE_FCMP,`CE_FCVT,`CE_FLOG,`CE_FATAN2,
      `CE_FEXP,`CE_FSIN,`CE_FCOS,`CE_FACOS:state<=FREQ;
      `CE_KEXEC:if(ar[ra]>RAM_WORDS-8)stop(8'he4);else state<=KREQ;
      default:stop(8'he1);
     endcase
    end
    MREQ:if(mem_req_ready)state<=MWAIT;
    MWAIT:if(mem_rsp_valid)begin
     if(mem_rsp_error)stop(8'he5);
     else begin
      if(opcode==`CE_LD)fr[rd]<=mem_rsp_data;
      if(opcode==`CE_LDI32)ar[rd]<=mem_rsp_data[31:0];
      advance(sequential);
     end
    end
    FREQ:if(alu_req_ready)state<=FWAIT;
    FWAIT:if(alu_rsp_valid)begin
     fp_flags<=fp_flags|alu_rsp_flags;
     if((TRAP_FP || guard_valid) && ((|alu_rsp_flags[2:0]) || (alu_rsp_data[62:52]==2047)))begin
      if(guard_valid)begin sp<=guard_sp;advance({2'b0,guard_pc});end else stop(8'he8);
     end
     else begin
     if(opcode==`CE_FCMP)begin less<=alu_less;equal<=alu_equal;unordered<=alu_unordered;end
     else fr[rd]<=alu_rsp_data;
     advance(sequential);
     end
    end
    KREQ:if(kernel_req_ready)state<=KWAIT;
    KWAIT:if(kernel_rsp_valid)begin
     last_kernel<=kernel_status;fp_flags<=fp_flags|kernel_flags;
     if(kernel_status!=0)stop(8'he6);else advance(sequential);
    end
    DONE:if(rsp_ready)state<=IDLE;
    HWAIT:if(svc_ready)advance(sequential);
    default:stop(8'he1);
   endcase
  end
 end
endmodule
