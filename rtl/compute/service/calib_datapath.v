// Calibration execution datapath: one sequencer and one shared workspace.
// Board entry: calib_top -> calib_execution_service -> this module.
// Board parameters: FP_SHARED=1, ENABLE_KERNELS=0, HOST_CALLS=1.
// Kernel ownership (when enabled) is exclusive, not priority-based;
// scalar execution stops until the kernel has acknowledged completion.
// External synchronous instruction input keeps the ROM build independent.
// Host port is available between tasks AND while a HOST instruction is paused.
// Here "host" means the RTL phase adapter, not a PC or an external CPU.
// Reset cancels all transactions. See docs/INSTRUCTION_CONTROL_GUIDE.md.
module calib_datapath #(parameter RAM_WORDS=4096,CONST_BASE=3584,
 parameter FP_SHARED=0,ENABLE_KERNELS=1,TRAP_FP=0,HOST_CALLS=0)(
 input wire clk,rst_n,start_valid,output wire start_ready,
 input wire [15:0] start_pc,program_words,
 output wire imem_en,output wire [15:0] imem_addr,input wire [31:0] imem_data,
 input wire host_en,host_write,input wire [31:0] host_addr,input wire [63:0] host_data,
 output wire host_ready,output reg host_valid,output reg host_error,output wire [63:0] host_result,
 output wire busy,rsp_valid,input wire rsp_ready,output wire [7:0] rsp_status,
 output wire [15:0] debug_pc,output wire [31:0] cycle_count,instruction_count,
 output wire shared_req_valid,input wire shared_req_ready,output wire [4:0] shared_req_op,
 output wire [63:0] shared_req_a,shared_req_b,output wire shared_rsp_ready,
 input wire shared_rsp_valid,input wire [63:0] shared_rsp_result,input wire [4:0] shared_rsp_flags,
 output wire svc_valid,input wire svc_ready,output wire [7:0] svc_id
);
 localparam AW=$clog2(RAM_WORDS);
 wire smv,smr,smw,smrv,smrr,sme, sfv,sfr,sfrv,sfrr;
 wire [31:0] sma;wire [63:0] smd,smrd,sfa,sfb,sfd;wire [4:0] sfo,sff;
 wire less,equal,unordered;
 wire kmv,kmr,kmw,kmrv,kmrr,kme,kfv,kfr,kfrv,kfrr;
 wire [31:0] kma;wire [63:0] kmd,kmrd,kfa,kfb,kfd;wire [4:0] kfo,kff;
 wire kv,kr,krv,krr;wire [2:0] kid;wire [31:0] descriptor;
 wire [7:0] kstatus;wire [4:0] kflags;
 reg owner;
 wire mv=owner?kmv:smv,mw=owner?kmw:smw,mrr=owner?kmrr:smrr;
 wire [31:0] address=owner?kma:sma;wire [63:0] data=owner?kmd:smd;
 reg [1:0] mem_state;reg mem_error;reg [63:0] mem_result;
 wire address_error=address>=RAM_WORDS || (mw && address>=CONST_BASE);
 wire mr=rst_n && mem_state==0;
 wire mrv=rst_n && mem_state==2;
 wire [63:0] ram_result;
 assign smr=mr&&!owner;assign kmr=mr&&owner;
 assign smrv=mrv&&!owner;assign kmrv=mrv&&owner;
 assign smrd=mem_result;assign kmrd=mem_result;
 assign sme=mem_error;assign kme=mem_error;
 wire fv=owner?kfv:sfv;
 wire frv,fr;
 wire frr=owner?kfrr:sfrr;
 wire [63:0] fd;wire [4:0] ff;
 assign sfr=fr&&!owner;assign kfr=fr&&owner;
 assign sfrv=frv&&!owner;assign kfrv=frv&&owner;
 assign sfd=fd;assign kfd=fd;assign sff=ff;assign kff=ff;
 assign host_ready=rst_n && ((start_ready && !start_valid) || svc_valid);
 wire host_accept=host_en&&host_ready;
 calib_workspace #(.WORDS(RAM_WORDS)) workspace(.clk(clk),
  .a_en(mv&&mr&&!address_error),.a_write(mw),.a_addr(address[AW-1:0]),.a_data(data),.a_result(ram_result),
  .b_en(host_accept&&host_addr<RAM_WORDS),.b_write(host_write),.b_addr(host_addr[AW-1:0]),.b_data(host_data),.b_result(host_result));
 generate if(FP_SHARED)begin : g_shared
  assign shared_req_valid=fv;assign fr=shared_req_ready;assign shared_req_op=owner?kfo:sfo;
  assign shared_req_a=owner?kfa:sfa;assign shared_req_b=owner?kfb:sfb;
  assign shared_rsp_ready=frr;assign frv=shared_rsp_valid;assign fd=shared_rsp_result;assign ff=shared_rsp_flags;
  // The legacy pool returns flags but no compare outputs. Operands remain
  // stable until response consumption, so derive the comparison locally.
  wire na=(shared_req_a[62:52]==2047)&&(|shared_req_a[51:0]);
  wire nb=(shared_req_b[62:52]==2047)&&(|shared_req_b[51:0]);
  wire zz=!(|shared_req_a[62:0]) && !(|shared_req_b[62:0]);
  assign unordered=na||nb;
  assign equal=!unordered && (shared_req_a==shared_req_b || zz);
  assign less=!unordered && !zz && ((shared_req_a[63]!=shared_req_b[63])?shared_req_a[63]:
   (shared_req_a[63]?(shared_req_a>shared_req_b):(shared_req_a<shared_req_b)));
 end else begin : g_local
 assign shared_req_valid=0;assign shared_req_op=0;assign shared_req_a=0;assign shared_req_b=0;assign shared_rsp_ready=0;
 calib_alu alu(.ce(1'b1),.clk(clk),.rst_n(rst_n),.req_valid(fv),.req_ready(fr),.req_op(owner?kfo:sfo),
  .req_a(owner?kfa:sfa),.req_b(owner?kfb:sfb),.rsp_valid(frv),.rsp_ready(frr),.rsp_result(fd),
  .rsp_flags(ff),.rsp_less(less),.rsp_equal(equal),.rsp_unordered(unordered));
 end endgenerate
 calib_sequencer #(.RAM_WORDS(RAM_WORDS),.CONST_BASE(CONST_BASE),.TRAP_FP(TRAP_FP),.HOST_CALLS(HOST_CALLS)) sequencer(
  .clk(clk),.rst_n(rst_n),.start_valid(start_valid),.start_ready(start_ready),.start_pc(start_pc),.program_words(program_words),
  .imem_en(imem_en),.imem_addr(imem_addr),.imem_data(imem_data),
  .mem_req_valid(smv),.mem_req_ready(smr),.mem_req_write(smw),.mem_req_addr(sma),.mem_req_data(smd),
  .mem_rsp_valid(smrv),.mem_rsp_ready(smrr),.mem_rsp_data(smrd),.mem_rsp_error(sme),
  .alu_req_valid(sfv),.alu_req_ready(sfr),.alu_req_op(sfo),.alu_req_a(sfa),.alu_req_b(sfb),
  .alu_rsp_valid(sfrv),.alu_rsp_ready(sfrr),.alu_rsp_data(sfd),.alu_rsp_flags(sff),.alu_less(less),.alu_equal(equal),.alu_unordered(unordered),
  .kernel_req_valid(kv),.kernel_req_ready(kr),.kernel_id(kid),.kernel_descriptor(descriptor),
  .kernel_rsp_valid(krv),.kernel_rsp_ready(krr),.kernel_status(kstatus),.kernel_flags(kflags),
  .busy(busy),.rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),
  .debug_pc(debug_pc),.cycle_count(cycle_count),.instruction_count(instruction_count),
  .svc_valid(svc_valid),.svc_ready(svc_ready),.svc_id(svc_id));
 generate if(ENABLE_KERNELS)begin : g_kernel
 calib_kernel_ctrl #(.RAM_WORDS(RAM_WORDS),.CONST_BASE(CONST_BASE)) kernel(
  .clk(clk),.rst_n(rst_n),.req_valid(kv),.req_ready(kr),.req_id(kid),.req_descriptor(descriptor),
  .rsp_valid(krv),.rsp_ready(krr),.rsp_status(kstatus),.rsp_flags(kflags),
  .mem_req_valid(kmv),.mem_req_ready(kmr),.mem_req_write(kmw),.mem_req_addr(kma),.mem_req_data(kmd),
  .mem_rsp_valid(kmrv),.mem_rsp_ready(kmrr),.mem_rsp_data(kmrd),.mem_rsp_error(kme),
  .alu_req_valid(kfv),.alu_req_ready(kfr),.alu_req_op(kfo),.alu_req_a(kfa),.alu_req_b(kfb),
  .alu_rsp_valid(kfrv),.alu_rsp_ready(kfrr),.alu_rsp_data(kfd),.alu_rsp_flags(kff));
 end else begin : g_no_kernel
  assign kr=1'b1;assign krv=owner;assign kstatus=8'd1;assign kflags=0;
  assign kmv=0;assign kmw=0;assign kma=0;assign kmd=0;assign kmrr=0;
  assign kfv=0;assign kfo=0;assign kfa=0;assign kfb=0;assign kfrr=0;
 end endgenerate
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin owner<=0;mem_state<=0;mem_error<=0;mem_result<=0;host_valid<=0;host_error<=0;end
  else begin
   if(kv&&kr)owner<=1;
   if(krv&&krr)owner<=0;
   host_valid<=host_accept;host_error<=host_accept && host_addr>=RAM_WORDS;
   case(mem_state)
    0:if(mv)begin mem_error<=address_error;mem_state<=1;end
    1:begin mem_result<=ram_result;mem_state<=2;end
    2:if(mrr)mem_state<=0;
   endcase
  end
 end
endmodule
