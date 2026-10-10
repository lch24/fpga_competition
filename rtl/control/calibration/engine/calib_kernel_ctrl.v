`include "calib_defs.vh"
// Exclusive, blocking CLEAR/COPY/DOT/AXPY/RANK1 engine. The sequencer waits
// while this unit owns the one ALU and workspace port. No duplicated MAC.
// Descriptor preflight uses iterative address additions, not 32-bit products.
// It validates all ranges before any write; conservative interval overlap
// checks reject interleaved overlapping bounds as well as real collisions.
module calib_kernel_ctrl #(parameter RAM_WORDS=4096,CONST_BASE=3584)(
 input wire clk,rst_n,req_valid,output wire req_ready,
 input wire [2:0] req_id,input wire [31:0] req_descriptor,
 output wire rsp_valid,input wire rsp_ready,output reg [7:0] rsp_status,output reg [4:0] rsp_flags,
 output wire mem_req_valid,input wire mem_req_ready,output wire mem_req_write,
 output wire [31:0] mem_req_addr,output wire [63:0] mem_req_data,
 input wire mem_rsp_valid,output wire mem_rsp_ready,input wire [63:0] mem_rsp_data,input wire mem_rsp_error,
 output wire alu_req_valid,input wire alu_req_ready,output wire [4:0] alu_req_op,
 output wire [63:0] alu_req_a,alu_req_b,
 input wire alu_rsp_valid,output wire alu_rsp_ready,input wire [63:0] alu_rsp_data,input wire [4:0] alu_rsp_flags
);
 localparam IDLE=0,DREQ=1,DWAIT=2,CHECK=3,SCAN=4,OVERLAP=5,
   READ0=6,WAIT0=7,READ1=8,WAIT1=9,MULREQ=10,MULWAIT=11,
   ADDREQ=12,ADDWAIT=13,WRITE=14,WRITEWAIT=15,NEXT=16,DONE=17,PAIRREAD=18,PAIRWAIT=19;
 reg [4:0] state;reg [2:0] kind,di;
 reg [31:0] descriptor,src0,src1,dst,count,stride0,stride1,strided,colspan;
 reg [63:0] alpha,x,y,product,acc,result;
 reg bad_descriptor;
 reg [32:0] end0,end1,endd;
 reg [31:0] scan_left,left,row_left,col_left,p0,p1,pd,row_address;
 wire uses0=kind!=0 && count!=0;
 wire uses1=(kind==2 || kind==3) && count!=0;
 wire usesd=count!=0 || kind==2;
 wire overlap0=uses0 && usesd && ({1'b0,src0}<=endd) && ({1'b0,dst}<=end0);
 wire overlap1=uses1 && usesd && ({1'b0,src1}<=endd) && ({1'b0,dst}<=end1);
 wire same0=src0==dst && stride0==strided;
 wire same1=src1==dst && stride1==strided;
 assign req_ready=rst_n && state==IDLE;
 assign rsp_valid=rst_n && state==DONE;
 assign mem_req_valid=rst_n && (state==DREQ || state==READ0 || state==READ1 || state==WRITE || state==PAIRREAD);
 assign mem_req_write=state==WRITE;
 assign mem_req_addr=state==DREQ?descriptor+{29'd0,di}:state==READ0?p0:state==PAIRREAD?p1:state==READ1?(kind==4?pd:p1):pd;
 assign mem_req_data=result;
 assign mem_rsp_ready=rst_n && (state==DWAIT || state==WAIT0 || state==WAIT1 || state==WRITEWAIT || state==PAIRWAIT);
 assign alu_req_valid=rst_n && (state==MULREQ || state==ADDREQ);
 assign alu_req_op=state==MULREQ ? `PAR_FP_MUL : `PAR_FP_ADD;
 assign alu_req_a=state==MULREQ?x:product;
 assign alu_req_b=state==MULREQ?(kind==3?alpha:y):(kind==2?acc:y);
 assign alu_rsp_ready=rst_n && (state==MULWAIT || state==ADDWAIT);
 task stop;
  input [7:0] status;
  begin rsp_status<=status;state<=DONE;end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;kind<=0;di<=0;descriptor<=0;src0<=0;src1<=0;dst<=0;count<=0;
   stride0<=0;stride1<=0;strided<=0;colspan<=0;alpha<=0;x<=0;y<=0;product<=0;acc<=0;result<=0;
   bad_descriptor<=0;end0<=0;end1<=0;endd<=0;scan_left<=0;left<=0;row_left<=0;col_left<=0;
   p0<=0;p1<=0;pd<=0;row_address<=0;rsp_status<=0;rsp_flags<=0;
  end else case(state)
   IDLE:if(req_valid)begin
    kind<=req_id;descriptor<=req_descriptor;di<=0;bad_descriptor<=0;rsp_status<=0;rsp_flags<=0;
    if(req_id>4 || req_descriptor>RAM_WORDS-8)stop(1);else state<=DREQ;
   end
   DREQ:if(mem_req_ready)state<=DWAIT;
   DWAIT:if(mem_rsp_valid)begin
    if(mem_rsp_error)stop(2);
    else begin
     case(di)
      0:begin src0<=mem_rsp_data[31:0];src1<=mem_rsp_data[63:32];end
      1:begin dst<=mem_rsp_data[31:0];count<=mem_rsp_data[63:32];end
      2:begin stride0<=mem_rsp_data[31:0];stride1<=mem_rsp_data[63:32];end
      3:begin strided<=mem_rsp_data[31:0];bad_descriptor<=bad_descriptor || (|mem_rsp_data[63:32]);end
      4:begin colspan<=mem_rsp_data[31:0];bad_descriptor<=bad_descriptor || (|mem_rsp_data[63:32]);end
      5:alpha<=mem_rsp_data;
      6,7:bad_descriptor<=bad_descriptor || (|mem_rsp_data);
     endcase
     if(di==7)state<=CHECK;else begin di<=di+1'b1;state<=DREQ;end
    end
   end
   CHECK:begin
    if(bad_descriptor || count>RAM_WORDS ||
       (kind==4 && (colspan!=count || strided!=1 || src1!=0 || alpha!=0)) ||
       (kind!=4 && colspan!=0))stop(1);
    else begin
     end0<={1'b0,src0};end1<={1'b0,src1};endd<={1'b0,dst};scan_left<=count;
     state<=SCAN;
    end
   end
   SCAN:begin
    if((uses0 && end0>=RAM_WORDS) || (uses1 && end1>=RAM_WORDS) || (usesd && endd>=CONST_BASE))stop(3);
    else if(scan_left>1)begin
     if(uses0)end0<=end0+{1'b0,stride0};
     if(uses1)end1<=end1+{1'b0,stride1};
     if(kind==4)endd<=endd+{1'b0,scan_left};
     else if(kind!=2)endd<=endd+{1'b0,strided};
     scan_left<=scan_left-1'b1;
    end else state<=OVERLAP;
   end
   OVERLAP:begin
    if((overlap0 && !(kind==1 && same0)) || (overlap1 && !(kind==3 && same1)))stop(4);
    else begin
     p0<=src0;p1<=kind==4?src0:src1;pd<=dst;row_address<=src0;
     left<=count;row_left<=count;col_left<=count;acc<=0;result<=0;
     if(count==0)begin if(kind==2)state<=WRITE;else stop(0);end
     else state<=kind==0?WRITE:READ0;
    end
   end
   READ0:if(mem_req_ready)state<=WAIT0;
   WAIT0:if(mem_rsp_valid)begin
    if(mem_rsp_error)stop(2);
    else begin
     x<=mem_rsp_data;
     if(kind==1)begin result<=mem_rsp_data;state<=WRITE;end
     else if(kind==4)begin
      // The first read of a rank-one element pair is x_i. Read x_j next
      // using READ0 again, tracked by a dedicated state below.
      state<=PAIRREAD;
     end else state<=READ1;
    end
   end
   PAIRREAD:if(mem_req_ready)state<=PAIRWAIT;
   PAIRWAIT:if(mem_rsp_valid)begin
    if(mem_rsp_error)stop(2);else begin y<=mem_rsp_data;state<=MULREQ;end
   end
   READ1:if(mem_req_ready)state<=WAIT1;
   WAIT1:if(mem_rsp_valid)begin
    if(mem_rsp_error)stop(2);
    else begin y<=mem_rsp_data;state<=kind==4?ADDREQ:MULREQ;end
   end
   MULREQ:if(alu_req_ready)state<=MULWAIT;
   MULWAIT:if(alu_rsp_valid)begin
    product<=alu_rsp_data;rsp_flags<=rsp_flags|alu_rsp_flags;
    state<=kind==4?READ1:ADDREQ;
   end
   ADDREQ:if(alu_req_ready)state<=ADDWAIT;
   ADDWAIT:if(alu_rsp_valid)begin
    rsp_flags<=rsp_flags|alu_rsp_flags;result<=alu_rsp_data;
    if(kind==2)begin acc<=alu_rsp_data;state<=left==1?WRITE:NEXT;end
    else state<=WRITE;
   end
   WRITE:if(mem_req_ready)state<=WRITEWAIT;
   WRITEWAIT:if(mem_rsp_valid)begin if(mem_rsp_error)stop(2);else if(kind==2)stop(0);else state<=NEXT;end
   NEXT:begin
    if(kind==4)begin
     if(row_left==1 && col_left==1)stop(0);
     else begin
      pd<=pd+1'b1;
      if(col_left==1)begin
       row_left<=row_left-1'b1;col_left<=row_left-1'b1;
       row_address<=row_address+stride0;p0<=row_address+stride0;p1<=row_address+stride0;
      end else begin col_left<=col_left-1'b1;p1<=p1+stride0;end
      state<=READ0;
     end
    end else if(left==1)stop(0);
    else begin
     left<=left-1'b1;p0<=p0+stride0;p1<=p1+stride1;
     if(kind!=2)pd<=pd+strided;
     state<=kind==0?WRITE:READ0;
    end
   end
   DONE:if(rsp_ready)state<=IDLE;
   default:stop(1);
  endcase
 end
endmodule
