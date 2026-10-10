// Shared detection add/subtract service. Each client has ONE request/result
// slot, so a stalled distance consumer cannot block a different accumulation
// client (which may be needed to unblock that consumer).
// pair=1 serializes X and Y, then publishes both results atomically. This keeps
// existing paired-result contracts while removing duplicate X/Y adders.
module fp32_pair_add_pool #(parameter CLIENTS=4,USE_CE=0)(
 input wire clk,rst_n,ce,
 input wire [CLIENTS-1:0] req_valid,req_sub,req_pair,
 output wire [CLIENTS-1:0] req_ready,
 input wire [CLIENTS*64-1:0] req_a,req_b,
 output wire [CLIENTS-1:0] rsp_valid,input wire [CLIENTS-1:0] rsp_ready,
 output wire [CLIENTS*64-1:0] result
);
 localparam OW=CLIENTS>1?$clog2(CLIENTS):1;
 localparam PICK=0,ISSUE=1,WAIT_RESULT=2;
 reg [1:0] state;
 reg [CLIENTS-1:0] occupied,pending,complete,subtract,pair;
 reg [63:0] a[0:CLIENTS-1],b[0:CLIENTS-1],data[0:CLIENTS-1];
 reg [OW-1:0] owner,next_owner;
 reg lane;
 wire enabled=!USE_CE || ce;
 wire ready,valid;wire [31:0] sum;
 integer i,j,k,selected;
 assign req_ready={CLIENTS{rst_n && enabled}} & ~occupied;
 assign rsp_valid={CLIENTS{rst_n && enabled}} & complete;
 genvar g;generate for(g=0;g<CLIENTS;g=g+1)begin:results assign result[g*64+:64]=data[g];end endgenerate
 always @*begin
  selected=-1;
  for(j=0;j<CLIENTS;j=j+1)begin
   k=next_owner+j;if(k>=CLIENTS)k=k-CLIENTS;
   if(selected<0 && pending[k])selected=k;
  end
 end
 wire [31:0] operand_a=lane?a[owner][63:32]:a[owner][31:0];
 wire [31:0] operand_b=lane?b[owner][63:32]:b[owner][31:0];
 fp32_add #(.USE_CE(USE_CE)) arithmetic(.clk(clk),.rst_n(rst_n),.ce(ce),
  .in_valid(state==ISSUE),.in_ready(ready),.in_a(operand_a),
  .in_b({operand_b[31]^subtract[owner],operand_b[30:0]}),
  .out_valid(valid),.out_ready(state==WAIT_RESULT),.out_r(sum));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=PICK;occupied<=0;pending<=0;complete<=0;subtract<=0;pair<=0;
   owner<=0;next_owner<=0;lane<=0;
   for(i=0;i<CLIENTS;i=i+1)begin a[i]<=0;b[i]<=0;data[i]<=0;end
  end else if(enabled)begin
   for(i=0;i<CLIENTS;i=i+1)begin
    if(rsp_valid[i] && rsp_ready[i])begin occupied[i]<=0;complete[i]<=0;end
    if(req_valid[i] && req_ready[i])begin
     occupied[i]<=1;pending[i]<=1;a[i]<=req_a[i*64+:64];b[i]<=req_b[i*64+:64];
     subtract[i]<=req_sub[i];pair[i]<=req_pair[i];
    end
   end
   case(state)
    PICK:if(selected>=0)begin owner<=selected;pending[selected]<=0;lane<=0;state<=ISSUE;
      next_owner<=selected==CLIENTS-1?0:selected+1;end
    ISSUE:if(ready)state<=WAIT_RESULT;
    WAIT_RESULT:if(valid)begin
      if(lane)data[owner][63:32]<=sum;
      else begin data[owner][31:0]<=sum;data[owner][63:32]<=0;end
      if(!lane && pair[owner])begin lane<=1;state<=ISSUE;end
      else begin complete[owner]<=1;state<=PICK;end
    end
   endcase
  end
 end
endmodule
