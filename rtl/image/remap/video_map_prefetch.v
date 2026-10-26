`timescale 1ns/1ps
// Two 64-pixel map blocks, 16 HMIC beats each. Four synchronous 64-bit RAM
// banks accept an entire physical return every clock. Only two buffers, not
// a full-frame cache. Descriptors are held under backpressure.
module video_map_prefetch #(parameter PIXELS=1280*720)(
 input wire clk,rst_n,start,input wire [31:0] map_base,
 output reg out_valid,input wire out_ready,output reg [63:0] descriptor,
 output reg error,
 output wire arvalid,input wire arready,output wire [27:0] araddr,output wire [3:0] arlen,
 input wire rvalid,input wire [255:0] rdata,input wire rlast,input wire [3:0] rid
);
 (* syn_ramstyle="block_ram" *) reg [63:0] bank0[0:31],bank1[0:31],bank2[0:31],bank3[0:31];
 reg [63:0] q0,q1,q2,q3;
 reg [1:0] ready_bank;reg fill_bank,read_bank,active,loading,requesting,pending;
 reg [3:0] beat;reg [5:0] pos;reg [1:0] lane;
 localparam BLOCK_BITS=$clog2(PIXELS/64+1),PIXEL_BITS=$clog2(PIXELS+1);
 reg [31:0] base;
 reg [BLOCK_BITS-1:0] issued;reg [PIXEL_BITS-1:0] launched;
 wire [27:0] issued_words=issued;
 assign arvalid=requesting;assign araddr=(base>>2)+(issued_words<<7);assign arlen=15;
 wire launch=active&&launched<PIXELS&&ready_bank[read_bank]&&!pending&&(!out_valid||out_ready);
 wire [4:0] ra={read_bank,pos[5:2]},wa={fill_bank,beat};
 always @(posedge clk)begin
  if(launch)begin q0<=bank0[ra];q1<=bank1[ra];q2<=bank2[ra];q3<=bank3[ra];end
  if(loading&&rvalid)begin bank0[wa]<=rdata[63:0];bank1[wa]<=rdata[127:64];bank2[wa]<=rdata[191:128];bank3[wa]<=rdata[255:192];end
 end
 reg [63:0] selected;
 always @*case(lane)0:selected=q0;1:selected=q1;2:selected=q2;default:selected=q3;endcase
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin ready_bank<=0;fill_bank<=0;read_bank<=0;active<=0;loading<=0;requesting<=0;pending<=0;beat<=0;pos<=0;lane<=0;base<=0;issued<=0;launched<=0;out_valid<=0;descriptor<=0;error<=0;end
  else if(start)begin
   ready_bank<=0;fill_bank<=0;read_bank<=0;active<=1;loading<=0;requesting<=0;pending<=0;beat<=0;pos<=0;base<=map_base;issued<=0;launched<=0;out_valid<=0;error<=0;
  end else begin
   if(out_valid&&out_ready)out_valid<=0;
   if(active&&!loading&&!requesting&&issued<PIXELS/64&&!ready_bank[fill_bank])requesting<=1;
   if(arvalid&&arready)begin requesting<=0;loading<=1;beat<=0;end
   if(loading&&rvalid)begin
    if(rid!=0||rlast!=(beat==15))error<=1;
    beat<=beat+1'b1;
    if(rlast)begin loading<=0;ready_bank[fill_bank]<=1;fill_bank<=~fill_bank;issued<=issued+1;end
   end
   if(launch)begin
    pending<=1;lane<=pos[1:0];launched<=launched+1;pos<=pos+1'b1;
    if(pos==63)begin ready_bank[read_bank]<=0;read_bank<=~read_bank;end
   end
   if(pending&&(!out_valid||out_ready))begin descriptor<=error?{4'b0,selected[59:0]}:selected;out_valid<=1;pending<=0;end
  end
 end
endmodule
