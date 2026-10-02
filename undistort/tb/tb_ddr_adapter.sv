`timescale 1ns/1ps
module tb_ddr_adapter;
 reg clk=0;always #5 clk=~clk;reg rst_n=0,ddr_ready=1;
 reg rd_valid=0,wr_valid=0,w_valid=0;wire rd_ready,wr_ready,w_ready;
 reg [31:0] rd_addr,rd_len,wr_addr,wr_len,w_data;reg [15:0] rd_tag,wr_tag;
 reg [3:0] w_keep;reg w_last;
 wire r_valid,b_valid;reg r_ready=0,b_ready=0;
 wire [31:0] r_data;wire [3:0] r_keep;wire [15:0] r_tag,b_tag;wire r_last,r_error,b_error;
 wire [27:0] axi_araddr,axi_awaddr;wire [3:0] axi_aruser_id,axi_awuser_id,axi_arlen,axi_awlen;
 wire axi_aruser_ap,axi_awuser_ap,axi_arvalid,axi_awvalid;
 reg axi_arready=0,axi_awready=0,axi_rvalid=0,axi_rlast=1,axi_wready=0,axi_wusero_last=0;
 reg [255:0] axi_rdata;wire [255:0] axi_wdata;wire [31:0] axi_wstrb;
 reg [3:0] axi_rid=0,axi_wusero_id=0;
 hmic_ddr_adapter dut(.*);
 reg [7:0] mem[0:255];
 integer ar_delay=-1,aw_delay=-1,done_delay=-1,block_addr,i,n,k;
 reg [255:0] saved_data;reg [31:0] saved_mask;
 reg [15:0] lfsr=16'hcafe;
 always @(posedge clk)if(rst_n)begin
  lfsr<={lfsr[14:0],lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
  axi_arready<=lfsr[0]&&ar_delay<0;axi_awready<=lfsr[1]&&aw_delay<0;
  axi_rvalid<=0;axi_wready<=0;axi_wusero_last<=0;
  if(axi_arvalid&&axi_arready)begin block_addr=axi_araddr*4;ar_delay<=3;end
  if(ar_delay>0)ar_delay<=ar_delay-1;
  if(ar_delay==0)begin
   for(k=0;k<32;k=k+1)axi_rdata[k*8+:8]<=mem[block_addr+k];
   axi_rvalid<=1;ar_delay<=-1;
  end
  if(axi_awvalid&&axi_awready)begin block_addr=axi_awaddr*4;saved_data<=axi_wdata;saved_mask<=axi_wstrb;aw_delay<=3;end
  if(aw_delay>0)aw_delay<=aw_delay-1;
  if(aw_delay==0)begin axi_wready<=1;aw_delay<=-1;end
  if(axi_wready)begin
   if(saved_data!==axi_wdata||saved_mask!==axi_wstrb)$fatal(1,"write payload not pre-staged");
   for(k=0;k<32;k=k+1)if(axi_wstrb[k])mem[block_addr+k]<=axi_wdata[k*8+:8];
   done_delay<=3;
  end
  if(done_delay>0)done_delay<=done_delay-1;
  if(done_delay==0)begin axi_wusero_last<=1;done_delay<=-1;end
  if(b_valid&&done_delay>=0)$fatal(1,"early write completion");
 end
 task write_bytes(input integer a,input integer len);
  begin
   @(negedge clk);wr_addr=a;wr_len=len;wr_tag=16'h1234;wr_valid=1;
   do @(posedge clk);while(!wr_ready);@(negedge clk);wr_valid=0;
   for(n=0;n<len;n=n+4)begin
    w_data=32'hdcba9876;w_keep=((len-n>=4)?15:((1<<(len-n))-1));w_last=n+4>=len;w_valid=1;
    do @(posedge clk);while(!w_ready);@(negedge clk);w_valid=0;
   end
   wait(b_valid);repeat(3)@(negedge clk);if(b_error||b_tag!=16'h1234)$fatal(1,"write response");
   b_ready=1;@(negedge clk);b_ready=0;
  end
 endtask
 task read_bytes(input integer a,input integer len);
  reg [31:0] expected;
  begin
   @(negedge clk);rd_addr=a;rd_len=len;rd_tag=16'h5678;rd_valid=1;
   do @(posedge clk);while(!rd_ready);@(negedge clk);rd_valid=0;
   for(n=0;n<len;n=n+4)begin
    wait(r_valid);expected=0;
    for(i=0;i<4;i=i+1)if(n+i<len)expected[i*8+:8]=mem[a+n+i];
    repeat(3)@(negedge clk);
    if(r_data!==expected||r_keep!=((len-n>=4)?15:((1<<(len-n))-1))||r_last!=(n+4>=len)||r_error||r_tag!=16'h5678)$fatal(1,"read mismatch at %d+%d got %h expected %h",a,n,r_data,expected);
    r_ready=1;@(negedge clk);r_ready=0;
   end
  end
 endtask
 initial begin
  for(i=0;i<256;i=i+1)mem[i]=i;
  repeat(4)@(negedge clk);rst_n=1;
  read_bytes(29,11);write_bytes(31,9);read_bytes(28,16);
  if(mem[30]!=30||mem[40]!=40)$fatal(1,"padding/outside bytes overwritten");
  write_bytes(63,2);read_bytes(62,5);
  $display("PASS DDR adapter: arbitrary byte offset, cross-block, multiword, keep, backpressure, write barrier");$finish;
 end
 initial begin #100000;$fatal(1,"adapter timeout");end
endmodule
