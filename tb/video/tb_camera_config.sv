`timescale 1ns/1ps
// SCCB slave checks the actual SCL/SDA byte stream. NACK each of the four
// bytes in turn; the sequencer must retry that register and not report done.
module tb_camera_config;
 reg clk=0,slow=0,rst=0,slave_low=0,enable=0,slow_run=1;
 always #2 clk=~clk;
 always #20 if(slow_run)slow=~slow;
 tri1 sda;
 assign sda=slave_low?1'b0:1'bz;
 wire scl,done,cfg_clk;wire [8:0] index;
 reg_config dut(.clk_25M(clk),.camera_rstn(rst),.initial_en(enable),
  .reg_conf_done(done),.i2c_sclk(scl),.i2c_sdat(sda),.clock_20k(cfg_clk),.reg_index(index));
 integer transactions=0,byte_index,bit_index,polarity_writes=0,stream_writes=0;
 integer round_index,round_transactions=0;
 reg [31:0] word;
 reg finished=0;
 initial begin
  // Accelerate only the divided clock; production divider is tested separately.
  force dut.clock_20k=slow;
  forever begin
   @(negedge sda);
   if(scl===1'b1 && rst)begin
    if(!enable)$fatal(1,"SCCB started before post-reset settling completed");
    word=0;
    for(byte_index=0;byte_index<4;byte_index=byte_index+1)begin
     for(bit_index=0;bit_index<8;bit_index=bit_index+1)begin
      @(posedge scl);word={word[30:0],sda};
     end
     @(negedge scl);#1;
     slave_low=!(round_transactions<4 && byte_index==round_transactions);
     @(posedge scl);
     @(negedge scl);#1;slave_low=0;
    end
    if(round_transactions<=4 && word!==32'h78310311)$fatal(1,"NACK did not retry first register: %h",word);
    if(word==32'h78474020)polarity_writes=polarity_writes+1;
    if(word==32'h78300802)begin
     if(index!=358 || polarity_writes!=round_index+1)
      $fatal(1,"sensor started streaming before final geometry/polarity write");
     stream_writes=stream_writes+1;
    end
    transactions=transactions+1;
    round_transactions=round_transactions+1;
   end
  end
 end
 initial begin
  for(round_index=0;round_index<2;round_index=round_index+1)begin
   // Cold start and powered-sensor restart both wait for initial_en.
   repeat(5)@(negedge clk);rst=1;
   repeat(100)@(negedge slow);
   if(round_transactions!=0 || index!=0 || done || scl!==1 || sda!==1)
    $fatal(1,"configuration/bus changed during sensor settling");
   #3;enable=1;
   wait(done);#1;
   if(round_transactions!=363||polarity_writes!=round_index+1||stream_writes!=round_index+1||index!=359)
    $fatal(1,"configuration completion transactions=%0d polarity=%0d stream=%0d index=%0d",round_transactions,polarity_writes,stream_writes,index);
   // No SCCB clock is available during this reset. Outputs must still reset.
   @(negedge slow);slow_run=0;#3;rst=0;enable=0;#1;
   if(done!==0 || index!==0 || scl!==1 || sda!==1 || dut.u1.cyc_count!==63)
    $fatal(1,"stopped SCCB clock prevented reset or bus release");
   round_transactions=0;slow_run=1;
  end
  finished=1;$display("PASS camera config: settling, four NACK retries, standby through final write, warm restart and stopped-clock reset");$finish;
 end
 initial begin #2000000;$fatal(1,"camera config timeout index=%0d transactions=%0d",index,transactions);end
endmodule
