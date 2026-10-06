`timescale 1ns/1ps
module tb_reg_config_clock;
    reg clk=0;
    always #20.202020 clk=~clk; // actual PLL output: 24.75 MHz
    reg rst_n=0;
    wire cfg_clk,done,scl;
    tri1 sda;
    wire [8:0] index;
    reg previous=0;
    integer ticks=0,edges=0;
    realtime last_rise=0;
    reg finished=0;
    reg_config dut(.clk_25M(clk),.camera_rstn(rst_n),.initial_en(1'b0),
      .reg_conf_done(done),.i2c_sclk(scl),.i2c_sdat(sda),.clock_20k(cfg_clk),.reg_index(index));
    initial begin
        repeat(3) @(negedge clk);
        rst_n=1;
        repeat(7200) begin
            @(posedge clk); #1;
            ticks=ticks+1;
            if(cfg_clk!==previous) begin
                if(ticks!=1200) $fatal(1,"divider half-period ticks=%0d",ticks);
                ticks=0;edges=edges+1;previous=cfg_clk;
                if(cfg_clk) begin
                    if(last_rise!=0 && (($realtime-last_rise)>=100000 ||
                       ($realtime-last_rise)<96960)) $fatal(1,"incorrect full period");
                    last_rise=$realtime;
                end
            end
        end
        if(edges!=6) $fatal(1,"missing divider edges");
        @(negedge clk);rst_n=0;
        @(posedge clk);#1;
        if(cfg_clk!==0 || dut.clock_20k_cnt!==0) $fatal(1,"divider reset failed");
        finished=1;
        $display("PASS tb_reg_config_clock edges=%0d half_period=1200 full_period=2400",edges);
        $finish;
    end
    initial begin #400000; if(!finished) $fatal(1,"clock test timeout");end
endmodule
