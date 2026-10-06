`timescale 1ns/1ps
module tb_bilinear_fixed #(parameter COUNT=5324);
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,ce=1,in_valid=0,out_ready=0;
    wire in_ready,out_valid;
    reg [7:0] p00,p10,p01,p11;
    reg [31:0] dx,dy;
    wire [31:0] out_r;
    reg [127:0] vectors[0:COUNT-1];
    reg [31:0] expected;
    bilinear_u8_q20 #(.USE_CE(1)) dut(.*);
    integer ticks=0,checked=0,i,wait_cycles;
    reg finished=0;
    always @(negedge clk) begin ticks=ticks+1;ce=(ticks%5!=0 && ticks%13!=0);end
    task send;
        begin
            @(negedge clk);#1;in_valid=1;
            @(posedge clk);while(!ce || !in_ready)@(posedge clk);
            @(negedge clk);#1;in_valid=0;
        end
    endtask
    initial begin
        $readmemh("vectors.hex",vectors);
        repeat(4)@(negedge clk);#1;rst_n=1;
        // Cancel during multiplication; a new operation must see no stale result.
        {p00,p10,p01,p11,dx,dy,expected}=vectors[1];
        send();repeat(3)@(negedge clk);#1;rst_n=0;
        repeat(2)@(negedge clk);#1;rst_n=1;
        if(out_valid)$fatal(1,"reset did not cancel");
        for(i=0;i<COUNT;i=i+1)begin
            {p00,p10,p01,p11,dx,dy,expected}=vectors[i];
            send();wait_cycles=0;
            while(!out_valid)begin @(negedge clk);#1;wait_cycles=wait_cycles+1;
                if(wait_cycles>100)$fatal(1,"timeout %0d",i);end
            repeat(i%4+1)begin
                @(negedge clk);#1;
                if(!out_valid || out_r!==expected || in_ready)$fatal(1,"case %0d got %h expected %h",i,out_r,expected);
            end
            out_ready=1;@(posedge clk);while(!ce)@(posedge clk);
            @(negedge clk);#1;out_ready=0;checked=checked+1;
        end
        finished=1;$display("PASS bilinear vectors=%0d",checked);$finish;
    end
    initial begin #20000000;$fatal(1,"watchdog");end
endmodule
