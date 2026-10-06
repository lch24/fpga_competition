`timescale 1ns/1ps
module tb_tensor_shared #(parameter COUNT=604);
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,ce=1,start=0;
    wire busy,done,out_ok;
    reg [63:0] in_a,in_b,in_c,in_bx,in_by,expected_dx,expected_dy;
    reg [3:0] expected_ok;
    wire [63:0] out_dx,out_dy;
    reg [451:0] vectors[0:COUNT-1];
    tensor_solve #(.USE_CE(1)) dut(.*);
    integer ticks=0,checked=0,i,cycles,total_cycles=0;
    reg finished=0;
    always @(negedge clk)begin ticks=ticks+1;ce=(ticks%7!=0 && ticks%11!=0);end
    task launch;
        begin
            @(negedge clk);#1;start=1;
            @(posedge clk);while(!ce)@(posedge clk);
            @(negedge clk);#1;start=0;
        end
    endtask
    initial begin
        $readmemh("vectors.hex",vectors);
        repeat(4)@(negedge clk);#1;rst_n=1;
        {expected_ok,in_a,in_b,in_c,in_bx,in_by,expected_dx,expected_dy}=vectors[3];
        launch();repeat(10)@(negedge clk);#1;rst_n=0;
        repeat(2)@(negedge clk);#1;rst_n=1;
        if(done || busy)$fatal(1,"reset cancellation");
        for(i=0;i<COUNT;i=i+1)begin
            {expected_ok,in_a,in_b,in_c,in_bx,in_by,expected_dx,expected_dy}=vectors[i];
            launch();cycles=0;
            while(!done)begin @(negedge clk);#1;cycles=cycles+1;
                if(cycles>2000)$fatal(1,"timeout case %0d",i);end
            if(out_ok!==expected_ok[0] || busy)$fatal(1,"validity case %0d",i);
            if(out_ok && (out_dx!==expected_dx || out_dy!==expected_dy))
                $fatal(1,"case %0d got %h %h expected %h %h",i,out_dx,out_dy,expected_dx,expected_dy);
            checked=checked+1;total_cycles=total_cycles+cycles;
            repeat(3)@(negedge clk);
        end
        finished=1;$display("PASS tensor checked=%0d",checked);$finish;
    end
    initial begin #20000000;$fatal(1,"watchdog");end
endmodule
