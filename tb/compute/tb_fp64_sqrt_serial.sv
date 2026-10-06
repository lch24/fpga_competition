`timescale 1ns/1ps
module tb_fp64_sqrt_serial;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0, in_valid=0, out_ready=0;
    reg [63:0] in_x=0;
    wire in_ready,out_valid; wire [63:0] out_r;
    fp64_sqrt dut(.ce(1'b1),.*);
    integer checked=0, resets=0, errors=0;
    reg finished=0;
    integer fd, scan, cycles, stall, j;
    reg [63:0] x, expected;
    task reset_core;
        begin
            @(negedge clk); rst_n=0; in_valid=0; out_ready=0;
            #1; if(out_valid || in_ready) $fatal(1,"reset handshake");
            repeat(2) @(negedge clk);
            rst_n=1;
            #1; if(out_valid || !in_ready) $fatal(1,"reset state");
            resets=resets+1;
        end
    endtask
    task send;
        input [63:0] value;
        begin
            @(negedge clk); in_valid=1; in_x=value;
            do @(posedge clk); while(!in_ready);
            @(negedge clk); in_valid=0;
        end
    endtask
    initial begin
        reset_core();
        // Cancel at beginning, middle, last root step and held response.
        for(j=0;j<4;j=j+1) begin
            send(64'h4000000000000000);
            case(j)
                0: repeat(1) @(negedge clk);
                1: repeat(27) @(negedge clk);
                2: repeat(55) @(negedge clk);
                3: repeat(60) @(negedge clk);
            endcase
            reset_core();
            repeat(60) begin
                @(negedge clk);
                if(out_valid) $fatal(1,"cancelled response leaked");
            end
        end
        fd=$fopen("../../data/system/sqrt_vectors.txt","r");
        if(!fd) $fatal(1,"missing vectors");
        while(!$feof(fd)) begin
            scan=$fscanf(fd,"%h %h\n",x,expected);
            if(scan==2) begin
                send(x);
                cycles=0;
                // Hold a prospective next request while busy. It must not be accepted.
                in_valid=1; in_x=~x;
                while(!out_valid) begin
                    if(in_ready) $fatal(1,"accepted while busy");
                    @(negedge clk); cycles=cycles+1;
                    if(cycles>56) $fatal(1,"latency timeout");
                end
                if(cycles!=56 || out_r!==expected)
                    $fatal(1,"sqrt x=%h actual=%h expected=%h cycles=%0d",x,out_r,expected,cycles);
                stall=checked%13;
                repeat(stall) begin
                    @(negedge clk);
                    if(!out_valid || out_r!==expected || in_ready) $fatal(1,"held response changed");
                end
                in_valid=0; out_ready=1;
                @(negedge clk); out_ready=0;
                if(out_valid || !in_ready) $fatal(1,"response not consumed");
                checked=checked+1;
            end
        end
        $fclose(fd);
        finished=1; $display("PASS sqrt checked=%0d resets=%0d",checked,resets); $finish;
    end
    initial begin #10000000; $fatal(1,"watchdog"); end
endmodule
