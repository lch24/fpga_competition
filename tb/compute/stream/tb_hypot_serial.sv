`timescale 1ns/1ps
module tb_hypot_serial;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,ce=1,valid=0,ready=0;
    reg [31:0] a=0,b=0;
    wire in_ready,out_valid;wire [31:0] result;
    fp32_hypot #(.USE_CE(1)) dut(.clk(clk),.rst_n(rst_n),.ce(ce),
        .in_valid(valid),.in_ready(in_ready),.in_a(a),.in_b(b),
        .out_valid(out_valid),.out_ready(ready),.out_r(result));
    integer fd,report,fields,cases=0,errors=0,cycles=0,max_cycles=0;
    reg done=0; reg [31:0] expected,held,seed=32'hbc7f3109;
    task fail(input [511:0] message);
        begin errors=errors+1;$fdisplay(report,"FAIL case=%0d a=%h b=%h got=%h expected=%h %0s",cases,a,b,result,expected,message);$fflush(report);end
    endtask
    task reset;
        begin
            @(negedge clk);rst_n=0;valid=0;ready=0;ce=1;
            repeat(2)@(negedge clk);rst_n=1;
        end
    endtask
    task send;
        begin
            @(negedge clk);ce=1;valid=1;
            if(!in_ready)fail("not ready before request");
            @(negedge clk);valid=0;a=32'h7fc00000;b=32'hff800000;
        end
    endtask
    initial begin
        report=$fopen("results.txt","w");fd=$fopen("vectors.txt","r");
        if(!report || !fd)$fatal(1,"missing vectors/report");
        reset;
        // Cancel both active computation and a held result, then recover.
        a=32'h40400000;b=32'h40800000;send;repeat(12)@(negedge clk);reset;
        repeat(160)begin @(negedge clk);if(out_valid)fail("stale response after reset");end
        while(!$feof(fd))begin
            fields=$fscanf(fd,"%h %h %h\n",a,b,expected);
            if(fields==3)begin
                send;cycles=0;
                while(!out_valid && cycles<250)begin
                    seed={seed[30:0],seed[31]^seed[21]^seed[1]^seed[0]};ce=seed[0]|seed[4];
                    @(negedge clk);cycles=cycles+1;
                end
                if(!out_valid)fail("timeout");
                if(result!==expected)fail("numerical mismatch");
                if(cycles>max_cycles)max_cycles=cycles;
                held=result;
                repeat(3)begin @(negedge clk);if(!out_valid || result!==held)fail("backpressure unstable");end
                ce=0;ready=1;
                repeat(3)begin @(negedge clk);if(!out_valid || result!==held)fail("CE stall consumed response");end
                ce=1;@(negedge clk);ready=0;
                if(out_valid)fail("response not consumed");
                cases=cases+1;
            end
        end
        a=32'h40400000;b=32'h40800000;send;
        while(!out_valid)@(negedge clk);
        reset;
        repeat(160)begin @(negedge clk);if(out_valid)fail("held result survived reset");end
        $fdisplay(report,"HYPOT cases=%0d errors=%0d max_wall_cycles=%0d",cases,errors,max_cycles);
        $fclose(report);done=1;$stop;
    end
endmodule
