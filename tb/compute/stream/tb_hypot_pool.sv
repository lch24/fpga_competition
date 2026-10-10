`timescale 1ns/1ps
module tb_hypot_pool;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,ce=1;
    reg [3:0] valid=0,ready=0;
    wire [3:0] input_ready,output_valid,qv,qr,rv,rr;
    wire [127:0] qa,qb;
    wire [31:0] result;
    wire [31:0] got[0:3];
    reg [31:0] av[0:3],bv[0:3],expected[0:3];
    genvar g;
    generate for(g=0;g<4;g=g+1)begin:clients
        hypot_port #(.SHARED(1),.USE_CE(1)) port(
            .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(valid[g]),.in_ready(input_ready[g]),
            .in_a(av[g]),.in_b(bv[g]),.out_valid(output_valid[g]),.out_ready(ready[g]),.out_r(got[g]),
            .math_hyp_valid(qv[g]),.math_hyp_ready(qr[g]),.math_hyp_a(qa[g*32+:32]),.math_hyp_b(qb[g*32+:32]),
            .math_hyp_rsp_valid(rv[g]),.math_hyp_rsp_ready(rr[g]),.math_hyp_result(result));
    end endgenerate
    hypot_pool #(.USE_CE(1)) pool(.clk(clk),.rst_n(rst_n),.ce(ce),
        .req_valid(qv),.req_ready(qr),.req_a(qa),.req_b(qb),
        .rsp_valid(rv),.rsp_ready(rr),.result(result));
    reg done=0;
    integer errors=0,checked=0,ticks=0,i,r,timeout,report;
    reg [3:0] received=0;
    always @(posedge clk)if(rst_n && ce)begin
        for(integer k=0;k<4;k=k+1)if(output_valid[k] && ready[k])begin
            if(received[k] || got[k]!==expected[k])begin
                errors=errors+1;$fdisplay(report,"FAIL owner=%0d got=%h expected=%h duplicate=%b",k,got[k],expected[k],received[k]);
            end
            received[k]=1;checked=checked+1;
        end
    end
    task reset;
        begin
            @(negedge clk);rst_n=0;ce=1;valid=0;ready=0;
            repeat(2)@(negedge clk);rst_n=1;received=0;
        end
    endtask
    initial begin
        report=$fopen("pool_results.txt","w");
        av[0]=32'h40400000;bv[0]=32'h40800000;expected[0]=32'h40a00000;
        av[1]=32'h40c00000;bv[1]=32'h41000000;expected[1]=32'h41200000;
        av[2]=32'h41100000;bv[2]=32'h41400000;expected[2]=32'h41700000;
        av[3]=32'h41400000;bv[3]=32'h41800000;expected[3]=32'h41a00000;
        reset;
        // Cancel a current owner together with three queued requests.
        valid=15;@(negedge clk);valid=0;repeat(15)@(negedge clk);reset;
        ready=15;repeat(160)@(negedge clk);
        if(checked!=0)errors=errors+1;
        for(r=0;r<40;r=r+1)begin
            ready=0;ce=1;received=0;
            if(input_ready!==15)errors=errors+1;
            valid=15;@(negedge clk);valid=0;timeout=0;
            // Every client starts together; uneven output stalls and CE.
            while(received!=15 && timeout<1200)begin
                ticks=ticks+1;ce=ticks%5!=0;
                ready={ticks%7!=0,ticks%11!=0,ticks%3!=0,ticks%13!=0};
                @(negedge clk);timeout=timeout+1;
            end
            if(received!=15)begin errors=errors+1;$fdisplay(report,"TIMEOUT round=%0d",r);end
            ready=0;ce=1;@(negedge clk);
        end
        $fdisplay(report,"POOL checked=%0d errors=%0d",checked,errors);$fclose(report);
        done=1;$stop;
    end
endmodule
