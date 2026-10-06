`timescale 1ns/1ps
// Exercise mixed-latency multiply/hypot joins and reuse after rejected grids.
module tb_grid_validate_serial;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0,start=0;
    wire busy,done,valid_out,rd_en;
    wire [5:0] rd_addr;
    reg [31:0] rd_x=0,rd_y=0;
    wire [31:0] cost_out;
    reg [31:0] px[0:39],py[0:39];
    grid_validate dut(.clk(clk),.rst_n(rst_n),.start(start),.n_in(16'd40),
        .busy(busy),.done(done),.rd_en(rd_en),.rd_addr(rd_addr),
        .rd_x(rd_x),.rd_y(rd_y),.valid_out(valid_out),.cost_out(cost_out));
    always @(posedge clk) if(rd_en) begin rd_x<=px[rd_addr];rd_y<=py[rd_addr];end
    integer checked=0,cycles=0,requests=0,responses=0;
    reg finished=0;
    integer t,i;
    // Positive exact integers are enough for rectangular geometry fixtures.
    function [31:0] as_float;
        input integer n;
        integer k; reg [31:0] fraction;
        begin
            if(n==0)as_float=0;
            else begin
                k=0;while((n>>k)>1)k=k+1;
                fraction=n<<(23-k);
                as_float=((127+k)<<23)|(fraction & 32'h007fffff);
            end
        end
    endfunction
    reg [7:0] last_request_state=255;
    reg stage_has_request=0;
    always @(posedge clk) if(rst_n)begin
        // Each arithmetic phase must issue exactly once, despite long waits.
        if(dut.state!=last_request_state)stage_has_request=0;
        if(dut.any_request)begin
            if(stage_has_request)$fatal(1,"duplicate arithmetic request");
            stage_has_request=1;requests=requests+1;
        end
        last_request_state=dut.state;
        if(dut.stage_done)responses=responses+1;
    end
    initial begin
        repeat(4)@(negedge clk);rst_n=1;
        for(t=0;t<6;t=t+1)begin
            for(i=0;i<40;i=i+1)begin
                px[i]=as_float(16+(i%8)*8);
                py[i]=as_float(16+(i/8)*8);
            end
            // Early and late rejection followed immediately by a valid job.
            if(t==0)px[1]=as_float(17);
            if(t==2)px[38]=as_float(17);
            if(t==4)for(i=0;i<40;i=i+1)py[i]=as_float(16);
            @(negedge clk);start=1;@(negedge clk);start=0;
            cycles=0;
            while(!done)begin
                @(negedge clk);cycles=cycles+1;
                if(cycles>200000)$fatal(1,"grid timeout");
            end
            if(!valid_out || cost_out!==((t%2==0)?32'h7149f2ca:32'd0))
                $fatal(1,"grid case=%0d cost=%h",t,cost_out);
            if(requests!=responses)$fatal(1,"unconsumed arithmetic result");
            checked=checked+1;
            repeat(3)@(negedge clk);
        end
        finished=1;$display("PASS grid validate serial cases=%0d",checked);$finish;
    end
    initial begin #20000000;$fatal(1,"watchdog");end
endmodule
