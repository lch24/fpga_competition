`timescale 1ns/1ps
// Exercise mixed-latency multiply/hypot joins and reuse after rejected grids.
module tb_grid_validate_serial;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0,start=0,ce=1; integer tick=0;
    always @(negedge clk)begin tick=tick+1;ce=(tick%9!=2 && tick%9!=3);end
    wire busy,done,valid_out,rd_en;
    wire [5:0] rd_addr;
    reg [31:0] rd_x=0,rd_y=0;
    wire [31:0] cost_out;
    reg [31:0] px[0:39],py[0:39];
    grid_validate #(.USE_CE(1)) dut(.ce(ce),.clk(clk),.rst_n(rst_n),.start(start),.n_in(16'd40),
        .busy(busy),.done(done),.rd_en(rd_en),.rd_addr(rd_addr),
        .rd_x(rd_x),.rd_y(rd_y),.valid_out(valid_out),.cost_out(cost_out));
    always @(posedge clk) if(ce && rd_en) begin rd_x<=px[rd_addr];rd_y<=py[rd_addr];end
    integer checked=0,cycles=0,requests=0,responses=0;
    reg finished=0;
    integer t,i,fd,rc;
    reg [63:0] expected_bits;
    real expected_cost,actual_cost,error;
    function real value32(input [31:0] bits);
        reg [63:0] wide;reg [10:0] e;
        begin e=bits[30:23]+11'd896;wide={bits[31],e,bits[22:0],29'b0};
          value32=bits[30:0]==0?0:$bitstoreal(wide);end
    endfunction
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
    always @(posedge clk) if(rst_n)begin
        if(dut.any_request)requests=requests+1;
        if(dut.stage_done)responses=responses+1;
        if(responses>requests || requests-responses>1)$fatal(1,"arithmetic transaction accounting");
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
            @(negedge clk);start=1;@(posedge clk);while(!ce)@(posedge clk);@(negedge clk);start=0;
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
        fd=$fopen("../../data/fixtures/squared_grids.txt","r");if(!fd)$fatal(1,"grid fixtures missing");
        for(t=0;t<32;t=t+1)begin
            rc=$fscanf(fd,"%h",expected_bits);if(rc!=1)$fatal(1,"grid expected format");
            for(i=0;i<40;i=i+1)begin rc=$fscanf(fd,"%h %h",px[i],py[i]);if(rc!=2)$fatal(1,"grid point format");end
            @(negedge clk);start=1;@(posedge clk);while(!ce)@(posedge clk);@(negedge clk);start=0;
            cycles=0;
            while(!done)begin @(negedge clk);cycles=cycles+1;if(cycles>200000)$fatal(1,"grid timeout");end
            expected_cost=$bitstoreal(expected_bits);actual_cost=value32(cost_out);
            error=actual_cost-expected_cost;if(error<0)error=-error;
            if(!valid_out || error>1e-4*(1+expected_cost))$fatal(1,"grid numeric fixture=%0d",t);
            if(requests!=responses)$fatal(1,"pending arithmetic");checked=checked+1;
            repeat(3)@(negedge clk);
        end
        $fclose(fd);
        finished=1;$display("PASS grid validate serial cases=%0d",checked);$finish;
    end
    initial begin #20000000;$fatal(1,"watchdog");end
endmodule
