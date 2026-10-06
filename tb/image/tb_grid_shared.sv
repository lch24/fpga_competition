`timescale 1ns/1ps
// Golden costs were captured from the pre-sharing validator. Fixtures include
// regular/sheared/curved grids and early/late rejection, reused without reset.
module tb_grid_shared #(parameter CAPTURE=0);
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,ce=1,start=0;
    wire busy,done,valid_out,rd_en;
    wire [5:0] rd_addr;
    reg [31:0] rd_x=0,rd_y=0;
    wire [31:0] cost_out;
    reg [31:0] px[0:39],py[0:39],expected[0:47];
    grid_validate #(.USE_CE(1)) dut(.*,.n_in(16'd40));
    always @(posedge clk) if(ce && rd_en) begin rd_x<=px[rd_addr];rd_y<=py[rd_addr];end
    integer ticks=0,checked=0,cycles=0,total_cycles=0,t,i,fd;
    reg finished=0;
    always @(negedge clk) begin ticks=ticks+1;ce=(ticks%7!=0 && ticks%11!=0);end
    function [31:0] as_float(input integer n);
        integer k;reg [31:0] fraction;
        begin
            if(n==0)as_float=0;
            else begin k=0;while((n>>k)>1)k=k+1;
                fraction=n<<(23-k);as_float=((127+k)<<23)|(fraction & 32'h007fffff);end
        end
    endfunction
    initial begin
        if(CAPTURE) fd=$fopen("../../data/system/grid_shared_costs.hex","w");
        else $readmemh("../../data/system/grid_shared_costs.hex",expected);
        repeat(4)@(negedge clk);rst_n=1;
        for(t=0;t<48;t=t+1)begin
            for(i=0;i<40;i=i+1)begin
                px[i]=as_float(64+(i%8)*(8+t%5)+(i/8)*(t%3)+(t%4==1?(i%8)*(i%8)/7:0));
                py[i]=as_float(64+(i/8)*(8+t%7)+(i%8)*(t%2)+(t%4==2?(i/8)*(i/8)/3:0));
            end
            if(t%8==0)px[1]=as_float(65);
            if(t%8==3)px[38]=as_float(64);
            if(t%8==6)for(i=0;i<40;i=i+1)py[i]=as_float(64);
            @(negedge clk);#1;start=1;
            @(posedge clk);while(!ce)@(posedge clk);
            @(negedge clk);#1;start=0;cycles=0;
            while(!done)begin @(negedge clk);#1;cycles=cycles+1;
                if(cycles>400000)$fatal(1,"grid timeout case %0d",t);end
            if(!valid_out)$fatal(1,"missing grid result");
            if(CAPTURE)$fdisplay(fd,"%08h",cost_out);
            else if(cost_out!==expected[t])$fatal(1,"case %0d got %h expected %h",t,cost_out,expected[t]);
            checked=checked+1;total_cycles=total_cycles+cycles;
            repeat(4)@(negedge clk);
        end
        if(CAPTURE)$fclose(fd);
        finished=1;$display("PASS grid shared checked=%0d cycles=%0d",checked,total_cycles);$finish;
    end
    initial begin #200000000;$fatal(1,"watchdog");end
endmodule
