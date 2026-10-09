`timescale 1ns/1ps
module tb_integer_detection;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0,ce=1,hv=0,hr=0,rv=0,rr=0,done=0;
    wire hi,ho,ri,ro,rpass,rd_en;
    reg [31:0] a,b,c,x,y,radius;
    wire [31:0] response;
    wire [11:0] rd_addr;
    reg [7:0] mem[0:4095],rd_data;
    integer tick=0,fd,rc,i,n=0,rings=0,expected_pass,timeout;
    reg [31:0] expected,held;
    harris_response #(.USE_CE(1)) h(.clk(clk),.rst_n(rst_n),.ce(ce),
        .in_valid(hv),.in_ready(hi),.in_a(a),.in_b(b),.in_c(c),
        .out_valid(ho),.out_ready(hr),.out_resp(response));
    ring_check #(.USE_CE(1),.W(64),.H(64),.GRAY_ADDR_W(12)) r(
        .clk(clk),.rst_n(rst_n),.ce(ce),.cfg_scale(3'd0),
        .in_valid(rv),.in_ready(ri),.in_x(x),.in_y(y),.in_radius(radius),
        .rd_en(rd_en),.rd_addr(rd_addr),.rd_data(rd_data),
        .out_valid(ro),.out_ready(rr),.out_pass(rpass));
    always @(posedge clk) if(ce && rd_en) rd_data<=mem[rd_addr];
    always @(negedge clk) begin tick=tick+1;ce=(tick%7!=2 && tick%7!=3);end
    task edge_enabled;begin @(posedge clk);while(!ce)@(posedge clk);#1;end endtask
    initial begin
        repeat(4)@(negedge clk);rst_n=1;
        fd=$fopen("harris.txt","r");if(!fd)$fatal(1,"harris file");
        while(!$feof(fd))begin
            rc=$fscanf(fd,"%h %h %h %h\n",a,b,c,expected);
            if(rc==4)begin
                #1;hv=1;edge_enabled();hv=0;timeout=0;
                while(!ho && timeout<100)begin edge_enabled();timeout=timeout+1;end
                if(!ho || response!==expected)$fatal(1,"Harris case %0d got %h expected %h",n,response,expected);
                held=response;repeat(5)begin edge_enabled();if(!ho || response!==held)$fatal(1,"Harris stall");end
                hr=1;edge_enabled();hr=0;n=n+1;
            end
        end
        $fclose(fd);fd=$fopen("rings.txt","r");if(!fd)$fatal(1,"ring file");
        while(!$feof(fd))begin
            rc=$fscanf(fd,"%h %h %h %d\n",x,y,radius,expected_pass);
            if(rc==4)begin
                for(i=0;i<4096;i=i+1)rc=$fscanf(fd,"%h",mem[i]);
                #1;rv=1;edge_enabled();rv=0;timeout=0;
                while(!ro && timeout<400)begin edge_enabled();timeout=timeout+1;end
                if(!ro || rpass!==expected_pass[0])$fatal(1,"Ring case %0d got %b expected %d",rings,rpass,expected_pass);
                repeat(5)begin edge_enabled();if(!ro || rpass!==expected_pass[0])$fatal(1,"ring stall");end
                rr=1;edge_enabled();rr=0;rings=rings+1;
            end
        end
        $fclose(fd);
        // Reset wins even when the external DDR clock enable is low.
        hv=1;edge_enabled();hv=0;rst_n=0;@(posedge clk);#1;
        if(ho || ro)$fatal(1,"reset leaked response");
        done=1;$display("PASS integer detection tensors=%0d rings=%0d",n,rings);$finish;
    end
    initial begin #50000000;$fatal(1,"watchdog");end
endmodule
