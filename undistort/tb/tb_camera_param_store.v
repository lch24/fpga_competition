`timescale 1ns/1ps

module tb_camera_param_store;

    reg                 clk;
    reg                 rst_n;
    reg                 s_param_valid;
    wire                s_param_ready;
    reg        [31:0]   s_fx;
    reg        [31:0]   s_fy;
    reg        [31:0]   s_cx;
    reg        [31:0]   s_cy;
    reg        [31:0]   s_k1;
    reg        [31:0]   s_k2;
    reg        [31:0]   s_p1;
    reg        [31:0]   s_p2;
    reg        [31:0]   s_k3;
    reg        [15:0]   s_calib_width;
    reg        [15:0]   s_calib_height;
    reg        [31:0]   s_calib_id;
    reg        [63:0]   s_rms_error;
    reg                 map_busy;

    wire                active_valid;
    wire                active_update;
    wire        [31:0]  active_fx;
    wire        [31:0]  active_fy;
    wire        [31:0]  active_cx;
    wire        [31:0]  active_cy;
    wire        [31:0]  active_k1;
    wire        [31:0]  active_k2;
    wire        [31:0]  active_p1;
    wire        [31:0]  active_p2;
    wire        [31:0]  active_k3;
    wire        [15:0]  active_calib_width;
    wire        [15:0]  active_calib_height;
    wire        [31:0]  active_calib_id;
    wire        [63:0]  active_rms_error;
    wire                shadow_pending;

    camera_param_store dut (
        .clk                  (clk),
        .rst_n                (rst_n),
        .s_param_valid        (s_param_valid),
        .s_param_ready        (s_param_ready),
        .s_fx                 (s_fx),
        .s_fy                 (s_fy),
        .s_cx                 (s_cx),
        .s_cy                 (s_cy),
        .s_k1                 (s_k1),
        .s_k2                 (s_k2),
        .s_p1                 (s_p1),
        .s_p2                 (s_p2),
        .s_k3                 (s_k3),
        .s_calib_width        (s_calib_width),
        .s_calib_height       (s_calib_height),
        .s_calib_id           (s_calib_id),
        .s_rms_error          (s_rms_error),
        .map_busy             (map_busy),
        .active_valid         (active_valid),
        .active_update        (active_update),
        .active_fx            (active_fx),
        .active_fy            (active_fy),
        .active_cx            (active_cx),
        .active_cy            (active_cy),
        .active_k1            (active_k1),
        .active_k2            (active_k2),
        .active_p1            (active_p1),
        .active_p2            (active_p2),
        .active_k3            (active_k3),
        .active_calib_width   (active_calib_width),
        .active_calib_height  (active_calib_height),
        .active_calib_id      (active_calib_id),
        .active_rms_error     (active_rms_error),
        .shadow_pending       (shadow_pending)
    );

    always #5 clk = ~clk;

    task fail;
        input [8*80-1:0] message;
        begin
            $display("FAIL: %0s", message);
            $finish;
        end
    endtask

    initial begin
        clk            = 1'b0;
        rst_n          = 1'b0;
        s_param_valid  = 1'b0;
        map_busy       = 1'b0;
        s_fx           = 32'sd0;
        s_fy           = 32'sd0;
        s_cx           = 32'sd0;
        s_cy           = 32'sd0;
        s_k1           = 32'sd0;
        s_k2           = 32'sd0;
        s_p1           = 32'sd0;
        s_p2           = 32'sd0;
        s_k3           = 32'sd0;
        s_calib_width  = 16'd0;
        s_calib_height = 16'd0;
        s_calib_id     = 32'd0;
        s_rms_error    = 64'd0;

        repeat (3) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        // First result: accepted and committed while the mapper is idle.
        s_fx           = 32'h44ab_ed92;
        s_fy           = 32'h44ab_52f3;
        s_cx           = 32'h4425_794f;
        s_cy           = 32'h4351_2f4f;
        s_k1           = 32'hbe82_16e0;
        s_k2           = 32'h3d27_a3d3;
        s_p1           = 32'h3c27_14ed;
        s_p2           = 32'hba3f_1f0f;
        s_k3           = 32'h0000_0000;
        s_calib_width  = 16'd1280;
        s_calib_height = 16'd720;
        s_calib_id     = 32'd1;
        s_rms_error    = 64'h3fe1_5cdc_092a_cc06;
        s_param_valid  = 1'b1;

        while (!s_param_ready)
            @(posedge clk);
        @(posedge clk);
        @(negedge clk);
        s_param_valid = 1'b0;

        wait (active_update);
        #1;
        if (!active_valid)
            fail("active_valid was not set");
        if (active_calib_id != 32'd1 || active_calib_width != 16'd1280 ||
            active_calib_height != 16'd720)
            fail("first metadata set was not committed atomically");
        if (active_fx != 32'h44ab_ed92 || active_k1 != 32'hbe82_16e0)
            fail("first parameter set is incorrect");

        @(negedge clk);

        // Second result arrives while mapping is busy. It must remain in the
        // shadow bank and must not modify the active bank.
        map_busy       = 1'b1;
        s_fx           = 32'h447a_0000;
        s_fy           = 32'h447a_0000;
        s_cx           = 32'h43fa_0000;
        s_cy           = 32'h438c_a000;
        s_k1           = 32'hbe4c_cccd;
        s_k2           = 32'h3ca3_d70a;
        s_p1           = 32'h0000_0000;
        s_p2           = 32'h0000_0000;
        s_k3           = 32'h0000_0000;
        s_calib_width  = 16'd1280;
        s_calib_height = 16'd720;
        s_calib_id     = 32'd2;
        s_rms_error    = 64'h3fe0_0000_0000_0000;
        s_param_valid  = 1'b1;

        while (!s_param_ready)
            @(posedge clk);
        @(posedge clk);
        @(negedge clk);
        s_param_valid = 1'b0;
        @(posedge clk);
        #1;

        if (!shadow_pending)
            fail("second parameter set was not held in the shadow bank");
        if (active_calib_id != 32'd1)
            fail("active parameters changed while map_busy was high");

        // Releasing map_busy allows an atomic commit on the next clock.
        map_busy = 1'b0;
        wait (active_update);
        #1;
        if (active_calib_id != 32'd2 || active_fx != 32'h447a_0000)
            fail("second parameter set was not committed");
        if (shadow_pending)
            fail("shadow_pending did not clear after commit");

        $display("PASS: camera_param_store handshake and atomic commit");
        $finish;
    end

endmodule
