`timescale 1ns/1ps

module tb_dwt2d;

// parameters 
localparam CLK_PERIOD = 10;
localparam IMG_SIZE   = 512;
localparam LV3_SIZE = 64;

// DWT Signals 
reg         clk, rst_n;
reg  [7:0]  pixel_in;
reg         pixel_valid, row_start;
wire        done;
wire        lv1_out_valid;
wire [1:0]  lv1_subband;
wire [8:0]  lv1_row, lv1_col;
wire signed [9:0] lv1_data;
wire        lv3_out_valid;
wire [1:0]  lv3_subband;
wire [5:0]  lv3_row, lv3_col;
wire signed [11:0] lv3_data;

// DWT Instantiation
dwt2d_top #(.IMG_SIZE(IMG_SIZE)) dwt (
    .clk          (clk),
    .rst_n        (rst_n),
    .pixel_in     (pixel_in),
    .pixel_valid  (pixel_valid),
    .row_start    (row_start),
    .done         (done),
    .lv1_out_valid(lv1_out_valid),
    .lv1_subband  (lv1_subband),
    .lv1_row      (lv1_row),
    .lv1_col      (lv1_col),
    .lv1_data     (lv1_data),
    .lv3_out_valid (lv3_out_valid),
    .lv3_subband   (lv3_subband),
    .lv3_row       (lv3_row),
    .lv3_col       (lv3_col),
    .lv3_data      (lv3_data)
);

initial clk = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// Memory Storage 
reg [7:0] image [0:IMG_SIZE*IMG_SIZE-1];
reg signed [9:0] dut_ll3 [0:LV3_SIZE-1][0:LV3_SIZE-1];
reg signed [9:0] gold_ll3 [0:LV3_SIZE-1][0:LV3_SIZE-1];

integer i, j;
integer img_fd, gold_fd;

// Task: Load image (.raw to memory) 
task load_image;
    integer i;
    begin
        img_fd = $fopen("woman.raw", "rb");
        if (img_fd == 0) begin
            $display("WARNING: woman.raw not found, generating ramp pattern for testing...");
            for (i = 0; i < IMG_SIZE*IMG_SIZE; i = i + 1) begin
                image[i] = i % 256; 
            end
        end else begin
            $fread(image, img_fd);
            $fclose(img_fd);
            $display("Successfully loaded woman.raw");
        end
    end
endtask

// Task: Load MATLAB Golden Data 
task load_golden;
    integer r_val;
begin
    gold_fd = $fopen("C:/VLSIDSPHW2/fixed_DWT_project/golden_ll3.txt", "r");
    if (gold_fd == 0) begin
        $display("WARNING: golden_ll3.txt NOT found. Simulation will fail comparison.");
    end
    
    for (i=0; i<LV3_SIZE; i=i+1) begin
        for (j=0; j<LV3_SIZE; j=j+1) begin
            if ($fscanf(gold_fd, "%d", r_val) != 1) begin
                $display("ERROR: Unexpected EOF in golden_ll3.txt at (%0d,%0d)", i, j);
                $finish;
            end
            gold_ll3[i][j] = r_val[11:0];
        end
    end
    $fclose(gold_fd);
    $display("Golden data loaded successfully.");
end
endtask

// Task: Drive Pixel Stream 
task drive_image;
    integer row, col;
begin
    wait(rst_n == 1);
    @(posedge clk);
    for (row=0; row<IMG_SIZE; row=row+1) begin
        // Row start pulse
        row_start <= 1;
        @(posedge clk);
        row_start <= 0;
        
        // Transmit pixels
        for (col=0; col<IMG_SIZE; col=col+1) begin
            pixel_in    <= image[row*IMG_SIZE + col];
            pixel_valid <= 1;
            @(posedge clk);
        end
        pixel_valid <= 0;
        
        // Gap for horizontal pipeline processing
        repeat(10) @(posedge clk); 
    end
end
endtask

// Collect outputs 
always @(posedge clk) begin
    if (lv3_out_valid && lv3_subband == 2'b00) begin
        // 防止 index 越界
        if (lv3_row < LV3_SIZE && lv3_col < LV3_SIZE)
            dut_ll3[lv3_row][lv3_col] <= lv3_data;
    end
end

// Main Processing 
initial begin
    // Setup
    $dumpfile("dwt2d_sim.vcd");
    $dumpvars(0, tb_dwt2d);
    
    clk = 0;
    rst_n = 0;
    pixel_in = 0;
    pixel_valid = 0;
    row_start = 0;
    i = 0; j = 0; 
    
    load_image;
    load_golden;

    // Reset Sequence
    #(CLK_PERIOD * 10);
    rst_n = 1;
    #(CLK_PERIOD * 5);

    $display("--- Starting DWT 2D Processing ---");
    drive_image;

    // Wait for vertical engine to finish 
    fork
        begin : wait_done
            wait(done == 1);
            repeat(100) @(posedge clk); 
            $display("--- DWT Reported DONE ---");
        end
        begin : timeout
            repeat(IMG_SIZE * IMG_SIZE * 5) @(posedge clk);
            $display("ERROR: Simulation TIMEOUT!");
            $finish;
        end
    join

    compare_results;
    
    $display("Simulation Finished at %0t", $time);
    $finish;
end

// Comparison Task 
task compare_results;
    integer errs;
begin
    errs = 0;
    for (i=0; i<LV3_SIZE; i=i+1) begin
        for (j=0; j<LV3_SIZE; j=j+1) begin
            if (dut_ll3[i][j] !== gold_ll3[i][j]) begin
                if (errs < 20)
                    $display("Mismatch @ LL3(%0d,%0d): DUT=%d, Gold=%d", i, j, dut_ll3[i][j], gold_ll3[i][j]);
                errs = errs + 1;
            end
        end
    end
    
    if (errs == 0)
        $display("********** SUCCESS: All %0d pixels matched! **********", LV3_SIZE*LV3_SIZE);
    else
        $display("********** FAILURE: %0d mismatches found. **********", errs);
end
endtask

endmodule