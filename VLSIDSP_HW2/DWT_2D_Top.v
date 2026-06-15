module DWT_2D_Top (
    input  wire        clk,
    input  wire        rst_n,
    
    // 輸入端：來自 Image Buffer 或 Testbench
    input  wire        valid_in,
    input  wire [7:0]  pixel_in,
    
    // 輸出端：最終四頻帶結果
    output wire        valid_out,
    output wire signed [10:0] LL, // WL_ver = 11
    output wire signed [10:0] LH,
    output wire signed [10:0] HL,
    output wire signed [10:0] HH
);

    // ==========================================
    // 1. 內部連線宣告 (Wires)
    // ==========================================
    
    // 水平濾波器 -> 轉置記憶體
    wire        hor_to_mem_valid;
    wire signed [9:0] hor_L; // WL_hor = 10
    wire signed [9:0] hor_H;
    
    // 轉置記憶體 -> 垂直濾波器 (9 Taps)
    wire        mem_to_ver_valid;
    wire signed [9:0] L_taps [0:8];
    wire signed [9:0] H_taps [0:8];

    // ==========================================
    // 2. 實作水平濾波器 (Stage 1)
    // ==========================================
    Filter_1D_Hor u_Filter_1D_Hor (
        .clk      (clk),
        .rst_n    (rst_n),
        .valid_in (valid_in),
        .pixel_in (pixel_in),
        
        .valid_out(hor_to_mem_valid),
        .L_out    (hor_L),
        .H_out    (hor_H)
    );

    // ==========================================
    // 3. 實作轉置記憶體 / Line Buffer (Stage 2)
    // ==========================================
    Transpose_Memory u_Transpose_Memory (
        .clk      (clk),
        .rst_n    (rst_n),
        .valid_in (hor_to_mem_valid),
        .L_in     (hor_L),
        .H_in     (hor_H),
        
        .valid_out(mem_to_ver_valid),
        
        // 將 9 個 Tap 依序接出
        .L_tap0(L_taps[0]), .L_tap1(L_taps[1]), .L_tap2(L_taps[2]), 
        .L_tap3(L_taps[3]), .L_tap4(L_taps[4]), .L_tap5(L_taps[5]), 
        .L_tap6(L_taps[6]), .L_tap7(L_taps[7]), .L_tap8(L_taps[8]),
        
        .H_tap0(H_taps[0]), .H_tap1(H_taps[1]), .H_tap2(H_taps[2]), 
        .H_tap3(H_taps[3]), .H_tap4(H_taps[4]), .H_tap5(H_taps[5]), 
        .H_tap6(H_taps[6]), .H_tap7(H_taps[7]), .H_tap8(H_taps[8])
    );

    // ==========================================
    // 4. 實作垂直濾波器 (Stage 3)
    // ==========================================
    Filter_1D_Ver u_Filter_1D_Ver (
        .clk      (clk),
        .rst_n    (rst_n),
        .valid_in (mem_to_ver_valid),
        
        // 接收來自記憶體的 9 個垂直像素
        .L_tap0(L_taps[0]), .L_tap1(L_taps[1]), .L_tap2(L_taps[2]), 
        .L_tap3(L_taps[3]), .L_tap4(L_taps[4]), .L_tap5(L_taps[5]), 
        .L_tap6(L_taps[6]), .L_tap7(L_taps[7]), .L_tap8(L_taps[8]),
        
        .H_tap0(H_taps[0]), .H_tap1(H_taps[1]), .H_tap2(H_taps[2]), 
        .H_tap3(H_taps[3]), .H_tap4(H_taps[4]), .H_tap5(H_taps[5]), 
        .H_tap6(H_taps[6]), .H_tap7(H_taps[7]), .H_tap8(H_taps[8]),
        
        .valid_out(valid_out),
        .LL_out   (LL),
        .LH_out   (LH),
        .HL_out   (HL),
        .HH_out   (HH)
    );

endmodule