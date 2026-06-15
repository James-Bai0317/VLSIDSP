`timescale 1ns / 1ps

module tb_DWT_2D;

    // ==========================================
    // 1. 參數與訊號宣告
    // ==========================================
    parameter IMG_W = 512;
    parameter IMG_H = 512;
    parameter TOTAL_PIXELS = IMG_W * IMG_H;

    reg         clk;
    reg         rst_n;
    reg         valid_in;
    reg  [7:0]  pixel_in;

    wire        valid_out;
    wire signed [10:0] LL, LH, HL, HH;

    // 影像記憶體 (用來模擬存放 Test Pattern)
    reg [7:0] img_mem [0:TOTAL_PIXELS-1];
    integer i;

    // 檔案指標
    integer f_out_LL;

    // ==========================================
    // 2. 實體化 (Instantiate) 你的頂層模組
    // ==========================================
    DWT_2D_Top u_DUT (
        .clk      (clk),
        .rst_n    (rst_n),
        .valid_in (valid_in),
        .pixel_in (pixel_in),
        .valid_out(valid_out),
        .LL       (LL),
        .LH       (LH),
        .HL       (HL),
        .HH       (HH)
    );

    // ==========================================
    // 3. Clock 產生器 (100MHz)
    // ==========================================
    always #5 clk = ~clk;

    // ==========================================
    // 4. 主測試流程 (Main Stimulus)
    // ==========================================
    initial begin
        // 初始化訊號
        clk = 0;
        rst_n = 0;
        valid_in = 0;
        pixel_in = 0;

        // 讀取 Python 產生的影像文字檔 (確保檔案放在與 TB 同一個目錄)
        $readmemh("input_image.txt", img_mem);
        
        // 開啟輸出檔案，準備記錄硬體算出來的結果
        f_out_LL = $fopen("hardware_out_LL.txt", "w");

        // 釋放 Reset
        #20 rst_n = 1;
        #10;

        $display("--- 開始影像資料輸入 (Simulation Start) ---");

        // 將 512x512 的像素一個一個推進硬體
        for (i = 0; i < TOTAL_PIXELS; i = i + 1) begin
            @(negedge clk);
            valid_in = 1'b1;
            pixel_in = img_mem[i];
        end

        // 影像傳輸完畢
        @(negedge clk);
        valid_in = 1'b0;
        
        // 等待管線 (Pipeline) 處理完剩下的資料
        // 因為 Line Buffer 有 9 行的延遲，需要等一陣子
        #20000; 

        $display("--- 模擬結束 (Simulation End) ---");
        $fclose(f_out_LL);
        $finish;
    end

    // ==========================================
    // 5. 監控輸出與寫入檔案 (Output Monitor)
    // ==========================================
    always @(posedge clk) begin
        if (valid_out) begin
            // 當硬體吐出有效資料時，將 LL 頻帶的結果寫入文字檔 (存為十進位)
            // 你可以根據需要把 LH, HL, HH 也存下來
            $fdisplay(f_out_LL, "%d", LL);
        end
    end
    initial
    begin
        $dumpfile("dwt_wave.vcd");  // 指定要產生的波形檔名稱
        $dumpvars(0, tb_DWT_2D);    // 紀錄 tb_DWT_2D 裡面所有的訊號變化
    end
endmodule