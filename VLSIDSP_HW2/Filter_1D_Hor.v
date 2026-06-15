module Filter_1D_Hor (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,    // 上游給資料的訊號
    input  wire [7:0]  pixel_in,    // 原始影像像素 (0~255)
    
    output reg         valid_out,   // 告訴下游(Line Buffer)資料準備好了
    output reg  signed [9:0] L_out, // 10-bit 水平低通輸出 (WL_hor=10)
    output reg  signed [9:0] H_out  // 10-bit 水平高通輸出 (WL_hor=10)
);

    // ==========================================
    // 1. 定義量化後的濾波器係數 (WL=9, WT=2^-7 -> 乘上 128 取整數)
    // ==========================================
    // Lowpass (h)
    localparam signed [8:0] H0  = 9'sd109;  // 0.8526 * 128
    localparam signed [8:0] H1  = 9'sd48;   // 0.3774 * 128
    localparam signed [8:0] H2  = -9'sd14;  // -0.1106 * 128
    localparam signed [8:0] H3  = -9'sd3;   // -0.0238 * 128
    localparam signed [8:0] H4  = 9'sd5;    // 0.0378 * 128

    // Highpass (g)
    localparam signed [8:0] G0  = -9'sd101; // -0.7884 * 128
    localparam signed [8:0] G1  = 9'sd54;   // 0.4180 * 128
    localparam signed [8:0] G2  = 9'sd5;    // 0.0406 * 128
    localparam signed [8:0] G3  = -9'sd8;   // -0.0645 * 128
    // G4 為 0 (CDF 9/7 高通只有 7 個 tap)

    // ==========================================
    // 2. 移位暫存器 (Shift Register) - 長度 9
    // ==========================================
    // 將 8-bit unsigned 轉為 10-bit signed 以防止運算過程中溢位
    reg signed [9:0] sr [0:8];
    integer i;

    // ==========================================
    // 3. 降採樣計數器 (Downsample Counter)
    // ==========================================
    reg cnt; // 0 或 1，每吃兩個 pixel 才觸發一次運算

    // ==========================================
    // 4. 乘加器管線 (MAC) 與控制邏輯
    // ==========================================
    // 宣告內部的大型運算線路 (保留足夠的 bit 數防止溢位)
    wire signed [20:0] mac_L;
    wire signed [20:0] mac_L_rounded;
    wire signed [20:0] mac_H;
    wire signed [20:0] mac_H_rounded;

    // Lowpass 摺積運算 (對稱相加)
    assign mac_L = (sr[4] * H0) + 
                   ((sr[3] + sr[5]) * H1) + 
                   ((sr[2] + sr[6]) * H2) + 
                   ((sr[1] + sr[7]) * H3) + 
                   ((sr[0] + sr[8]) * H4);

    // Highpass 摺積運算 (對稱相加)
    assign mac_H = (sr[4] * G0) + 
                   ((sr[3] + sr[5]) * G1) + 
                   ((sr[2] + sr[6]) * G2) + 
                   ((sr[1] + sr[7]) * G3);

    // 加上 64 (即 2^6) 進行四捨五入，然後向右 Shift 7 位 (因為 WT=2^-7)
    // 這樣出來的結果就是純整數 (WT_hor = 1)
    assign mac_L_rounded = (mac_L + 21'sd64) >>> 7;
    assign mac_H_rounded = (mac_H + 21'sd64) >>> 7;

    // ==========================================
    // 5. 循序邏輯 (Sequential Logic)
    // ==========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            L_out     <= 10'sd0;
            H_out     <= 10'sd0;
            cnt       <= 1'b0;
            for (i = 0; i < 9; i = i + 1) begin
                sr[i] <= 10'sd0;
            end
        end else begin
            valid_out <= 1'b0; // 預設不輸出

            if (valid_in) begin
                // 資料推入 Shift Register (將無號數補 0 轉為有號數)
                sr[8] <= sr[7]; sr[7] <= sr[6]; sr[6] <= sr[5];
                sr[5] <= sr[4]; sr[4] <= sr[3]; sr[3] <= sr[2];
                sr[2] <= sr[1]; sr[1] <= sr[0]; sr[0] <= {2'b00, pixel_in};

                // 每讀入一筆資料，計數器翻轉一次
                cnt <= ~cnt;

                // 破解 Note 3：每 2 個 Clock 也就是 cnt == 1 時，才輸出一次有效結果 (Downsampling)
                if (cnt == 1'b1) begin
                    valid_out <= 1'b1;
                    
                    // 截斷為 10-bit 輸出，完全符合你的 Q1 最佳化參數 WL_hor=10
                    L_out     <= mac_L_rounded[9:0];
                    H_out     <= mac_H_rounded[9:0];
                end
            end
        end
    end

endmodule