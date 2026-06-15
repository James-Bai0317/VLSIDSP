module Filter_1D_Ver (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,    // 來自 Transpose Memory 的 valid 訊號
    
    // 9 個垂直相鄰的水平 Lowpass (L) 資料
    input  wire signed [9:0] L_tap0, L_tap1, L_tap2, L_tap3, L_tap4, L_tap5, L_tap6, L_tap7, L_tap8,
    // 9 個垂直相鄰的水平 Highpass (H) 資料
    input  wire signed [9:0] H_tap0, H_tap1, H_tap2, H_tap3, H_tap4, H_tap5, H_tap6, H_tap7, H_tap8,
    
    output reg         valid_out,   // 最終的 2-D DWT 輸出有效訊號
    
    // 最終四個頻帶的輸出，根據 Q1 找出的極限參數：WL_ver = 11 bits
    output reg signed [10:0] LL_out,
    output reg signed [10:0] LH_out,
    output reg signed [10:0] HL_out,
    output reg signed [10:0] HH_out
);

    // ==========================================
    // 1. 濾波器係數 (與水平相同，WL=9, WT=2^-7)
    // ==========================================
    localparam signed [8:0] H0 = 9'sd109, H1 = 9'sd48, H2 = -9'sd14, H3 = -9'sd3, H4 = 9'sd5;
    localparam signed [8:0] G0 = -9'sd101, G1 = 9'sd54, G2 = 9'sd5, G3 = -9'sd8;

    // ==========================================
    // 2. MAC 乘加器管線 (對 L 頻帶做垂直濾波 -> 產生 LL 和 LH)
    // ==========================================
    wire signed [21:0] mac_LL, mac_LH;
    
    // 垂直 Lowpass on L (對稱相加 Folding)
    assign mac_LL = (L_tap4 * H0) + 
                    ((L_tap3 + L_tap5) * H1) + 
                    ((L_tap2 + L_tap6) * H2) + 
                    ((L_tap1 + L_tap7) * H3) + 
                    ((L_tap0 + L_tap8) * H4);

    // 垂直 Highpass on L
    assign mac_LH = (L_tap4 * G0) + 
                    ((L_tap3 + L_tap5) * G1) + 
                    ((L_tap2 + L_tap6) * G2) + 
                    ((L_tap1 + L_tap7) * G3);

    // ==========================================
    // 3. MAC 乘加器管線 (對 H 頻帶做垂直濾波 -> 產生 HL 和 HH)
    // ==========================================
    wire signed [21:0] mac_HL, mac_HH;
    
    // 垂直 Lowpass on H
    assign mac_HL = (H_tap4 * H0) + 
                    ((H_tap3 + H_tap5) * H1) + 
                    ((H_tap2 + H_tap6) * H2) + 
                    ((H_tap1 + H_tap7) * H3) + 
                    ((H_tap0 + H_tap8) * H4);

    // 垂直 Highpass on H
    assign mac_HH = (H_tap4 * G0) + 
                    ((H_tap3 + H_tap5) * G1) + 
                    ((H_tap2 + H_tap6) * G2) + 
                    ((H_tap1 + H_tap7) * G3);

    // ==========================================
    // 4. 垂直方向的降採樣控制 (Vertical Downsampling)
    // ==========================================
    reg [7:0] col_cnt; 
    reg       row_flag; // 1: 輸出此行, 0: 丟棄此行 (降採樣)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            col_cnt   <= 8'd0;
            row_flag  <= 1'b1;
            LL_out <= 11'sd0; LH_out <= 11'sd0;
            HL_out <= 11'sd0; HH_out <= 11'sd0;
        end else begin
            valid_out <= 1'b0; // 預設關閉
            
            if (valid_in) begin
                // 【破解點】只有當 row_flag 為 1 時，才輸出有效資料
                if (row_flag == 1'b1) begin
                    valid_out <= 1'b1;
                    LL_out <= (mac_LL + 22'sd64) >>> 7;
                    LH_out <= (mac_LH + 22'sd64) >>> 7;
                    HL_out <= (mac_HL + 22'sd64) >>> 7;
                    HH_out <= (mac_HH + 22'sd64) >>> 7;
                end
                
                // 計算是否已經讀完一行 (256 個有效水平像素)
                if (col_cnt == 8'd255) begin
                    col_cnt  <= 8'd0;
                    row_flag <= ~row_flag; // 換行時翻轉 flag (丟掉下一行)
                end else begin
                    col_cnt  <= col_cnt + 1'b1;
                end
            end
        end
    end
endmodule