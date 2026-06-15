module Transpose_Memory (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,    // 來自水平濾波器的 valid 訊號
    input  wire signed [9:0] L_in,  // 來自水平濾波器的 Lowpass 結果
    input  wire signed [9:0] H_in,  // 來自水平濾波器的 Highpass 結果
    
    output reg         valid_out,   // 告訴垂直濾波器：9行資料到齊了，可以開工！
    
    // 輸出 9 個垂直相鄰的 L 資料 (Tap0 是最舊的上面那行，Tap8 是最新讀進來的那行)
    output reg signed [9:0] L_tap0, L_tap1, L_tap2, L_tap3, L_tap4, L_tap5, L_tap6, L_tap7, L_tap8,
    // 輸出 9 個垂直相鄰的 H 資料
    output reg signed [9:0] H_tap0, H_tap1, H_tap2, H_tap3, H_tap4, H_tap5, H_tap6, H_tap7, H_tap8
);

    // ==========================================
    // 1. 宣告 10 條 Line Buffer (SRAM 陣列)
    // 寬度 20 bits (10 L + 10 H)，深度 256
    // ==========================================
    reg [19:0] SRAM_0 [0:255];
    reg [19:0] SRAM_1 [0:255];
    reg [19:0] SRAM_2 [0:255];
    reg [19:0] SRAM_3 [0:255];
    reg [19:0] SRAM_4 [0:255];
    reg [19:0] SRAM_5 [0:255];
    reg [19:0] SRAM_6 [0:255];
    reg [19:0] SRAM_7 [0:255];
    reg [19:0] SRAM_8 [0:255];
    reg [19:0] SRAM_9 [0:255];

    // ==========================================
    // 2. 指標控制 (Pointers)
    // ==========================================
    reg [7:0] col_ptr;  // 水平位置指標 (0~255)
    reg [3:0] line_ptr; // 目前正在寫入哪一條 Line Buffer (0~9)
    reg [3:0] line_count; // 記錄目前總共填滿了幾條線 (用來觸發 valid_out)

    wire [19:0] data_in = {L_in, H_in};

    // ==========================================
    // 3. SRAM 寫入邏輯 (Write Logic)
    // ==========================================
    always @(posedge clk) begin
        if (valid_in) begin
            case (line_ptr)
                4'd0: SRAM_0[col_ptr] <= data_in;
                4'd1: SRAM_1[col_ptr] <= data_in;
                4'd2: SRAM_2[col_ptr] <= data_in;
                4'd3: SRAM_3[col_ptr] <= data_in;
                4'd4: SRAM_4[col_ptr] <= data_in;
                4'd5: SRAM_5[col_ptr] <= data_in;
                4'd6: SRAM_6[col_ptr] <= data_in;
                4'd7: SRAM_7[col_ptr] <= data_in;
                4'd8: SRAM_8[col_ptr] <= data_in;
                4'd9: SRAM_9[col_ptr] <= data_in;
            endcase
        end
    end

    // ==========================================
    // 4. SRAM 非同步讀取邏輯 (Read Logic)
    // (為了對齊時序，我們直接組合邏輯讀取當下的 col_ptr)
    // ==========================================
    wire [19:0] read_out [0:9];
    assign read_out[0] = SRAM_0[col_ptr];
    assign read_out[1] = SRAM_1[col_ptr];
    assign read_out[2] = SRAM_2[col_ptr];
    assign read_out[3] = SRAM_3[col_ptr];
    assign read_out[4] = SRAM_4[col_ptr];
    assign read_out[5] = SRAM_5[col_ptr];
    assign read_out[6] = SRAM_6[col_ptr];
    assign read_out[7] = SRAM_7[col_ptr];
    assign read_out[8] = SRAM_8[col_ptr];
    assign read_out[9] = SRAM_9[col_ptr];

    // ==========================================
    // 5. 狀態控制與旋轉多工器 (State & Rotation MUX)
    // ==========================================
    integer i;
    reg [3:0] tap_idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_ptr    <= 8'd0;
            line_ptr   <= 4'd0;
            line_count <= 4'd0;
            valid_out  <= 1'b0;
        end else begin
            valid_out <= 1'b0; // 預設關閉

            if (valid_in) begin
                // 當我們收集滿 9 條線後，每次寫入新的像素，同時也代表可以輸出 9 個垂直像素了！
                if (line_count >= 4'd9) begin
                    valid_out <= 1'b1;
                    
                    // 【旋轉多工魔法】：動態分配 9 個 Tap
                    // line_ptr 是我們"正在寫入"的那條線，所以最舊的線會是 line_ptr + 1
                    L_tap0 <= read_out[(line_ptr + 1) % 10][19:10]; H_tap0 <= read_out[(line_ptr + 1) % 10][9:0];
                    L_tap1 <= read_out[(line_ptr + 2) % 10][19:10]; H_tap1 <= read_out[(line_ptr + 2) % 10][9:0];
                    L_tap2 <= read_out[(line_ptr + 3) % 10][19:10]; H_tap2 <= read_out[(line_ptr + 3) % 10][9:0];
                    L_tap3 <= read_out[(line_ptr + 4) % 10][19:10]; H_tap3 <= read_out[(line_ptr + 4) % 10][9:0];
                    L_tap4 <= read_out[(line_ptr + 5) % 10][19:10]; H_tap4 <= read_out[(line_ptr + 5) % 10][9:0];
                    L_tap5 <= read_out[(line_ptr + 6) % 10][19:10]; H_tap5 <= read_out[(line_ptr + 6) % 10][9:0];
                    L_tap6 <= read_out[(line_ptr + 7) % 10][19:10]; H_tap6 <= read_out[(line_ptr + 7) % 10][9:0];
                    L_tap7 <= read_out[(line_ptr + 8) % 10][19:10]; H_tap7 <= read_out[(line_ptr + 8) % 10][9:0];
                    L_tap8 <= read_out[(line_ptr + 9) % 10][19:10]; H_tap8 <= read_out[(line_ptr + 9) % 10][9:0];
                end

                // 更新指標
                if (col_ptr == 8'd255) begin
                    col_ptr <= 8'd0; // 換下一行
                    if (line_ptr == 4'd9)
                        line_ptr <= 4'd0; // 繞回第 0 條 SRAM
                    else
                        line_ptr <= line_ptr + 1'b1;
                        
                    // 記錄目前累積了幾條線 (最多鎖定在 9，代表已經可以開始做垂直濾波)
                    if (line_count < 4'd9)
                        line_count <= line_count + 1'b1;
                end else begin
                    col_ptr <= col_ptr + 1'b1;
                end
            end
        end
    end

endmodule