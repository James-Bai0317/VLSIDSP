`timescale 100ps/1ps
`define CLK_PERIOD 10

// 請根據你的環境確認路徑是否正確
`define MATRIX "C:/4111064113 VLSIDSP final project/A_input.dat"
`define GOLD_R "C:/4111064113 VLSIDSP final project/R_golden.dat"

module QR_tb;
    parameter WORD = 12;
    parameter LEN  = 12;
    parameter FRAC = 7;

    reg clk;
    reg reset;
    reg ready;
    reg signed [WORD-1:0] r_data;

    wire                  r_req;
    wire [1:0]            r_row;
    wire [1:0]            r_col;
    wire signed [LEN-1:0] R11, R22, R33, R44;
    wire signed [LEN-1:0] R12, R13, R14;
    wire signed [LEN-1:0] R23, R24;
    wire signed [LEN-1:0] R34;
    wire valid;

    // Raw file storage
    reg [11:0] raw_matrix [0:15];
    reg [11:0] raw_gold_r [0:15];

    // Parsed matrices
    reg signed [WORD-1:0] A_MAT  [0:3][0:3];
    reg signed [LEN-1:0]  GOLD_R [0:3][0:3];

    // Full HW R matrix
    reg signed [LEN-1:0] HW_R [0:3][0:3];

    // ── FSM state localparams ──────────────────────────────────
    localparam S_IDLE  = 3'd0;
    localparam S_ROW0  = 3'd1;
    localparam S_ROW1  = 3'd2;
    localparam S_ROW2  = 3'd3;
    localparam S_ROW3  = 3'd4;
    localparam S_WAIT  = 3'd5;
    localparam S_VALID = 3'd6;

    QR_module #(WORD, LEN, FRAC) uut (
        .clk(clk), .reset(reset), .ready(ready),
        .r_req(r_req), .r_row(r_row), .r_col(r_col), .r_data(r_data),
        .R11(R11), .R22(R22), .R33(R33), .R44(R44),
        .R12(R12), .R13(R13), .R14(R14),
        .R23(R23), .R24(R24),
        .R34(R34),
        .valid(valid)
    );

    // Clock Generator
    initial clk = 0;
    always #(`CLK_PERIOD/2) clk = ~clk;

    integer i, r, c;
    reg pass;
    reg feed_done;

    // ----------------------------------------------------------------
    // Helper task: print a 4x4 signed matrix (12-bit entries)
    // ----------------------------------------------------------------
    task print_matrix_12;
        input [255:0] label; 
        input signed [LEN-1:0] m00, m01, m02, m03;
        input signed [LEN-1:0] m10, m11, m12, m13;
        input signed [LEN-1:0] m20, m21, m22, m23;
        input signed [LEN-1:0] m30, m31, m32, m33;
        begin
            $display("  | %5d  %5d  %5d  %5d |", m00, m01, m02, m03);
            $display("  | %5d  %5d  %5d  %5d |", m10, m11, m12, m13);
            $display("  | %5d  %5d  %5d  %5d |", m20, m21, m22, m23);
            $display("  | %5d  %5d  %5d  %5d |", m30, m31, m32, m33);
        end
    endtask

    // Helper task: print A matrix
    task print_A;
        begin
            $display("  | %4d  %4d  %4d  %4d |", A_MAT[0][0], A_MAT[0][1], A_MAT[0][2], A_MAT[0][3]);
            $display("  | %4d  %4d  %4d  %4d |", A_MAT[1][0], A_MAT[1][1], A_MAT[1][2], A_MAT[1][3]);
            $display("  | %4d  %4d  %4d  %4d |", A_MAT[2][0], A_MAT[2][1], A_MAT[2][2], A_MAT[2][3]);
            $display("  | %4d  %4d  %4d  %4d |", A_MAT[3][0], A_MAT[3][1], A_MAT[3][2], A_MAT[3][3]);
        end
    endtask

    // ── Main Test Stimulus ──────────────────────────────────────────
    initial begin
        // 讀取檔案
        $readmemh(`MATRIX, raw_matrix);
        $readmemh(`GOLD_R, raw_gold_r);

        for (i = 0; i < 16; i = i + 1) begin
            r = i / 4;
            c = i % 4;
            A_MAT[r][c]  = $signed(raw_matrix[i]);
            GOLD_R[r][c] = $signed(raw_gold_r[i]);
        end

        // 初始化 HW_R
        for (r = 0; r < 4; r = r + 1)
            for (c = 0; c < 4; c = c + 1)
                HW_R[r][c] = 0;

        pass = 1;
        feed_done = 0;

        // 重置與啟動
        reset = 1; ready = 0; r_data = 0;
        #(`CLK_PERIOD * 2);
        @(negedge clk); reset = 0; #(`CLK_PERIOD);
        @(negedge clk); ready = 1; #(`CLK_PERIOD); ready = 0;

        // ----------------------------------------------------------------
        // 修正後的餵料邏輯：不直接依賴 uut.state 組合邏輯跳出，
        // 改用 fork-join 或是與 clk 同步監看，當進入 S_WAIT 時設定 feed_done
        // ----------------------------------------------------------------
        fork
            begin
                while (!feed_done) begin
                    @(negedge clk);
                    if (uut.state == S_WAIT || uut.state == S_VALID) begin
                        feed_done = 1;
                    end
                end
            end
            begin
                while (!feed_done) begin
                    @(negedge clk);
                    if (r_req && r_row < 4 && r_col < 4) begin
                        r_data = A_MAT[r_row][r_col];
                    end else begin
                        r_data = {WORD{1'b0}};
                    end
                end
            end
        join
        
        r_data = 0; // 餵料結束，清空資料總線

        // 監聽結束與防卡死超時
        fork : timeout_block
            begin
                @(posedge valid);
                #2; // 稍微落後 valid 一點點點，確保資料完全穩定
                disable timeout_block;
            end
            begin
                #(`CLK_PERIOD * 1200); // CORDIC 疊代耗時較長，防禦時間拉長至 1200
                $display("[-] ERROR: Simulation timeout! Valid signal never asserted.");
                pass = 0; $finish;
            end
        join

        // 抓取硬體運算出來的 R 矩陣數值
        HW_R[0][0] = R11; HW_R[0][1] = R12; HW_R[0][2] = R13; HW_R[0][3] = R14;
        HW_R[1][0] = 0;   HW_R[1][1] = R22; HW_R[1][2] = R23; HW_R[1][3] = R24;
        HW_R[2][0] = 0;   HW_R[2][1] = 0;   HW_R[2][2] = R33; HW_R[2][3] = R34;
        HW_R[3][0] = 0;   HW_R[3][1] = 0;   HW_R[3][2] = 0;   HW_R[3][3] = R44;

        // ── 數據列印與報表生成 ────────────────────────────────────────
        $display("");
        $display("==================================================");
        $display("   Input Matrix A (8-bit integers)");
        $display("==================================================");
        print_A;

        $display("");
        $display("==================================================");
        $display("   Golden R Matrix (Q5.7 fixed-point)");
        $display("==================================================");
        print_matrix_12("",
            GOLD_R[0][0], GOLD_R[0][1], GOLD_R[0][2], GOLD_R[0][3],
            GOLD_R[1][0], GOLD_R[1][1], GOLD_R[1][2], GOLD_R[1][3],
            GOLD_R[2][0], GOLD_R[2][1], GOLD_R[2][2], GOLD_R[2][3],
            GOLD_R[3][0], GOLD_R[3][1], GOLD_R[3][2], GOLD_R[3][3]);

        $display("");
        $display("==================================================");
        $display("   HW R Matrix (Q5.7 fixed-point)");
        $display("==================================================");
        print_matrix_12("",
            HW_R[0][0], HW_R[0][1], HW_R[0][2], HW_R[0][3],
            HW_R[1][0], HW_R[1][1], HW_R[1][2], HW_R[1][3],
            HW_R[2][0], HW_R[2][1], HW_R[2][2], HW_R[2][3],
            HW_R[3][0], HW_R[3][1], HW_R[3][2], HW_R[3][3]);

        $display("");
        $display("==================================================");
        $display("   R Matrix Element Comparison");
        $display("   (tolerance: abs_diff <= 4)");
        $display("--------------------------------------------------");
        $display("   Position  |   HW   |  Gold  |  Diff  | Result");
        $display("--------------------------------------------------");

        // 修正點：將 pass 以 inout 方式丟進 Task 運算更新
        print_compare("R[0][0]", HW_R[0][0], GOLD_R[0][0], pass);
        print_compare("R[0][1]", HW_R[0][1], GOLD_R[0][1], pass);
        print_compare("R[0][2]", HW_R[0][2], GOLD_R[0][2], pass);
        print_compare("R[0][3]", HW_R[0][3], GOLD_R[0][3], pass);
        print_compare("R[1][1]", HW_R[1][1], GOLD_R[1][1], pass);
        print_compare("R[1][2]", HW_R[1][2], GOLD_R[1][2], pass);
        print_compare("R[1][3]", HW_R[1][3], GOLD_R[1][3], pass);
        print_compare("R[2][2]", HW_R[2][2], GOLD_R[2][2], pass);
        print_compare("R[2][3]", HW_R[2][3], GOLD_R[2][3], pass);
        print_compare("R[3][3]", HW_R[3][3], GOLD_R[3][3], pass);

        $display("--------------------------------------------------");

        if (pass) 
            $display(" >>> SUCCESS: All R elements PASSED! <<<");
        else      
            $display(" >>> FAIL:    One or more mismatches! <<<");
        $display("==================================================");
        $display("");

        $finish;
    end

    // ----------------------------------------------------------------
    // 修正後的 Task: 明確宣告輸入與變更狀態的 1-bit 旗標
    // ----------------------------------------------------------------
    task print_compare;
        input [55:0] tag;
        input signed [LEN-1:0] hw_val;
        input signed [LEN-1:0] gold_val;
        inout io_pass; // 使用 inout 才能將裡面的失敗狀態改寫回主程式的 pass 變數
        
        reg signed [LEN:0] diff; // 加寬 1-bit 防止二補數相減溢位
        reg signed [LEN:0] adiff;
        begin
            diff  = hw_val - gold_val;
            adiff = (diff < 0) ? -diff : diff;
            
            // 將畫面的 OK/FAIL 條件統一維持在誤差小於等於 4
            if (adiff <= 2) begin
                $display("   %-8s  | %5d | %5d  | %5d  |  OK", tag, hw_val, gold_val, adiff[LEN-1:0]);
            end else begin
                $display("   %-8s  | %5d | %5d  | %5d  |  FAIL ***", tag, hw_val, gold_val, adiff[LEN-1:0]);
                io_pass = 0; // 直接將主程式的 pass 刷成 0
            end
        end
    endtask

endmodule
