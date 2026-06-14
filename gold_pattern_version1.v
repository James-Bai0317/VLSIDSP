clc; clear;
rng(42); % 固定隨機種子，方便硬體 Testbench 除錯

%% system specification
FRAC = 7;       % 放大 2^7
K_FRAC = 10;    % K 因子右移 10 位
K = 622;        % CORDIC 補償增益 (0.6073 * 1024 截斷後為 622)

% 隨機產生 4x4 的 A 矩陣 (元素在合理有號數範圍內)
%A = randi([-15, 15], 4, 4); 
A = [
    5 2 1 0;
    2 5 2 1;
    1 2 5 2;
    0 1 2 5
    ];
[row, column] = size(A);

% 將矩陣轉換成硬體內部的有號整數型態，並左移 7 bits
mat = int32(A * 2^FRAC);

%% ideal QR factorization 
[Q_h, R_h] = qr(A, 'econ');

%% CORDIC 脈動陣列硬體行為模擬 (Givens Rotation)
for c = 1:column
    for r = row:-1:c+1
        
        fprintf('Elim r=%d, c=%d...\n', r, c);
        
        % 進行 12 次 CORDIC 迭代 (iter 0 到 11)
        for iter = 0:11
            x_current = mat(r-1, :); % 基準列
            y_current = mat(r, :);   % 被消去列
            
            % 修正：嚴格對齊硬體 GG.v 邏輯
            % 硬體中 d = ~y[WORD_LENGTH-1]。也就是說當 y >= 0 時，d = 1 旋轉；當 y < 0 時，d = 0 旋轉。
            % 這樣可以確保 X 軸最後收斂在正確的象限，消滅負號差異。
            val_y = mat(r, c);
            
            if val_y >= 0
                % 當 y >= 0, 驅動順時針旋轉壓回 0
                for col = c : column
                    shift_x = floor(double(x_current(col)) / 2^iter);
                    shift_y = floor(double(y_current(col)) / 2^iter);
                    
                    mat(r-1, col) = int32(double(x_current(col)) + shift_y);
                    mat(r, col)   = int32(double(y_current(col)) - shift_x);
                end
            else
                % 當 y < 0, 驅動逆時針旋轉壓回 0
                for col = c : column
                    shift_x = floor(double(x_current(col)) / 2^iter);
                    shift_y = floor(double(y_current(col)) / 2^iter);
                    
                    mat(r-1, col) = int32(double(x_current(col)) - shift_y);
                    mat(r, col)   = int32(double(y_current(col)) + shift_x);
                end
            end
        end
        
        % 12次 CORDIC 迭代結束後，整列乘上 K 因子並做硬體右移截斷 (>>> 10)
        for col = c:column
            mat(r-1, col) = int32(floor(double(mat(r-1, col)) * K / 2^K_FRAC));
            mat(r, col)   = int32(floor(double(mat(r, col)) * K / 2^K_FRAC));
        end
        
        % 硬體強制將已被消去的下三角目標點清零
        mat(r, c) = 0;
    end
end

R_CORDIC = double(mat) / 2^FRAC;
R_h_aligned = R_h;
for i = 1:row
    if sign(R_CORDIC(i,i)) ~= sign(R_h(i,i)) && R_CORDIC(i,i) ~= 0
        R_h_aligned(i, :) = -R_h(i, :);
    end
end
Error_Matrix = abs(R_h_aligned - R_CORDIC);
signal_power = sum(R_h_aligned(:).^2);
noise_power = sum((R_h_aligned(:) - R_CORDIC(:)).^2);
sqnr = 10 * log10(signal_power / noise_power);

%% 顯示定點數運算後的 R 矩陣浮點數值
disp('=== 顯示輸入的原始 A 矩陣 ===');
disp(A);

disp('=== ideal QR factorization R matrix ===')
disp(R_h_aligned);

disp('=== 顯示 CORDIC 運算後的 R 矩陣浮點數值 (除以 2^7 還原) ===');
disp(R_CORDIC);

disp('=== 3. 兩者之間的絕對誤差分佈 ===');
disp(Error_Matrix);
fprintf('系統量化訊噪比 (SQNR) : %.2f dB\n', sqnr);

%% Golden Pattern Row-major flatten
write_data_hex('A_input.dat', double(A), 12);  % 原始 A 矩陣的 12-bit 定點數
write_data_hex('R_golden.dat', double(R_CORDIC), 12);       % 運算後的 R 矩陣 12-bit 定點數

disp('MATLAB 4x4 R 矩陣 Golden Pattern 產生成功！');
disp('已產出符合硬體脈動陣列規格的 A_input.dat 與 R_golden.dat');


%% 輔助函數：將 4x4 矩陣變成十六進位檔案 
function write_data_hex(file_path, data, width)
    data_flat = data'; 
    data_flat = data_flat(:);
    
    max_val = 2^(width-1) - 1;
    min_val = -2^(width-1);
    
    % 計算十六進位需要的字元數 (每 4 bits 一個 hex 字元)
    hex_digits = ceil(width / 4);
    
    fid = fopen(file_path, 'w');
    for i = 1:length(data_flat)
        val = floor(data_flat(i));
        
        if val > max_val, val = max_val; end
        if val < min_val, val = min_val; end
        
        if val < 0
            val = val + 2^width;
        end
        
        % 先轉二進位確保二補數正確，再轉成指定長度的十六進位
        bin_str = dec2bin(val, width);
        hex_str = dec2hex(bin2dec(bin_str), hex_digits);
        
        fprintf(fid, '%s\n', hex_str);
    end
    fclose(fid);
end
