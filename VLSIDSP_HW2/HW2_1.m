clc; clear; close all;

% Read image
OGimg = imread('woman.bmp'); % 512 X 512 image
DWT_out = zeros(512,512); 
  
L = 512; 
 
% dynamic range extension to prevent each level of calculating truncation
layer1_fixed = numerictype(true, 10, 0); 
layer2_fixed = numerictype(true, 11, 0); 
layer3_fixed = numerictype(true, 12, 0);  
 
%% fixed point DWT in four parts of image: LL: left up HL: right up LH: left
%% down HH: right down. L: low freq. H: high freq.
% first-level 2D-DWT (row by column)
[out_l , out_h] = DWT_row_processing(L , OGimg , layer1_fixed); 
[out_ll , out_hl , out_lh , out_hh] = DWT_column_processing(L , out_l , out_h , layer1_fixed); 
DWT_out(1 : 256 , 257 : 512)   = out_hl; 
DWT_out(257 : 512 , 1 : 256)   = out_lh; 
DWT_out(257 : 512 , 257 : 512) = out_hh; 

% second-level 2D-DWT (row -> column)
[out_l_2 , out_h_2] = DWT_row_processing(256 , out_ll , layer2_fixed); 
[out_ll_2 , out_hl_2 , out_lh_2 , out_hh_2] = DWT_column_processing(256 , out_l_2 , out_h_2 , layer2_fixed); 
DWT_out(1 : 128 , 129 : 256)   = out_hl_2; 
DWT_out(129 : 256 , 1 : 128)   = out_lh_2; 
DWT_out(129 : 256 , 129 : 256) = out_hh_2; 
 
% third-level 2D-DWT (row -> column)
[out_l_3 , out_h_3] = DWT_row_processing(128 , out_ll_2 , layer3_fixed); 
[out_ll_3 , out_hl_3 , out_lh_3 , out_hh_3] = DWT_column_processing(128 , out_l_3 , out_h_3 , layer3_fixed); 
DWT_out(1 : 64 , 1 : 64)     = out_ll_3; 
DWT_out(1 : 64 , 65 : 128)   = out_hl_3; 
DWT_out(65 : 128 , 1 : 64)   = out_lh_3; 
DWT_out(65 : 128 , 65 : 128) = out_hh_3; 

%% floating point IDWT in four parts of image 
% first-level 2D-IDWT (column -> row) 
[I_out_l_2 , I_out_h_2] = IDWT_column_processing(128 , out_ll_3 , out_lh_3 , out_hl_3 , out_hh_3); 
[IDWT_img_2] = IDWT_row_processing(128 , I_out_l_2 , I_out_h_2); 

% second-level 2D-IDWT (column -> row)
[I_out_l_1 ,I_out_h_1] = IDWT_column_processing(256 ,IDWT_img_2 ,out_lh_2 ,out_hl_2 ,out_hh_2); 
[IDWT_img_1] = IDWT_row_processing(256 ,I_out_l_1 ,I_out_h_1); 

% third-level 2D-IDWT (column -> row)
[I_out_l ,I_out_h] = IDWT_column_processing(L , IDWT_img_1 ,out_lh ,out_hl ,out_hh); 
[IDWT_img] = IDWT_row_processing(L ,I_out_l ,I_out_h); 
 
%%  PSNR calculation, MAXI = 255 for 8-bit image
MSE = 0; 
for i = 1 : 512 
    for j = 1 : 512 
        MSE = MSE + ((double(OGimg(i,j)) - double(IDWT_img(i,j))) ^ 2); 
    end 
end 
MSE = MSE / (512 ^ 2); 
PSNR = 10 * (log10((255 ^ 2) / MSE)); 
fprintf('PSNR = %3.4f dB\n', PSNR);

WL = 9; % fixed point word length = 9
FL = 7; % fixed point fractional length = 7

%% Bit True Algorithm sheet
fprintf('\n------------------------------------------------------------\n');
fprintf('      Final 3-Level 2-D DWT Bit-True Design Summary\n');
fprintf('------------------------------------------------------------\n');
fprintf('%-25s | %-10s | %-10s\n', 'Variable Name', 'Word Length', 'Fractional');
fprintf('------------------------------------------------------------\n');
fprintf('%-25s | %-11d | %-10d\n', 'Filter Coefficients (h,g)', 9, 7);
fprintf('%-25s | %-11d | %-10d\n', 'Level 1 DWT (out_l/h)', WL, FL);
fprintf('%-25s | %-11d | %-10d\n', 'Level 2 DWT (out_l_2/h_2)', WL+1, FL);
fprintf('%-25s | %-11d | %-10d\n', 'Level 3 DWT (out_l_3/h_3)', WL+2, FL);
fprintf('------------------------------------------------------------\n');

%% Image Processing Result  
figure(1) 
imshow(mat2gray(double(DWT_out))); 
title('DWT Result');
 
figure(2) 
imshow(mat2gray(double(IDWT_img)));
title('IDWT Result');

LL3_vector = reshape(DWT_out', [], 1);

%% Matlab 產生 Golden Pattern 腳本範例
% 1. 設定檔名
output_file = 'golden_ll3.txt';

% 2. 處理資料 (假設 LL1 是你的結果)
% 如果是 fi 物件，取 int；如果是普通浮點數，請先用 round() 取整
gold_data = round(DWT_out); 


% 4. 寫入檔案
fid = fopen(output_file, 'w');
[rows, cols] = size(gold_data);

for r = 1:rows
    for c = 1:cols
        % 寫入整數並換行
        fprintf(fid, '%d\n', gold_data(r, c));
    end
end

% 建立並開啟檔案
fd = fopen('golden_ll3.txt', 'w');

if fd ~= -1
    % 使用迴圈或直接利用 fprintf 的自動映射功能
    % %d 代表十進位整數，\n 代表換行
    fprintf(fd, '%d\n', LL3_vector); 
    fclose(fd);
    disp('Golden LL3 pattern generated successfully!');
else
    error('File open failed!');
end

fclose(fid);
fprintf('已完成！共寫入 %d 筆資料到 %s\n', rows*cols, output_file);

% 寫入成 .raw 檔案 (Binary 格式)
fid = fopen('woman.raw', 'wb');
fwrite(fid, DWT_out', 'uint8'); % 要轉置 (img')，因為 Matlab 是 Column-major
fclose(fid);

%% DWT row processing function (down-sampling)
function [out_l ,out_h] = DWT_row_processing(L , img , fixed_size) 
 
% Analysis filter coefficients 
h_floating = [ 0.037828455507; -0.023849465020; -0.110624404418; 0.377402855613; 0.852698679009; 0.377402855613; -0.110624404418; -0.023849465020; 0.037828455507]; 
g_floating = [-0.064538882629;  0.040689417609;  0.418092273222; -0.788485616406; 0.418092273222; 0.040689417609; -0.064538882629]; 
 
% Fixed point coefficients (find minimum word length)
h = fi(h_floating, 1, 9, 7); % fixed point word length = 9, fractional length = 7
g = fi(g_floating, 1, 9, 7); % fixed point word length = 9, fractional length = 7
 
% Symmetric extension for image boundary
DWT_row_l = zeros(L, L + 8, 'like', fi([], fixed_size)); % 左右各補4個點，'like'為按照fixed型別創造0矩陣
DWT_row_l = [img( : , 5) img( : , 4) img( : , 3) img( : , 2) img img( : , L - 1) img( : , L - 2) img( : , L - 3) img( : , L - 4)]; 
DWT_row_h = zeros(L, L + 6 ,'like', fi([], fixed_size)); % 左右各補3個點
DWT_row_h = [img( : , 4) img( : , 3) img( : , 2) img img( : , L - 1) img( : , L - 2) img( : , L - 3)]; 
 
% Output image
for i = 1 : L 
    % lowpass filter for odd output data
    temp_row_l  = conv(DWT_row_l(i , :) , h); % 1D convolution for each row patch and filter coefficient
    % from the 9th patch, stride = 2, to the (L+7)th patch for full conv
    out_l(i , 1 : (L / 2)) = fi(temp_row_l(1 , 9 : 2 : (L + 7)) ,fixed_size); 
 
    % highpass filter for even output data
    temp_row_h  = conv(DWT_row_h(i , :) , g); 
    % from the 8th patch, stride = 2, to the (L+6)th patch for full conv
    out_h(i , 1 : (L / 2)) = fi(temp_row_h(1 , 8 : 2 : (L+6)) , fixed_size); 
end  
end 

%% DWT column processing function (down-sampling)
function [out_ll , out_hl , out_lh , out_hh] = DWT_column_processing(L ,input_l ,input_h ,fixed_size) 
 
% Analysis filter coefficients 
h_floating = [ 0.037828455507; -0.023849465020; -0.110624404418; 0.377402855613; 0.852698679009; 0.377402855613; -0.110624404418; -0.023849465020; 0.037828455507]; 
g_floating = [-0.064538882629;  0.040689417609;  0.418092273222; -0.788485616406; 0.418092273222; 0.040689417609; -0.064538882629]; 
 
% Fixed point coefficients (find minimum word length)
h = fi(h_floating, 1, 9, 7); % fixed point word length = 9, fractional length = 7
g = fi(g_floating, 1, 9, 7); % fixed point word length = 9, fractional length = 7
 
% Symmetric extension for image boundary, LL HL上下各補4個點(9-tap low pass)、LH HH上下各補3個點(7-tap high pass) 
DWT_col_ll = zeros(L + 8, L / 2, 'like', fi([], fixed_size)); 
DWT_col_ll = [input_l(5 , : ); input_l(4 , : ); input_l(3 , : ); input_l(2 , : ); input_l; input_l(L - 1 , : ); input_l(L - 2 , : ); input_l(L - 3, : ); input_l(L - 4, : )]; 

DWT_col_lh = zeros(L + 6, L / 2, 'like', fi([], fixed_size));  
DWT_col_lh = [input_l(4 , : ); input_l(3 , : ); input_l(2 , : ); input_l; input_l(L - 1 , : ); input_l(L - 2 , : ); input_l(L - 3, : )]; 

DWT_col_hl = zeros(L + 8, L / 2, 'like', fi([], fixed_size)); 
DWT_col_hl = [input_h(5 , : ); input_h(4 , : ); input_h(3 , : ); input_h(2 , : ); input_h; input_h(L - 1 , : ); input_h(L - 2 , : ); input_h(L - 3, : ); input_h(L - 4, : )]; 

DWT_col_hh = zeros(L + 6, L / 2, 'like', fi([], fixed_size));  
DWT_col_hh = [input_h(4 , : ); input_h(3 , : ); input_h(2 , : ); input_h; input_h(L - 1 , : ); input_h(L - 2 , : ); input_h(L - 3, : )]; 
 
% Output image
for i = 1 : L/2 
    % lowpass filter for odd output data
    temp_col_ll = conv(DWT_col_ll( : ,i) , h); 
    out_ll(1 : (L / 2) , i) = fi(temp_col_ll(9 : 2 : (L + 7) , 1) , fixed_size); 
 
    temp_col_hl = conv(DWT_col_hl( : ,i) , h); 
    out_hl(1 : (L / 2) , i) = fi(temp_col_hl(9 : 2 : (L + 7) , 1) , fixed_size); 
 
    % highpass filter for even output data
    temp_col_lh = conv(DWT_col_lh( : , i) , g); 
    out_lh(1 : (L / 2) , i) = fi(temp_col_lh(8 : 2 : (L + 6) , 1) , fixed_size); 
 
    temp_col_hh = conv(DWT_col_hh( : , i) , g); 
    out_hh(1 : (L / 2) , i) = fi(temp_col_hh(8 : 2 : (L + 6) , 1) , fixed_size); 
end 
end

%% IDWT column processing function (up-sampling)
function [out_l , out_h] = IDWT_column_processing(L , input_ll , input_lh , input_hl , input_hh) 
 
% Synthesis filter coefficients 
q = [-0.064538882629; -0.040689417609; 0.418092273222; 0.788485616406; 0.418092273222; -0.040689417609; -0.064538882629]; 
p = [-0.037828455507; -0.023849465020; 0.110624404418; 0.377402855613; -0.852698679009; 0.377402855613; 0.110624404418; -0.023849465020; -0.037828455507]; 
 
% Symmetric extension for image boundary, LL HL上下各3個點(7-tap low pass)、LH HH上下各4個點(9-tap high pass)
IDWT_col_ll = zeros(L + 6 , L / 2); % 先建立0矩陣，後續填值可自動補0，不須額外插值
IDWT_col_ll(4 : 2 : (L + 2) , : ) = input_ll; % Insert input from the 4th patch, stride = 2, to the (L+2)th patch 
IDWT_col_ll(1 : 3 , : ) = [IDWT_col_ll(7 , : ); IDWT_col_ll(6 , : ); IDWT_col_ll(5 , : )]; % 上部的對稱延展對應
IDWT_col_ll((L + 4) : (L + 6) , : ) = [IDWT_col_ll(L + 2 , : ); IDWT_col_ll(L + 1 , : ); IDWT_col_ll(L + 0 , : )]; % 下部的對稱延展對應 
 
IDWT_col_hl = zeros(L + 6 ,L / 2); 
IDWT_col_hl(4 : 2 : (L + 2) , : ) = input_hl; 
IDWT_col_hl(1 : 3 , : ) = [IDWT_col_hl(7 , : ); IDWT_col_hl(6 , : ); IDWT_col_hl(5 , : )]; 
IDWT_col_hl((L + 4) : (L + 6) , : ) = [IDWT_col_hl(L + 2 , : ); IDWT_col_hl(L + 1 , : ); IDWT_col_hl(L + 0 , : )]; 
 
IDWT_col_lh = zeros(L + 8 ,L / 2); 
IDWT_col_lh(6 : 2 : (L + 4) , : ) = input_lh; 
IDWT_col_lh(1 : 4 , : ) = [IDWT_col_lh(9 , : ); IDWT_col_lh(8 , : ); IDWT_col_lh(7 , : ); IDWT_col_lh(6 , : )]; 
IDWT_col_lh((L + 5) : (L + 8) , : ) = [IDWT_col_lh(L + 3 , : ); IDWT_col_lh(L + 2 , : ); IDWT_col_lh(L + 1 , : ); IDWT_col_lh(L - 0 , : )]; 
 
IDWT_col_hh = zeros(L + 8 ,L / 2); 
IDWT_col_hh(6 : 2 : (L + 4) , : ) = input_hh; 
IDWT_col_hh(1 : 4 , : ) = [IDWT_col_hh(9 , : ); IDWT_col_hh(8 , : ); IDWT_col_hh(7 , : ); IDWT_col_hh(6 , : )]; 
IDWT_col_hh((L + 5) : (L + 8) , : ) = [IDWT_col_hh(L + 3 , : ); IDWT_col_hh(L + 2 , : ); IDWT_col_hh(L + 1 , : ); IDWT_col_hh(L - 0 , : )]; 
 
% Output image
for i = 1 : (L / 2)
    % lowpass filter for odd output data
    temp_col_ll = conv(IDWT_col_ll( : , i) , q); 
    out_ll(1 : L , i) = temp_col_ll(7 : L + 6 , 1); 
 
    temp_col_hl = conv(IDWT_col_hl( : , i) , q); 
    out_hl(1 : L , i) = temp_col_hl(7 : L + 6 , 1); 

    % highpass filter for even output data
    temp_col_lh = conv(IDWT_col_lh( : , i) , p); 
    out_lh(1 : L , i) = temp_col_lh(9 : L + 8 , 1); 
 
    temp_col_hh = conv(IDWT_col_hh( : , i) , p); 
    out_hh(1 : L , i) = temp_col_hh(9 : L + 8 , 1); 
end 
out_l = out_ll + out_lh; 
out_h = out_hl + out_hh; 
end 

%% IDWT row processing function (up-sampling)
function [IDWT_result] = IDWT_row_processing(L , input_l , input_h) 

% Synthesis filter coefficients 
q = [-0.064538882629; -0.040689417609; 0.418092273222; 0.788485616406; 0.418092273222; -0.040689417609; -0.064538882629]; 
p = [-0.037828455507; -0.023849465020; 0.110624404418; 0.377402855613; -0.852698679009; 0.377402855613; 0.110624404418; -0.023849465020; -0.037828455507];  
 
% Symmetric extension for image boundary 
IDWT_row_l = zeros(L ,L + 6); 
IDWT_row_l( : ,4 : 2 : (L + 2)) = input_l; 
IDWT_row_l( : ,1 : 3) = [IDWT_row_l( : ,7) IDWT_row_l( : ,6) IDWT_row_l( : ,5)]; 
IDWT_row_l( : ,(L + 4) : (L + 6)) = [IDWT_row_l( : , L + 2 ) IDWT_row_l( : , L + 1 ) IDWT_row_l( : , L + 0 )]; 
 
IDWT_row_h = zeros(L ,L + 8); 
IDWT_row_h( : ,6 : 2 : (L + 4)) = input_h; 
IDWT_row_h( : ,1 : 4) = [IDWT_row_h( : ,9) IDWT_row_h( : ,8) IDWT_row_h( : ,7) IDWT_row_h( : ,6)]; 
IDWT_row_h( : ,(L + 5) : (L + 8)) = [IDWT_row_h( : , L + 3 ) IDWT_row_h( : , L + 2 ) IDWT_row_h( : , L + 1 ) IDWT_row_h( : , L + 0 )]; 
 
% Output image
for i = 1 : L 
    % lowpass filter for odd output data
    temp_l = conv(IDWT_row_l(i , : ) ,q); 
    out_l(i ,1 : L)  = temp_l(1 ,7 : L + 6); 
 
    % highpass filter for even output data
    temp_h = conv(IDWT_row_h(i , : ) ,p); 
    out_h(i ,1 : L)  = temp_h(1 ,9 : L + 8); 
end 
IDWT_result = out_l + out_h;  
end 