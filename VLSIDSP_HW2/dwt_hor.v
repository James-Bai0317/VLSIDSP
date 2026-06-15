// ============================================================
// dwt_hor.v, first processing horizontal DWT processing
// ============================================================
module horiz_dwt #(
    parameter L     = 512, 
    parameter IN_W  = 9,   // input pixel bit width
    parameter OUT_W = 10   // output pixel bit width
)(
    input                          clk,
    input                          rst_n,
    input        signed [IN_W-1:0] pixel_in,     // input pixel
    input                          pixel_valid,  // input valid
    input                          row_start,    // change to the new row 
    output wire signed [OUT_W-1:0] out_l,        // low pass output coef.
    output wire signed [OUT_W-1:0] out_h,        // high pass output coef.
    output wire                    out_valid     // output valid
);

localparam LOG2L = $clog2(L + 10); // 522 pixels needs 10-bits above
    
// FSM definition
localparam ST_IDLE      = 3'd0;  // 閒置狀態 
localparam ST_LEFT_PAD  = 3'd1;  // left boundray processing
localparam ST_PROCESS   = 3'd2;  // middle image processing
localparam ST_RIGHT_PAD = 3'd3;  // right boundary processing
localparam ST_FINISH    = 3'd4;  // DONE
    
reg [2:0] current_state, next_state; // FSM state transiton
reg [15:0] in_cnt; // input pixel counter
reg [3:0] pad_cnt; // extension pixel counter
reg [15:0] ext_cnt; // pixel in filter amount     
reg        out_phase; // each two down-sampling phase 

reg signed [IN_W-1:0] sr9 [0:8];        // 9-tap sliding window
reg signed [IN_W-1:0] left_buf [0:4];   // first 5 pixels for left boundary
reg signed [IN_W-1:0] right_buf [0:4];  // last 5 pixels for right boundary

// Symmetric extension signal
reg signed [IN_W-1:0] ext_sample;   // samples into window
reg                   ext_valid;    // entension valid signal
    
integer i;

// FSM current state logic
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) current_state <= ST_IDLE;
    else        current_state <= next_state;
end

// FSM next state logic
always @(*) begin
    case (current_state)
        ST_IDLE:      if (row_start) next_state = ST_LEFT_PAD;    // row_start 開始偵測
        ST_LEFT_PAD:  if (in_cnt == 5) next_state = ST_PROCESS;   // 存好5個前面pixel後開始處理
        ST_PROCESS:   if (in_cnt == L) next_state = ST_RIGHT_PAD; // 處理完原始pixel後開始處理右邊界
        ST_RIGHT_PAD: if (pad_cnt == 4) next_state = ST_FINISH;   // 右邊邊界補齊後進入結束狀態
        ST_FINISH:    next_state = ST_IDLE; 
        default:      next_state = ST_IDLE;
    endcase
end

// input pixel counter and boundary buffer processing
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        in_cnt <= 0;
        for (i = 0; i < 5; i = i + 1) begin
            left_buf[i] <= 0;
            right_buf[i] <= 0;
        end
    end else begin
        case (current_state)
            ST_IDLE: in_cnt <= 0;
            ST_LEFT_PAD, ST_PROCESS: begin
                if (pixel_valid && in_cnt < L) begin
                    in_cnt <= in_cnt + 1;
                    if (in_cnt < 5) left_buf[in_cnt] <= pixel_in;
                    right_buf[0] <= pixel_in; 
                    for (i = 1; i < 5; i = i + 1) right_buf[i] <= right_buf[i-1];
                end
            end
            ST_RIGHT_PAD: in_cnt <= 0; 
        endcase
    end
end

// extension sample stream
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ext_sample <= 0; ext_valid <= 0; pad_cnt <= 0;
    end else begin
        ext_valid <= 0;
        case (current_state)
            ST_LEFT_PAD: begin
                if (in_cnt == 5) begin 
                    ext_sample <= left_buf[4-pad_cnt];
                    ext_valid  <= 1;
                    pad_cnt    <= pad_cnt + 1;
                    if (pad_cnt == 4) pad_cnt <= 0;
                end
            end
            ST_PROCESS: begin
                if (pixel_valid) begin
                    ext_sample <= pixel_in;
                    ext_valid  <= 1;
                end
            end
            ST_RIGHT_PAD: begin
                if (pad_cnt < 4) begin
                    ext_sample <= right_buf[pad_cnt+1];
                    ext_valid  <= 1;
                    pad_cnt    <= pad_cnt + 1;
                end
            end
            default: pad_cnt <= 0;
        endcase
    end
end

// sliding window 
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i=0; i<9; i=i+1) sr9[i] <= 0;
        ext_cnt <= 0;
        out_phase <= 0;
    end else if (row_start) begin
        ext_cnt <= 0;
        out_phase <= 0;
        for (i=0; i<9; i=i+1) sr9[i] <= 0;
    end else if (ext_valid) begin
        for (i=8; i>0; i=i-1) sr9[i] <= sr9[i-1];
        sr9[0] <= ext_sample;
        if (ext_cnt < 65535) ext_cnt <= ext_cnt + 1;
        if (ext_cnt >= 8) out_phase <= ~out_phase;
    end
end

// Bus 
wire [IN_W*9-1:0] win_l_flat;
wire [IN_W*7-1:0] win_h_flat;
    
genvar g;
generate
    for (g=0; g<9; g=g+1) begin assign win_l_flat[g*IN_W +: IN_W] = sr9[g]; end
    for (g=0; g<7; g=g+1) begin assign win_h_flat[g*IN_W +: IN_W] = sr9[g+1]; end
endgenerate

dwt1d_engine #(.OUT_W(OUT_W), .WL(IN_W)) u_engine (
    .clk(clk),
    .rst_n(rst_n),
    .win_l_in(win_l_flat),
    .win_h_in(win_h_flat),
    .valid_in(ext_valid && (ext_cnt >= 8) && (out_phase == 1)),
    .out_l(out_l),
    .out_h(out_h),
    .valid_out(out_valid)
);

endmodule