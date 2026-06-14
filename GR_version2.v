// ============================================================
//  GR.v  –  Givens Rotation (Rotation-mode CORDIC)
//  Fixed Pipelined Direction Fetch & Safe Hold Stage
// ============================================================
module GR #(
    parameter WORD_LENGTH = 12,
    parameter FRAC        = 7
)(
    input                           clk,
    input                           reset,
    input                           start,    // driven by upstream done
    input                           mode,     // 0: init, 1: accumulate
    input  [11:0]                   di,       // direction bits from GG
    input  signed [WORD_LENGTH-1:0] x_in,     // residual from above
    input  signed [WORD_LENGTH-1:0] y_in,     // new element (left wave)

    output signed [WORD_LENGTH-1:0] data_out_x,  // downward
    output signed [WORD_LENGTH-1:0] data_out_y,  // rightward (R off-diag)
    output                          done
);

reg done_reg;
assign done = done_reg;

localparam signed [11:0] K      = 12'd622;
localparam               K_FRAC = 10;

wire signed [23:0] scale_mult_x = $signed(x_reg) * $signed(K);
wire signed [23:0] scale_mult_y = $signed(y_reg) * $signed(K);

localparam IDLE      = 2'd0;
localparam CORDIC_IN = 2'd1;
localparam ITERATION = 2'd2;
localparam SCALE     = 2'd3;

reg [1:0]  current_state, next_state;
reg [1:0]  counter;

reg signed [WORD_LENGTH-1:0] x_reg, y_reg;
reg signed [WORD_LENGTH-1:0] x_reg_next, y_reg_next;

reg [11:0] di_lat;

// ── Combinational CORDIC Layers ──────────────────────────────
// 修正點：顯式提取當前 Pass 應該要使用的 4-bit 旋轉方向
reg [3:0] dir;
always @(*) begin
    case (counter)
        2'd0:    dir = di_lat[3:0];
        2'd1:    dir = di_lat[7:4];
        2'd2:    dir = di_lat[11:8];
        default: dir = 4'b0000;
    endcase
end

wire [3:0] base = {counter, 2'b00};

wire signed [WORD_LENGTH-1:0] L1_x, L1_y;
wire signed [WORD_LENGTH-1:0] L2_x, L2_y;
wire signed [WORD_LENGTH-1:0] L3_x, L3_y;
wire signed [WORD_LENGTH-1:0] L4_x, L4_y;

assign L1_x = dir[0] ? x_reg + (y_reg >>> base)   : x_reg - (y_reg >>> base);
assign L1_y = dir[0] ? y_reg - (x_reg >>> base)   : y_reg + (x_reg >>> base);

assign L2_x = dir[1] ? L1_x + (L1_y >>> (base+1)) : L1_x - (L1_y >>> (base+1));
assign L2_y = dir[1] ? L1_y - (L1_x >>> (base+1)) : L1_y + (L1_x >>> (base+1));

assign L3_x = dir[2] ? L2_x + (L2_y >>> (base+2)) : L2_x - (L2_y >>> (base+2));
assign L3_y = dir[2] ? L2_y - (L2_x >>> (base+2)) : L2_y + (L2_x >>> (base+2));

assign L4_x = dir[3] ? L3_x + (L3_y >>> (base+3)) : L3_x - (L3_y >>> (base+3));
assign L4_y = dir[3] ? L3_y - (L3_x >>> (base+3)) : L3_y + (L3_x >>> (base+3));

/*reg signed [WORD_LENGTH-1:0] out_x_reg, out_y_reg;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        out_x_reg <= 0;
        out_y_reg <= 0;
    end else begin
        if (current_state == SCALE) begin
            out_x_reg <= scale_mult_x >>> K_FRAC;
            out_y_reg <= scale_mult_y >>> K_FRAC;
        end
    end
en*/

assign data_out_x = x_reg;
assign data_out_y = y_reg;

// ── FSM Sequential Block ──────────────────────────────────────
always @(posedge clk or posedge reset) begin
    if (reset) begin
        current_state <= IDLE;
        counter       <= 0;
        x_reg         <= 0;
        y_reg         <= 0;
        di_lat        <= 0;
        done_reg      <= 0;
    end else begin
        current_state <= next_state;
        x_reg         <= x_reg_next;
        y_reg         <= y_reg_next;
        done_reg <= (current_state == SCALE && next_state == IDLE);
        
        if (current_state == CORDIC_IN)begin
             $display("[T=%0t] GR CORDIC_IN: mode=%0d x_in=%0d y_in=%0d x_reg(before)=%0d di=0x%h -> x_reg_next=%0d y_reg_next=%0d",
                      $time, mode, x_in, y_in, x_reg, di, x_reg_next, y_reg_next);
            di_lat <= di;   
        end
        if (current_state == ITERATION)
            counter <= counter + 1;
        else
            counter <= 0;
    end
end

// ── Next-State Logic ──────────────────────────────────────────
always @(*) begin
    case (current_state)
        IDLE:      next_state = start ? CORDIC_IN : IDLE;
        CORDIC_IN: next_state = ITERATION;
        ITERATION: next_state = (counter == 2'd2) ? SCALE : ITERATION;
        SCALE:     next_state = IDLE;
        default:   next_state = IDLE;
    endcase
end

// ── Datapath Combinational Logic ──────────────────────────────
always @(*) begin
    x_reg_next = x_reg;
    y_reg_next = y_reg;

    case (current_state)
        CORDIC_IN: begin
            if (mode == 1'b0) begin
                x_reg_next = x_in; // 載入基礎主元列的 off-diagonal 元素
                y_reg_next = y_in;
            end else begin
                x_reg_next = x_reg; // 保持原本儲存在內部的 R 矩陣值，不被 x_in 覆蓋！
                y_reg_next = y_in;  // 旋轉新進來的元素
            end
        end

        ITERATION: begin
            x_reg_next = L4_x;
            y_reg_next = L4_y;
        end

        SCALE: begin
            //x_reg_next = scale_mult_x >>> K_FRAC;  // 兩個模式都要乘K
            //y_reg_next = scale_mult_y >>> K_FRAC;
            x_reg_next = scale_mult_x[WORD_LENGTH-1+K_FRAC:K_FRAC];
            y_reg_next = scale_mult_y[WORD_LENGTH-1+K_FRAC:K_FRAC];
        end

        default: begin
            x_reg_next = x_reg;
            y_reg_next = y_reg;
        end
    endcase
end

endmodule
