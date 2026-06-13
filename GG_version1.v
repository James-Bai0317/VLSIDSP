// ============================================================
//  GG.v  –  Givens Generation (Vectoring-mode CORDIC)
//  Fixed Quadrant Alignment and Correct Direction Shifting
// ============================================================
module GG #(
    parameter WORD_LENGTH = 12,
    parameter FRAC        = 7
)(
    input                           clk,
    input                           reset,
    input                           start,    // driven by upstream done
    input                           mode,     // 0: first row, 1: subsequent rows
    input  signed [WORD_LENGTH-1:0] x_in,     // diagonal residual from above
    input  signed [WORD_LENGTH-1:0] y_in,     // new row element to eliminate

    output [11:0]                   do,       // 12-bit direction vector
    output signed [WORD_LENGTH-1:0] data_out,
    output                          done
);

reg done_reg;
assign done = done_reg;

localparam signed [11:0] K      = 12'd622;   // 0.607421875 in Q10
localparam               K_FRAC = 10;

localparam IDLE      = 2'd0;
localparam CORDIC_IN = 2'd1;
localparam ITERATION = 2'd2;
localparam SCALE     = 2'd3;

reg [1:0]  current_state, next_state;
reg [1:0]  counter;      // 0..2 passes

reg signed [WORD_LENGTH-1:0] x_reg, y_reg;
reg signed [WORD_LENGTH-1:0] x_reg_next, y_reg_next;
reg [11:0] do_reg, do_reg_next;

// ── Combinational CORDIC Layers ──────────────────────────────
wire [3:0] base = counter << 2;   // 0, 4, 8

wire d0_w, d1_w, d2_w, d3_w;
wire signed [WORD_LENGTH-1:0] L1_x, L1_y;
wire signed [WORD_LENGTH-1:0] L2_x, L2_y;
wire signed [WORD_LENGTH-1:0] L3_x, L3_y;
wire signed [WORD_LENGTH-1:0] L4_x, L4_y;

// Vectoring mode: Move towards positive X axis (d = ~y[MSB])
assign d0_w = ~y_reg[WORD_LENGTH-1];
assign L1_x = d0_w ? x_reg + (y_reg >>> base)   : x_reg - (y_reg >>> base);
assign L1_y = d0_w ? y_reg - (x_reg >>> base)   : y_reg + (x_reg >>> base);

assign d1_w = ~L1_y[WORD_LENGTH-1];
assign L2_x = d1_w ? L1_x + (L1_y >>> (base+1)) : L1_x - (L1_y >>> (base+1));
assign L2_y = d1_w ? L1_y - (L1_x >>> (base+1)) : L1_y + (L1_x >>> (base+1));

assign d2_w = ~L2_y[WORD_LENGTH-1];
assign L3_x = d2_w ? L2_x + (L2_y >>> (base+2)) : L2_x - (L2_y >>> (base+2));
assign L3_y = d2_w ? L2_y - (L2_x >>> (base+2)) : L2_y + (L2_x >>> (base+2));

assign d3_w = ~L3_y[WORD_LENGTH-1];
assign L4_x = d3_w ? L3_x + (L3_y >>> (base+3)) : L3_x - (L3_y >>> (base+3));
assign L4_y = d3_w ? L3_y - (L3_x >>> (base+3)) : L3_y + (L3_x >>> (base+3));

assign do       = do_reg;
assign data_out = x_reg;

// ── FSM Sequential Block ──────────────────────────────────────
always @(posedge clk or posedge reset) begin
    if (reset) begin
        current_state <= IDLE;
        counter       <= 0;
        x_reg         <= 0;
        y_reg         <= 0;
        do_reg        <= 0;
        done_reg      <= 0;
    end else begin
        current_state <= next_state;
        x_reg         <= x_reg_next;
        y_reg         <= y_reg_next;
        do_reg        <= do_reg_next;
        done_reg      <= (current_state == SCALE);
        
        if (current_state == ITERATION) begin
            counter <= counter + 1;
        end else begin
            counter <= 0;
        end
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
    x_reg_next  = x_reg;
    y_reg_next  = y_reg;
    do_reg_next = do_reg;

    case (current_state)
        CORDIC_IN: begin
            if (mode == 1'b0) begin
                // 這是主元列（比如你定錨的主列）：x 才是對角線，y 是新進來要被消去的資料
                x_reg_next = x_in; // 或是來自特定的主元暫存器
                y_reg_next = y_in;
            end else begin
                // 拿先前疊代完、留在暫存器裡的 x_reg，跟新進來的列（y_in）進行消去
                x_reg_next = x_reg; 
                y_reg_next = y_in;
            end
            do_reg_next = 0;
        end

        ITERATION: begin
            x_reg_next = L4_x;
            y_reg_next = L4_y;
            // 修正點：從低位往高位依序填入，對齊 GR 的讀取順序 (Pass 0 填在 [3:0], Pass 1 填在 [7:4]...)
            if (counter == 2'd0) do_reg_next[3:0]  = {d3_w, d2_w, d1_w, d0_w};
            if (counter == 2'd1) do_reg_next[7:4]  = {d3_w, d2_w, d1_w, d0_w};
            if (counter == 2'd2) do_reg_next[11:8] = {d3_w, d2_w, d1_w, d0_w};
        end

        SCALE: begin
            x_reg_next = $signed(x_reg * K) >>> K_FRAC;
            y_reg_next = 0;
        end
    endcase
end

endmodule
