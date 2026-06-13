// ============================================================
//  QR_module.v  –  4×4 Systolic QR Factorisation (R only)
//  Pure dataflow design  ─  Registered Control & Pipelined Data
// ============================================================
module QR_module #(
    parameter WORD = 8,    // input element width
    parameter LEN  = 12,   // internal / output width
    parameter FRAC = 7     // fractional bits in internal representation
)(
    input                        clk,
    input                        reset,
    input                        ready,

    // External memory / testbench feed interface
    output reg                   r_req,   // request next element
    output reg [1:0]             r_row,   // which row  [0..3]
    output reg [1:0]             r_col,   // which column [0..3]
    input  signed [WORD-1:0]     r_data,  // element value

    // R matrix outputs (12-bit fixed-point, Q4.7 format)
    output reg signed [LEN-1:0] R11, R22, R33, R44,
    output reg signed [LEN-1:0] R12, R13, R14,
    output reg signed [LEN-1:0] R23, R24,
    output reg signed [LEN-1:0] R34,
    output reg                   valid
);

// ── Sign-extend & align input data to internal fixed-point ───
localparam ALIGN = FRAC - (LEN - WORD);   // = 3
wire signed [LEN-1:0] data_ext = $signed({{(LEN-WORD){r_data[WORD-1]}}, r_data}) <<< ALIGN;

// ── Top-level FSM States ──────────────────────────────────────
localparam S_IDLE  = 3'd0;
localparam S_ROW0  = 3'd1;  // feed row 0
localparam S_ROW1  = 3'd2;  // feed row 1
localparam S_ROW2  = 3'd3;  // feed row 2
localparam S_ROW3  = 3'd4;  // feed row 3
localparam S_WAIT  = 3'd5;  // wait for last PE to finish
localparam S_VALID = 3'd6;  // assert valid

reg [2:0] state, state_next;
reg [1:0] col_cnt;  
reg       feeding;  

// ── Dataflow Interconnect Wires ───────────────────────────────
// GG & GR Done Signals
wire gg11_done, gg22_done, gg33_done, gg44_done;
wire gr12_done, gr13_done, gr14_done;
wire gr23_done, gr24_done;
wire gr34_done;

// Data Path Wires (Residuals)
wire signed [LEN-1:0] gg11_x, gg22_x, gg33_x, gg44_x;
wire signed [LEN-1:0] gr12_x, gr13_x, gr14_x;
wire signed [LEN-1:0] gr23_x, gr24_x;
wire signed [LEN-1:0] gr34_x;
wire signed [LEN-1:0] gr12_y, gr13_y, gr14_y;
wire signed [LEN-1:0] gr23_y, gr24_y;
wire signed [LEN-1:0] gr34_y;

// GG Direction Buses (Givens parameters)
wire [11:0] do11, do22, do33;

// ── Registered Control & Data Path for PEs ────────────────────
reg gg11_start, gg22_start, gg33_start, gg44_start;
reg gg11_mode,  gg22_mode,  gg33_mode,  gg44_mode;
reg signed [LEN-1:0] gg11_xin, gg22_xin, gg33_xin, gg44_xin;
reg signed [LEN-1:0] gg11_yin, gg22_yin, gg33_yin, gg44_yin;

reg gr12_start, gr13_start, gr14_start;
reg gr23_start, gr24_start;
reg gr34_start;
reg gr12_mode, gr13_mode, gr14_mode;
reg gr23_mode, gr24_mode;
reg gr34_mode;
reg signed [LEN-1:0] gr12_xin, gr13_xin, gr14_xin;
reg signed [LEN-1:0] gr23_xin, gr24_xin;
reg signed [LEN-1:0] gr34_xin;
reg signed [LEN-1:0] gr12_yin, gr13_yin, gr14_yin;
reg signed [LEN-1:0] gr23_yin, gr24_yin;
reg signed [LEN-1:0] gr34_yin;

// Direction Latches (Registered alongside done)
reg [11:0] do11_lat, do22_lat, do33_lat;

// Residual Latches (X/Y passing down/right with registers)
reg signed [LEN-1:0] gg11_x_lat, gg22_x_lat, gg33_x_lat;
reg signed [LEN-1:0] gr12_x_lat, gr13_x_lat, gr14_x_lat;
reg signed [LEN-1:0] gr23_x_lat, gr24_x_lat, gr34_x_lat;
reg signed [LEN-1:0] gr12_y_lat, gr23_y_lat, gr34_y_lat;

// ── Column Buffers (Input Latch Registers) ────────────────────
reg signed [LEN-1:0] buf_a12, buf_a13, buf_a14;
reg signed [LEN-1:0] buf_a22, buf_a23, buf_a24;
reg signed [LEN-1:0] buf_a33, buf_a34;
reg signed [LEN-1:0] buf_a44;
reg signed [LEN-1:0] buf_a_diag;
reg signed [LEN-1:0] buf_a21_r2;
reg signed [LEN-1:0] buf_a31, buf_a32;

reg [1:0] row_tag;

// ── PE Invocation Counters ────────────────────────────────────
reg [1:0] gg11_inv, gg22_inv, gg33_inv, gg44_inv;
reg [1:0] gr12_inv, gr13_inv, gr14_inv;
reg [1:0] gr23_inv, gr24_inv;
reg [1:0] gr34_inv;

// ── Feed-FSM Sequential Block ──────────────────────────────────
always @(posedge clk or posedge reset) begin
    if (reset) begin
        state   <= S_IDLE;
        col_cnt <= 0;
        feeding <= 0;
    end else begin
        state <= state_next;
        case (state)
            S_IDLE: begin
                col_cnt <= 0;
                feeding <= ready;
            end
            S_ROW0: begin
                if (col_cnt == 2'd3) begin
                    col_cnt <= 0;
                    feeding <= 0;
                end else begin
                    col_cnt <= col_cnt + 1;
                end
            end
            S_ROW1, S_ROW2, S_ROW3: begin
                if (!feeding) begin
                    col_cnt <= 0;
                    if (gg11_done) feeding <= 1; // Wait for previous wave's boundary clear
                end else if (col_cnt == 2'd3) begin
                    col_cnt <= 0;
                    feeding <= 0;
                end else begin
                    col_cnt <= col_cnt + 1;
                end
            end
            default: begin
                col_cnt <= 0;
                feeding <= 0;
            end
        endcase
    end
end

// ── Feed-FSM Combinational Next-State Logic ────────────────────
always @(*) begin
    state_next = state;
    r_req   = 0;
    r_row   = 0;
    r_col   = col_cnt;
    row_tag = 0;

    case (state)
        S_IDLE: begin
            r_col = 0;
            if (ready) state_next = S_ROW0;
        end
        S_ROW0: begin
            r_req   = 1;
            r_row   = 0;
            row_tag = 0;
            if (col_cnt == 2'd3) state_next = S_ROW1;
        end
        S_ROW1: begin
            row_tag = 1;
            r_row   = 1;
            if (feeding) begin
                r_req = 1;
                if (col_cnt == 2'd3) state_next = S_ROW2;
            end
        end
        S_ROW2: begin
            row_tag = 2;
            r_row   = 2;
            if (feeding) begin
                r_req = 1;
                if (col_cnt == 2'd3) state_next = S_ROW3;
            end
        end
        S_ROW3: begin
            row_tag = 3;
            r_row   = 3;
            if (feeding) begin
                r_req = 1;
                if (col_cnt == 2'd3) state_next = S_WAIT;
            end
        end
        S_WAIT: begin
            if (gg44_done) state_next = S_VALID;
        end
        S_VALID: begin
            state_next = S_IDLE;
        end
        default: state_next = S_IDLE;
    endcase
end

// ── Input Stream Latch into Buffers ───────────────────────────
always @(posedge clk or posedge reset) begin
    if (reset) begin
        buf_a12 <= 0; buf_a13 <= 0; buf_a14 <= 0;
        buf_a22 <= 0; buf_a23 <= 0; buf_a24 <= 0;
        buf_a33 <= 0; buf_a34 <= 0;
        buf_a44 <= 0;
        buf_a_diag <= 0;
        buf_a21_r2 <= 0; buf_a31 <= 0; buf_a32 <= 0;
    end else if (r_req) begin
        if (r_col == 2'd0) buf_a_diag <= data_ext;

        case (row_tag)
            2'd0: begin
                if (r_col == 2'd1) buf_a12 <= data_ext;
                if (r_col == 2'd2) buf_a13 <= data_ext;
                if (r_col == 2'd3) buf_a14 <= data_ext;
            end
            2'd1: begin
                if (r_col == 2'd1) buf_a22 <= data_ext;
                if (r_col == 2'd2) buf_a23 <= data_ext;
                if (r_col == 2'd3) buf_a24 <= data_ext;
            end
            2'd2: begin
                if (r_col == 2'd1) buf_a21_r2 <= data_ext;
                if (r_col == 2'd2) buf_a33    <= data_ext;
                if (r_col == 2'd3) buf_a34    <= data_ext;
            end
            2'd3: begin
                if (r_col == 2'd1) buf_a31 <= data_ext;
                if (r_col == 2'd2) buf_a32 <= data_ext;
                if (r_col == 2'd3) buf_a44 <= data_ext;
            end
        endcase
    end
end

// ── Pure Dataflow Trigger & Latch Logic (Registered Control) ──
always @(posedge clk or posedge reset) begin
    if (reset) begin
        // Latches
        do11_lat <= 0; do22_lat <= 0; do33_lat <= 0;
        gg11_x_lat <= 0; gg22_x_lat <= 0; gg33_x_lat <= 0;
        gr12_x_lat <= 0; gr13_x_lat <= 0; gr14_x_lat <= 0;
        gr23_x_lat <= 0; gr24_x_lat <= 0;
        gr12_y_lat <= 0; gr23_y_lat <= 0; gr34_y_lat <= 0;
        
        // Invocation counters
        gg11_inv <= 0; gg22_inv <= 0; gg33_inv <= 0; gg44_inv <= 0;
        gr12_inv <= 0; gr13_inv <= 0; gr14_inv <= 0;
        gr23_inv <= 0; gr24_inv <= 0; gr34_inv <= 0;

        // Registered Starts
        gg11_start <= 0; gg22_start <= 0; gg33_start <= 0; gg44_start <= 0;
        gr12_start <= 0; gr13_start <= 0; gr14_start <= 0;
        gr23_start <= 0; gr24_start <= 0; gr34_start <= 0;
    end else begin
        // Clear start pulses by default (Single-cycle strobe)
        gg11_start <= 0; gg22_start <= 0; gg33_start <= 0; gg44_start <= 0;
        gr12_start <= 0; gr13_start <= 0; gr14_start <= 0;
        gr23_start <= 0; gr24_start <= 0; gr34_start <= 0;

        // ── Row 0 / Row-Feed Boundary ──
        if (r_req && r_col == 2'd0) begin
            gg11_start <= 1;
            gg11_inv   <= gg11_inv + 1;
        end

        // ── GG11 Cascade ──
        if (gg11_done) begin
            do11_lat   <= do11;   
            gg11_x_lat <= gg11_x;
            
            gr12_start <= 1; gr12_inv <= gr12_inv + 1;
            gr13_start <= 1; gr13_inv <= gr13_inv + 1;
            gr14_start <= 1; gr14_inv <= gr14_inv + 1;
        end

        // ── GR12 Cascade ──
        if (gr12_done) begin
            gr12_x_lat <= gr12_x;
            gr12_y_lat <= gr12_y;
            
            gg22_start <= 1;
            gg22_inv   <= gg22_inv + 1;
        end

        // ── GG22 Cascade ──
        if (gg22_done) begin
            do22_lat   <= do22;
            gg22_x_lat <= gg22_x;
            
            gr23_start <= 1; gr23_inv <= gr23_inv + 1;
            gr24_start <= 1; gr24_inv <= gr24_inv + 1;
        end

        // ── GR23 Cascade ──
        if (gr23_done) begin
            gr23_x_lat <= gr23_x;
            gr23_y_lat <= gr23_y;
            
            gg33_start <= 1;
            gg33_inv   <= gg33_inv + 1;
        end

        // ── GG33 Cascade ──
        if (gg33_done) begin
            do33_lat   <= do33;
            gg33_x_lat <= gg33_x;
            
            gr34_start <= 1; gr34_inv <= gr34_inv + 1;
        end

        // ── GR34 Cascade ──
        if (gr34_done) begin
            gr34_x_lat <= gr34_x;
            gr34_y_lat <= gr34_y;
            
            gg44_start <= 1;
            gg44_inv   <= gg44_inv + 1;
        end

        // ── GR Off-diagonal Propagation ──
        if (gr13_done) gr13_x_lat <= gr13_x;
        if (gr14_done) gr14_x_lat <= gr14_x;
        if (gr24_done) gr24_x_lat <= gr24_x;
        if (gg44_done) gg44_inv   <= gg44_inv + 1;
    end
end

// ── Dynamic Mode & MUX Assignment (Combinational Lookahead) ──
always @(*) begin
    // Modes depend on internal invocation counters
    gg11_mode = (gg11_inv == 2'd0); 
    gg22_mode = (gg22_inv == 2'd0);
    gg33_mode = (gg33_inv == 2'd0);
    gg44_mode = (gg44_inv == 2'd0);

    gr12_mode = (gr12_inv == 2'd0) ? 1'b0 : 1'b1;
    gr13_mode = (gr13_inv == 2'd0) ? 1'b0 : 1'b1;
    gr14_mode = (gr14_inv == 2'd0) ? 1'b0 : 1'b1;
    gr23_mode = (gr23_inv == 2'd0) ? 1'b0 : 1'b1;
    gr24_mode = (gr24_inv == 2'd0) ? 1'b0 : 1'b1;
    gr34_mode = (gr34_inv == 2'd0) ? 1'b0 : 1'b1;

    // Data-routing (Driven by stable buffered data waves)
    gg11_xin = 0;
    gg22_xin = gg11_x_lat;
    gg33_xin = gg22_x_lat;
    gg44_xin = gg33_x_lat;

    gr12_xin = gg11_x_lat;
    gr13_xin = gr12_x_lat;
    gr14_xin = gr13_x_lat;
    gr23_xin = gg22_x_lat;
    gr24_xin = gr23_x_lat;
    gr34_xin = gg33_x_lat;

    gg11_yin = buf_a_diag;
    gg22_yin = gr12_y_lat;
    gg33_yin = gr23_y_lat;
    gg44_yin = gr34_y_lat;

    case (gr12_inv)
        2'd0: gr12_yin = buf_a12;
        2'd1: gr12_yin = buf_a22;
        2'd2: gr12_yin = buf_a21_r2;
        2'd3: gr12_yin = buf_a31;
    endcase

    case (gr13_inv)
        2'd0: gr13_yin = buf_a13;
        2'd1: gr13_yin = buf_a23;
        2'd2: gr13_yin = buf_a33;
        2'd3: gr13_yin = buf_a32;
    endcase

    case (gr14_inv)
        2'd0: gr14_yin = buf_a14;
        2'd1: gr14_yin = buf_a24;
        2'd2: gr14_yin = buf_a34;
        2'd3: gr14_yin = buf_a44;
    endcase

    gr23_yin = (gr23_inv == 2'd0) ? buf_a23 : buf_a33;
    gr24_yin = (gr24_inv == 2'd0) ? buf_a24 : buf_a34;
    gr34_yin = buf_a34;
end

// ── Output Capture Zone ───────────────────────────────────────
always @(posedge clk or posedge reset) begin
    if (reset) begin
        R11 <= 0; R22 <= 0; R33 <= 0; R44 <= 0;
        R12 <= 0; R13 <= 0; R14 <= 0;
        R23 <= 0; R24 <= 0; R34 <= 0;
        valid <= 0;
    end else begin
        valid <= (state == S_VALID);
        if (state == S_VALID) begin
            // Captured from PEs output registers upon full factorization completion
            R11 <= gg11_x_lat;
            R22 <= gg22_x_lat;
            R33 <= gg33_x_lat;
            R44 <= gg44_x;       // current live stable output
            R12 <= gr12_x_lat;
            R13 <= gr13_x_lat;
            R14 <= gr14_x_lat;
            R23 <= gr23_x_lat;
            R24 <= gr24_x_lat;
            R34 <= gr34_x_lat;
        end
    end
end

// ── PE Instantiations ─────────────────────────────────────────

// ── Row 1 ──
GG #(LEN, FRAC) GG_11 (
    .clk(clk), .reset(reset), .start(gg11_start), .mode(gg11_mode),
    .x_in(gg11_xin), .y_in(gg11_yin), .do(do11), .data_out(gg11_x), .done(gg11_done)
);
GR #(LEN, FRAC) GR_12 (
    .clk(clk), .reset(reset), .start(gr12_start), .mode(gr12_mode), .di(do11_lat),
    .x_in(gr12_xin), .y_in(gr12_yin), .data_out_x(gr12_x), .data_out_y(gr12_y), .done(gr12_done)
);
GR #(LEN, FRAC) GR_13 (
    .clk(clk), .reset(reset), .start(gr13_start), .mode(gr13_mode), .di(do11_lat),
    .x_in(gr13_xin), .y_in(gr13_yin), .data_out_x(gr13_x), .data_out_y(gr13_y), .done(gr13_done)
);
GR #(LEN, FRAC) GR_14 (
    .clk(clk), .reset(reset), .start(gr14_start), .mode(gr14_mode), .di(do11_lat),
    .x_in(gr14_xin), .y_in(gr14_yin), .data_out_x(gr14_x), .data_out_y(gr14_y), .done(gr14_done)
);

// ── Row 2 ──
GG #(LEN, FRAC) GG_22 (
    .clk(clk), .reset(reset), .start(gg22_start), .mode(gg22_mode),
    .x_in(gg22_xin), .y_in(gg22_yin), .do(do22), .data_out(gg22_x), .done(gg22_done)
);
GR #(LEN, FRAC) GR_23 (
    .clk(clk), .reset(reset), .start(gr23_start), .mode(gr23_mode), .di(do22_lat),
    .x_in(gr23_xin), .y_in(gr23_yin), .data_out_x(gr23_x), .data_out_y(gr23_y), .done(gr23_done)
);
GR #(LEN, FRAC) GR_24 (
    .clk(clk), .reset(reset), .start(gr24_start), .mode(gr24_mode), .di(do22_lat),
    .x_in(gr24_xin), .y_in(gr24_yin), .data_out_x(gr24_x), .data_out_y(gr24_y), .done(gr24_done)
);

// ── Row 3 ──
GG #(LEN, FRAC) GG_33 (
    .clk(clk), .reset(reset), .start(gg33_start), .mode(gg33_mode),
    .x_in(gg33_xin), .y_in(gg33_yin), .do(do33), .data_out(gg33_x), .done(gg33_done)
);
GR #(LEN, FRAC) GR_34 (
    .clk(clk), .reset(reset), .start(gr34_start), .mode(gr34_mode), .di(do33_lat),
    .x_in(gr34_xin), .y_in(gr34_yin), .data_out_x(gr34_x), .data_out_y(gr34_y), .done(gr34_done)
);

// ── Row 4 ──
GG #(LEN, FRAC) GG_44 (
    .clk(clk), .reset(reset), .start(gg44_start), .mode(gg44_mode),
    .x_in(gg44_xin), .y_in(gg44_yin), .do(), .data_out(gg44_x), .done(gg44_done)
);

endmodule
