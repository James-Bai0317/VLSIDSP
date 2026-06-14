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
//localparam ALIGN = FRAC - (LEN - WORD);   // = 3
localparam ALIGN = FRAC;  // = 7（原本錯誤地算為 3）
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
reg signed [LEN-1:0] gg11_x_lat, gg22_x_lat, gg33_x_lat, gg44_x_lat;
reg signed [LEN-1:0] gr12_x_lat, gr13_x_lat, gr14_x_lat;
reg signed [LEN-1:0] gr23_x_lat, gr24_x_lat, gr34_x_lat;
reg signed [LEN-1:0] gr12_y_lat, gr23_y_lat, gr34_y_lat;

reg gg11_r_set, gg22_r_set, gg33_r_set, gg44_r_set;
reg signed [LEN-1:0] R11_val, R22_val, R33_val, R44_val;

// [BUG-M6 FIX] 分別儲存第一次呼叫的 x_out（off-diagonal R 元素）
reg signed [LEN-1:0] gr12_x_r12;  // R[0][1]
reg signed [LEN-1:0] gr13_x_r13;  // R[0][2]
reg signed [LEN-1:0] gr14_x_r14;  // R[0][3]
reg signed [LEN-1:0] gr23_x_r23;  // R[1][2]
reg signed [LEN-1:0] gr24_x_r24;  // R[1][3]
reg signed [LEN-1:0] gr34_x_r34;  // R[2][3]

// ── Column Buffers (Input Latch Registers) ────────────────────
reg signed [LEN-1:0] buf_a12, buf_a13, buf_a14;  // Row0: col1,2,3
reg signed [LEN-1:0] buf_a21, buf_a22, buf_a23, buf_a24;  // Row1
reg signed [LEN-1:0] buf_a31, buf_a32, buf_a33, buf_a34;  // Row2
reg signed [LEN-1:0] buf_a41, buf_a42, buf_a43, buf_a44;  // Row3
reg signed [LEN-1:0] buf_a_diag;  // current diagonal element

//reg signed [LEN-1:0] buf_a21_r2;
//reg signed [LEN-1:0] buf_a31, buf_a32;

reg [1:0] row_tag;

// ── PE Invocation Counters ────────────────────────────────────
reg [2:0] gg11_inv, gg22_inv, gg33_inv, gg44_inv;
reg [2:0] gr12_inv, gr13_inv, gr14_inv;
reg [2:0] gr23_inv, gr24_inv;
reg [2:0] gr34_inv;

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
        buf_a21 <= 0; buf_a22 <= 0; buf_a23 <= 0; buf_a24 <= 0;
        buf_a31 <= 0; buf_a32 <= 0; buf_a33 <= 0; buf_a34 <= 0;
        buf_a41 <= 0; buf_a42 <= 0; buf_a43 <= 0; buf_a44 <= 0;
        buf_a_diag <= 0;
        //buf_a21_r2 <= 0; buf_a31 <= 0; buf_a32 <= 0;
    end else if (r_req) begin
        if (r_col == 2'd0) buf_a_diag <= data_ext;

        case (row_tag)
            2'd0: begin
                // Row 0: A[0][0..3]
                if (r_col == 2'd1) buf_a12 <= data_ext;  // A[0][1]
                if (r_col == 2'd2) buf_a13 <= data_ext;  // A[0][2]
                if (r_col == 2'd3) buf_a14 <= data_ext;  // A[0][3]
            end
            2'd1: begin
                // Row 1: A[1][0..3]
                if (r_col == 2'd0) buf_a21 <= data_ext;  // A[1][0]（與 buf_a_diag 同步）
                if (r_col == 2'd1) buf_a22 <= data_ext;  // A[1][1]
                if (r_col == 2'd2) buf_a23 <= data_ext;  // A[1][2]
                if (r_col == 2'd3) buf_a24 <= data_ext;  // A[1][3]
            end
            2'd2: begin
                // Row 2: A[2][0..3]
                if (r_col == 2'd0) buf_a31 <= data_ext;  // A[2][0]
                if (r_col == 2'd1) buf_a32 <= data_ext;  // A[2][1]
                if (r_col == 2'd2) buf_a33 <= data_ext;  // A[2][2]
                if (r_col == 2'd3) buf_a34 <= data_ext;  // A[2][3]
            end
            2'd3: begin
                // Row 3: A[3][0..3]
                if (r_col == 2'd0) buf_a41 <= data_ext;  // A[3][0]
                if (r_col == 2'd1) buf_a42 <= data_ext;  // A[3][1]
                if (r_col == 2'd2) buf_a43 <= data_ext;  // A[3][2]
                if (r_col == 2'd3) buf_a44 <= data_ext;  // A[3][3]
            end
        endcase
    end
end

// ── Pure Dataflow Trigger & Latch Logic (Registered Control) ──
always @(posedge clk or posedge reset) begin
    if (reset) begin
        // Latches
        do11_lat <= 0; do22_lat <= 0; do33_lat <= 0;
        gg11_x_lat <= 0; gg22_x_lat <= 0; gg33_x_lat <= 0; gg44_x_lat <= 0;
        gr12_x_lat <= 0; gr13_x_lat <= 0; gr14_x_lat <= 0;
        gr23_x_lat <= 0; gr24_x_lat <= 0; gr34_x_lat <= 0;
        gr12_y_lat <= 0; gr23_y_lat <= 0; gr34_y_lat <= 0;
        gr12_x_r12 <= 0; gr13_x_r13 <= 0; gr14_x_r14 <= 0;
        gr23_x_r23 <= 0; gr24_x_r24 <= 0; gr34_x_r34 <= 0;
        gg11_r_set <= 0; gg22_r_set <= 0; gg33_r_set <= 0; gg44_r_set <= 0;
        R11_val <= 0; R22_val <= 0; R33_val <= 0; R44_val <= 0;
        
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
            $display("[T=%0t] GG11 done: gg11_x=%0d (0x%h)  buf_a_diag=%0d  do11=0x%h  gg11_inv=%0d",
                      $time, gg11_x, gg11_x, buf_a_diag, do11, gg11_inv);
            do11_lat   <= do11;   
            gg11_x_lat <= gg11_x;
            R11_val    <= gg11_x;
            gg11_r_set <= 1;
            if (gg11_inv >= 3'd2) begin
                gr12_start <= 1; gr12_inv <= gr12_inv + 1;
                gr13_start <= 1; gr13_inv <= gr13_inv + 1;
                gr14_start <= 1; gr14_inv <= gr14_inv + 1;
            end
        end

        // ── GR12 Cascade ──
        /*if (gr12_done) begin
            gr12_x_lat <= gr12_x;
            gr12_y_lat <= gr12_y;
            
            gg22_start <= 1;
            gg22_inv   <= gg22_inv + 1;
        end*/
        if (gr12_done) begin
            $display("[T=%0t] GR12 done: gr12_inv=%0d  mode=%0d  di_lat=0x%h  yin=%0d -> x=%0d y=%0d",
              $time, gr12_inv, gr12_mode, do11_lat, gr12_yin, gr12_x, gr12_y);
            gr12_x_lat <= gr12_x;
            // gr12 inv=0 處理的是 row0 的 col1 元素（旋轉後 y ≈ 0，不應觸發 GG22）
            // gr12 inv=1,2,3 的 y_out 才是需要餵給 GG22 的 col2 殘差
            /*if (gr12_inv >= 2'd1) begin
                gr12_y_lat <= gr12_y;
                gg22_start <= 1;
                gg22_inv   <= gg22_inv + 1;
            end*/
            if (gr12_inv == 3'd3) gr12_x_r12 <= gr12_x;
            if (gr12_inv >= 3'd1) begin
                gr12_y_lat <= gr12_y;
                gg22_start <= 1; gg22_inv <= gg22_inv + 1;
            end
        end

        // ── GG22 Cascade ──
        if (gg22_done) begin
            $display("[T=%0t] GG22 done: gg22_x=%0d (0x%h)  gr12_y_lat=%0d  do22=0x%h  gg22_inv=%0d",
              $time, gg22_x, gg22_x, gr12_y_lat, do22, gg22_inv);
            do22_lat   <= do22;
            gg22_x_lat <= gg22_x;
            R22_val    <= gg22_x;
            gg22_r_set <= 1;
            if (gg22_inv >= 3'd2) begin
                gr23_start <= 1; gr23_inv <= gr23_inv + 1;
                gr24_start <= 1; gr24_inv <= gr24_inv + 1;
            end
        end

        // ── GR23 Cascade ──
        /*if (gr23_done) begin
            gr23_x_lat <= gr23_x;
            gr23_y_lat <= gr23_y;
            
            gg33_start <= 1;
            gg33_inv   <= gg33_inv + 1;
        end*/
        if (gr23_done) begin
            $display("[T=%0t] GR23 done: gr23_inv=%0d  mode=%0d  di_lat=0x%h  yin=%0d -> x=%0d y=%0d",
              $time, gr23_inv, gr23_mode, do22_lat, gr23_yin, gr23_x, gr23_y);
            gr23_x_lat <= gr23_x;
            // gr23 inv=0 處理的是 row1 的 col2 元素（不觸發 GG33）
            // gr23 inv=1,2 的 y_out 才餵給 GG33
            /*if (gr23_inv >= 2'd1) begin
                gr23_y_lat <= gr23_y;
                gg33_start <= 1;
                gg33_inv   <= gg33_inv + 1;
            end*/
            if (gr23_inv == 3'd2) gr23_x_r23 <= gr23_x;
            if (gr23_inv >= 3'd1) begin
                gr23_y_lat <= gr23_y;
                gg33_start <= 1; gg33_inv <= gg33_inv + 1;
            end
        end

        // ── GG33 Cascade ──
        if (gg33_done) begin
            $display("[T=%0t] GG33 done: gg33_x=%0d (0x%h)  gr23_y_lat=%0d  do33=0x%h  gg33_inv=%0d",
              $time, gg33_x, gg33_x, gr23_y_lat, do33, gg33_inv);
            do33_lat   <= do33;
            gg33_x_lat <= gg33_x;
            if (!gg33_r_set) begin
                R33_val    <= gg33_x;
                gg33_r_set <= 1;
            end
            gr34_start <= 1; gr34_inv <= gr34_inv + 1;
        end

        // ── GR34 Cascade ──
        /*if (gr34_done) begin
            gr34_x_lat <= gr34_x;
            gr34_y_lat <= gr34_y;
            
            gg44_start <= 1;
            gg44_inv   <= gg44_inv + 1;
        end*/

        if (gr34_done) begin
            $display("[T=%0t] GR34 done: gr34_inv=%0d  mode=%0d  di_lat=0x%h  yin=%0d -> x=%0d y=%0d",
              $time, gr34_inv, gr34_mode, do33_lat, gr34_yin, gr34_x, gr34_y);
            gr34_x_lat <= gr34_x;
            // gr34 inv=0 處理的是 row2 的 col3 元素（不觸發 GG44）
            // gr34 inv=1 的 y_out 才餵給 GG44
            /*if (gr34_inv >= 2'd1) begin
                gr34_y_lat <= gr34_y;
                gg44_start <= 1;
                gg44_inv   <= gg44_inv + 1;
            end*/
            if (gr34_inv == 3'd1) gr34_x_r34 <= gr34_x;
            if (gr34_inv >= 3'd2) begin
                gr34_y_lat <= gr34_y;
                gg44_start <= 1; gg44_inv <= gg44_inv + 1;
            end
        end

        if (gg44_done && !gg44_r_set) begin
            $display("[T=%0t] GG44 done: gg44_x=%0d (0x%h)  gr34_y_lat=%0d  gg44_inv=%0d",
              $time, gg44_x, gg44_x, gr34_y_lat, gg44_inv);
            R44_val    <= gg44_x;
            gg44_r_set <= 1;
        end

        // ── GR Off-diagonal Propagation ──
        if (gr13_done) begin
            gr13_x_lat <= gr13_x;
            if (gr13_inv == 3'd1) gr13_x_r13 <= gr13_x;
        end
        if (gr14_done) begin
            gr14_x_lat <= gr14_x;
            if (gr14_inv == 3'd1) gr14_x_r14 <= gr14_x;
        end
        if (gr24_done) begin
            gr24_x_lat <= gr24_x;
            if (gr24_inv == 3'd1) gr24_x_r24 <= gr24_x;
        end
    end
end

// ── Dynamic Mode & MUX Assignment (Combinational Lookahead) ──
always @(*) begin
    // Modes depend on internal invocation counters
    /*gg11_mode = (gg11_inv == 2'd0); 
    gg22_mode = (gg22_inv == 2'd0);
    gg33_mode = (gg33_inv == 2'd0);
    gg44_mode = (gg44_inv == 2'd0);

    gr12_mode = (gr12_inv == 2'd0) ? 1'b0 : 1'b1;
    gr13_mode = (gr13_inv == 2'd0) ? 1'b0 : 1'b1;
    gr14_mode = (gr14_inv == 2'd0) ? 1'b0 : 1'b1;
    gr23_mode = (gr23_inv == 2'd0) ? 1'b0 : 1'b1;
    gr24_mode = (gr24_inv == 2'd0) ? 1'b0 : 1'b1;
    gr34_mode = (gr34_inv == 2'd0) ? 1'b0 : 1'b1;*/
    gg11_mode = (gg11_inv == 3'd1) ? 1'b0 : 1'b1;
    gg22_mode = (gg22_inv == 3'd1) ? 1'b0 : 1'b1;
    gg33_mode = (gg33_inv == 3'd1) ? 1'b0 : 1'b1;
    gg44_mode = (gg44_inv == 3'd1) ? 1'b0 : 1'b1;

    gr12_mode = (gr12_inv == 3'd1) ? 1'b0 : 1'b1;
    gr13_mode = (gr13_inv == 3'd1) ? 1'b0 : 1'b1;
    gr14_mode = (gr14_inv == 3'd1) ? 1'b0 : 1'b1;
    gr23_mode = (gr23_inv == 3'd1) ? 1'b0 : 1'b1;
    gr24_mode = (gr24_inv == 3'd1) ? 1'b0 : 1'b1;
    gr34_mode = (gr34_inv == 3'd1) ? 1'b0 : 1'b1;

    // Data-routing (Driven by stable buffered data waves)
    gg11_xin = 0;
    gg22_xin = 0;
    gg33_xin = 0;
    gg44_xin = 0;

    gr12_xin = gg11_x_lat;   // GR12 x_in ← GG11 output
    gr13_xin = gr12_x_lat;   // GR13 x_in ← GR12 x output（systolic 向下）
    gr14_xin = gr13_x_lat;   // GR14 x_in ← GR13 x output
    gr23_xin = gg22_x_lat;   // GR23 x_in ← GG22 output
    gr24_xin = gr23_x_lat;   // GR24 x_in ← GR23 x output
    gr34_xin = gg33_x_lat;   // GR34 x_in ← GG33 output
    /*gr12_xin = 0;
    gr13_xin = 0;
    gr14_xin = 0;
    gr23_xin = 0;
    gr24_xin = 0;
    gr34_xin = 0;*/

    gg11_yin = buf_a_diag;
    gg22_yin = gr12_y_lat;
    gg33_yin = gr23_y_lat;
    gg44_yin = gr34_y_lat;

    case (gr12_inv)
        3'd1:    gr12_yin = buf_a12;   // A[0][1] → 旋轉後 y→0，確立旋轉角
        3'd2:    gr12_yin = buf_a22;   // A[1][1] → 殘差送 GG22
        3'd3:    gr12_yin = buf_a32;   // A[2][1] → 殘差送 GG22
        3'd4:    gr12_yin = buf_a42;   // A[3][1] → 殘差送 GG22
        default: gr12_yin = 0;
    endcase
 
    // GR13: 依呼叫次序選擇 col3 元素
    case (gr13_inv)
        3'd1:    gr13_yin = buf_a13;   // A[0][2]
        3'd2:    gr13_yin = buf_a23;   // A[1][2]
        3'd3:    gr13_yin = buf_a33;   // A[2][2]
        3'd4:    gr13_yin = buf_a43;   // A[3][2]
        default: gr13_yin = 0;
    endcase
 
    // GR14: 依呼叫次序選擇 col4 元素
    case (gr14_inv)
        3'd1:    gr14_yin = buf_a14;   // A[0][3]
        3'd2:    gr14_yin = buf_a24;   // A[1][3]
        3'd3:    gr14_yin = buf_a34;   // A[2][3]
        3'd4:    gr14_yin = buf_a44;   // A[3][3]
        default: gr14_yin = 0;
    endcase
 
    // GR23: col3 元素（row1 和 row2 的 col3）
    case (gr23_inv)
        3'd1:    gr23_yin = buf_a23;   // A[1][2]
        3'd2:    gr23_yin = buf_a33;   // A[2][2]
        3'd3:    gr23_yin = buf_a43;   // A[3][2]
        default: gr23_yin = 0;
    endcase
 
    // GR24: col4 元素
    case (gr24_inv)
        3'd1:    gr24_yin = buf_a24;   // A[1][3]
        3'd2:    gr24_yin = buf_a34;   // A[2][3]
        3'd3:    gr24_yin = buf_a44;   // A[3][3]
        default: gr24_yin = 0;
    endcase
 
    // GR34: col4 元素（row2 和 row3 的 col4）
    case (gr34_inv)
        3'd1:    gr34_yin = buf_a34;   // A[2][3]
        3'd2:    gr34_yin = buf_a44;   // A[3][3]
        default: gr34_yin = 0;
    endcase
    /*case (gr12_inv)
        3'd1: gr12_yin = buf_a12;
        3'd2: gr12_yin = buf_a22;
        3'd3: gr12_yin = buf_a21_r2;
        3'd4: gr12_yin = buf_a31;
        default: gr12_yin = 0;
    endcase

    case (gr13_inv)
        3'd1: gr13_yin = buf_a13;
        3'd2: gr13_yin = buf_a23;
        3'd3: gr13_yin = buf_a33;
        3'd4: gr13_yin = buf_a32;
        default: gr13_yin = 0;
    endcase

    case (gr14_inv)
        3'd1: gr14_yin = buf_a14;
        3'd2: gr14_yin = buf_a24;
        3'd3: gr14_yin = buf_a34;
        3'd4: gr14_yin = buf_a44;
        default: gr14_yin = 0;
    endcase*/

    //gr23_yin = (gr23_inv == 3'd1) ? buf_a23 : buf_a33;
    //gr24_yin = (gr24_inv == 3'd1) ? buf_a24 : buf_a34;
    //gr23_yin = (gr23_inv == 2'd0) ? buf_a23 : buf_a33;
    //gr24_yin = (gr24_inv == 2'd0) ? buf_a24 : buf_a34;
    //gr34_yin = buf_a34;
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
            R11 <= R11_val >>> FRAC;
            R22 <= R22_val >>> FRAC;
            R33 <= R33_val >>> FRAC;
            R44 <= R44_val >>> FRAC;
            R12 <= gr12_x_r12 >>> FRAC;
            R13 <= gr13_x_r13 >>> FRAC;
            R14 <= gr14_x_r14 >>> FRAC;
            R23 <= gr23_x_r23 >>> FRAC;
            R24 <= gr24_x_r24 >>> FRAC;
            R34 <= gr34_x_r34 >>> FRAC;
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
