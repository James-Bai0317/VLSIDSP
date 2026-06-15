// ============================================================
// dwt_level3.v, level 2 3 DWT
// ============================================================
module dwt_level3 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire signed [9:0]  data_in,     // LL1
    input  wire               valid_in,
    output wire signed [11:0] data_out,    // LL3
    output wire               valid_out,
    output wire [1:0]         subband_out,
    output wire [5:0]         row_out,
    output wire [5:0]         col_out
);

// Level 2: 256x256 LL1 -> 128x128 LL2
// horizontal: horiz_dwt L=256, IN_W=10, OUT_W=11
// vertical: 9 line buffer + dwt1d_engine

// level 2 horizontal 
wire signed [10:0] h2_l, h2_h;
wire               h2_v;
reg  [7:0]         c2_cnt;   // input pixel counter

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) c2_cnt <= 0;
    else if (valid_in) c2_cnt <= (c2_cnt == 255) ? 8'd0 : c2_cnt + 1;
end
horiz_dwt #(.L(256), .IN_W(10), .OUT_W(11)) u_h2 (
    .clk(clk), .rst_n(rst_n),
    .pixel_in(data_in), .pixel_valid(valid_in),
    .row_start(valid_in && (c2_cnt == 0)),
    .out_l(h2_l), .out_h(h2_h), .out_valid(h2_v)
);

// level 2 Vertical Line Buffers 
reg  [6:0]  v2_wr_addr;
reg  [3:0]  v2_wr_line;   
reg  [8:0]  v2_row_total; // 已寫入幾行

wire signed [10:0] v2_lb_l [0:8];
reg  [6:0]  v2_rd_addr;

genvar gv2;
generate
    for (gv2 = 0; gv2 < 9; gv2 = gv2 + 1) begin : V2LB
        line_buffer #(.WIDTH(11), .DEPTH(128)) lb_v2 (
            .clk(clk),
            .wr_en  (h2_v && (v2_wr_line == gv2)),
            .wr_addr(v2_wr_addr),
            .wr_data(h2_l),         
            .rd_addr(v2_rd_addr),
            .rd_data(v2_lb_l[gv2])
        );
    end
endgenerate

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        v2_wr_addr  <= 0; v2_wr_line <= 0; v2_row_total <= 0;
    end else if (h2_v) begin
        v2_wr_addr <= v2_wr_addr + 1;
        if (v2_wr_addr == 127) begin
            v2_wr_addr  <= 0;
            v2_row_total <= v2_row_total + 1;
            v2_wr_line  <= (v2_wr_line == 9) ? 4'd0 : v2_wr_line + 1;
        end
    end
end

// level 2 Vertical Control FSM 
reg [1:0]  v2_vstate;
reg [7:0]  v2_vrow;   // 輸出行號 0~127
reg [6:0]  v2_vcol;   // 輸出列號 0~127
reg        v2_eng_valid;
reg [3:0]  v2_lb_sel [0:8];

localparam VS_IDLE=2'd0, VS_PREP=2'd1, VS_COMP=2'd2, VS_WAIT=2'd3;

integer kk;
reg signed [10:0] ty2;
always @(*) begin
    for (kk = 0; kk < 9; kk = kk + 1) begin
        ty2 = $signed({1'b0, v2_vrow}) * 2 + kk - 4;
        if (ty2 < 0) ty2 = -ty2;
        else if (ty2 > 255) ty2 = 510 - ty2;
        v2_lb_sel[kk] = ty2 % 9;
    end
end

// 9-tap window for vertical engine
wire [98:0] v2_win_l_bus; // 9 × 11 bits
wire [76:0] v2_win_h_bus; // 7 × 11 bits
genvar gv2f;
generate
    for (gv2f = 0; gv2f < 9; gv2f = gv2f + 1)
        assign v2_win_l_bus[gv2f*11 +: 11] = v2_lb_l[v2_lb_sel[gv2f]];
    for (gv2f = 0; gv2f < 7; gv2f = gv2f + 1)
        assign v2_win_h_bus[gv2f*11 +: 11] = v2_lb_l[v2_lb_sel[gv2f+1]];
endgenerate

wire signed [10:0] v2_res_ll; // LL2 output
wire               v2_out_v;
dwt1d_engine #(.WL(11), .OUT_W(11), .FL(7)) u_v2_vert (
    .clk(clk), .rst_n(rst_n),
    .win_l_in(v2_win_l_bus), .win_h_in(v2_win_h_bus),
    .valid_in(v2_eng_valid),
    .out_l(v2_res_ll), .out_h(/* HL2 unused */),
    .valid_out(v2_out_v)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        v2_vstate <= VS_IDLE; v2_vrow <= 0; v2_vcol <= 0;
        v2_eng_valid <= 0; v2_rd_addr <= 0;
    end else begin
        case (v2_vstate)
            VS_IDLE: if (v2_row_total >= 5) begin
                v2_rd_addr <= 0; v2_vstate <= VS_PREP;
            end
            VS_PREP: begin
                v2_rd_addr <= 1; v2_eng_valid <= 1; v2_vstate <= VS_COMP;
            end
            VS_COMP: begin
                if (v2_vcol == 127) begin
                    v2_eng_valid <= 0; v2_vcol <= 0; v2_rd_addr <= 0;
                    if (v2_vrow == 127) begin
                        v2_vstate <= VS_IDLE;
                    end else begin
                        v2_vrow <= v2_vrow + 1; v2_vstate <= VS_WAIT;
                    end
                end else begin
                    v2_vcol    <= v2_vcol + 1;
                    v2_rd_addr <= v2_vcol + 2;
                end
            end
            VS_WAIT: if (v2_row_total >= (v2_vrow << 1) + 5)
                v2_vstate <= VS_PREP;
        endcase
    end
end

// Level 3: 128x128 LL2 -> 64x64 LL3
// horizontal: horiz_dwt L=128, IN_W=11, OUT_W=12
// vertical: 9 line buffer + dwt1d_engine

// level 3 Horizontal 
wire signed [11:0] h3_l, h3_h;
wire               h3_v;
reg  [6:0]         c3_cnt;

always @(posedge clk or negedge rst_n)
    if (!rst_n) c3_cnt <= 0;
    else if (v2_out_v) c3_cnt <= (c3_cnt == 127) ? 7'd0 : c3_cnt + 1;

horiz_dwt #(.L(128), .IN_W(11), .OUT_W(12)) u_h3 (
    .clk(clk), .rst_n(rst_n),
    .pixel_in(v2_res_ll), .pixel_valid(v2_out_v),
    .row_start(v2_out_v && (c3_cnt == 0)),
    .out_l(h3_l), .out_h(h3_h), .out_valid(h3_v)
);

// level 3 Vertical 9 Line Buffers
reg  [5:0]  v3_wr_addr;
reg  [3:0]  v3_wr_line;
reg  [7:0]  v3_row_total;

wire signed [11:0] v3_lb_l [0:8];
reg  [5:0]  v3_rd_addr;

genvar gv3;
generate
    for (gv3 = 0; gv3 < 9; gv3 = gv3 + 1) begin : V3LB
        line_buffer #(.WIDTH(12), .DEPTH(64)) lb_v3 (
            .clk(clk),
            .wr_en  (h3_v && (v3_wr_line == gv3)),
            .wr_addr(v3_wr_addr),
            .wr_data(h3_l),
            .rd_addr(v3_rd_addr),
            .rd_data(v3_lb_l[gv3])
        );
    end
endgenerate

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        v3_wr_addr <= 0; v3_wr_line <= 0; v3_row_total <= 0;
    end else if (h3_v) begin
        v3_wr_addr <= v3_wr_addr + 1;
        if (v3_wr_addr == 63) begin
            v3_wr_addr   <= 0;
            v3_row_total <= v3_row_total + 1;
            v3_wr_line   <= (v3_wr_line == 9) ? 4'd0 : v3_wr_line + 1;
        end
    end
end

// level 3 Vertical Control FSM 
reg [1:0]  v3_vstate;
reg [6:0]  v3_vrow;
reg [5:0]  v3_vcol;
reg        v3_eng_valid;
reg [3:0]  v3_lb_sel [0:8];

integer kk3;
reg signed [10:0] ty3;
always @(*) begin
    for (kk3 = 0; kk3 < 9; kk3 = kk3 + 1) begin
        ty3 = $signed({1'b0, v3_vrow}) * 2 + kk3 - 4;
        if (ty3 < 0) ty3 = -ty3;
        else if (ty3 > 127) ty3 = 254 - ty3;
        v3_lb_sel[kk3] = ty3 % 9;
    end
end

wire [107:0] v3_win_l_bus; // 9 × 12 bits
wire [83:0]  v3_win_h_bus; // 7 × 12 bits
genvar gv3f;
generate
    for (gv3f = 0; gv3f < 9; gv3f = gv3f + 1)
        assign v3_win_l_bus[gv3f*12 +: 12] = v3_lb_l[v3_lb_sel[gv3f]];
    for (gv3f = 0; gv3f < 7; gv3f = gv3f + 1)
        assign v3_win_h_bus[gv3f*12 +: 12] = v3_lb_l[v3_lb_sel[gv3f+1]];
endgenerate

wire signed [11:0] v3_res_ll;
wire               v3_out_v;
dwt1d_engine #(.WL(12), .OUT_W(12), .FL(7)) u_v3_vert (
    .clk(clk), .rst_n(rst_n),
    .win_l_in(v3_win_l_bus), .win_h_in(v3_win_h_bus),
    .valid_in(v3_eng_valid),
    .out_l(v3_res_ll), .out_h(/* HL3 unused */),
    .valid_out(v3_out_v)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        v3_vstate <= VS_IDLE; v3_vrow <= 0; v3_vcol <= 0;
        v3_eng_valid <= 0; v3_rd_addr <= 0;
    end else begin
        case (v3_vstate)
            VS_IDLE: if (v3_row_total >= 5) begin
                v3_rd_addr <= 0; v3_vstate <= VS_PREP;
            end
            VS_PREP: begin
                v3_rd_addr <= 1; v3_eng_valid <= 1; v3_vstate <= VS_COMP;
            end
            VS_COMP: begin
                if (v3_vcol == 63) begin
                    v3_eng_valid <= 0; v3_vcol <= 0; v3_rd_addr <= 0;
                    if (v3_vrow == 63) begin
                        v3_vstate <= VS_IDLE;
                    end else begin
                        v3_vrow <= v3_vrow + 1; v3_vstate <= VS_WAIT;
                    end
                end else begin
                    v3_vcol    <= v3_vcol + 1;
                    v3_rd_addr <= v3_vcol + 2;
                end
            end
            VS_WAIT: if (v3_row_total >= (v3_vrow << 1) + 5)
                v3_vstate <= VS_PREP;
        endcase
    end
end

// --- Output ---
reg [5:0] r3, c3;
always @(posedge clk) begin
    if (!rst_n) begin {r3, c3} <= 0; end
    else if (v3_out_v) begin
        c3 <= c3 + 1;
        if (c3 == 63) r3 <= r3 + 1;
    end
end

assign data_out    = v3_res_ll;
assign valid_out   = v3_out_v;
assign subband_out = 2'b00;
assign row_out     = r3;
assign col_out     = c3;

endmodule