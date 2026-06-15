module dwt2d_top #(
    parameter IMG_SIZE = 512
)(
    input  wire        clk,
    input  wire        rst_n,

    // Streaming input
    input  wire [7:0]  pixel_in,
    input  wire        pixel_valid,
    input  wire        row_start,

    output reg         done,
    
    // Level-1 sub-band output
    output wire        lv1_out_valid, 
    output wire [1:0]  lv1_subband,
    output wire [8:0]  lv1_row,
    output wire [8:0]  lv1_col,
    output wire signed [9:0] lv1_data,

    // Level-3 sub-band output
    output wire lv3_out_valid,
    output wire [1:0] lv3_subband,
    output wire [5:0] lv3_row,
    output wire [5:0] lv3_col,
    output wire signed [11:0] lv3_data
);

// level-1 Horizontal DWT
wire signed [9:0] h1_out_l, h1_out_h;
wire              h1_out_valid;
horiz_dwt #(.L(512), .IN_W(9), .OUT_W(10)) u_horiz1 (
    .clk(clk), .rst_n(rst_n), .pixel_in({1'b0, pixel_in}), 
    .pixel_valid(pixel_valid), .row_start(row_start),
    .out_l(h1_out_l), .out_h(h1_out_h), .out_valid(h1_out_valid)
);

// Line Buffers
localparam NLB = 10;
reg [7:0] hcol_cnt; 
reg [3:0] lb_wr_line; 
reg [9:0] hrow_total; 
wire signed [9:0] lb_rd_data_l [0:9]; 
wire signed [9:0] lb_rd_data_h [0:9];
reg [7:0] lb_rd_addr;

genvar gi;
generate
    for (gi=0; gi<NLB; gi=gi+1) begin : LB_INST
        line_buffer #(.WIDTH(10), .DEPTH(256)) lb_l_inst (
            .clk(clk), .wr_en(h1_out_valid && (lb_wr_line == gi)),
            .wr_addr(hcol_cnt), .wr_data(h1_out_l), .rd_addr(lb_rd_addr), .rd_data(lb_rd_data_l[gi])
        );
        line_buffer #(.WIDTH(10), .DEPTH(256)) lb_h_inst (
            .clk(clk), .wr_en(h1_out_valid && (lb_wr_line == gi)),
            .wr_addr(hcol_cnt), .wr_data(h1_out_h), .rd_addr(lb_rd_addr), .rd_data(lb_rd_data_h[gi])
        );
    end
endgenerate

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin hcol_cnt<=0; lb_wr_line<=0; hrow_total<=0; end
    else if (h1_out_valid) begin
        hcol_cnt <= hcol_cnt + 1;
        if (hcol_cnt == 255) begin
            hcol_cnt <= 0; hrow_total <= hrow_total + 1;
            lb_wr_line <= (lb_wr_line == 9) ? 4'd0 : lb_wr_line + 4'd1;
        end
    end
end

// Vertical Control & FSM Registers
reg [1:0] vstate;
localparam VS_IDLE    = 2'd0;
localparam VS_PREPARE = 2'd1;
localparam VS_COMPUTE = 2'd2;
localparam VS_WAIT    = 2'd3;
localparam VS_FINISH  = 2'd4;

reg [8:0] vrow_idx; 
reg [7:0] vcol_idx;
reg [1:0] subband_sel;
reg       v_eng_valid;

reg signed [10:0] ty; 
reg [3:0] v_lb_sel [0:8];
integer k_idx;

always @(*) begin
    for (k_idx = 0; k_idx < 9; k_idx = k_idx + 1) begin
        ty = (vrow_idx << 1) + k_idx - 4; // vrow_idx * 2
        if (ty < 0) ty = -ty;
        else if (ty > 511) ty = 1022 - ty;
        v_lb_sel[k_idx] = ty % 10;
    end
end


// Flattening & Vertical Engines
wire [89:0] vwin_l_bus, vwin_hl_bus; 
wire [69:0] vwin_h_bus, vwin_hh_bus;
generate
    for (gi=0; gi<9; gi=gi+1) begin : FLAT_9
        assign vwin_l_bus[gi*10 +: 10]  = lb_rd_data_l[v_lb_sel[gi]];
        assign vwin_hl_bus[gi*10 +: 10] = lb_rd_data_h[v_lb_sel[gi]];
    end
    for (gi=0; gi<7; gi=gi+1) begin : FLAT_7
        assign vwin_h_bus[gi*10 +: 10]  = lb_rd_data_l[v_lb_sel[gi+1]];
        assign vwin_hh_bus[gi*10 +: 10] = lb_rd_data_h[v_lb_sel[gi+1]];
    end
endgenerate

wire signed [9:0] res_ll, res_lh, res_hl, res_hh;
wire v_out_valid_l, v_out_valid_h;

dwt1d_engine #(.WL(10), .OUT_W(10)) u_vert_L (
    .clk(clk), .rst_n(rst_n), .win_l_in(vwin_l_bus), .win_h_in(vwin_h_bus),
    .valid_in(v_eng_valid), .out_l(res_ll), .out_h(res_lh), .valid_out(v_out_valid_l)
);

dwt1d_engine #(.WL(10), .OUT_W(10)) u_vert_H (
    .clk(clk), .rst_n(rst_n), .win_l_in(vwin_hl_bus), .win_h_in(vwin_hh_bus),
    .valid_in(v_eng_valid), .out_l(res_hl), .out_h(res_hh), .valid_out(v_out_valid_h)
);

// level-3 Instance (LL1 Path)
dwt_level3 u_l3 (
    .clk(clk), .rst_n(rst_n), 
    .data_in(res_ll), .valid_in(v_out_valid_l),
    .data_out(lv3_data), .valid_out(lv3_out_valid),
    .subband_out(lv3_subband), .row_out(lv3_row), .col_out(lv3_col)
);

// FSM 
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        vstate      <= VS_IDLE;
        vrow_idx    <= 0;
        vcol_idx    <= 0;
        done        <= 0;
        lb_rd_addr  <= 0;
        v_eng_valid <= 0;
        subband_sel <= 0;
    end else begin
        case (vstate)
            VS_IDLE: begin
                if (hrow_total >= 5) begin
                    lb_rd_addr <= 0;
                    vstate     <= VS_PREPARE;
                end
            end
            VS_PREPARE: begin
                // 預留一拍讓 SRAM 讀出第一筆資料
                lb_rd_addr  <= 1;
                v_eng_valid <= 1;
                vstate      <= VS_COMPUTE;
            end
            VS_COMPUTE: begin
                if (vcol_idx == 255) begin
                    v_eng_valid <= 0;
                    vcol_idx    <= 0;
                    lb_rd_addr  <= 0;
                    if (vrow_idx == 255) begin
                        // done <= 1;
                        vstate <= VS_FINISH;
                    end else begin
                        vrow_idx <= vrow_idx + 1;
                        vstate   <= VS_WAIT;
                    end
                end else begin
                    vcol_idx   <= vcol_idx + 1;
                    lb_rd_addr <= vcol_idx + 2; // 超前讀取
                end
            end
            VS_WAIT: begin
                // 檢查是否有足夠的新行可以進行下一次垂直濾波 (vrow_idx*2 + 5)
                if (hrow_total >= 512 || hrow_total >= (vrow_idx << 1) + 5) begin
                    vstate <= VS_PREPARE;
                end
            end
            VS_FINISH: begin
                repeat(100) @(posedge clk); // 簡單粗暴的等待 Pipeline 結束
                done   <= 1;
                vstate <= VS_IDLE;
            end
        endcase
    end
end

// output Subband Control 
assign lv1_data      = (subband_sel == 2'b00) ? res_ll :
                        (subband_sel == 2'b01) ? res_hl :
                        (subband_sel == 2'b10) ? res_lh : res_hh;
    
assign lv1_out_valid = v_out_valid_l; 
assign lv1_subband   = subband_sel;
assign lv1_row       = vrow_idx;
assign lv1_col       = vcol_idx;

endmodule