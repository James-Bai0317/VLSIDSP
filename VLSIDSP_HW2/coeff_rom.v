module coeff_rom (
    input  wire [3:0] h_addr,   // 0..8
    input  wire [2:0] g_addr,   // 0..6
    output reg  signed [8:0] h_coef,
    output reg  signed [8:0] g_coef
);
 
// h coefficients  (symmetric)
always @(*) begin
    case (h_addr)
        4'd0: h_coef = 9'sd5;
        4'd1: h_coef = -9'sd3;
        4'd2: h_coef = -9'sd14;
        4'd3: h_coef = 9'sd48;
        4'd4: h_coef = 9'sd109;
        4'd5: h_coef = 9'sd48;
        4'd6: h_coef = -9'sd14;
        4'd7: h_coef = -9'sd3;
        4'd8: h_coef = 9'sd5;
        default: h_coef = 9'sd0;
    endcase
end
 
// g coefficients (symmetric)
always @(*) begin
    case (g_addr)
        3'd0: g_coef = -9'sd8;
        3'd1: g_coef = 9'sd5;
        3'd2: g_coef = 9'sd54;
        3'd3: g_coef = -9'sd101;
        3'd4: g_coef = 9'sd54;
        3'd5: g_coef = 9'sd5;
        3'd6: g_coef = -9'sd8;
        default: g_coef = 9'sd0;
    endcase
end
 
endmodule
