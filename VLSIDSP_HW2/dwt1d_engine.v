// ============================================================
// dwt1d_engine.v , parameters in module synthesize easily
// ============================================================
module dwt1d_engine #(    
    parameter OUT_W = 10,   // signed output width (Level 1: 10, Level 2: 11, Level 3: 12)
    parameter WL    = 9,   // signed input sample width from
    parameter FL    = 7    // Fractional Length from design of Q1
)(    
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire        [WL*9-1:0]      win_l_in,  // 9-sample sliding window for low pass filter (h)
    input  wire        [WL*7-1:0]      win_h_in,  // 7-sample sliding window for high pass filter (g)
    input                              valid_in,
    output reg  signed [OUT_W-1:0]     out_l,
    output reg  signed [OUT_W-1:0]     out_h,
    output reg                         valid_out
);

// fixed-point Analysis filter coefficient LUT (multiply by 2^7=128)
// h_floating = [ 0.037828455507; -0.023849465020; -0.110624404418; 0.377402855613; 0.852698679009; 0.377402855613; -0.110624404418; -0.023849465020; 0.037828455507]; 

localparam signed [8:0] H0 = 9'sd5;   
localparam signed [8:0] H1 = -9'sd3;  
localparam signed [8:0] H2 = -9'sd14;  
localparam signed [8:0] H3 = 9'sd48;  
localparam signed [8:0] H4 = 9'sd109; // middle term
localparam signed [8:0] H5 = 9'sd48;  
localparam signed [8:0] H6 = -9'sd14; 
localparam signed [8:0] H7 = -9'sd3;  
localparam signed [8:0] H8 = 9'sd5;

// g_floating = [-0.064538882629;  0.040689417609;  0.418092273222; -0.788485616406; 0.418092273222; 0.040689417609; -0.064538882629]; 
localparam signed [8:0] G0 = -9'sd8;
localparam signed [8:0] G1 = 9'sd5;
localparam signed [8:0] G2 = 9'sd54;
localparam signed [8:0] G3 = -9'sd101; // middle term
localparam signed [8:0] G4 = 9'sd54;
localparam signed [8:0] G5 = 9'sd5;
localparam signed [8:0] G6 = -9'sd8;

wire signed [WL-1:0] d_l [0:8]; // low pass filter conv. term
wire signed [WL-1:0] d_h [0:6]; // high pass filter conv. term
    
genvar i;
generate 
    for (i = 0; i < 9; i = i + 1) begin assign d_l[i] = win_l_in[i*WL +: WL]; end
    for (i = 0; i < 7; i = i + 1) begin assign d_h[i] = win_h_in[i*WL +: WL]; end
endgenerate 

// reg signed [23:0] sum_l, sum_h;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        out_l <= 0; out_h <= 0; valid_out <= 0;
    end else if (valid_in) begin
        // Low-pass Convolution
        out_l <= (d_l[0]*H0 + d_l[1]*H1 + d_l[2]*H2 + d_l[3]*H3 + d_l[4]*H4 + 
                    d_l[5]*H5 + d_l[6]*H6 + d_l[7]*H7 + d_l[8]*H8) >>> FL;
        // High-pass Convolution
        out_h <= (d_h[0]*G0 + d_h[1]*G1 + d_h[2]*G2 + d_h[3]*G3 + d_h[4]*G4 + 
                    d_h[5]*G5 + d_h[6]*G6) >>> FL;
        valid_out <= 1'b1;
    end else begin
        valid_out <= 1'b0;
    end
end

endmodule