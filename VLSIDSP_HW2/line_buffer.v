module line_buffer #(
    parameter WIDTH = 10, // data bits
    parameter DEPTH = 512 // memory depth
)(
    input  wire                          clk,
    input  wire                          wr_en,    // write enable
    input  wire     [$clog2(DEPTH)-1:0]  wr_addr,  // write address
    input  wire     [WIDTH-1:0]          wr_data,  // write data
    input  wire     [$clog2(DEPTH)-1:0]  rd_addr,  // read address
    output reg      [WIDTH-1:0]          rd_data   // read data
);

reg [WIDTH-1:0] mem [0:DEPTH-1]; // Memory declaration

always @(posedge clk) begin
    if (wr_en) mem[wr_addr] <= wr_data; // write data into memeory
    rd_data <= mem[rd_addr];            // read data from memory
end

endmodule