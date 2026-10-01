`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/26/2026 03:41:03 PM
// Design Name: 
// Module Name: secded_encoder
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module secded_encoder(
    input  wire [31:0] data_in,
    output reg  [6:0]  syndrome // {overall_parity, p5, p4, p3, p2, p1, p0}
    );

    integer i, pos;
    reg [5:0] p;
    reg p_all;

    always @(*) begin
        p = 6'b0;
        p_all = 1'b0;
        pos = 3; // Positions 1, 2, 4, 8, 16, 32 are reserved for parity bits
        
        for (i = 0; i < 32; i = i + 1) begin
            // Skip power-of-2 positions
            while (pos == 4 || pos == 8 || pos == 16 || pos == 32) begin
                pos = pos + 1;
            end
            
            // Calculate Hamming parity via XOR tree
            if (data_in[i]) begin
                p = p ^ pos;
            end
            
            // Accumulate data bits for overall parity
            p_all = p_all ^ data_in[i];
            
            pos = pos + 1;
        end
        
        // Final overall parity includes the calculated Hamming parity bits
        p_all = p_all ^ p[0] ^ p[1] ^ p[2] ^ p[3] ^ p[4] ^ p[5];
        syndrome = {p_all, p};
    end
endmodule
