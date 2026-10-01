`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/26/2026 04:53:28 PM
// Design Name: 
// Module Name: crc8_parallel
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


module crc8_parallel(
    input  wire [31:0] data_in,
    output reg  [7:0]  crc_out
    );

    integer i;
    reg [7:0] crc;
    
    // Standard CRC-8 polynomial: x^8 + x^2 + x + 1 (0x07)
    always @(*) begin
        crc = 8'h00; // Initial state
        
        for (i = 31; i >= 0; i = i - 1) begin
            if (crc[7] ^ data_in[i]) begin
                crc = (crc << 1) ^ 8'h07;
            end else begin
                crc = (crc << 1);
            end
        end
        
        crc_out = crc;
    end
endmodule
