`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/26/2026 03:42:58 PM
// Design Name: 
// Module Name: secded_decoder
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



module secded_decoder(
    input  wire [31:0] data_in,
    input  wire [6:0]  syndrome_in,
    output reg  [31:0] data_out,         // The corrected data candidate
    output reg         single_error,
    output reg         double_error
    );

    integer i, pos;
    reg [5:0] p_calc;
    reg p_all_calc;
    reg [5:0] syndrome_diff;
    reg overall_parity_diff;

    always @(*) begin
        // 1. Recompute the parities on the incoming data
        p_calc = 6'b0;
        p_all_calc = 1'b0;
        pos = 3;
        
        for (i = 0; i < 32; i = i + 1) begin
            while (pos == 4 || pos == 8 || pos == 16 || pos == 32) begin
                pos = pos + 1;
            end
            if (data_in[i]) begin
                p_calc = p_calc ^ pos;
            end
            p_all_calc = p_all_calc ^ data_in[i];
            pos = pos + 1;
        end
        p_all_calc = p_all_calc ^ p_calc[0] ^ p_calc[1] ^ p_calc[2] ^ p_calc[3] ^ p_calc[4] ^ p_calc[5];

        // 2. Compare incoming syndrome with calculated syndrome
        syndrome_diff = syndrome_in[5:0] ^ p_calc;
        overall_parity_diff = ^data_in ^ ^syndrome_in;
        
        // Default outputs
        data_out = data_in;
        single_error = 1'b0;
        double_error = 1'b0;

        // 3. Error Classification & Correction
        if (syndrome_diff != 6'b0) begin
            if (overall_parity_diff == 1'b1) begin
                // Single Error Detected -> Generate 1-bit correction candidate
                single_error = 1'b1;
                
                // Locate the flipped bit and invert it
                pos = 3;
                for (i = 0; i < 32; i = i + 1) begin
                    while (pos == 4 || pos == 8 || pos == 16 || pos == 32) begin
                        pos = pos + 1;
                    end
                    if (pos == syndrome_diff) begin
                        data_out[i] = ~data_in[i];
                    end
                    pos = pos + 1;
                end
            end else begin
                // Multi-bit error detected
                double_error = 1'b1;
            end
        end else if (overall_parity_diff == 1'b1) begin
            // Parity bit itself is corrupted, data is fine
            single_error = 1'b1;
        end
    end
endmodule
