`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/24/2026 09:55:20 PM
// Design Name: 
// Module Name: instruction_fetch_unit
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

module instruction_fetch_unit(
    input wire clk,
    input wire rst,
    
    // Inputs from Datapath / Control
    input  wire  en,
    input  wire  [31:0] pc_target,
    input  wire  pc_src,
    
    // ARBIT Recovery Inputs
    input  wire arbit_redo,
    input  wire [31:0] redo_pc,
    
    // Outputs
    output wire  [31:0] pc,            // Current Program Counter
    output wire [31:0] pc_plus_4,      // Return Address (for JAL/JALR writeback)

    // Fault Tolerance Outputs
    output wire cfi_fault      // Flags alignment or bounds violations to ARBIT
    );
    
    // Internal Signals
    wire  [31:0] next_pc;
    
    // ----------------------------------------------------------------------
    // Triplicated PC Registers (Local TMR)
    // ----------------------------------------------------------------------
    reg [31:0] pc_1, pc_2, pc_3;
    wire [31:0] voted_pc;
    
    // Hardware Majority Voter (Combinational)
    assign voted_pc = (pc_1 == pc_2) ? pc_1 :
                      (pc_2 == pc_3) ? pc_2 :
                      (pc_1 == pc_3) ? pc_1 : 
                      pc_1; // Fallback

    // ----------------------------------------------------------------------
    // Semantic Bound Enforcement (CFI Watchdog)
    // ----------------------------------------------------------------------
    wire alignment_fault = (voted_pc[1:0] != 2'b00);
    wire bounds_fault = (voted_pc >= 32'h00001000); 
    
    assign cfi_fault = alignment_fault | bounds_fault;

    // ----------------------------------------------------------------------
    // PC Routing & Next PC Calculation
    // ----------------------------------------------------------------------
    assign pc = voted_pc; 
    assign pc_plus_4 = voted_pc + 32'd4;
        
    // ARBIT REDO has highest priority. Replaces standard execution flow.
    assign next_pc = (arbit_redo) ? redo_pc : 
                     (pc_src)     ? pc_target : 
                     pc_plus_4;
    
    // ----------------------------------------------------------------------
    // Register Update 
    // ----------------------------------------------------------------------
    always @ (posedge clk) begin
        if(rst) begin
            pc_1 <= 32'b0; 
            pc_2 <= 32'b0;
            pc_3 <= 32'b0;
        end
        // Override stall condition if ARBIT is forcing a rewind
        else if(en | arbit_redo) begin
            pc_1 <= next_pc;
            pc_2 <= next_pc;
            pc_3 <= next_pc;
        end
    end
   
endmodule
