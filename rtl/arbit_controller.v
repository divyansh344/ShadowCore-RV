`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/26/2026 08:43:58 PM
// Design Name: 
// Module Name: arbit_controller
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


module arbit_controller(
    input  wire clk,
    input  wire rst,
    
    // Fault Signals from Datapath
    input  wire uncorrectable_flag, // From Writeback (Data payload corrupted)
    input  wire cfi_fault_sig,      // From Fetch (PC misaligned/out-of-bounds)
    input  wire ctrl_fault_E,       // From Execute (Control signal parity mismatch)
    
    // Pipeline Context
    input  wire branch_flush,       // Indicates instruction is already being flushed by a branch prediction

    // ARBIT Output Controls
    output reg  wb_valid,           // Gates architectural commitment (safe_regwrite)
    output reg  arbit_flush,        // Flushes pipeline stages
    output reg  arbit_redo,         // Signals Fetch unit to rewind PC to checkpoint
    output reg  persistent_fault    // Triggers global safe reset
);

    // FSM State Encoding
    localparam NORMAL     = 2'b00;
    localparam REDO       = 2'b01;
    localparam SAFE_RESET = 2'b10;

    reg [1:0] current_state, next_state;
    
    // Fault Escalation Counter
    reg [2:0] fault_counter;
    localparam MAX_RETRIES = 3'd5; // Allow 5 recovery attempts before locking down

    // Unified fault detection (Combinational)
    wire any_fault = uncorrectable_flag | cfi_fault_sig | ctrl_fault_E;

    // 1. State Register & Counter Update
    always @(posedge clk) begin
        if (rst) begin
            current_state <= NORMAL;
            fault_counter <= 3'b0;
        end else begin
            current_state <= next_state;
            
            // Reset counter if a cycle completes normally without faults
            if (current_state == NORMAL && !any_fault) begin
                fault_counter <= 3'b0;
            end 
            // Increment on entering REDO state
            else if (current_state == NORMAL && any_fault && !branch_flush) begin
                fault_counter <= fault_counter + 1;
            end
        end
    end

    // 2. Next State Logic & Output Generation
    always @(*) begin
        // Default Outputs for normal pipeline execution
        wb_valid         = 1'b1;  
        arbit_flush      = 1'b0;
        arbit_redo       = 1'b0;
        persistent_fault = 1'b0;
        next_state       = current_state;

        case (current_state)
            NORMAL: begin
                if (any_fault) begin
                    // 1. Gated Commitment: Block writeback instantly
                    wb_valid = 1'b0; 
                    
                    if (branch_flush) begin
                        // 2. Selective Rollback: Instruction already invalidated by a branch
                        // Trigger standard FLUSH rather than a rollback
                        arbit_flush = 1'b1;
                        next_state  = NORMAL;
                    end else if (fault_counter >= MAX_RETRIES) begin
                        // 3. Fault Escalation: Too many retries indicate permanent silicon damage
                        next_state = SAFE_RESET;
                    end else begin
                        // 4. Centralized Recovery: Trigger 1-cycle REDO
                        arbit_flush = 1'b1;
                        arbit_redo  = 1'b1;
                        next_state  = REDO;
                    end
                end
            end

            REDO: begin
                // Pipeline is cleared, PC is rewound. 
                // Keep writeback gated during the REDO setup cycle, then return to NORMAL.
                wb_valid   = 1'b0; 
                next_state = NORMAL;
            end

            SAFE_RESET: begin
                // Lock core into a safe state due to persistent fault
                persistent_fault = 1'b1; 
                wb_valid         = 1'b0;
                next_state       = SAFE_RESET;
            end
            
            default: next_state = NORMAL;
        endcase
    end

endmodule
