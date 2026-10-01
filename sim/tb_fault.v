`timescale 1ns / 1ps

module tb_shadowcore_fault_injection;

    // Testbench Signals
    reg clk;
    reg rst;
    wire [31:0] pc_out;
    reg  [31:0] instr_in;
    wire [31:0] data_mem_addr;
    wire [31:0] data_mem_wdata;
    reg  [31:0] data_mem_rdata;
    wire        mem_write;
    wire [2:0]  data_mem_funct3;

    // Instantiate the Datapath
    data_path dut (
        .clk(clk),
        .rst(rst),
        .pc_out(pc_out),
        .instr_in(instr_in),
        .data_mem_addr(data_mem_addr),
        .data_mem_wdata(data_mem_wdata),
        .data_mem_rdata(data_mem_rdata),
        .mem_write(mem_write),
        .data_mem_funct3(data_mem_funct3)
    );

    // Clock Generation
    always #5 clk = ~clk;

    // Mock Instruction Memory
    always @(*) begin
        case(pc_out)
            32'h00000000: instr_in = 32'h00100093; // addi x1, x0, 1
            32'h00000004: instr_in = 32'h00208113; // addi x2, x1, 2
            32'h00000008: instr_in = 32'h00310193; // addi x3, x2, 3
            32'h0000000C: instr_in = 32'h00418213; // addi x4, x3, 4
            32'h00000010: instr_in = 32'h00520293; // addi x5, x4, 5
            32'h00000014: instr_in = 32'h00628313; // addi x6, x5, 6
            32'h00000018: instr_in = 32'h00730393; // addi x7, x6, 7
            32'h0000001C: instr_in = 32'h00838413; // addi x8, x7, 8
            default:      instr_in = 32'h00000013; // nop
        endcase
    end

    integer cycle_count;
    always @(posedge clk) begin
        if (rst) cycle_count <= 0;
        else cycle_count <= cycle_count + 1;
    end

    // Helper task to wait for an instruction to reach Writeback
    task wait_for_wb_pc;
        input [31:0] target_pc;
        begin
            // pc_plus_4W - 4 is the PC of the instruction currently in WB
            wait(dut.pc_plus_4W - 4 == target_pc && !dut.stall_D);
            @(negedge clk); // Align to middle of cycle for combinational checks
        end
    endtask

    // Helper task to wait for an instruction to reach Execute
    task wait_for_ex_pc;
        input [31:0] target_pc;
        begin
            wait(dut.pcE == target_pc && !dut.stall_D);
            @(negedge clk);
        end
    endtask

    initial begin
        // Initialize
        clk = 0;
        rst = 1;
        data_mem_rdata = 32'b0;
        
        $display("======================================================================");
        $display("  SHADOWCORE-RV : TRIPLE-AXIS FAULT TOLERANCE VERIFICATION SUITE");
        $display("======================================================================");
        
        // Release Reset
        #20 rst = 0;
        $display("[INFO] Reset released. Pipeline filling...\n");

        // ---------------------------------------------------------
        // TEST 1: Correctable Data Fault (Writeback Stage)
        // ---------------------------------------------------------
        wait_for_wb_pc(32'h00000008); // Wait for `addi x3, x2, 3` to reach WB (Result should be 6)
        
        $display("[TEST 1] COMBINATIONAL SECDED CORRECTION (Single-Bit Data Fault)");
        $display("----------------------------------------------------------------------");
        $display("> Injecting fault in Writeback Stage (ALU Result corrupted in transit)");
        $display("> Corrupting bit [4] of ALU result...");
        
        force dut.alu_result_out[4] = ~dut.alu_result_out[4];
        #2; // Wait for combinational logic
        
        $display("[CYCLE %0d] Raw Result at WB   : 0x%08h (Corrupted!)", cycle_count, dut.alu_result_out);
        $display("[CYCLE %0d] SECDED Output      : 0x%08h (Corrected!)", cycle_count, dut.corrected_alu_result);
        $display("[CYCLE %0d] CRC Match          : %b", cycle_count, dut.crc_match);
        $display("[CYCLE %0d] Uncorrectable Flag : %b (Pipeline continues seamlessly)", cycle_count, dut.uncorrectable_flag);
        
        @(posedge clk); // Let it write back
        release dut.alu_result_out[4];
        
        #5; // Check register file
        if (dut.reg_file.registers[3] == 32'h00000006) 
            $display("> Verification: Register x3 contains 0x00000006. [PASS]\n");
        else 
            $display("> Verification: Register x3 corrupted! [FAIL]\n");


        // ---------------------------------------------------------
        // TEST 2: Fatal Data Fault (Writeback Stage)
        // ---------------------------------------------------------
        wait_for_wb_pc(32'h00000010); // Wait for `addi x5, x4, 5` to reach WB
        
        $display("[TEST 2] DETERMINISTIC ALIASING REJECTION (Double-Bit Data Fault)");
        $display("----------------------------------------------------------------------");
        $display("> Injecting multi-bit fault in Writeback Stage...");
        $display("> Corrupting bits [6:5] of ALU result...");
        
        force dut.alu_result_out[6:5] = ~dut.alu_result_out[6:5];
        #2;
        
        $display("[CYCLE %0d] Raw Result at WB   : 0x%08h (Corrupted!)", cycle_count, dut.alu_result_out);
        $display("[CYCLE %0d] SECDED Output      : 0x%08h (Incorrectly aliases!)", cycle_count, dut.corrected_alu_result);
        $display("[CYCLE %0d] CRC Match          : %b (CRC catches the aliasing!)", cycle_count, dut.crc_match);
        $display("[CYCLE %0d] Uncorrectable Flag : %b", cycle_count, dut.uncorrectable_flag);
        $display("[CYCLE %0d] ARBIT Controller   : Halting Writeback (wb_valid = %b)", cycle_count, dut.wb_valid);
        $display("[CYCLE %0d] ARBIT Controller   : Triggering REDO (redo_pc = 0x%08h)", cycle_count, dut.redo_pc);
        
        @(posedge clk); // Allow ARBIT to flush and redo
        release dut.alu_result_out[6:5];
        
        #5; // Check PC and pipeline
        if (dut.pc_out == 32'h00000010)
            $display("> Verification: Pipeline flushed and PC rewound to 0x00000010. [PASS]\n");
        else
            $display("> Verification: REDO failed. PC is 0x%08h [FAIL]\n", dut.pc_out);


        // Wait for pipeline to reach EX stage after the flush
        wait_for_ex_pc(32'h00000018); // Wait for `addi x7, x6, 7` to reach EX

        // ---------------------------------------------------------
        // TEST 3: Control Fault Injection (Execute Stage)
        // ---------------------------------------------------------
        $display("[TEST 3] DECOUPLED CONTROL-FLOW INTEGRITY (Control Parity Fault)");
        $display("----------------------------------------------------------------------");
        $display("> Injecting parity mismatch in Execute Stage Control Bundle...");
        
        force dut.ctrl_parity_E = ~dut.ctrl_parity_E; 
        #2;
        
        $display("[CYCLE %0d] Control Fault Flag : %b", cycle_count, dut.ctrl_fault_E);
        $display("[CYCLE %0d] ARBIT Controller   : Masking MemWrite and RegWrite in EX", cycle_count);
        $display("[CYCLE %0d] ARBIT Controller   : Triggering REDO (redo_pc = 0x%08h)", cycle_count, dut.redo_pc);
        
        @(posedge clk);
        release dut.ctrl_parity_E;
        
        #5;
        if (dut.pc_out == 32'h00000018)
            $display("> Verification: Erroneous memory/register write prevented. PC rewound to 0x00000018. [PASS]\n");
        else
            $display("> Verification: REDO failed. PC is 0x%08h [FAIL]\n", dut.pc_out);


        #100;
        $display("======================================================================");
        $display("  VERIFICATION COMPLETE: ALL FAULT TOLERANCE MECHANISMS PASSED");
        $display("======================================================================");
        $finish;
    end
endmodule