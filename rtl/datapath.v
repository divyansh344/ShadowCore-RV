`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/26/2026 01:41:53 AM
// Design Name: 
// Module Name: data_path
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


module data_path(
    input  wire        clk,
    input  wire        rst,

    // ----------------------------------------------------------------------
    // Interface to Instruction Memory
    // ----------------------------------------------------------------------
    output wire [31:0] pc_out,          // Current PC -> Goes to Instruction Memory
    input  wire [31:0] instr_in,        // Instruction <- Comes from Instruction Memory
    
    // ----------------------------------------------------------------------
    // Interface to Data Memory
    // ----------------------------------------------------------------------
    output wire [31:0] data_mem_addr,   // Address -> Goes to Data Memory
    output wire [31:0] data_mem_wdata,  // Write Data -> Goes to Data Memory
    input  wire [31:0] data_mem_rdata,  // Read Data <- Comes from Data Memory
    output wire        mem_write,       // Control Signal -> Goes to Data Memory
    output wire [2:0]  data_mem_funct3  // Access size/sign -> Goes to Data Memory
    );

    // ======================================================================
    // INTERNAL WIRES
    // ======================================================================
    
    // Control Unit Signals
    wire [5:0] alu_control;
    wire       reg_write;
    wire       alu_src;
    wire       [1:0] result_src;
    wire       branch;
    wire       jump;
    wire       jalr;
    wire       auipc;
    wire       mem_writeD;  // ID-stage mem_write (feeds pipeline registers only)
    
    // Datapath Wires
    wire [31:0] pc_plus_4;
    wire [31:0] imm_val;
    wire [31:0] reg_rdata1;
    wire [31:0] reg_rdata2;
    wire [31:0] alu_result;
    wire [31:0] src_AE, src_BE;
    wire        branch_taken;
    wire pc_srcE;
    // WB→ID branch forwarding — feeds corrected source operands to branch_unit
    // Needed because register file now uses posedge write: at posedge N the
    // register still holds the pre-N value, so if WB is writing the same
    // register that a branch in ID reads, we must forward write_back_data.
    wire [31:0] branch_src1, branch_src2;
    wire [31:0] pc_targetE;
    reg  [31:0] write_back_data;// Data to be written to Register File

    // Decoupled Control Signal Parity Wires
    wire ctrl_parity_D, ctrl_parity_E; // ctrl_parity_M, ctrl_parity_W; (maybe later use)
    wire ctrl_fault_E; // Flags a corrupted control bundle

    // ALU Flags
    wire        zero_flag;
    wire        sign_flag;
       
    // Pipeline Wires
    // 1. Fetch->Decode
    wire [31:0] instrD, pc_plus_4D, pcD;
    // 2. Decode->Execute
    wire reg_writeE, mem_writeE, jumpE, jalrE, branchE, alu_srcE, branch_takenE, auipcE;
    wire [1:0] result_srcE;
    wire [5:0] alu_controlE;
    wire [31:0] rd1E, rd2E, pcE, imm_extE, pc_plus_4E;
    wire [4:0] rs1E, rs2E, rdE;
    wire [2:0] funct3E;
    // 3. Execute->Memory
    wire reg_writeM, mem_writeM;
    wire [1:0] result_srcM;
    wire [31:0] write_dataE, alu_resultM, write_dataM, pc_plus_4M;
    wire [4:0] rdM;
    wire [2:0] funct3M;
    // 4. Memory->Writeback
    wire reg_writeW;
    wire [1:0] result_srcW;
    wire [31:0] alu_result_out, read_dataW, pc_plus_4W;
    wire [4:0] rdW;
    wire [2:0] funct3W;
    // Hazard Unit
    wire stall_F, stall_D, Flush_D, Flush_E;
    wire [1:0] forward_AE, forward_BE;

    // Protection bits out of Execute Stage
    wire [6:0] alu_syndrome_E;
    wire [7:0] alu_crc_E;
    
    // Protection bits traversing to MEM and WB
    wire [6:0] alu_syndrome_M, alu_syndrome_W;
    wire [7:0] alu_crc_M, alu_crc_W;
    
    // Writeback validation and correction
    wire [31:0] corrected_alu_result;
    wire [7:0]  validation_crc;
    wire single_error, double_error;
    wire crc_match;
    wire uncorrectable_flag; // Feeds to ARBIT controller
    wire cfi_fault_sig; // Feeds to ARBIT controller    

    // ARBIT Controller Wires
    wire wb_valid;
    wire arbit_flush;
    wire arbit_redo;
    wire persistent_fault;
    wire [31:0] redo_pc; // Incase ARBIT needs to redirect execution to a known-good PC
  
    // ======================================================================
    // MODULE INSTANTIATIONS
    // ======================================================================
    
    // Datapath Units
    
    // 1. CONTROL UNIT
    control_unit ctrl_unit (
        .rst(rst),
        .opcode(instrD[6:0]),
        .funct3(instrD[14:12]),
        .funct7(instrD[31:25]),
        .alu_control(alu_control),
        .mem_write(mem_writeD),  // ID stage → pipeline only; port driven by mem_writeM
        .result_src(result_src),
        .alu_src(alu_src),
        .reg_write(reg_write),
        .branch(branch),
        .jump(jump),
        .jalr(jalr),
        .auipc(auipc),
        .ctrl_parity(ctrl_parity_D)
    );

    // 2. IMMEDIATE GENERATOR
    immediate_generator imm_gen (
        .instruction(instrD),
        .imm_val(imm_val)
    );

    // 3. REGISTER FILE
    register_file reg_file (
        .clk(clk),
        .rst(rst),
        .reg_write_en(reg_writeW & wb_valid),
        .read_addr1(instrD[19:15]), // rs1
        .read_addr2(instrD[24:20]), // rs2
        .write_addr(rdW),  // rd
        .write_data(write_back_data), // From Writeback Mux
        .read_data1(reg_rdata1),
        .read_data2(reg_rdata2)
    );

    // ── WB→ID GENERAL FORWARDING ────────────────────────────────────────────
    // The register file uses synchronous (posedge) write and combinational read.
    // When WB is writing register R at the same clock edge that ID is reading R,
    // the combinational read returns the OLD value (written next posedge).
    // The existing WB→EX forwarding (forward_AE/BE = 2'b01) cannot fix this
    // because the writing instruction leaves WB before the dependent instruction
    // reaches EX — the pipeline register has already latched the stale value.
    //
    // Solution: forward write_back_data directly to rd1/rd2 inputs of the
    // ID→EX register. This mirrors the branch_src1/2 forwarding but applies
    // to ALL instructions, not just branches.
    wire [31:0] rd1_fwd = (reg_writeW && rdW != 5'b0 && rdW == instrD[19:15])
                          ? write_back_data : reg_rdata1;
    wire [31:0] rd2_fwd = (reg_writeW && rdW != 5'b0 && rdW == instrD[24:20])
                          ? write_back_data : reg_rdata2;

    // 4. ALU SOURCE MUX (Selects between Register B and Immediate)
    assign src_BE = (alu_srcE) ? imm_extE : write_dataE;
    
    // 5. ALU
    alu my_alu (
        .src_a(src_AE),
        .src_b(src_BE),
        .alu_control(alu_controlE),
        .result(alu_result),
        .zero_flag(zero_flag),
        .sign_flag(sign_flag)
    );
    
    secded_encoder alu_encoder (
        .data_in(alu_result), // Output from ALU
        .syndrome(alu_syndrome_E)
    );

    crc8_parallel alu_crc_gen (
        .data_in(alu_result),
        .crc_out(alu_crc_E)
    );

    // ── BRANCH SOURCE FORWARDING ────────────────────────────────────────────
    // Three forwarding cases for branch source operands (branch resolved in ID):
    //   Priority 1 – MEM→ID: Result from EX stage just moved to MEM register.
    //                For ALU ops: forward alu_resultM (the computed result).
    //                For LOAD ops: forward data_mem_rdata (the loaded value).
    //                NOTE: alu_resultM for a load is the LOAD ADDRESS, not the
    //                loaded data — so we must use data_mem_rdata instead.
    //                This fixes the LW→BGE hazard (lw_stall fires, LW reaches
    //                MEM, branch needs the actual memory content, not address).
    //   Priority 2 – WB→ID: write_back_data covers the 3-instruction gap.
    //   Default     – Use WB→ID forwarded register file output.
    //   EX→ID case:   Branch-EX stall forces a 1-cycle bubble so result is in MEM.
    wire [31:0] mem_fwd_val = result_srcM[0] ? data_mem_rdata : alu_resultM;
    assign branch_src1 = (reg_writeM && rdM != 5'b0 && rdM == instrD[19:15]) ? mem_fwd_val :
                         (reg_writeW && rdW != 5'b0 && rdW == instrD[19:15]) ? write_back_data :
                         rd1_fwd;
    assign branch_src2 = (reg_writeM && rdM != 5'b0 && rdM == instrD[24:20]) ? mem_fwd_val :
                         (reg_writeW && rdW != 5'b0 && rdW == instrD[24:20]) ? write_back_data :
                         rd2_fwd;

    branch_unit br_unit (
        .branch(branch),
        .funct3(instrD[14:12]),
        .src_a(branch_src1),
        .src_b(branch_src2),
        .branch_taken(branch_taken)
    );

    // 7. INSTRUCTION FETCH UNIT
    instruction_fetch_unit fetch_unit (
        .clk(clk),
        .rst(rst),
        .en(~stall_F),
        .pc_target(pc_targetE),
        .pc_src(pc_srcE),
        .arbit_redo(arbit_redo),
        .redo_pc(redo_pc),
        .pc(pc_out),
        .pc_plus_4(pc_plus_4),
        .cfi_fault(cfi_fault_sig)
    );
    
    // Pipeline Registers
    
    // 1. Fetch->Decode
    IF_ID_REG R1(
    .clk(clk),
    .rst(rst),
    .en(~stall_D),
    .clr(Flush_D | arbit_flush),
    
    .instruction_in(instr_in),
    .pcF(pc_out),
    .pc_plus_4F(pc_plus_4),
    
    .instrD(instrD),
    .pcD(pcD),
    .pc_plus_4D(pc_plus_4D)
    );
    
    // 2. Decode->Execute
    ID_EX_REG R2(
    .clk(clk),
    .rst(rst),
    .clr(Flush_E | arbit_flush),
    
    
    .alu_controlD(alu_control),
    .mem_writeD(mem_writeD),    // registered → mem_writeE → mem_writeM
    .result_srcD(result_src),
    .alu_src(alu_src),        
    .reg_writeD(reg_write),      
    .branchD(branch),         
    .jumpD(jump),
    .jalrD(jalr),           
    .rdD(instrD[11:7]),
    .imm_extD(imm_val),
    .pcD(pcD),
    .pc_plus_4D(pc_plus_4D),
    .rd1(rd1_fwd),           // WB→ID forwarded rs1 (was reg_rdata1)
    .rd2(rd2_fwd),           // WB→ID forwarded rs2 (was reg_rdata2)
    .rs1D(instrD[19:15]),
    .rs2D(instrD[24:20]),
    .funct3D(instrD[14:12]),
    .branch_takenD(branch_taken),
    .auipcD(auipc),
    .ctrl_parity_in(ctrl_parity_D),

    
    .alu_controlE(alu_controlE),
    .mem_writeE(mem_writeE),       
    .result_srcE(result_srcE), 
    .alu_srcE(alu_srcE),  
    .reg_writeE(reg_writeE),      
    .branchE(branchE),          
    .jumpE(jumpE),
    .jalrE(jalrE),            
    .rdE(rdE),
    .imm_extE(imm_extE),
    .pcE(pcE),
    .pc_plus_4E(pc_plus_4E),
    .rd1E(rd1E),
    .rd2E(rd2E),
    .rs1E(rs1E),
    .rs2E(rs2E),
    .funct3E(funct3E),
    .branch_takenE(branch_takenE),
    .auipcE(auipcE),
    .ctrl_parity_out(ctrl_parity_E)
    );
    
    // 3. Execute->Memory
    EX_MEM_REG R3(
    .clk(clk),
    .rst(rst | arbit_flush),
    
    .reg_writeE(reg_writeE & ~ctrl_fault_E),
    .result_srcE(result_srcE),
    .mem_writeE(mem_writeE & ~ctrl_fault_E),
    .alu_result(alu_result),
    .write_dataE(write_dataE),
    .rdE(rdE),
    .pc_plus_4E(pc_plus_4E),
    .funct3E(funct3E),
    .syndrome_in(alu_syndrome_E),
    .crc_in(alu_crc_E),
    //.ctrl_parity_in(ctrl_parity_E),

    
    .reg_writeM(reg_writeM),
    .result_srcM(result_srcM),
    .mem_writeM(mem_writeM),
    .alu_resultM(alu_resultM),
    .write_dataM(write_dataM),
    .rdM(rdM),
    .pc_plus_4M(pc_plus_4M),
    .funct3M(funct3M),
    .syndrome_out(alu_syndrome_M),
    .crc_out(alu_crc_M)
    //.ctrl_parity_out(ctrl_parity_M)
    );
    
    // 4. Memory->Writeback
    MEM_WB_REG R4(
    .clk(clk),
    .rst(rst | arbit_flush),
    
    .reg_writeM(reg_writeM),
    .result_srcM(result_srcM),
    .alu_resultM(alu_resultM),
    .rd(data_mem_rdata),
    .rdM(rdM),
    .pc_plus_4M(pc_plus_4M),
    .funct3M(funct3M),
    .syndrome_in(alu_syndrome_M),
    .crc_in(alu_crc_M),
    //.ctrl_parity_in(ctrl_parity_M),

    
    .reg_writeW(reg_writeW),
    .result_srcW(result_srcW),
    .alu_result_out(alu_result_out),
    .read_dataW(read_dataW),
    .rdW(rdW),
    .pc_plus_4W(pc_plus_4W),
    .funct3W(funct3W),
    .syndrome_out(alu_syndrome_W),
    .crc_out(alu_crc_W)
   //.ctrl_parity_out(ctrl_parity_W)
    );
    
    // Hazard Unit
    Hazard_unit H1(
    // Forwarding
    .rs1E(rs1E),
    .rs2E(rs2E),
    .rdM(rdM),
    .reg_writeM(reg_writeM),
    .rdW(rdW),
    .reg_writeW(reg_writeW),
    
    .forward_AE(forward_AE),
    .forward_BE(forward_BE),
    
    // Stalling
    .rs1D(instrD[19:15]),
    .rs2D(instrD[24:20]),
    .rdE(rdE),
    .result_srcE(result_srcE),
    // Branch stall: detect EX-stage RAW for branch source registers
    .branchD(branch),
    .reg_writeE(reg_writeE),
  
    .stall_F(stall_F),
    .stall_D(stall_D),
    .Flush_E(Flush_E),
    
    // Control Hazards
    .pc_srcE(pc_srcE),
    .Flush_D(Flush_D)
    );

    // ======================================================================
    // FAULT TOLERANCE: ARBIT RECOVERY CONTROLLER
    // ======================================================================
    arbit_controller arbit_unit (
        .clk(clk),
        .rst(rst),
        .uncorrectable_flag(uncorrectable_flag),
        .cfi_fault_sig(cfi_fault_sig),
        .ctrl_fault_E(ctrl_fault_E),
        .branch_flush(pc_srcE), // High if a branch is currently redirecting execution
        .wb_valid(wb_valid),
        .arbit_flush(arbit_flush),
        .arbit_redo(arbit_redo),
        .persistent_fault(persistent_fault)
    );
    
    // ======================================================================
    // Internal Units
    // ======================================================================
  
    assign pc_srcE = jumpE || branch_takenE;
    assign pc_targetE = (jalrE) ? (src_AE + imm_extE):(pcE + imm_extE);
    assign src_AE =     auipcE             ? pcE :          // AUIPC: use PC as operand A
                        (forward_AE == 2'b00)? rd1E:
                        (forward_AE == 2'b01)? write_back_data:
                        (forward_AE == 2'b10)? alu_resultM: 32'b0;
    
    assign write_dataE =(forward_BE == 2'b00)? rd2E:
                        (forward_BE == 2'b01)? write_back_data:
                        (forward_BE == 2'b10)? alu_resultM: 32'b0;

    assign ctrl_fault_E = ctrl_parity_E ^ (mem_writeE ^ reg_writeE ^ branchE ^ jumpE ^ jalrE); // Parity for control bundle in Execute stage

    assign redo_pc = (ctrl_fault_E) ? pcE : (pc_plus_4W - 32'd4);   // If control bundle in Execute stage is corrupted, redo from that PC;
                                                                    // else redo from the instruction that just completed Writeback (PC+4W - 4 = PC of instruction that just completed WB)    
                                           
    // ======================================================================
    // OUTPUT ASSIGNMENTS & WRITEBACK LOGIC
    // ======================================================================

    // Decode and Correct the ALU Result
    secded_decoder alu_decoder (
        .data_in(alu_result_out),         // Raw result arriving at WB
        .syndrome_in(alu_syndrome_W),     // Syndrome that traveled with it
        .data_out(corrected_alu_result),  // Safe, corrected candidate
        .single_error(single_error),
        .double_error(double_error)
    );

    // Recalculate CRC on the corrected candidate
    crc8_parallel alu_crc_validator (
        .data_in(corrected_alu_result), 
        .crc_out(validation_crc)
    );

    // Aliasing Rejection Logic
    // Verify the CRC of the proposed correction matches the original CRC
    assign crc_match = (validation_crc == alu_crc_W);
    
    // Flag ARBIT if uncorrectable (double error OR aliased single error)
    assign uncorrectable_flag = double_error | (single_error & ~crc_match);


    // Data Memory Connections
    // mem_write uses mem_writeM (MEM stage) — aligned with alu_resultM and write_dataM
    assign mem_write       = mem_writeM;
    assign data_mem_addr   = alu_resultM;
    assign data_mem_wdata  = write_dataM;
    assign data_mem_funct3 = funct3M;       // Byte/half/word select for stores
    
    // Writeback Mux — handles ALU result, Jump return address,
    // and all LOAD variants (LW, LH, LHU, LB, LBU)
    always @(*) begin
        case(result_srcW)
            // ALU result (R-Type, I-Type, AUIPC, LUI)
            2'b00: write_back_data = corrected_alu_result;
            
            // Load — apply size/sign extension using funct3W + byte offset from address
            2'b01: begin
                case(funct3W)
                    // LB  — signed byte
                    3'b000: case(corrected_alu_result[1:0])
                                2'b00: write_back_data = {{24{read_dataW[7]}},  read_dataW[7:0]};
                                2'b01: write_back_data = {{24{read_dataW[15]}}, read_dataW[15:8]};
                                2'b10: write_back_data = {{24{read_dataW[23]}}, read_dataW[23:16]};
                                2'b11: write_back_data = {{24{read_dataW[31]}}, read_dataW[31:24]};
                                default: write_back_data = 32'b0;
                            endcase
                    // LH  — signed half-word
                    3'b001: case(corrected_alu_result[1])
                                1'b0: write_back_data = {{16{read_dataW[15]}}, read_dataW[15:0]};
                                1'b1: write_back_data = {{16{read_dataW[31]}}, read_dataW[31:16]};
                                default: write_back_data = 32'b0;
                            endcase
                    // LW  — full word, pass through
                    3'b010: write_back_data = read_dataW;
                    // LBU — unsigned byte
                    3'b100: case(corrected_alu_result[1:0])
                                2'b00: write_back_data = {24'b0, read_dataW[7:0]};
                                2'b01: write_back_data = {24'b0, read_dataW[15:8]};
                                2'b10: write_back_data = {24'b0, read_dataW[23:16]};
                                2'b11: write_back_data = {24'b0, read_dataW[31:24]};
                                default: write_back_data = 32'b0;
                            endcase
                    // LHU — unsigned half-word
                    3'b101: case(corrected_alu_result[1])
                                1'b0: write_back_data = {16'b0, read_dataW[15:0]};
                                1'b1: write_back_data = {16'b0, read_dataW[31:16]};
                                default: write_back_data = 32'b0;
                            endcase
                    // Default (safety): treat as LW
                    default: write_back_data = read_dataW;
                endcase
            end
            
            // JAL/JALR — return address (PC+4)
            2'b10: write_back_data = pc_plus_4W;
            
            default: write_back_data = 32'b0;
        endcase   
    end
endmodule
