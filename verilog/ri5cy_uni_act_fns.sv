`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/27/2026 11:57:26 AM
// Design Name: 
// Module Name: ri5cy_uni_act_fns
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

// Top level wrapper for RI5CY

// Copyright (C) 2017 Embecosm Limited <www.embecosm.com>

// Contributor: Jeremy Bennett <jeremy.bennett@embecosm.com>

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along with
// this program.  If not, see <http://www.gnu.org/licenses/>.


// This instantiates memory and (eventually) a debug interface for the core.


module top
#(
  parameter INSTR_RDATA_WIDTH = 128,
  parameter ADDR_WIDTH        = 13,
  parameter BOOT_ADDR         = 'h80,
  // ---------------- HDC parameters ----------------
  parameter HDC_D          = 1760,   // HDC_D / HDC_BLOCK_SIZE must == 80
  parameter HDC_BLOCK_SIZE = 22,     // and HDC_D % CHUNK_SIZE(110) == 0
  parameter HDC_FUNC_LSB   = 16      // func_sel = data_path[HDC_FUNC_LSB +: 2]
)
(
  input  logic        clk_i,
  input  logic        rstn_i,
  input  logic [31:0] irq_i,

  input  logic        debug_req_i,
  output logic        debug_gnt_o,
  output logic        debug_rvalid_o,
  input  logic [14:0] debug_addr_i,
  input  logic        debug_we_i,
  input  logic [31:0] debug_wdata_i,
  output logic [31:0] debug_rdata_o,
  output logic        debug_halted_o,

  input  logic        load_i,
  input  logic        fetch_enable_i,
  output logic        core_busy_o,
  output logic        top_done,

  output logic signed [15:0] y          // HDC activation result
);

  // ---- core <-> memory signals ----
  logic                  instr_req, instr_gnt, instr_rvalid;
  logic [ADDR_WIDTH-1:0] instr_addr;
  logic [127:0]          instr_rdata;

  logic                  data_req, data_gnt, data_rvalid, data_we;
  logic [ADDR_WIDTH-1:0] data_addr;
  logic [3:0]            data_be;
  logic [31:0]           data_rdata;
  logic [31:0]           data_wdata;

  logic                  start;   // custom HDC trigger from core

  // =====================================================================
  //  CORE
  // =====================================================================
  riscv_core #(.INSTR_RDATA_WIDTH(INSTR_RDATA_WIDTH)) riscv_core_i (
     .clk_i               ( clk_i          ),
     .rst_ni              ( rstn_i         ),
     .clock_en_i          ( '1             ),
     .test_en_i           ( '1             ),
     .boot_addr_i         ( BOOT_ADDR      ),
     .core_id_i           ( 4'h0           ),
     .cluster_id_i        ( 6'h0           ),
     .instr_addr_o        ( instr_addr     ),
     .instr_req_o         ( instr_req      ),
     .instr_rdata_i       ( instr_rdata    ),
     .instr_gnt_i         ( instr_gnt      ),
     .instr_rvalid_i      ( instr_rvalid   ),
     .data_addr_o         ( data_addr      ),
     .data_wdata_o        ( data_wdata     ),
     .data_we_o           ( data_we        ),
     .data_req_o          ( data_req       ),
     .data_be_o           ( data_be        ),
     .data_rdata_i        ( data_rdata     ),
     .data_gnt_i          ( data_gnt       ),
     .data_rvalid_i       ( data_rvalid    ),
     .data_err_i          ( 1'b0           ),
     .irq_i               ( irq_i          ),
     .debug_req_i         ( debug_req_i    ),
     .debug_gnt_o         ( debug_gnt_o    ),
     .debug_rvalid_o      ( debug_rvalid_o ),
     .debug_addr_i        ( debug_addr_i   ),
     .debug_we_i          ( debug_we_i     ),
     .debug_wdata_i       ( debug_wdata_i  ),
     .debug_rdata_o       ( debug_rdata_o  ),
     .debug_halted_o      ( debug_halted_o ),
     .debug_halt_i        ( 1'b0           ),
     .debug_resume_i      ( 1'b0           ),
     .fetch_enable_i      ( fetch_enable_i ),
     .core_busy_o         ( core_busy_o    ),
     .ext_perf_counters_i (                ),
     .start               ( start          )
  );

  // =====================================================================
  //  MEMORY  (N removed; 16-bit result path)
  // =====================================================================
  ram #(.ADDR_WIDTH(ADDR_WIDTH - 2)) ram_i (
     .clk            ( clk_i        ),
     .instr_req_i    ( instr_req    ),
     .instr_addr_i   ( instr_addr   ),
     .instr_rdata_o  ( instr_rdata  ),
     .instr_rvalid_o ( instr_rvalid ),
     .instr_gnt_o    ( instr_gnt    ),
     .data_req_i     ( data_req     ),
     .data_addr_i    ( data_addr    ),
     .data_we_i      ( data_we      ),
     .data_be_i      ( data_be      ),
     .data_wdata_i   ( data_wdata   ),
     .data_rdata_o   ( data_rdata   ),
     .data_rvalid_o  ( data_rvalid  ),
     .data_gnt_o     ( data_gnt     ),
     .y_i            ( y            )   // full signed 16-bit result
  );

  // =====================================================================
  //  HDC ACCELERATOR
  //    data_path[15:0]  -> x_in
  //    data_path[17:16] -> func_sel
  // =====================================================================
  hdc_split #(
     .D          ( HDC_D          ),
     .BLOCK_SIZE ( HDC_BLOCK_SIZE ),
     .FUNC_LSB   ( HDC_FUNC_LSB   )
  ) hdc_split_i (
     .clk        ( clk_i       ),
     .rst        ( !rstn_i     ),
     .top_start  ( start       ),
     .data_path  ( data_rdata  ),
     .load       ( data_rvalid ),
     .top_done   ( top_done    ),
     .y          ( y           )
  );

endmodule

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Matthias Baer - baermatt@student.ethz.ch                   //
//                                                                            //
// Additional contributions by:                                               //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
//                                                                            //
// Design Name:    RISC-V processor core                                      //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Defines for various constants used by the processor core.  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

package riscv_defines;

////////////////////////////////////////////////
//    ___         ____          _             //
//   / _ \ _ __  / ___|___   __| | ___  ___   //
//  | | | | '_ \| |   / _ \ / _` |/ _ \/ __|  //
//  | |_| | |_) | |__| (_) | (_| |  __/\__ \  //
//   \___/| .__/ \____\___/ \__,_|\___||___/  //
//        |_|                                 //
////////////////////////////////////////////////

parameter OPCODE_SYSTEM    = 7'h73;
parameter OPCODE_FENCE     = 7'h0f;
parameter OPCODE_OP        = 7'h33;
parameter OPCODE_OPIMM     = 7'h13;
parameter OPCODE_STORE     = 7'h23;
parameter OPCODE_LOAD      = 7'h03;
parameter OPCODE_BRANCH    = 7'h63;
parameter OPCODE_JALR      = 7'h67;
parameter OPCODE_JAL       = 7'h6f;
parameter OPCODE_AUIPC     = 7'h17;
parameter OPCODE_LUI       = 7'h37;
//parameter OPCODE_HPC     = 7'h4a;

parameter OPCODE_HDC       = 7'h6b;

// those opcodes are now used for PULP custom instructions
// parameter OPCODE_CUST0     = 7'h0b
// parameter OPCODE_CUST1     = 7'h2b

// PULP custom
parameter OPCODE_LOAD_POST  = 7'h0b;
parameter OPCODE_STORE_POST = 7'h2b;
parameter OPCODE_PULP_OP    = 7'h5b;
parameter OPCODE_VECOP      = 7'h57;
parameter OPCODE_HWLOOP     = 7'h7b;

parameter REGC_S1   = 2'b10;
parameter REGC_RD   = 2'b01;
parameter REGC_ZERO = 2'b11;


//////////////////////////////////////////////////////////////////////////////
//      _    _    _   _    ___                       _   _                  //
//     / \  | |  | | | |  / _ \ _ __   ___ _ __ __ _| |_(_) ___  _ __  ___  //
//    / _ \ | |  | | | | | | | | '_ \ / _ \ '__/ _` | __| |/ _ \| '_ \/ __| //
//   / ___ \| |__| |_| | | |_| | |_) |  __/ | | (_| | |_| | (_) | | | \__ \ //
//  /_/   \_\_____\___/   \___/| .__/ \___|_|  \__,_|\__|_|\___/|_| |_|___/ //
//                             |_|                                          //
//////////////////////////////////////////////////////////////////////////////

parameter ALU_OP_WIDTH = 6;

parameter ALU_ADD   = 6'b011000;
parameter ALU_SUB   = 6'b011001;
parameter ALU_ADDU  = 6'b011010;
parameter ALU_SUBU  = 6'b011011;
parameter ALU_ADDR  = 6'b011100;
parameter ALU_SUBR  = 6'b011101;
parameter ALU_ADDUR = 6'b011110;
parameter ALU_SUBUR = 6'b011111;

parameter ALU_XOR   = 6'b101111;
parameter ALU_OR    = 6'b101110;
parameter ALU_AND   = 6'b010101;

// Shifts
parameter ALU_SRA   = 6'b100100;
parameter ALU_SRL   = 6'b100101;
parameter ALU_ROR   = 6'b100110;
parameter ALU_SLL   = 6'b100111;

// bit manipulation
parameter ALU_BEXT  = 6'b101000;
parameter ALU_BEXTU = 6'b101001;
parameter ALU_BINS  = 6'b101010;
parameter ALU_BCLR  = 6'b101011;
parameter ALU_BSET  = 6'b101100;

// Bit counting
parameter ALU_FF1   = 6'b110110;
parameter ALU_FL1   = 6'b110111;
parameter ALU_CNT   = 6'b110100;
parameter ALU_CLB   = 6'b110101;

// Sign-/zero-extensions
parameter ALU_EXTS  = 6'b111110;
parameter ALU_EXT   = 6'b111111;

// Comparisons
parameter ALU_LTS   = 6'b000000;
parameter ALU_LTU   = 6'b000001;
parameter ALU_LES   = 6'b000100;
parameter ALU_LEU   = 6'b000101;
parameter ALU_GTS   = 6'b001000;
parameter ALU_GTU   = 6'b001001;
parameter ALU_GES   = 6'b001010;
parameter ALU_GEU   = 6'b001011;
parameter ALU_EQ    = 6'b001100;
parameter ALU_NE    = 6'b001101;

// Set Lower Than operations
parameter ALU_SLTS  = 6'b000010;
parameter ALU_SLTU  = 6'b000011;
parameter ALU_SLETS = 6'b000110;
parameter ALU_SLETU = 6'b000111;

// Absolute value
parameter ALU_ABS   = 6'b010100;
parameter ALU_CLIP  = 6'b010110;
parameter ALU_CLIPU = 6'b010111;

// Insert/extract
parameter ALU_INS   = 6'b101101;

// min/max
parameter ALU_MIN   = 6'b010000;
parameter ALU_MINU  = 6'b010001;
parameter ALU_MAX   = 6'b010010;
parameter ALU_MAXU  = 6'b010011;

// div/rem
parameter ALU_DIVU  = 6'b110000; // bit 0 is used for signed mode, bit 1 is used for remdiv
parameter ALU_DIV   = 6'b110001; // bit 0 is used for signed mode, bit 1 is used for remdiv
parameter ALU_REMU  = 6'b110010; // bit 0 is used for signed mode, bit 1 is used for remdiv
parameter ALU_REM   = 6'b110011; // bit 0 is used for signed mode, bit 1 is used for remdiv

parameter ALU_SHUF  = 6'b111010;
parameter ALU_SHUF2 = 6'b111011;
parameter ALU_PCKLO = 6'b111000;
parameter ALU_PCKHI = 6'b111001;


parameter MUL_MAC32 = 3'b000;
parameter MUL_MSU32 = 3'b001;
parameter MUL_I     = 3'b010;
parameter MUL_IR    = 3'b011;
parameter MUL_DOT8  = 3'b100;
parameter MUL_DOT16 = 3'b101;
parameter MUL_H     = 3'b110;

// vector modes
parameter VEC_MODE32 = 2'b00;
parameter VEC_MODE16 = 2'b10;
parameter VEC_MODE8  = 2'b11;


/////////////////////////////////////////////////////////
//    ____ ____    ____            _     _             //
//   / ___/ ___|  |  _ \ ___  __ _(_)___| |_ ___ _ __  //
//  | |   \___ \  | |_) / _ \/ _` | / __| __/ _ \ '__| //
//  | |___ ___) | |  _ <  __/ (_| | \__ \ ||  __/ |    //
//   \____|____/  |_| \_\___|\__, |_|___/\__\___|_|    //
//                           |___/                     //
/////////////////////////////////////////////////////////

// CSR operations
parameter CSR_OP_NONE  = 2'b00;
parameter CSR_OP_WRITE = 2'b01;
parameter CSR_OP_SET   = 2'b10;
parameter CSR_OP_CLEAR = 2'b11;


// SPR for debugger, not accessible by CPU
parameter SP_DVR0       = 16'h3000;
parameter SP_DCR0       = 16'h3008;
parameter SP_DMR1       = 16'h3010;
parameter SP_DMR2       = 16'h3011;

parameter SP_DVR_MSB = 8'h00;
parameter SP_DCR_MSB = 8'h01;
parameter SP_DMR_MSB = 8'h02;
parameter SP_DSR_MSB = 8'h04;


///////////////////////////////////////////////
//   ___ ____    ____  _                     //
//  |_ _|  _ \  / ___|| |_ __ _  __ _  ___   //
//   | || | | | \___ \| __/ _` |/ _` |/ _ \  //
//   | || |_| |  ___) | || (_| | (_| |  __/  //
//  |___|____/  |____/ \__\__,_|\__, |\___|  //
//                              |___/        //
///////////////////////////////////////////////

// forwarding operand mux
parameter SEL_REGFILE      = 2'b00;
parameter SEL_FW_EX        = 2'b01;
parameter SEL_FW_WB        = 2'b10;

// operand a selection
parameter OP_A_REGA_OR_FWD = 2'b00;
parameter OP_A_CURRPC      = 2'b01;
parameter OP_A_IMM         = 2'b10;
parameter OP_A_REGB_OR_FWD = 2'b11;

// immediate a selection
parameter IMMA_Z      = 1'b0;
parameter IMMA_ZERO   = 1'b1;

// operand b selection
parameter OP_B_REGB_OR_FWD = 2'b00;
parameter OP_B_REGC_OR_FWD = 2'b01;
parameter OP_B_IMM         = 2'b10;

// immediate b selection
parameter IMMB_I      = 4'b0000;
parameter IMMB_S      = 4'b0001;
parameter IMMB_U      = 4'b0010;
parameter IMMB_PCINCR = 4'b0011;
parameter IMMB_S2     = 4'b0100;
parameter IMMB_S3     = 4'b0101;
parameter IMMB_VS     = 4'b0110;
parameter IMMB_VU     = 4'b0111;
parameter IMMB_SHUF   = 4'b1000;
parameter IMMB_CLIP   = 4'b1001;
parameter IMMB_BI     = 4'b1011;

// bit mask selection
parameter BMASK_A_ZERO = 1'b0;
parameter BMASK_A_S3   = 1'b1;

parameter BMASK_B_S2   = 2'b00;
parameter BMASK_B_S3   = 2'b01;
parameter BMASK_B_ZERO = 2'b10;
parameter BMASK_B_ONE  = 2'b11;

// multiplication immediates
parameter MIMM_ZERO    = 1'b0;
parameter MIMM_S3      = 1'b1;

// operand c selection
parameter OP_C_REGC_OR_FWD = 2'b00;
parameter OP_C_REGB_OR_FWD = 2'b01;
parameter OP_C_JT          = 2'b10;

// branch types
parameter BRANCH_NONE = 2'b00;
parameter BRANCH_JAL  = 2'b01;
parameter BRANCH_JALR = 2'b10;
parameter BRANCH_COND = 2'b11; // conditional branches

// jump target mux
parameter JT_JAL  = 2'b01;
parameter JT_JALR = 2'b10;
parameter JT_COND = 2'b11;


///////////////////////////////////////////////
//   ___ _____   ____  _                     //
//  |_ _|  ___| / ___|| |_ __ _  __ _  ___   //
//   | || |_    \___ \| __/ _` |/ _` |/ _ \  //
//   | ||  _|    ___) | || (_| | (_| |  __/  //
//  |___|_|     |____/ \__\__,_|\__, |\___|  //
//                              |___/        //
///////////////////////////////////////////////

// PC mux selector defines
parameter PC_BOOT          = 3'b000;
parameter PC_JUMP          = 3'b010;
parameter PC_BRANCH        = 3'b011;
parameter PC_EXCEPTION     = 3'b100;
parameter PC_ERET          = 3'b101;
parameter PC_DBG_NPC       = 3'b111;

// Exception PC mux selector defines
parameter EXC_PC_ILLINSN   = 2'b00;
parameter EXC_PC_ECALL     = 2'b01;
parameter EXC_PC_LOAD      = 2'b10;
parameter EXC_PC_STORE     = 2'b10;
parameter EXC_PC_IRQ       = 2'b11;

// Exceptions offsets
// target address = {boot_addr[31:8], EXC_OFF} (boot_addr must be 32 BYTE aligned!)
// offset 00 to 7e is used for external interrupts
parameter EXC_OFF_RST      = 8'h80;
parameter EXC_OFF_ILLINSN  = 8'h84;
parameter EXC_OFF_ECALL    = 8'h88;
parameter EXC_OFF_LSUERR   = 8'h8c;


// Debug module
parameter DBG_SETS_W = 6;

parameter DBG_SETS_IRQ    = 5;
parameter DBG_SETS_ECALL  = 4;
parameter DBG_SETS_EILL   = 3;
parameter DBG_SETS_ELSU   = 2;
parameter DBG_SETS_EBRK   = 1;
parameter DBG_SETS_SSTE   = 0;

parameter DBG_CAUSE_HALT   = 6'h1F;

endpackage

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the âLicenseâ?); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an âAS ISâ? BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Matthias Baer - baermatt@student.ethz.ch                   //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Andreas Traber - atraber@student.ethz.ch                   //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
// Design Name:    Top level module                                           //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Top level module of the RISC-V core.                       //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

//`include "riscv_config.sv"

import riscv_defines::*;

module riscv_core
#(
  parameter N_EXT_PERF_COUNTERS = 0,
  parameter INSTR_RDATA_WIDTH   = 32
)
(
  // Clock and Reset
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic        clock_en_i,    // enable clock, otherwise it is gated
  input  logic        test_en_i,     // enable all clock gates for testing

  // Core ID, Cluster ID and boot address are considered more or less static
  input  logic [31:0] boot_addr_i,
  input  logic [ 3:0] core_id_i,
  input  logic [ 5:0] cluster_id_i,

  // Instruction memory interface
  output logic                         instr_req_o,
  input  logic                         instr_gnt_i,
  input  logic                         instr_rvalid_i,
  output logic                  [31:0] instr_addr_o,
  input  logic [INSTR_RDATA_WIDTH-1:0] instr_rdata_i,

  // Data memory interface
  output logic        data_req_o,
  input  logic        data_gnt_i,
  input  logic        data_rvalid_i,
  output logic        data_we_o,
  output logic [3:0]  data_be_o,
  output logic [31:0] data_addr_o,
  output logic [31:0] data_wdata_o,
  input  logic [31:0] data_rdata_i,
  input  logic        data_err_i,

  // Interrupt inputs
  input  logic [31:0] irq_i,                 // level sensitive IR lines

  // Debug Interface
  input  logic        debug_req_i,
  output logic        debug_gnt_o,
  output logic        debug_rvalid_o,
  input  logic [14:0] debug_addr_i,
  input  logic        debug_we_i,
  input  logic [31:0] debug_wdata_i,
  output logic [31:0] debug_rdata_o,
  output logic        debug_halted_o,
  input  logic        debug_halt_i,
  input  logic        debug_resume_i,

  // CPU Control Signals
  input  logic        fetch_enable_i,
  output logic        core_busy_o,

  input  logic [N_EXT_PERF_COUNTERS-1:0] ext_perf_counters_i,
   
  //HDC predicted class output from CSRegisters
//  output logic [3:0]  hdc_predicted_class,
  output logic [31:0] y,
  output logic        start
);

  localparam N_HWLP      = 2;
  localparam N_HWLP_BITS = $clog2(N_HWLP);

  // IF/ID signals
  logic              is_hwlp_id;
  logic [N_HWLP-1:0] hwlp_dec_cnt_id;
  logic              instr_valid_id;
  logic [31:0]       instr_rdata_id;    // Instruction sampled inside IF stage
  logic              is_compressed_id;
  logic              illegal_c_insn_id; // Illegal compressed instruction sent to ID stage
  logic [31:0]       pc_if;             // Program counter in IF stage
  logic [31:0]       pc_id;             // Program counter in ID stage

  logic              clear_instr_valid;
  logic              pc_set;
  logic [2:0]        pc_mux_id;     // Mux selector for next PC
  logic [1:0]        exc_pc_mux_id;     // Mux selector for exception PC
  logic [4:0]        exc_vec_pc_mux_id; // Mux selector for vectorized IR lines

  logic              lsu_load_err;
  logic              lsu_store_err;

  // ID performance counter signals
  logic        is_decoding;

  logic        useincr_addr_ex;   // Active when post increment
  logic        data_misaligned;

  logic        mult_multicycle;

  // Jump and branch target and decision (EX->IF)
  logic [31:0] jump_target_id, jump_target_ex;
  logic        branch_in_ex;
  logic        branch_decision;

  logic        ctrl_busy;
  logic        if_busy;
  logic        lsu_busy;


  logic [31:0] pc_ex; // PC of last executed branch or p.elw

  // ALU Control
  logic [ALU_OP_WIDTH-1:0] alu_operator_ex;
  logic [31:0] alu_operand_a_ex;
  logic [31:0] alu_operand_b_ex;
  logic [31:0] alu_operand_c_ex;
  logic [ 4:0] bmask_a_ex;
  logic [ 4:0] bmask_b_ex;
  logic [ 1:0] imm_vec_ext_ex;
  logic [ 1:0] alu_vec_mode_ex;

  // Multiplier Control
  logic [ 2:0] mult_operator_ex;
  logic [31:0] mult_operand_a_ex;
  logic [31:0] mult_operand_b_ex;
  logic [31:0] mult_operand_c_ex;
  logic        mult_en_ex;
  logic        mult_sel_subword_ex;
  logic [ 1:0] mult_signed_mode_ex;
  logic [ 4:0] mult_imm_ex;
  logic [31:0] mult_dot_op_a_ex;
  logic [31:0] mult_dot_op_b_ex;
  logic [31:0] mult_dot_op_c_ex;
  logic [ 1:0] mult_dot_signed_ex;

  // Register Write Control
  logic [4:0]  regfile_waddr_ex;
  logic        regfile_we_ex;
  logic [4:0]  regfile_waddr_fw_wb_o;        // From WB to ID
  logic        regfile_we_wb;
  logic [31:0] regfile_wdata;

  logic [4:0]  regfile_alu_waddr_ex;
  logic        regfile_alu_we_ex;

  logic [4:0]  regfile_alu_waddr_fw;
  logic        regfile_alu_we_fw;
  logic [31:0] regfile_alu_wdata_fw;

  // CSR control
  logic        csr_access_ex;
  logic  [1:0] csr_op_ex;

  logic        csr_access;
  logic  [1:0] csr_op;
  logic [11:0] csr_addr;
  logic [11:0] csr_addr_int;
  logic [31:0] csr_rdata;
  logic [31:0] csr_wdata;

  // Data Memory Control:  From ID stage (id-ex pipe) <--> load store unit
  logic        data_we_ex;
  logic [1:0]  data_type_ex;
  logic        data_sign_ext_ex;
  logic [1:0]  data_reg_offset_ex;
  logic        data_req_ex;
  logic [31:0] data_pc_ex;
  logic        data_load_event_ex;
  logic        data_misaligned_ex;

  // stall control
  logic        halt_if;
  logic        if_ready;
  logic        id_ready;
  logic        ex_ready;

  logic        if_valid;
  logic        id_valid;
  logic        ex_valid;
  logic        wb_valid;

  logic        lsu_ready_ex;
  logic        lsu_ready_wb;

  // Signals between instruction core interface and pipe (if and id stages)
  logic        instr_req_int;    // Id stage asserts a req to instruction core interface

  // Interrupts
  logic        irq_enable;
  logic [31:0] mepc;

  logic [5:0]  exc_cause;
  logic        save_exc_cause;
  logic        exc_save_if;
  logic        exc_save_id;
  logic        exc_restore_id;


  // Hardware loop controller signals
  logic [N_HWLP-1:0] [31:0] hwlp_start;
  logic [N_HWLP-1:0] [31:0] hwlp_end;
  logic [N_HWLP-1:0] [31:0] hwlp_cnt;

  // used to write from CS registers to hardware loop registers
  logic   [N_HWLP_BITS-1:0] csr_hwlp_regid;
  logic               [2:0] csr_hwlp_we;
  logic              [31:0] csr_hwlp_data;


  // Debug Unit
  logic [DBG_SETS_W-1:0] dbg_settings;
  logic        dbg_req;
  logic        dbg_ack;
  logic        dbg_stall;
  logic        dbg_trap;

  // Debug GPR Read Access
  logic        dbg_reg_rreq;
  logic [ 4:0] dbg_reg_raddr;
  logic [31:0] dbg_reg_rdata;

  // Debug GPR Write Access
  logic        dbg_reg_wreq;
  logic [ 4:0] dbg_reg_waddr;
  logic [31:0] dbg_reg_wdata;

  // Debug CSR Access
  logic        dbg_csr_req;
  logic [11:0] dbg_csr_addr;
  logic        dbg_csr_we;
  logic [31:0] dbg_csr_wdata;

  logic [31:0] dbg_jump_addr;
  logic        dbg_jump_req;

  // Performance Counters
  logic        perf_imiss;
  logic        perf_jump;
  logic        perf_jr_stall;
  logic        perf_ld_stall;


  //////////////////////////////////////////////////////////////////////////////////////////////
  //   ____ _            _      __  __                                                   _    //
  //  / ___| | ___   ___| | __ |  \/  | __ _ _ __   __ _  __ _  ___ _ __ ___   ___ _ __ | |_  //
  // | |   | |/ _ \ / __| |/ / | |\/| |/ _` | '_ \ / _` |/ _` |/ _ \ '_ ` _ \ / _ \ '_ \| __| //
  // | |___| | (_) | (__|   <  | |  | | (_| | | | | (_| | (_| |  __/ | | | | |  __/ | | | |_  //
  //  \____|_|\___/ \___|_|\_\ |_|  |_|\__,_|_| |_|\__,_|\__, |\___|_| |_| |_|\___|_| |_|\__| //
  //                                                     |___/                                //
  //////////////////////////////////////////////////////////////////////////////////////////////

  logic        clk;

  logic        clock_en;
  logic        dbg_busy;

  logic        sleeping;

  // if we are sleeping on a barrier let's just wait on the instruction
  // interface to finish loading instructions
  assign core_busy_o = (data_load_event_ex & data_req_o) ? if_busy : (if_busy | ctrl_busy | lsu_busy);

  assign dbg_busy = dbg_req | dbg_csr_req | dbg_jump_req | dbg_reg_wreq | debug_req_i;

  assign clock_en = clock_en_i | core_busy_o | dbg_busy;

  assign sleeping = (~fetch_enable_i) & (~core_busy_o);


  // main clock gate of the core
  // generates all clocks except the one for the debug unit which is
  // independent
  cluster_clock_gating core_clock_gate_i
  (
    .clk_i     ( clk_i           ),
    .en_i      ( clock_en        ),
    .test_en_i ( test_en_i       ),
    .clk_o     ( clk             )
  );

  //////////////////////////////////////////////////
  //   ___ _____   ____ _____  _    ____ _____    //
  //  |_ _|  ___| / ___|_   _|/ \  / ___| ____|   //
  //   | || |_    \___ \ | | / _ \| |  _|  _|     //
  //   | ||  _|    ___) || |/ ___ \ |_| | |___    //
  //  |___|_|     |____/ |_/_/   \_\____|_____|   //
  //                                              //
  //////////////////////////////////////////////////
  riscv_if_stage
  #(
    .N_HWLP              ( N_HWLP            ),
    .RDATA_WIDTH         ( INSTR_RDATA_WIDTH )
  )
  if_stage_i
  (
    .clk                 ( clk               ),
    .rst_n               ( rst_ni            ),

    // boot address (trap vector location)
    .boot_addr_i         ( boot_addr_i       ),

    // instruction request control
    .req_i               ( instr_req_int     ),

    // instruction cache interface
    .instr_req_o         ( instr_req_o       ),
    .instr_addr_o        ( instr_addr_o      ),
    .instr_gnt_i         ( instr_gnt_i       ),
    .instr_rvalid_i      ( instr_rvalid_i    ),
    .instr_rdata_i       ( instr_rdata_i     ),

    // outputs to ID stage
    .hwlp_dec_cnt_id_o   ( hwlp_dec_cnt_id   ),
    .is_hwlp_id_o        ( is_hwlp_id        ),
    .instr_valid_id_o    ( instr_valid_id    ),
    .instr_rdata_id_o    ( instr_rdata_id    ),
    .is_compressed_id_o  ( is_compressed_id  ),
    .illegal_c_insn_id_o ( illegal_c_insn_id ),
    .pc_if_o             ( pc_if             ),
    .pc_id_o             ( pc_id             ),

    // control signals
    .clear_instr_valid_i ( clear_instr_valid ),
    .pc_set_i            ( pc_set            ),
    .exception_pc_reg_i  ( mepc              ), // exception return address
    .pc_mux_i            ( pc_mux_id         ), // sel for pc multiplexer
    .exc_pc_mux_i        ( exc_pc_mux_id     ),
    .exc_vec_pc_mux_i    ( exc_vec_pc_mux_id ),

    // from hwloop registers
    .hwlp_start_i        ( hwlp_start        ),
    .hwlp_end_i          ( hwlp_end          ),
    .hwlp_cnt_i          ( hwlp_cnt          ),

    // from debug unit
    .dbg_jump_addr_i     ( dbg_jump_addr     ),
    .dbg_jump_req_i      ( dbg_jump_req      ),

    // Jump targets
    .jump_target_id_i    ( jump_target_id    ),
    .jump_target_ex_i    ( jump_target_ex    ),

    // pipeline stalls
    .halt_if_i           ( halt_if           ),
    .if_ready_o          ( if_ready          ),
    .id_ready_i          ( id_ready          ),
    .if_valid_o          ( if_valid          ),

    .if_busy_o           ( if_busy           ),
    .perf_imiss_o        ( perf_imiss        )
  );


  /////////////////////////////////////////////////
  //   ___ ____    ____ _____  _    ____ _____   //
  //  |_ _|  _ \  / ___|_   _|/ \  / ___| ____|  //
  //   | || | | | \___ \ | | / _ \| |  _|  _|    //
  //   | || |_| |  ___) || |/ ___ \ |_| | |___   //
  //  |___|____/  |____/ |_/_/   \_\____|_____|  //
  //                                             //
  /////////////////////////////////////////////////
  riscv_id_stage
  #(
    .N_HWLP                       ( N_HWLP               )
  )
  id_stage_i
  (
    .clk                          ( clk                  ),
    .rst_n                        ( rst_ni               ),

    .test_en_i                    ( test_en_i            ),

    // Processor Enable
    .fetch_enable_i               ( fetch_enable_i       ),
    .ctrl_busy_o                  ( ctrl_busy            ),
    .is_decoding_o                ( is_decoding          ),

    // Interface to instruction memory
    .hwlp_dec_cnt_i               ( hwlp_dec_cnt_id      ),
    .is_hwlp_i                    ( is_hwlp_id           ),
    .instr_valid_i                ( instr_valid_id       ),
    .instr_rdata_i                ( instr_rdata_id       ),
    .instr_req_o                  ( instr_req_int        ),

    // Jumps and branches
    .branch_in_ex_o               ( branch_in_ex         ),
    .branch_decision_i            ( branch_decision      ),
    .jump_target_o                ( jump_target_id       ),

    // IF and ID control signals
    .clear_instr_valid_o          ( clear_instr_valid    ),
    .pc_set_o                     ( pc_set               ),
    .pc_mux_o                     ( pc_mux_id            ),
    .exc_pc_mux_o                 ( exc_pc_mux_id        ),
    .exc_vec_pc_mux_o             ( exc_vec_pc_mux_id    ),

    .illegal_c_insn_i             ( illegal_c_insn_id    ),
    .is_compressed_i              ( is_compressed_id     ),

    .pc_if_i                      ( pc_if                ),
    .pc_id_i                      ( pc_id                ),

    // Stalls
    .halt_if_o                    ( halt_if              ),

    .if_ready_i                   ( if_ready             ),
    .id_ready_o                   ( id_ready             ),
    .ex_ready_i                   ( ex_ready             ),

    .if_valid_i                   ( if_valid             ),
    .id_valid_o                   ( id_valid             ),
    .ex_valid_i                   ( ex_valid             ),
    .wb_valid_i                   ( wb_valid             ),

    // From the Pipeline ID/EX
    .pc_ex_o                      ( pc_ex                ),

    .alu_operator_ex_o            ( alu_operator_ex      ),
    .alu_operand_a_ex_o           ( alu_operand_a_ex     ),
    .alu_operand_b_ex_o           ( alu_operand_b_ex     ),
    .alu_operand_c_ex_o           ( alu_operand_c_ex     ),
    .bmask_a_ex_o                 ( bmask_a_ex           ),
    .bmask_b_ex_o                 ( bmask_b_ex           ),
    .imm_vec_ext_ex_o             ( imm_vec_ext_ex       ),
    .alu_vec_mode_ex_o            ( alu_vec_mode_ex      ),

    .regfile_waddr_ex_o           ( regfile_waddr_ex     ),
    .regfile_we_ex_o              ( regfile_we_ex        ),

    .regfile_alu_we_ex_o          ( regfile_alu_we_ex    ),
    .regfile_alu_waddr_ex_o       ( regfile_alu_waddr_ex ),

    // MUL
    .mult_operator_ex_o           ( mult_operator_ex     ), // from ID to EX stage
    .mult_en_ex_o                 ( mult_en_ex           ), // from ID to EX stage
    .mult_sel_subword_ex_o        ( mult_sel_subword_ex  ), // from ID to EX stage
    .mult_signed_mode_ex_o        ( mult_signed_mode_ex  ), // from ID to EX stage
    .mult_operand_a_ex_o          ( mult_operand_a_ex    ), // from ID to EX stage
    .mult_operand_b_ex_o          ( mult_operand_b_ex    ), // from ID to EX stage
    .mult_operand_c_ex_o          ( mult_operand_c_ex    ), // from ID to EX stage
    .mult_imm_ex_o                ( mult_imm_ex          ), // from ID to EX stage

    .mult_dot_op_a_ex_o           ( mult_dot_op_a_ex     ), // from ID to EX stage
    .mult_dot_op_b_ex_o           ( mult_dot_op_b_ex     ), // from ID to EX stage
    .mult_dot_op_c_ex_o           ( mult_dot_op_c_ex     ), // from ID to EX stage
    .mult_dot_signed_ex_o         ( mult_dot_signed_ex   ), // from ID to EX stage

    // CSR ID/EX
    .csr_access_ex_o              ( csr_access_ex        ),
    .csr_op_ex_o                  ( csr_op_ex            ),

    // hardware loop signals to IF hwlp controller
    .hwlp_start_o                 ( hwlp_start           ),
    .hwlp_end_o                   ( hwlp_end             ),
    .hwlp_cnt_o                   ( hwlp_cnt             ),

    // hardware loop signals from CSR
    .csr_hwlp_regid_i             ( csr_hwlp_regid       ),
    .csr_hwlp_we_i                ( csr_hwlp_we          ),
    .csr_hwlp_data_i              ( csr_hwlp_data        ),

    // LSU
    .data_req_ex_o                ( data_req_ex          ), // to load store unit
    .data_we_ex_o                 ( data_we_ex           ), // to load store unit
    .data_type_ex_o               ( data_type_ex         ), // to load store unit
    .data_sign_ext_ex_o           ( data_sign_ext_ex     ), // to load store unit
    .data_reg_offset_ex_o         ( data_reg_offset_ex   ), // to load store unit
    .data_load_event_ex_o         ( data_load_event_ex   ), // to load store unit

    .data_misaligned_ex_o         ( data_misaligned_ex   ), // to load store unit

    .prepost_useincr_ex_o         ( useincr_addr_ex      ),
    .data_misaligned_i            ( data_misaligned      ),

    // Interrupt Signals
    .irq_i                        ( irq_i                ), // incoming interrupts
    .irq_enable_i                 ( irq_enable           ), // global interrupt enable
    .exc_cause_o                  ( exc_cause            ),
    .save_exc_cause_o             ( save_exc_cause       ),
    .exc_save_if_o                ( exc_save_if          ), // control signal to save pc
    .exc_save_id_o                ( exc_save_id          ), // control signal to save pc
    .exc_restore_id_o             ( exc_restore_id       ), // control signal to restore pc
    .lsu_load_err_i               ( lsu_load_err         ),
    .lsu_store_err_i              ( lsu_store_err        ),

    // Debug Unit Signals
    .dbg_settings_i               ( dbg_settings         ),
    .dbg_req_i                    ( dbg_req              ),
    .dbg_ack_o                    ( dbg_ack              ),
    .dbg_stall_i                  ( dbg_stall            ),
    .dbg_trap_o                   ( dbg_trap             ),

    .dbg_reg_rreq_i               ( dbg_reg_rreq         ),
    .dbg_reg_raddr_i              ( dbg_reg_raddr        ),
    .dbg_reg_rdata_o              ( dbg_reg_rdata        ),

    .dbg_reg_wreq_i               ( dbg_reg_wreq         ),
    .dbg_reg_waddr_i              ( dbg_reg_waddr        ),
    .dbg_reg_wdata_i              ( dbg_reg_wdata        ),

    .dbg_jump_req_i               ( dbg_jump_req         ),

    // Forward Signals
    .regfile_waddr_wb_i           ( regfile_waddr_fw_wb_o),  // Write address ex-wb pipeline
    .regfile_we_wb_i              ( regfile_we_wb        ),  // write enable for the register file
    .regfile_wdata_wb_i           ( regfile_wdata        ),  // write data to commit in the register file

    .regfile_alu_waddr_fw_i       ( regfile_alu_waddr_fw ),
    .regfile_alu_we_fw_i          ( regfile_alu_we_fw    ),
    .regfile_alu_wdata_fw_i       ( regfile_alu_wdata_fw ),

    // from ALU
    .mult_multicycle_i            ( mult_multicycle      ),

    // Performance Counters
    .perf_jump_o                  ( perf_jump            ),
    .perf_jr_stall_o              ( perf_jr_stall        ),
    .perf_ld_stall_o              ( perf_ld_stall        ),
    .start                        ( start                )
  );


  /////////////////////////////////////////////////////
  //   _______  __  ____ _____  _    ____ _____      //
  //  | ____\ \/ / / ___|_   _|/ \  / ___| ____|     //
  //  |  _|  \  /  \___ \ | | / _ \| |  _|  _|       //
  //  | |___ /  \   ___) || |/ ___ \ |_| | |___      //
  //  |_____/_/\_\ |____/ |_/_/   \_\____|_____|     //
  //                                                 //
  /////////////////////////////////////////////////////
  riscv_ex_stage  ex_stage_i
  (
    // Global signals: Clock and active low asynchronous reset
    .clk                        ( clk                          ),
    .rst_n                      ( rst_ni                       ),

    // Alu signals from ID stage
    .alu_operator_i             ( alu_operator_ex              ), // from ID/EX pipe registers
    .alu_operand_a_i            ( alu_operand_a_ex             ), // from ID/EX pipe registers
    .alu_operand_b_i            ( alu_operand_b_ex             ), // from ID/EX pipe registers
    .alu_operand_c_i            ( alu_operand_c_ex             ), // from ID/EX pipe registers
    .bmask_a_i                  ( bmask_a_ex                   ), // from ID/EX pipe registers
    .bmask_b_i                  ( bmask_b_ex                   ), // from ID/EX pipe registers
    .imm_vec_ext_i              ( imm_vec_ext_ex               ), // from ID/EX pipe registers
    .alu_vec_mode_i             ( alu_vec_mode_ex              ), // from ID/EX pipe registers

    // Multipler
    .mult_operator_i            ( mult_operator_ex             ), // from ID/EX pipe registers
    .mult_operand_a_i           ( mult_operand_a_ex            ), // from ID/EX pipe registers
    .mult_operand_b_i           ( mult_operand_b_ex            ), // from ID/EX pipe registers
    .mult_operand_c_i           ( mult_operand_c_ex            ), // from ID/EX pipe registers
    .mult_en_i                  ( mult_en_ex                   ), // from ID/EX pipe registers
    .mult_sel_subword_i         ( mult_sel_subword_ex          ), // from ID/EX pipe registers
    .mult_signed_mode_i         ( mult_signed_mode_ex          ), // from ID/EX pipe registers
    .mult_imm_i                 ( mult_imm_ex                  ), // from ID/EX pipe registers
    .mult_dot_op_a_i            ( mult_dot_op_a_ex             ), // from ID/EX pipe registers
    .mult_dot_op_b_i            ( mult_dot_op_b_ex             ), // from ID/EX pipe registers
    .mult_dot_op_c_i            ( mult_dot_op_c_ex             ), // from ID/EX pipe registers
    .mult_dot_signed_i          ( mult_dot_signed_ex           ), // from ID/EX pipe registers

    .mult_multicycle_o          ( mult_multicycle              ), // to ID/EX pipe registers

    // interface with CSRs
    .csr_access_i               ( csr_access_ex                ),
    .csr_rdata_i                ( csr_rdata                    ),

    // From ID Stage: Regfile control signals
    .branch_in_ex_i             ( branch_in_ex                 ),
    .regfile_alu_waddr_i        ( regfile_alu_waddr_ex         ),
    .regfile_alu_we_i           ( regfile_alu_we_ex            ),

    .regfile_waddr_i            ( regfile_waddr_ex             ),
    .regfile_we_i               ( regfile_we_ex                ),

    // Output of ex stage pipeline
    .regfile_waddr_wb_o         ( regfile_waddr_fw_wb_o        ),
    .regfile_we_wb_o            ( regfile_we_wb                ),

    // To IF: Jump and branch target and decision
    .jump_target_o              ( jump_target_ex               ),
    .branch_decision_o          ( branch_decision              ),

    // To ID stage: Forwarding signals
    .regfile_alu_waddr_fw_o     ( regfile_alu_waddr_fw         ),
    .regfile_alu_we_fw_o        ( regfile_alu_we_fw            ),
    .regfile_alu_wdata_fw_o     ( regfile_alu_wdata_fw         ),

    // stall control
    .lsu_ready_ex_i             ( lsu_ready_ex                 ),

    .ex_ready_o                 ( ex_ready                     ),
    .ex_valid_o                 ( ex_valid                     ),
    .wb_ready_i                 ( lsu_ready_wb                 )
  );


  ////////////////////////////////////////////////////////////////////////////////////////
  //    _     ___    _    ____    ____ _____ ___  ____  _____   _   _ _   _ ___ _____   //
  //   | |   / _ \  / \  |  _ \  / ___|_   _/ _ \|  _ \| ____| | | | | \ | |_ _|_   _|  //
  //   | |  | | | |/ _ \ | | | | \___ \ | || | | | |_) |  _|   | | | |  \| || |  | |    //
  //   | |__| |_| / ___ \| |_| |  ___) || || |_| |  _ <| |___  | |_| | |\  || |  | |    //
  //   |_____\___/_/   \_\____/  |____/ |_| \___/|_| \_\_____|  \___/|_| \_|___| |_|    //
  //                                                                                    //
  ////////////////////////////////////////////////////////////////////////////////////////

  riscv_load_store_unit  load_store_unit_i
  (
    .clk                   ( clk                ),
    .rst_n                 ( rst_ni             ),

    //output to data memory
    .data_req_o            ( data_req_o         ),
    .data_gnt_i            ( data_gnt_i         ),
    .data_rvalid_i         ( data_rvalid_i      ),
    .data_err_i            ( data_err_i         ),

    .data_addr_o           ( data_addr_o        ),
    .data_we_o             ( data_we_o          ),
    .data_be_o             ( data_be_o          ),
    .data_wdata_o          ( data_wdata_o       ),
    .data_rdata_i          ( data_rdata_i       ),

    // signal from ex stage
    .data_we_ex_i          ( data_we_ex         ),
    .data_type_ex_i        ( data_type_ex       ),
    .data_wdata_ex_i       ( alu_operand_c_ex   ),
    .data_reg_offset_ex_i  ( data_reg_offset_ex ),
    .data_sign_ext_ex_i    ( data_sign_ext_ex   ),  // sign extension

    .data_rdata_ex_o       ( regfile_wdata      ),
    .data_req_ex_i         ( data_req_ex        ),
    .operand_a_ex_i        ( alu_operand_a_ex   ),
    .operand_b_ex_i        ( alu_operand_b_ex   ),
    .addr_useincr_ex_i     ( useincr_addr_ex    ),

    .data_misaligned_ex_i  ( data_misaligned_ex ), // from ID/EX pipeline
    .data_misaligned_o     ( data_misaligned    ),

    // exception signals
    .load_err_o            ( lsu_load_err       ),
    .store_err_o           ( lsu_store_err      ),

    // control signals
    .lsu_ready_ex_o        ( lsu_ready_ex       ),
    .lsu_ready_wb_o        ( lsu_ready_wb       ),

    .ex_valid_i            ( ex_valid           ),
    .busy_o                ( lsu_busy           )
  );

  assign wb_valid = lsu_ready_wb;


  //////////////////////////////////////
  //        ____ ____  ____           //
  //       / ___/ ___||  _ \ ___      //
  //      | |   \___ \| |_) / __|     //
  //      | |___ ___) |  _ <\__ \     //
  //       \____|____/|_| \_\___/     //
  //                                  //
  //   Control and Status Registers   //
  //////////////////////////////////////

  riscv_cs_registers
  #(
    .N_EXT_CNT       ( N_EXT_PERF_COUNTERS   )
  )
  cs_registers_i
  (
    .clk                     ( clk                ),
    .rst_n                   ( rst_ni             ),

    // Core and Cluster ID from outside
    .core_id_i               ( core_id_i          ),
    .cluster_id_i            ( cluster_id_i       ),

    // Interface to CSRs (SRAM like)
    .csr_access_i            ( csr_access         ),
    .csr_addr_i              ( csr_addr           ),
    .csr_wdata_i             ( csr_wdata          ),
    .csr_op_i                ( csr_op             ),
    .csr_rdata_o             ( csr_rdata          ),

    // Interrupt related control signals
    .irq_enable_o            ( irq_enable         ),
    .mepc_o                  ( mepc               ),

    .pc_if_i                 ( pc_if              ),
    .pc_id_i                 ( pc_id              ), // from IF stage
    .pc_ex_i                 ( pc_ex              ), // from ID/EX pipeline
    .data_load_event_ex_i    ( data_load_event_ex ), // from ID/EX pipeline
    .exc_save_if_i           ( exc_save_if        ),
    .exc_save_id_i           ( exc_save_id        ),
    .exc_restore_i           ( exc_restore_id     ),

    .exc_cause_i             ( exc_cause          ),
    .save_exc_cause_i        ( save_exc_cause     ),

    // from hwloop registers
    .hwlp_start_i            ( hwlp_start         ),
    .hwlp_end_i              ( hwlp_end           ),
    .hwlp_cnt_i              ( hwlp_cnt           ),

    .hwlp_regid_o            ( csr_hwlp_regid     ),
    .hwlp_we_o               ( csr_hwlp_we        ),
    .hwlp_data_o             ( csr_hwlp_data      ),

    // performance counter related signals
    .id_valid_i              ( id_valid           ),
    .is_compressed_i         ( is_compressed_id   ),
    .is_decoding_i           ( is_decoding        ),

    .imiss_i                 ( perf_imiss         ),
    .pc_set_i                ( pc_set             ),
    .jump_i                  ( perf_jump          ),
    .branch_i                ( branch_in_ex       ),
    .branch_taken_i          ( branch_decision    ),
    .ld_stall_i              ( perf_ld_stall      ),
    .jr_stall_i              ( perf_jr_stall      ),

    .mem_load_i              ( data_req_o & data_gnt_i & (~data_we_o) ),
    .mem_store_i             ( data_req_o & data_gnt_i & data_we_o    ),

    .ext_counters_i          ( ext_perf_counters_i                    ),
//    .hdc_predicted_class     ( hdc_predicted_class                    )
    .y(y)
  );

  // Mux for CSR access through Debug Unit
  assign csr_access   = (dbg_csr_req == 1'b0) ? csr_access_ex : 1'b1;
  assign csr_addr     = (dbg_csr_req == 1'b0) ? csr_addr_int     : dbg_csr_addr;
  assign csr_wdata    = (dbg_csr_req == 1'b0) ? alu_operand_a_ex : dbg_csr_wdata;
  assign csr_op       = (dbg_csr_req == 1'b0) ? csr_op_ex
                                              : (dbg_csr_we == 1'b1 ? CSR_OP_WRITE
                                                                    : CSR_OP_NONE );
  assign csr_addr_int = csr_access_ex ? alu_operand_b_ex[11:0] : '0;


  /////////////////////////////////////////////////////////////
  //  ____  _____ ____  _   _  ____   _   _ _   _ ___ _____  //
  // |  _ \| ____| __ )| | | |/ ___| | | | | \ | |_ _|_   _| //
  // | | | |  _| |  _ \| | | | |  _  | | | |  \| || |  | |   //
  // | |_| | |___| |_) | |_| | |_| | | |_| | |\  || |  | |   //
  // |____/|_____|____/ \___/ \____|  \___/|_| \_|___| |_|   //
  //                                                         //
  /////////////////////////////////////////////////////////////

  riscv_debug_unit debug_unit_i
  (
    .clk               ( clk_i              ), // always-running clock for debug
    .rst_n             ( rst_ni             ),

    // Debug Interface
    .debug_req_i       ( debug_req_i        ),
    .debug_gnt_o       ( debug_gnt_o        ),
    .debug_rvalid_o    ( debug_rvalid_o     ),
    .debug_addr_i      ( debug_addr_i       ),
    .debug_we_i        ( debug_we_i         ),
    .debug_wdata_i     ( debug_wdata_i      ),
    .debug_rdata_o     ( debug_rdata_o      ),
    .debug_halt_i      ( debug_halt_i       ),
    .debug_resume_i    ( debug_resume_i     ),
    .debug_halted_o    ( debug_halted_o     ),

    // To/From Core
    .settings_o        ( dbg_settings       ),
    .trap_i            ( dbg_trap           ),
    .exc_cause_i       ( exc_cause          ),
    .stall_o           ( dbg_stall          ),
    .dbg_req_o         ( dbg_req            ),
    .dbg_ack_i         ( dbg_ack            ),

    // register file read port
    .regfile_rreq_o    ( dbg_reg_rreq       ),
    .regfile_raddr_o   ( dbg_reg_raddr      ),
    .regfile_rdata_i   ( dbg_reg_rdata      ),

    // register file write port
    .regfile_wreq_o    ( dbg_reg_wreq       ),
    .regfile_waddr_o   ( dbg_reg_waddr      ),
    .regfile_wdata_o   ( dbg_reg_wdata      ),

    // CSR read/write port
    .csr_req_o         ( dbg_csr_req        ),
    .csr_addr_o        ( dbg_csr_addr       ),
    .csr_we_o          ( dbg_csr_we         ),
    .csr_wdata_o       ( dbg_csr_wdata      ),
    .csr_rdata_i       ( csr_rdata          ),

    // signals for PPC and NPC
    .pc_if_i           ( pc_if              ), // from IF stage
    .pc_id_i           ( pc_id              ), // from IF stage
    .pc_ex_i           ( pc_ex              ), // PC of last executed branch (in EX stage) or p.elw

    .data_load_event_i ( data_load_event_ex ),
    .instr_valid_id_i  ( instr_valid_id     ),

    .sleeping_i        ( sleeping           ),

    .branch_in_ex_i    ( branch_in_ex       ),
    .branch_taken_i    ( branch_decision    ),

    .jump_addr_o       ( dbg_jump_addr      ), // PC from debug unit
    .jump_req_o        ( dbg_jump_req       )  // set PC to new value
  );


`ifdef TRACE_EXECUTION
  riscv_tracer riscv_tracer_i
  (
    .clk            ( clk_i                                ), // always-running clock for tracing
    .rst_n          ( rst_ni                               ),

    .fetch_enable   ( fetch_enable_i                       ),
    .core_id        ( core_id_i                            ),
    .cluster_id     ( cluster_id_i                         ),

    .pc             ( id_stage_i.pc_id_i                   ),
    .instr          ( id_stage_i.instr                     ),
    .compressed     ( id_stage_i.is_compressed_i           ),
    .id_valid       ( id_stage_i.id_valid_o                ),
    .is_decoding    ( id_stage_i.is_decoding_o             ),
    .pipe_flush     ( id_stage_i.controller_i.pipe_flush_i ),

    .rs1_value      ( id_stage_i.operand_a_fw_id           ),
    .rs2_value      ( id_stage_i.operand_b_fw_id           ),
    .rs3_value      ( id_stage_i.alu_operand_c             ),
    .rs2_value_vec  ( id_stage_i.alu_operand_b             ),

    .ex_valid       ( ex_valid                             ),
    .ex_reg_addr    ( regfile_alu_waddr_fw                 ),
    .ex_reg_we      ( regfile_alu_we_fw                    ),
    .ex_reg_wdata   ( regfile_alu_wdata_fw                 ),

    .ex_data_addr   ( data_addr_o                          ),
    .ex_data_req    ( data_req_o                           ),
    .ex_data_gnt    ( data_gnt_i                           ),
    .ex_data_we     ( data_we_o                            ),
    .ex_data_wdata  ( data_wdata_o                         ),

    .wb_bypass      ( ex_stage_i.branch_in_ex_i            ),

    .wb_valid       ( wb_valid                             ),
    .wb_reg_addr    ( regfile_waddr_fw_wb_o                ),
    .wb_reg_we      ( regfile_we_wb                        ),
    .wb_reg_wdata   ( regfile_wdata                        ),

    .imm_u_type     ( id_stage_i.imm_u_type                ),
    .imm_uj_type    ( id_stage_i.imm_uj_type               ),
    .imm_i_type     ( id_stage_i.imm_i_type                ),
    .imm_iz_type    ( id_stage_i.imm_iz_type[11:0]         ),
    .imm_z_type     ( id_stage_i.imm_z_type                ),
    .imm_s_type     ( id_stage_i.imm_s_type                ),
    .imm_sb_type    ( id_stage_i.imm_sb_type               ),
    .imm_s2_type    ( id_stage_i.imm_s2_type               ),
    .imm_s3_type    ( id_stage_i.imm_s3_type               ),
    .imm_vs_type    ( id_stage_i.imm_vs_type               ),
    .imm_vu_type    ( id_stage_i.imm_vu_type               ),
    .imm_clip_type  ( id_stage_i.instr_rdata_i[11:7]       )
  );
`endif

`ifdef SIMCHECKER
  logic is_interrupt;
  assign is_interrupt = (pc_mux_id == PC_EXCEPTION) && (exc_pc_mux_id == EXC_PC_IRQ);

  riscv_simchecker riscv_simchecker_i
  (
    .clk              ( clk_i                                ), // always-running clock for tracing
    .rst_n            ( rst_ni                               ),

    .fetch_enable     ( fetch_enable_i                       ),
    .boot_addr        ( boot_addr_i                          ),
    .core_id          ( core_id_i                            ),
    .cluster_id       ( cluster_id_i                         ),

    .instr_compressed ( if_stage_i.fetch_rdata[15:0]         ),
    .pc_set           ( pc_set                               ),
    .if_valid         ( if_valid                             ),

    .pc               ( id_stage_i.pc_id_i                   ),
    .instr            ( id_stage_i.instr                     ),
    .is_compressed    ( is_compressed_id                     ),
    .id_valid         ( id_stage_i.id_valid_o                ),
    .is_decoding      ( id_stage_i.is_decoding_o             ),
    .is_illegal       ( id_stage_i.illegal_insn_dec          ),
    .is_interrupt     ( is_interrupt                         ),
    .irq_no           ( exc_vec_pc_mux_id                    ),
    .pipe_flush       ( id_stage_i.controller_i.pipe_flush_i ),

    .ex_valid         ( ex_valid                             ),
    .ex_reg_addr      ( id_stage_i.registers_i.waddr_b_i     ),
    .ex_reg_we        ( id_stage_i.registers_i.we_b_i        ),
    .ex_reg_wdata     ( id_stage_i.registers_i.wdata_b_i     ),

    .ex_data_addr     ( data_addr_o                          ),
    .ex_data_req      ( data_req_o                           ),
    .ex_data_gnt      ( data_gnt_i                           ),
    .ex_data_we       ( data_we_o                            ),
    .ex_data_wdata    ( data_wdata_o                         ),

    .wb_bypass        ( ex_stage_i.branch_in_ex_i            ),
    .lsu_misaligned   ( data_misaligned                      ),

    .wb_valid         ( wb_valid                             ),
    .wb_reg_addr      ( id_stage_i.registers_i.waddr_a_i     ),
    .wb_reg_we        ( id_stage_i.registers_i.we_a_i        ),
    .wb_reg_wdata     ( id_stage_i.registers_i.wdata_a_i     ),

    .wb_data_rvalid   ( data_rvalid_i                        ),
    .wb_data_rdata    ( data_rdata_i                         )
  );
`endif

endmodule

// Copied from pulpino/rtl/components

module cluster_clock_gating
(
    input  logic clk_i,
    input  logic en_i,
    input  logic test_en_i,
    output logic clk_o
  );

`ifdef PULP_FPGA_EMUL
  // no clock gates in FPGA flow
  assign clk_o = clk_i;
`else
  logic clk_en;

  always_latch
  begin
     if (clk_i == 1'b0)
       clk_en <= en_i | test_en_i;
  end

  assign clk_o = clk_i & clk_en;
`endif

endmodule // cluster_clock_gating

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Renzo Andri - andrire@student.ethz.ch                      //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Andreas Traber - atraber@student.ethz.ch                   //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
// Design Name:    Instruction Fetch Stage                                    //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Instruction fetch unit: Selection of the next PC, and      //
//                 buffering (sampling) of the read instruction               //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


import riscv_defines::*;

module riscv_if_stage
#(
  parameter N_HWLP      = 2,
  parameter RDATA_WIDTH = 32
)
(
    input  logic        clk,
    input  logic        rst_n,

    // the boot address is used to calculate the exception offsets
    input  logic [31:0] boot_addr_i,

    // instruction request control
    input  logic        req_i,

    // instruction cache interface
    output logic                   instr_req_o,
    output logic            [31:0] instr_addr_o,
    input  logic                   instr_gnt_i,
    input  logic                   instr_rvalid_i,
    input  logic [RDATA_WIDTH-1:0] instr_rdata_i,

    // Output of IF Pipeline stage
    output logic [N_HWLP-1:0] hwlp_dec_cnt_id_o,     // currently served instruction was the target of a hwlp
    output logic              is_hwlp_id_o,          // currently served instruction was the target of a hwlp
    output logic              instr_valid_id_o,      // instruction in IF/ID pipeline is valid
    output logic       [31:0] instr_rdata_id_o,      // read instruction is sampled and sent to ID stage for decoding
    output logic              is_compressed_id_o,    // compressed decoder thinks this is a compressed instruction
    output logic              illegal_c_insn_id_o,   // compressed decoder thinks this is an invalid instruction
    output logic       [31:0] pc_if_o,
    output logic       [31:0] pc_id_o,

    // Forwarding ports - control signals
    input  logic        clear_instr_valid_i,   // clear instruction valid bit in IF/ID pipe
    input  logic        pc_set_i,              // set the program counter to a new value
    input  logic [31:0] exception_pc_reg_i,    // address used to restore PC when the interrupt/exception is served
    input  logic  [2:0] pc_mux_i,              // sel for pc multiplexer
    input  logic  [1:0] exc_pc_mux_i,          // selects ISR address
    input  logic  [4:0] exc_vec_pc_mux_i,      // selects ISR address for vectorized interrupt lines

    // jump and branch target and decision
    input  logic [31:0] jump_target_id_i,      // jump target address
    input  logic [31:0] jump_target_ex_i,      // jump target address

    // from hwloop controller
    input  logic [N_HWLP-1:0] [31:0] hwlp_start_i,          // hardware loop start addresses
    input  logic [N_HWLP-1:0] [31:0] hwlp_end_i,            // hardware loop end addresses
    input  logic [N_HWLP-1:0] [31:0] hwlp_cnt_i,            // hardware loop counters

    // from debug unit
    input  logic [31:0] dbg_jump_addr_i,
    input  logic        dbg_jump_req_i,

    // pipeline stall
    input  logic        halt_if_i,
    output logic        if_ready_o,
    input  logic        id_ready_i,
    output logic        if_valid_o,

    // misc signals
    output logic        if_busy_o,             // is the IF stage busy fetching instructions?
    output logic        perf_imiss_o           // Instruction Fetch Miss
);

  // offset FSM
  enum logic[0:0] {WAIT, IDLE } offset_fsm_cs, offset_fsm_ns;

  logic              valid;

  // prefetch buffer related signals
  logic              prefetch_busy;
  logic              branch_req;
  logic       [31:0] fetch_addr_n;

  logic              fetch_valid;
  logic              fetch_ready;
  logic       [31:0] fetch_rdata;
  logic       [31:0] fetch_addr;
  logic              is_hwlp_id_q, fetch_is_hwlp;

  logic       [31:0] exc_pc;

  // hardware loop related signals
  logic              hwlp_jump;
  logic       [31:0] hwlp_target;
  logic [N_HWLP-1:0] hwlp_dec_cnt, hwlp_dec_cnt_if;


  // exception PC selection mux
  always_comb
  begin : EXC_PC_MUX
    exc_pc = 'x;

    unique case (exc_pc_mux_i)
      EXC_PC_ILLINSN: exc_pc = { boot_addr_i[31:8], EXC_OFF_ILLINSN };
      EXC_PC_ECALL:   exc_pc = { boot_addr_i[31:8], EXC_OFF_ECALL   };
      EXC_PC_LOAD:    exc_pc = { boot_addr_i[31:8], EXC_OFF_LSUERR  };
      EXC_PC_IRQ:     exc_pc = { boot_addr_i[31:8], 1'b0, exc_vec_pc_mux_i[4:0], 2'b0 };
      // TODO: Add case for EXC_PC_STORE as soon as it differs from load

      default:;
    endcase
  end

  // fetch address selection
  always_comb
  begin
    fetch_addr_n = 'x;

    unique case (pc_mux_i)
      PC_BOOT:      fetch_addr_n = {boot_addr_i[31:8], EXC_OFF_RST};
      PC_JUMP:      fetch_addr_n = jump_target_id_i;
      PC_BRANCH:    fetch_addr_n = jump_target_ex_i;
      PC_EXCEPTION: fetch_addr_n = exc_pc;             // set PC to exception handler
      PC_ERET:      fetch_addr_n = exception_pc_reg_i; // PC is restored when returning from IRQ/exception
      PC_DBG_NPC:   fetch_addr_n = dbg_jump_addr_i;    // PC is taken from debug unit

      default:;
    endcase
  end

  generate
    if (RDATA_WIDTH == 32) begin : prefetch_32
      // prefetch buffer, caches a fixed number of instructions
      riscv_prefetch_buffer prefetch_buffer_i
      (
        .clk               ( clk                         ),
        .rst_n             ( rst_n                       ),

        .req_i             ( req_i                       ),

        .branch_i          ( branch_req                  ),
        .addr_i            ( {fetch_addr_n[31:1], 1'b0}  ),

        .hwloop_i          ( hwlp_jump                   ),
        .hwloop_target_i   ( hwlp_target                 ),

        .ready_i           ( fetch_ready                 ),
        .valid_o           ( fetch_valid                 ),
        .rdata_o           ( fetch_rdata                 ),
        .addr_o            ( fetch_addr                  ),
        .is_hwlp_o         ( fetch_is_hwlp               ),

        // goes to instruction memory / instruction cache
        .instr_req_o       ( instr_req_o                 ),
        .instr_addr_o      ( instr_addr_o                ),
        .instr_gnt_i       ( instr_gnt_i                 ),
        .instr_rvalid_i    ( instr_rvalid_i              ),
        .instr_rdata_i     ( instr_rdata_i               ),

        // Prefetch Buffer Status
        .busy_o            ( prefetch_busy               )
      );
    end else if (RDATA_WIDTH == 128) begin : prefetch_128
      // prefetch buffer, caches a fixed number of instructions
      riscv_prefetch_L0_buffer prefetch_buffer_i
      (
        .clk               ( clk                         ),
        .rst_n             ( rst_n                       ),

        .req_i             ( 1'b1                        ),

        .branch_i          ( branch_req                  ),
        .addr_i            ( {fetch_addr_n[31:1], 1'b0}  ),

        .hwloop_i          ( hwlp_jump                   ),
        .hwloop_target_i   ( hwlp_target                 ),

        .ready_i           ( fetch_ready                 ),
        .valid_o           ( fetch_valid                 ),
        .rdata_o           ( fetch_rdata                 ),
        .addr_o            ( fetch_addr                  ),
        .is_hwlp_o         ( fetch_is_hwlp               ),

        // goes to instruction memory / instruction cache
        .instr_req_o       ( instr_req_o                 ),
        .instr_addr_o      ( instr_addr_o                ),
        .instr_gnt_i       ( instr_gnt_i                 ),
        .instr_rvalid_i    ( instr_rvalid_i              ),
        .instr_rdata_i     ( instr_rdata_i               ),

        // Prefetch Buffer Status
        .busy_o            ( prefetch_busy               )
      );
    end
  endgenerate


  // offset FSM state
  always_ff @(posedge clk, negedge rst_n)
  begin
    if (rst_n == 1'b0) begin
      offset_fsm_cs     <= IDLE;
    end else begin
      offset_fsm_cs     <= offset_fsm_ns;
    end
  end

  // offset FSM state transition logic
  always_comb
  begin
    offset_fsm_ns = offset_fsm_cs;

    fetch_ready   = 1'b0;
    branch_req    = 1'b0;
    valid         = 1'b0;

    unique case (offset_fsm_cs)
      // no valid instruction data for ID stage
      // assume aligned
      IDLE: begin
        if (req_i) begin
          branch_req    = 1'b1;
          offset_fsm_ns = WAIT;
        end
      end

      // serving aligned 32 bit or 16 bit instruction, we don't know yet
      WAIT: begin
        if (fetch_valid) begin
          valid   = 1'b1; // an instruction is ready for ID stage

          if (req_i && if_valid_o) begin
            fetch_ready   = 1'b1;
            offset_fsm_ns = WAIT;
          end
        end
      end

      default: begin
        offset_fsm_ns = IDLE;
      end
    endcase


    // take care of jumps and branches
    if (pc_set_i) begin
      valid = 1'b0;

      // switch to new PC from ID stage
      branch_req = 1'b1;
      offset_fsm_ns = WAIT;
    end
  end

  // Hardware Loops
  riscv_hwloop_controller
  #(
    .N_REGS ( N_HWLP )
  )
  hwloop_controller_i
  (
    .current_pc_i          ( fetch_addr        ),

    .hwlp_jump_o           ( hwlp_jump         ),
    .hwlp_targ_addr_o      ( hwlp_target       ),

    // from hwloop_regs
    .hwlp_start_addr_i     ( hwlp_start_i      ),
    .hwlp_end_addr_i       ( hwlp_end_i        ),
    .hwlp_counter_i        ( hwlp_cnt_i        ),

    // to hwloop_regs
    .hwlp_dec_cnt_o        ( hwlp_dec_cnt      ),
    .hwlp_dec_cnt_id_i     ( hwlp_dec_cnt_id_o & {N_HWLP{is_hwlp_id_o}} )
  );


  assign pc_if_o         = fetch_addr;

  assign if_busy_o       = prefetch_busy;

  assign perf_imiss_o    = (~fetch_valid) | branch_req;


  // compressed instruction decoding, or more precisely compressed instruction
  // expander
  //
  // since it does not matter where we decompress instructions, we do it here
  // to ease timing closure
  logic [31:0] instr_decompressed;
  logic        illegal_c_insn;
  logic        instr_compressed_int;

  riscv_compressed_decoder compressed_decoder_i
  (
    .instr_i         ( fetch_rdata          ),
    .instr_o         ( instr_decompressed   ),
    .is_compressed_o ( instr_compressed_int ),
    .illegal_instr_o ( illegal_c_insn       )
  );

  // prefetch -> IF registers
  always_ff @(posedge clk, negedge rst_n)
  begin
    if (rst_n == 1'b0)
    begin
      hwlp_dec_cnt_if <= '0;
    end
    else
    begin
      if (hwlp_jump)
        hwlp_dec_cnt_if <= hwlp_dec_cnt;
    end
  end

  // IF-ID pipeline registers, frozen when the ID stage is stalled
  always_ff @(posedge clk, negedge rst_n)
  begin : IF_ID_PIPE_REGISTERS
    if (rst_n == 1'b0)
    begin
      instr_valid_id_o      <= 1'b0;
      instr_rdata_id_o      <= '0;
      illegal_c_insn_id_o   <= 1'b0;
      is_compressed_id_o    <= 1'b0;
      pc_id_o               <= '0;
      is_hwlp_id_q          <= 1'b0;
      hwlp_dec_cnt_id_o     <= '0;
    end
    else
    begin

      if (if_valid_o)
      begin
        instr_valid_id_o    <= 1'b1;
        instr_rdata_id_o    <= instr_decompressed;
        illegal_c_insn_id_o <= illegal_c_insn;
        is_compressed_id_o  <= instr_compressed_int;
        pc_id_o             <= pc_if_o;
        is_hwlp_id_q        <= fetch_is_hwlp;

        if (fetch_is_hwlp)
          hwlp_dec_cnt_id_o   <= hwlp_dec_cnt_if;

      end else if (clear_instr_valid_i) begin
        instr_valid_id_o    <= 1'b0;
      end

    end
  end

  assign is_hwlp_id_o = is_hwlp_id_q & instr_valid_id_o;

  assign if_ready_o = valid & id_ready_i;
  assign if_valid_o = (~halt_if_i) & if_ready_o;

`ifndef verilator
  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------

  // there should never be a grant when there is no request
  assert property (
    @(posedge clk) (instr_gnt_i) |-> (instr_req_o) )
    else $warning("There was a grant without a request");

  // make sure LSB of fetch_addr_n is always 0
  assert property (
    @(posedge clk) (req_i) |-> (~fetch_addr_n[0]) )
    else $warning("There was a request while the fetch_addr_n LSB is set");
`endif

endmodule

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Michael Gautschi - gautschi@iis.ee.ethz.ch                 //
//                                                                            //
// Design Name:    hwloop controller                                          //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Hardware loop controller unit. This unit is responsible to //
//                 handle hardware loops. Tasks are:                          //
//                 a) compare PC to all stored end addresses                  //
//                 b) jump to the right start address if counter =/ 0         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module riscv_hwloop_controller
#(
  parameter N_REGS = 2
)
(
  // from id stage
  input  logic [31:0]              current_pc_i,

  // from hwloop_regs
  input  logic [N_REGS-1:0] [31:0] hwlp_start_addr_i,
  input  logic [N_REGS-1:0] [31:0] hwlp_end_addr_i,
  input  logic [N_REGS-1:0] [31:0] hwlp_counter_i,

  // to hwloop_regs
  output logic [N_REGS-1:0]        hwlp_dec_cnt_o,

  // from pipeline stages
  input  logic [N_REGS-1:0]        hwlp_dec_cnt_id_i,

  // to id stage
  output logic                     hwlp_jump_o,
  output logic [31:0]              hwlp_targ_addr_o
);


  logic [N_REGS-1:0] pc_is_end_addr;

  // end address detection
  integer j;


  // generate comparators. check for end address and the loop counter
  genvar i;
  generate
    for (i = 0; i < N_REGS; i++) begin
      always @(*)
      begin
        pc_is_end_addr[i] = 1'b0;

        if (current_pc_i == hwlp_end_addr_i[i]) begin
          if (hwlp_counter_i[i][31:2] != 30'h0) begin
            pc_is_end_addr[i] = 1'b1;
          end else begin
            // hwlp_counter_i[i][31:2] == 32'h0
            case (hwlp_counter_i[i][1:0])
              2'b11:        pc_is_end_addr[i] = 1'b1;
              2'b10:        pc_is_end_addr[i] = ~hwlp_dec_cnt_id_i[i]; // only when there is nothing in flight
              2'b01, 2'b00: pc_is_end_addr[i] = 1'b0;
            endcase
          end
        end
      end
    end
  endgenerate

  // select corresponding start address and decrement counter
  always_comb
  begin
    hwlp_targ_addr_o = 'x;
    hwlp_dec_cnt_o   = '0;

    for (j = 0; j < N_REGS; j++) begin
      if (pc_is_end_addr[j]) begin
        hwlp_targ_addr_o  = hwlp_start_addr_i[j];
        hwlp_dec_cnt_o[j] = 1'b1;
        break;
      end
    end
  end

  // output signal for ID stage
  assign hwlp_jump_o = (|pc_is_end_addr);

endmodule

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
// Design Name:    Compressed instruction decoder                             //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Decodes RISC-V compressed instructions into their RV32     //
//                 equivalent. This module is fully combinatorial.            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


import riscv_defines::*;

module riscv_compressed_decoder
(
  input  logic [31:0] instr_i,
  output logic [31:0] instr_o,
  output logic        is_compressed_o,
  output logic        illegal_instr_o
);

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //   ____                                                 _   ____                     _            //
  //  / ___|___  _ __ ___  _ __  _ __ ___  ___ ___  ___  __| | |  _ \  ___  ___ ___   __| | ___ _ __  //
  // | |   / _ \| '_ ` _ \| '_ \| '__/ _ \/ __/ __|/ _ \/ _` | | | | |/ _ \/ __/ _ \ / _` |/ _ \ '__| //
  // | |__| (_) | | | | | | |_) | | |  __/\__ \__ \  __/ (_| | | |_| |  __/ (_| (_) | (_| |  __/ |    //
  //  \____\___/|_| |_| |_| .__/|_|  \___||___/___/\___|\__,_| |____/ \___|\___\___/ \__,_|\___|_|    //
  //                      |_|                                                                         //
  //////////////////////////////////////////////////////////////////////////////////////////////////////

  always_comb
  begin
    illegal_instr_o = 1'b0;
    instr_o         = 'x;

    unique case (instr_i[1:0])
      // C0
      2'b00: begin
        unique case (instr_i[15:13])
          3'b000: begin
            // c.addi4spn -> addi rd', x2, imm
            instr_o = {2'b0, instr_i[10:7], instr_i[12:11], instr_i[5], instr_i[6], 2'b00, 5'h02, 3'b000, 2'b01, instr_i[4:2], OPCODE_OPIMM};
            if (instr_i[12:5] == 8'b0)  illegal_instr_o = 1'b1;
          end

          3'b010: begin
            // c.lw -> lw rd', imm(rs1')
            instr_o = {5'b0, instr_i[5], instr_i[12:10], instr_i[6], 2'b00, 2'b01, instr_i[9:7], 3'b010, 2'b01, instr_i[4:2], OPCODE_LOAD};
          end

          3'b110: begin
            // c.sw -> sw rs2', imm(rs1')
            instr_o = {5'b0, instr_i[5], instr_i[12], 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b010, instr_i[11:10], instr_i[6], 2'b00, OPCODE_STORE};
          end

          default: begin
            illegal_instr_o = 1'b1;
          end
        endcase
      end

      // C1
      2'b01: begin
        unique case (instr_i[15:13])
          3'b000: begin
            // c.addi -> addi rd, rd, nzimm
            // c.nop
            instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], instr_i[11:7], 3'b0, instr_i[11:7], OPCODE_OPIMM};
          end

          3'b001, 3'b101: begin
            // 001: c.jal -> jal x1, imm
            // 101: c.j   -> jal x0, imm
            instr_o = {instr_i[12], instr_i[8], instr_i[10:9], instr_i[6], instr_i[7], instr_i[2], instr_i[11], instr_i[5:3], {9 {instr_i[12]}}, 4'b0, ~instr_i[15], OPCODE_JAL};
          end

          3'b010: begin
            // c.li -> addi rd, x0, nzimm
            instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], 5'b0, 3'b0, instr_i[11:7], OPCODE_OPIMM};
            if (instr_i[11:7] == 5'b0)  illegal_instr_o = 1'b1;
          end

          3'b011: begin
            // c.lui -> lui rd, imm
            instr_o = {{15 {instr_i[12]}}, instr_i[6:2], instr_i[11:7], OPCODE_LUI};

            if (instr_i[11:7] == 5'h02) begin
              // c.addi16sp -> addi x2, x2, nzimm
              instr_o = {{3 {instr_i[12]}}, instr_i[4:3], instr_i[5], instr_i[2], instr_i[6], 4'b0, 5'h02, 3'b000, 5'h02, OPCODE_OPIMM};
            end else if (instr_i[11:7] == 5'b0) begin
              illegal_instr_o = 1'b1;
            end

            if ({instr_i[12], instr_i[6:2]} == 6'b0) illegal_instr_o = 1'b1;
          end

          3'b100: begin
            unique case (instr_i[11:10])
              2'b00,
              2'b01: begin
                // 00: c.srli -> srli rd, rd, shamt
                // 01: c.srai -> srai rd, rd, shamt
                instr_o = {1'b0, instr_i[10], 5'b0, instr_i[6:2], 2'b01, instr_i[9:7], 3'b101, 2'b01, instr_i[9:7], OPCODE_OPIMM};
                if (instr_i[12] == 1'b1)  illegal_instr_o = 1'b1;
                if (instr_i[6:2] == 5'b0) illegal_instr_o = 1'b1;
              end

              2'b10: begin
                // c.andi -> andi rd, rd, imm
                instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], 2'b01, instr_i[9:7], 3'b111, 2'b01, instr_i[9:7], OPCODE_OPIMM};
              end

              2'b11: begin
                unique case ({instr_i[12], instr_i[6:5]})
                  3'b000: begin
                    // c.sub -> sub rd', rd', rs2'
                    instr_o = {2'b01, 5'b0, 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b000, 2'b01, instr_i[9:7], OPCODE_OP};
                  end

                  3'b001: begin
                    // c.xor -> xor rd', rd', rs2'
                    instr_o = {7'b0, 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b100, 2'b01, instr_i[9:7], OPCODE_OP};
                  end

                  3'b010: begin
                    // c.or  -> or  rd', rd', rs2'
                    instr_o = {7'b0, 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b110, 2'b01, instr_i[9:7], OPCODE_OP};
                  end

                  3'b011: begin
                    // c.and -> and rd', rd', rs2'
                    instr_o = {7'b0, 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b111, 2'b01, instr_i[9:7], OPCODE_OP};
                  end

                  3'b100,
                  3'b101,
                  3'b110,
                  3'b111: begin
                    // 100: c.subw
                    // 101: c.addw
                    illegal_instr_o = 1'b1;
                  end
                endcase
              end
            endcase
          end

          3'b110, 3'b111: begin
            // 0: c.beqz -> beq rs1', x0, imm
            // 1: c.bnez -> bne rs1', x0, imm
            instr_o = {{4 {instr_i[12]}}, instr_i[6:5], instr_i[2], 5'b0, 2'b01, instr_i[9:7], 2'b00, instr_i[13], instr_i[11:10], instr_i[4:3], instr_i[12], OPCODE_BRANCH};
          end

          default: begin
            illegal_instr_o = 1'b1;
          end
        endcase
      end

      // C2
      2'b10: begin
        unique case (instr_i[15:13])
          3'b000: begin
            // c.slli -> slli rd, rd, shamt
            instr_o = {7'b0, instr_i[6:2], instr_i[11:7], 3'b001, instr_i[11:7], OPCODE_OPIMM};
            if (instr_i[11:7] == 5'b0)  illegal_instr_o = 1'b1;
            if (instr_i[12] == 1'b1 || instr_i[6:2] == 5'b0)  illegal_instr_o = 1'b1;
          end

          3'b010: begin
            // c.lwsp -> lw rd, imm(x2)
            instr_o = {4'b0, instr_i[3:2], instr_i[12], instr_i[6:4], 2'b00, 5'h02, 3'b010, instr_i[11:7], OPCODE_LOAD};
            if (instr_i[11:7] == 5'b0)  illegal_instr_o = 1'b1;
          end

          3'b100: begin
            if (instr_i[12] == 1'b0) begin
              // c.mv -> add rd/rs1, x0, rs2
              instr_o = {7'b0, instr_i[6:2], 5'b0, 3'b0, instr_i[11:7], OPCODE_OP};

              if (instr_i[6:2] == 5'b0) begin
                // c.jr -> jalr x0, rd/rs1, 0
                instr_o = {12'b0, instr_i[11:7], 3'b0, 5'b0, OPCODE_JALR};
              end
            end else begin
              // c.add -> add rd, rd, rs2
              instr_o = {7'b0, instr_i[6:2], instr_i[11:7], 3'b0, instr_i[11:7], OPCODE_OP};

              if (instr_i[11:7] == 5'b0) begin
                // c.ebreak -> ebreak
                instr_o = {32'h00_10_00_73};
                if (instr_i[6:2] != 5'b0)
                  illegal_instr_o = 1'b1;
              end else if (instr_i[6:2] == 5'b0) begin
                // c.jalr -> jalr x1, rs1, 0
                instr_o = {12'b0, instr_i[11:7], 3'b000, 5'b00001, OPCODE_JALR};
              end
            end
          end

          3'b110: begin
            // c.swsp -> sw rs2, imm(x2)
            instr_o = {4'b0, instr_i[8:7], instr_i[12], instr_i[6:2], 5'h02, 3'b010, instr_i[11:9], 2'b00, OPCODE_STORE};
          end

          default: begin
            illegal_instr_o = 1'b1;
          end
        endcase
      end

      default: begin
        // 32 bit (or more) instruction
        instr_o = instr_i;
      end
    endcase
  end

  assign is_compressed_o = (instr_i[1:0] != 2'b11);

endmodule

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Igor Loi - igor.loi@unibo.it                               //
//                                                                            //
// Additional contributions by:                                               //
//                 Andreas Traber - atraber@iis.ee.ethz.ch                    //
//                                                                            //
// Design Name:    Prefetcher Buffer for 128 bit memory interface             //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Prefetch Buffer that caches instructions. This cuts overly //
//                 long critical paths to the instruction cache               //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module riscv_prefetch_L0_buffer
#(
  parameter                                   RDATA_IN_WIDTH = 128
)
(
  input  logic                                clk,
  input  logic                                rst_n,

  input  logic                                req_i,

  input  logic                                branch_i,
  input  logic [31:0]                         addr_i,

  input  logic                                hwloop_i,
  input  logic [31:0]                         hwloop_target_i,

  input  logic                                ready_i,
  output logic                                valid_o,
  output logic [31:0]                         rdata_o,
  output logic [31:0]                         addr_o,
  output logic                                is_hwlp_o, // is set when the currently served data is from a hwloop

  // goes to instruction memory / instruction cache
  output logic                                instr_req_o,
  output logic [31:0]                         instr_addr_o,
  input  logic                                instr_gnt_i,
  input  logic                                instr_rvalid_i,
  input  logic [RDATA_IN_WIDTH/32-1:0][31:0]  instr_rdata_i,

  // Prefetch Buffer Status
  output logic                                busy_o
);

  logic                               busy_L0;

  enum logic [3:0] { IDLE, BRANCHED,
                     HWLP_WAIT_GNT, HWLP_GRANTED, HWLP_GRANTED_WAIT, HWLP_FETCH_DONE,
                     NOT_VALID, NOT_VALID_GRANTED, NOT_VALID_CROSS, NOT_VALID_CROSS_GRANTED,
                     VALID, VALID_CROSS, VALID_GRANTED, VALID_FETCH_DONE } CS, NS;

  logic                               do_fetch;
  logic                               do_hwlp, do_hwlp_int;
  logic                               use_last;
  logic                               save_rdata_last;
  logic                               use_hwlp;
  logic                               save_rdata_hwlp;
  logic                               valid;

  logic                               hwlp_is_crossword;
  logic                               is_crossword;
  logic                               next_is_crossword;
  logic                               next_valid;
  logic                               next_upper_compressed;
  logic                               fetch_possible;
  logic                               upper_is_compressed;

  logic                       [31:0]  addr_q, addr_n, addr_int, addr_aligned_next, addr_real_next;
  logic                               is_hwlp_q, is_hwlp_n;

  logic                       [31:0]  rdata_last_q;

  logic                               valid_L0;
  logic [RDATA_IN_WIDTH/32-1:0][31:0] rdata_L0;
  logic                        [31:0] addr_L0;

  logic                               fetch_valid;
  logic                               fetch_gnt;

  // prepared data for output
  logic                        [31:0] rdata, rdata_unaligned;

  logic                               aligned_is_compressed, unaligned_is_compressed;
  logic                               hwlp_aligned_is_compressed, hwlp_unaligned_is_compressed;


  prefetch_L0_buffer_L0
  #(
    .RDATA_IN_WIDTH ( RDATA_IN_WIDTH )
  )
  L0_buffer_i
  (
    .clk                  ( clk                ),
    .rst_n                ( rst_n              ),

    .prefetch_i           ( do_fetch           ),
    .prefetch_addr_i      ( addr_real_next   ), //addr_aligned_next

    .branch_i             ( branch_i           ),
    .branch_addr_i        ( addr_i             ),

    .hwlp_i               ( do_hwlp | do_hwlp_int ),
    .hwlp_addr_i          ( hwloop_target_i    ),

    .fetch_gnt_o          ( fetch_gnt          ),
    .fetch_valid_o        ( fetch_valid        ),

    .valid_o              ( valid_L0           ),
    .rdata_o              ( rdata_L0           ),
    .addr_o               ( addr_L0            ),

    .instr_req_o          ( instr_req_o        ),
    .instr_addr_o         ( instr_addr_o       ),
    .instr_gnt_i          ( instr_gnt_i        ),
    .instr_rvalid_i       ( instr_rvalid_i     ),
    .instr_rdata_i        ( instr_rdata_i      ),

    .busy_o               ( busy_L0            )
  );


  assign rdata = (use_last || use_hwlp) ? rdata_last_q : rdata_L0[addr_o[3:2]];

  // the lower part of rdata_unaligned is always the higher part of rdata
  assign rdata_unaligned[15:0] = rdata[31:16];

  always_comb
  begin
    rdata_unaligned[31:16] = 'x;

    case(addr_o[3:2])
       2'b00: begin rdata_unaligned[31:16] = rdata_L0[1][15:0]; end
       2'b01: begin rdata_unaligned[31:16] = rdata_L0[2][15:0]; end
       2'b10: begin rdata_unaligned[31:16] = rdata_L0[3][15:0]; end
       2'b11: begin rdata_unaligned[31:16] = rdata_L0[0][15:0]; end
    endcase // addr_o
  end


  assign unaligned_is_compressed = rdata[17:16] != 2'b11;
  assign aligned_is_compressed   = rdata[1:0] != 2'b11;
  assign upper_is_compressed     = rdata_L0[3][17:16] != 2'b11;
  assign is_crossword            = (addr_o[3:1] == 3'b111) && (~upper_is_compressed);
  assign next_is_crossword       = ((addr_o[3:1] == 3'b110) && (aligned_is_compressed) && (~upper_is_compressed)) || ((addr_o[3:1] == 3'b101) && (~unaligned_is_compressed) && (~upper_is_compressed));
  assign next_upper_compressed   = ((addr_o[3:1] == 3'b110) && (aligned_is_compressed) && upper_is_compressed) || ((addr_o[3:1] == 3'b101) && (~unaligned_is_compressed) && upper_is_compressed);
  assign next_valid              = ((addr_o[3:2] != 2'b11) || next_upper_compressed) && (~next_is_crossword) && valid;

  //addr_o[3:2] == 2'b11;// ((addr_o[3:1] == 3'b101) & (~upper_is_compressed)) | addr_o[3:2] == 2'b11; //  
  assign fetch_possible          =  (addr_o[3:2] == 2'b11 );

  assign addr_aligned_next = { addr_o[31:2], 2'b00 } + 32'h4;
  assign addr_real_next    = (next_is_crossword) ? { addr_o[31:4], 4'b0000 } + 32'h16 : { addr_o[31:2], 2'b00 } + 32'h4;

  assign hwlp_unaligned_is_compressed = rdata_L0[2][17:16] != 2'b11;
  assign hwlp_aligned_is_compressed   = rdata_L0[3][1:0] != 2'b11;
  assign hwlp_is_crossword            = (hwloop_target_i[3:1] == 3'b111) && (~upper_is_compressed);

  always_comb
  begin
    addr_int    = addr_o;

    // advance address when pipeline is unstalled
    if (ready_i) begin

      if (addr_o[1]) begin
        // unaligned case
        // always move to next entry in the FIFO

        if (unaligned_is_compressed) begin
          addr_int = { addr_aligned_next[31:2], 2'b00};
        end else begin
          addr_int = { addr_aligned_next[31:2], 2'b10};
        end

      end else begin
        // aligned case

        if (aligned_is_compressed) begin
          // just increase address, do not move to next entry in the FIFO
          addr_int = { addr_o[31:2], 2'b10 };
        end else begin
          // move to next entry in the FIFO
          addr_int = { addr_aligned_next[31:2], 2'b00 };
        end
      end

    end
  end

  always_comb
  begin
    NS              = CS;
    do_fetch        = 1'b0;
    do_hwlp         = 1'b0;
    do_hwlp_int     = 1'b0;
    use_last        = 1'b0;
    use_hwlp        = 1'b0;
    save_rdata_last = 1'b0;
    save_rdata_hwlp = 1'b0;
    valid           = 1'b0;
    addr_n          = addr_int;
    is_hwlp_n       = is_hwlp_q;

    if (ready_i)
      is_hwlp_n = 1'b0;

    case (CS)
      IDLE: begin
        // wait here for something to happen
      end

      BRANCHED: begin
        valid    = 1'b0;
        do_fetch = fetch_possible;

        if (fetch_valid && (~is_crossword))
          valid = 1'b1;

        if (ready_i) begin
          if (hwloop_i) begin
            addr_n = addr_o; // keep the old address for now
            NS = HWLP_WAIT_GNT;
          end else begin
            if (next_valid) begin
              if (fetch_gnt) begin
                save_rdata_last = 1'b1;
                NS = VALID_GRANTED;
              end else
                NS = VALID;
            end else if (next_is_crossword) begin
              if (fetch_gnt) begin
                save_rdata_last = 1'b1;
                NS = NOT_VALID_CROSS_GRANTED;
              end else begin
                NS = NOT_VALID_CROSS;
              end
            end else begin
              if (fetch_gnt)
                NS = NOT_VALID_GRANTED;
              else
                NS = NOT_VALID;
            end
          end
        end else begin
          if (fetch_valid) begin
            if (is_crossword) begin
              save_rdata_last = 1'b1;
              if (fetch_gnt)
                NS = NOT_VALID_CROSS_GRANTED;
              else
                NS = NOT_VALID_CROSS;
            end else begin
              if (fetch_gnt) begin
                save_rdata_last = 1'b1;
                NS = VALID_GRANTED;
              end else
                NS = VALID;
            end
          end
        end
      end

      NOT_VALID: begin
        do_fetch = 1'b1;

        if (fetch_gnt)
          NS = NOT_VALID_GRANTED;
      end

      NOT_VALID_GRANTED: begin
        valid   = fetch_valid;
        do_hwlp = hwloop_i;

        if (fetch_valid)
          NS = VALID;
      end

      NOT_VALID_CROSS: 
      begin
        do_fetch = 1'b1;

        if (fetch_gnt)
        begin
          save_rdata_last = 1'b1;
          NS = NOT_VALID_CROSS_GRANTED;
        end
      end

      NOT_VALID_CROSS_GRANTED:
      begin
        valid    = fetch_valid;
        use_last = 1'b1;
        do_hwlp  = hwloop_i;

        if (fetch_valid) 
        begin
          if (ready_i)
            NS = VALID;
          else
            NS = VALID_CROSS;
        end
      end

      VALID: begin
         valid    = 1'b1;
         do_fetch = fetch_possible;  // fetch_possible  =  addr_o[3:2] == 2'b11;//
         do_hwlp  = hwloop_i;

         if (ready_i)
         begin
            if (next_is_crossword)
            begin
               do_fetch = 1'b1;
               
               if (fetch_gnt)
               begin
                  save_rdata_last = 1'b1;
                  NS = NOT_VALID_CROSS_GRANTED;
               end
               else // not fetching
               begin
                  NS = NOT_VALID_CROSS;
               end
            end 
            else // Next is not crossword
               if (~next_valid) 
               begin
                  if (fetch_gnt)
                     NS = NOT_VALID_GRANTED;
                  else
                     NS = NOT_VALID;
               end
               else // Next is valid
               begin
                  if (fetch_gnt)
                  begin
                     if (next_upper_compressed)
                     begin
                        save_rdata_last = 1'b1;
                        NS = VALID_GRANTED;
                     end
                  end
               end
         end 
         else // NOT ready
         begin
            if (fetch_gnt) 
               begin
                  save_rdata_last = 1'b1;
                  NS = VALID_GRANTED;
               end
         end
      end

      VALID_CROSS: begin
        valid    = 1'b1;
        use_last = 1'b1;
        do_hwlp  = hwloop_i;

        if (ready_i)
          NS = VALID;
      end

      VALID_GRANTED: begin
        valid    = 1'b1;
        use_last = 1'b1;
        do_hwlp  = hwloop_i;

        if (ready_i) begin
          if (fetch_valid) begin
            if (next_is_crossword)
              NS = VALID_CROSS;
            else if(next_upper_compressed)
              NS = VALID_FETCH_DONE;
            else
              NS = VALID;
          end else begin
            if (next_is_crossword)
              NS = NOT_VALID_CROSS_GRANTED;
            else if (next_upper_compressed)
              NS = VALID_GRANTED;
            else
              NS = NOT_VALID_GRANTED;
          end
        end else begin
          if (fetch_valid)
            NS = VALID_FETCH_DONE;
        end
      end

      VALID_FETCH_DONE: begin
        valid    = 1'b1;
        use_last = 1'b1;
        do_hwlp  = hwloop_i;

        if (ready_i) begin
          if (next_is_crossword)
            NS = VALID_CROSS;
          else if (next_upper_compressed)
            NS = VALID_FETCH_DONE;
          else
            NS = VALID;
        end
      end

      HWLP_WAIT_GNT: begin
        do_hwlp_int = 1'b1;

        if (fetch_gnt) begin
          is_hwlp_n = 1'b1;
          addr_n = hwloop_target_i;
          NS = BRANCHED;
        end
      end

      HWLP_GRANTED: begin
        valid    = 1'b1;
        use_hwlp = 1'b1;

        if (ready_i) begin
          addr_n = hwloop_target_i;

          if (fetch_valid) begin
            is_hwlp_n = 1'b1;

            if (hwlp_is_crossword) begin
              NS = NOT_VALID_CROSS;
            end else begin
              NS = VALID;
            end
          end else begin
            NS = HWLP_GRANTED_WAIT;
          end
        end else begin
          if (fetch_valid)
            NS = HWLP_FETCH_DONE;
        end
      end

      HWLP_GRANTED_WAIT: begin
        use_hwlp = 1'b1;

        if (fetch_valid) begin
          is_hwlp_n = 1'b1;

          if (hwlp_is_crossword) begin
            NS = NOT_VALID_CROSS;
          end else begin
            NS = VALID;
          end
        end
      end

      HWLP_FETCH_DONE: begin
        valid    = 1'b1;
        use_hwlp = 1'b1;

        if (ready_i) begin
          is_hwlp_n = 1'b1;
          addr_n = hwloop_target_i;

          if (hwlp_is_crossword) begin
            NS = NOT_VALID_CROSS;
          end else begin
            NS = VALID;
          end
        end
      end
    endcase

    // branches always have priority
    if (branch_i) begin
      is_hwlp_n = 1'b0;
      addr_n    = addr_i;
      NS        = BRANCHED;

    end else if (hwloop_i) begin
      if (do_hwlp) begin
        if (ready_i) begin
          if (fetch_gnt) begin
            is_hwlp_n = 1'b1;
            addr_n = hwloop_target_i;
            NS = BRANCHED;
          end else begin
            addr_n = addr_o; // keep the old address for now
            NS = HWLP_WAIT_GNT;
          end
        end else begin
          if (fetch_gnt) begin
            save_rdata_hwlp = 1'b1;
            NS = HWLP_GRANTED;
          end
        end
      end
    end
  end


  //////////////////////////////////////////////////////////////////////////////
  // registers
  //////////////////////////////////////////////////////////////////////////////

  always_ff @(posedge clk, negedge rst_n)
  begin
    if (~rst_n)
    begin
      addr_q         <= '0;
      is_hwlp_q      <= 1'b0;
      CS             <= IDLE;
      rdata_last_q   <= '0;
    end
    else
    begin
      addr_q    <= addr_n;
      is_hwlp_q <= is_hwlp_n;

      CS <= NS;

      if (save_rdata_hwlp)
        rdata_last_q <= rdata_o;
      else if(save_rdata_last)
           begin
              //rdata_last_q <= rdata_L0[3];
              if(ready_i)
              begin
                   rdata_last_q <= rdata_L0[3];//rdata;
              end
              else
              begin
                   rdata_last_q <= rdata;//rdata;
              end
           end
    end
  end

  //////////////////////////////////////////////////////////////////////////////
  // output ports
  //////////////////////////////////////////////////////////////////////////////

  assign rdata_o = ((~addr_o[1]) || use_hwlp) ? rdata : rdata_unaligned;
  assign valid_o = valid & (~branch_i);

  assign addr_o = addr_q;

  assign is_hwlp_o = is_hwlp_q & (~branch_i);

  assign busy_o = busy_L0;


`ifndef verilator
  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------

  // there should never be a ready_i without valid_o
  assert property (
    @(posedge clk) (ready_i) |-> (valid_o) ) else $warning("IF Stage is ready without prefetcher having valid data");

  // never is_crossword while also next_is_crossword
  assert property (
    @(posedge clk) (next_is_crossword) |-> (~is_crossword) ) else $warning("Cannot have two crossword accesses back-to-back");
  assert property (
    @(posedge clk) (is_crossword) |-> (~next_is_crossword) ) else $warning("Cannot have two crossword accesses back-to-back");
`endif

endmodule // prefetch_L0_buffer


module prefetch_L0_buffer_L0
#(
  parameter                                   RDATA_IN_WIDTH = 128
)
(
  input  logic                                clk,
  input  logic                                rst_n,

  input  logic                                prefetch_i,
  input  logic [31:0]                         prefetch_addr_i,

  input  logic                                branch_i,
  input  logic [31:0]                         branch_addr_i,

  input  logic                                hwlp_i,
  input  logic [31:0]                         hwlp_addr_i,


  output logic                                fetch_gnt_o,
  output logic                                fetch_valid_o,

  output logic                                valid_o,
  output logic [RDATA_IN_WIDTH/32-1:0][31:0]  rdata_o,
  output logic [31:0]                         addr_o,

  // goes to instruction memory / instruction cache
  output logic                                instr_req_o,
  output logic [31:0]                         instr_addr_o,
  input  logic                                instr_gnt_i,
  input  logic                                instr_rvalid_i,
  input  logic [RDATA_IN_WIDTH/32-1:0][31:0]  instr_rdata_i,

  output logic                                busy_o
);

  enum logic [2:0] { EMPTY, VALID_L0, WAIT_GNT, WAIT_RVALID, ABORTED_BRANCH, WAIT_HWLOOP } CS, NS;

  logic [3:0][31:0]   L0_buffer;
  logic      [31:0]   addr_q, instr_addr_int;
  logic               valid;


  //////////////////////////////////////////////////////////////////////////////
  // FSM
  //////////////////////////////////////////////////////////////////////////////

  always_comb
  begin
    NS             = CS;
    valid          = 1'b0;
    instr_req_o    = 1'b0;
    instr_addr_int = 'x;
    fetch_valid_o  = 1'b0;

    case(CS)

      // wait for the first branch request before fetching any instructions
      EMPTY:
      begin
        if (branch_i)
          instr_addr_int = branch_addr_i;
        else if (hwlp_i)
          instr_addr_int = hwlp_addr_i;
        else
          instr_addr_int = prefetch_addr_i;

        if (branch_i | hwlp_i | prefetch_i) // make the request to icache
        begin
          instr_req_o    = 1'b1;

          if (instr_gnt_i)
            NS = WAIT_RVALID;
          else
            NS = WAIT_GNT;
        end
      end //~EMPTY

      WAIT_GNT:
      begin
        if (branch_i)
          instr_addr_int = branch_addr_i;
        else if (hwlp_i)
          instr_addr_int = hwlp_addr_i;
        else
          instr_addr_int = addr_q;

        if (branch_i)
        begin
          instr_req_o    = 1'b1;

          if (instr_gnt_i)
            NS = WAIT_RVALID;
          else
            NS = WAIT_GNT;
        end
        else
        begin
          instr_req_o    = 1'b1;

          if (instr_gnt_i)
            NS = WAIT_RVALID;
          else
            NS = WAIT_GNT;
        end
      end //~WAIT_GNT


      WAIT_RVALID:
      begin
        valid   = instr_rvalid_i;

        if (branch_i)
          instr_addr_int = branch_addr_i;
        else if (hwlp_i)
          instr_addr_int = hwlp_addr_i;
        else
          instr_addr_int = prefetch_addr_i;

        if (branch_i)
        begin
          if (instr_rvalid_i)
          begin
            fetch_valid_o  = 1'b1;
            instr_req_o    = 1'b1;

            if (instr_gnt_i)
              NS = WAIT_RVALID;
            else
              NS = WAIT_GNT;
          end else begin
            NS = ABORTED_BRANCH; // TODO: THIS STATE IS IDENTICAL WITH THIS ONE
          end

        end
        else
        begin

          if (instr_rvalid_i)
          begin
            fetch_valid_o = 1'b1;

            if (prefetch_i | hwlp_i) // we are receiving the last packet, then prefetch the next one
            begin
              instr_req_o    = 1'b1;

              if (instr_gnt_i)
                NS = WAIT_RVALID;
              else
                NS = WAIT_GNT;
            end
            else // not the last chunk
            begin
              NS = VALID_L0;
            end
          end
        end
      end //~WAIT_RVALID

      VALID_L0:
      begin
        valid   = 1'b1;

        if (branch_i)
          instr_addr_int = branch_addr_i;
        else if (hwlp_i)
          instr_addr_int = hwlp_addr_i;
        else
          instr_addr_int = prefetch_addr_i;

        if (branch_i | hwlp_i | prefetch_i)
        begin
          instr_req_o    = 1'b1;

          if (instr_gnt_i)
            NS = WAIT_RVALID;
          else
            NS = WAIT_GNT;
        end
      end //~VALID_L0

      ABORTED_BRANCH:
      begin

        // prepare address even if we don't need it
        // this removes the dependency for instr_addr_o on instr_rvalid_i
        if (branch_i)
          instr_addr_int = branch_addr_i;
        else
          instr_addr_int = addr_q;

        if (instr_rvalid_i)
        begin
          instr_req_o    = 1'b1;

          if (instr_gnt_i)
            NS = WAIT_RVALID;
          else
            NS = WAIT_GNT;
        end
      end //~ABORTED_BRANCH

      default:
      begin
         NS = EMPTY;
      end
    endcase //~CS
  end


  //////////////////////////////////////////////////////////////////////////////
  // registers
  //////////////////////////////////////////////////////////////////////////////

  always_ff @(posedge clk, negedge rst_n)
  begin
    if (~rst_n)
    begin
      CS             <= EMPTY;
      L0_buffer      <= '0;
      addr_q         <= '0;
    end
    else
    begin
      CS             <= NS;

      if (instr_rvalid_i)
      begin
        L0_buffer <= instr_rdata_i;
      end

      if (branch_i | hwlp_i | prefetch_i)
        addr_q <= instr_addr_int;
    end
  end


  //////////////////////////////////////////////////////////////////////////////
  // output ports
  //////////////////////////////////////////////////////////////////////////////

  assign instr_addr_o = { instr_addr_int[31:4], 4'b0000 };

  assign rdata_o = (instr_rvalid_i) ? instr_rdata_i : L0_buffer;
  assign addr_o  = addr_q;

  assign valid_o = valid & (~branch_i);

  assign busy_o = (CS != EMPTY) && (CS != VALID_L0) || instr_req_o;

  assign fetch_gnt_o   = instr_gnt_i;

endmodule

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Andreas Traber - atraber@iis.ee.ethz.ch                    //
//                                                                            //
// Design Name:    Prefetcher Buffer for 32 bit memory interface              //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Prefetch Buffer that caches instructions. This cuts overly //
//                 long critical paths to the instruction cache               //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

// input port: send address one cycle before the data
// clear_i clears the FIFO for the following cycle. in_addr_i can be sent in
// this cycle already
module riscv_fetch_fifo
(
    input  logic        clk,
    input  logic        rst_n,

    // control signals
    input  logic        clear_i,          // clears the contents of the fifo

    // input port
    input  logic [31:0] in_addr_i,
    input  logic [31:0] in_rdata_i,
    input  logic        in_valid_i,
    output logic        in_ready_o,

    input  logic        in_replace2_i, // replaces second entry if there is one: "to be served after this instr"
    input  logic        in_is_hwlp_i,

    // output port
    output logic        out_valid_o,
    input  logic        out_ready_i,
    output logic [31:0] out_rdata_o,
    output logic [31:0] out_addr_o,

    output logic        out_valid_stored_o, // same as out_valid_o, except that if something is incoming now it is not included. This signal is available immediately as it comes directly out of FFs
    output logic        out_is_hwlp_o
  );

  localparam DEPTH = 4; // must be 3 or greater

  // index 0 is used for output
  logic [0:DEPTH-1] [31:0]  addr_n,    addr_int,    addr_Q;
  logic [0:DEPTH-1] [31:0]  rdata_n,   rdata_int,   rdata_Q;
  logic [0:DEPTH-1]         valid_n,   valid_int,   valid_Q;
  logic [0:1      ]         is_hwlp_n, is_hwlp_int, is_hwlp_Q;

  logic             [31:0]  addr_next;
  logic             [31:0]  rdata, rdata_unaligned;
  logic                     valid, valid_unaligned;

  logic                     aligned_is_compressed, unaligned_is_compressed;
  logic                     aligned_is_compressed_st, unaligned_is_compressed_st;

  //////////////////////////////////////////////////////////////////////////////
  // output port
  //////////////////////////////////////////////////////////////////////////////


  assign rdata = (valid_Q[0]) ? rdata_Q[0] : in_rdata_i;
  assign valid = valid_Q[0] || in_valid_i || is_hwlp_Q[1];

  assign rdata_unaligned = (valid_Q[1]) ? {rdata_Q[1][15:0], rdata[31:16]} : {in_rdata_i[15:0], rdata[31:16]};
  // it is implied that rdata_valid_Q[0] is set
  assign valid_unaligned = (valid_Q[1] || (valid_Q[0] && in_valid_i));

  assign unaligned_is_compressed    = rdata[17:16] != 2'b11;
  assign aligned_is_compressed      = rdata[1:0] != 2'b11;
  assign unaligned_is_compressed_st = rdata_Q[0][17:16] != 2'b11;
  assign aligned_is_compressed_st   = rdata_Q[0][1:0] != 2'b11;

  //////////////////////////////////////////////////////////////////////////////
  // instruction aligner (if unaligned)
  //////////////////////////////////////////////////////////////////////////////

  always_comb
  begin
    // serve the aligned case even though the output address is unaligned when
    // the next instruction will be from a hardware loop target
    // in this case the current instruction is already prealigned in element 0
    if (out_addr_o[1] && (~is_hwlp_Q[1])) begin
      // unaligned case
      out_rdata_o = rdata_unaligned;

      if (unaligned_is_compressed)
        out_valid_o = valid;
      else
        out_valid_o = valid_unaligned;
    end else begin
      // aligned case
      out_rdata_o = rdata;
      out_valid_o = valid;
    end
  end

  assign out_addr_o    = (valid_Q[0]) ? addr_Q[0] : in_addr_i;
  assign out_is_hwlp_o = (valid_Q[0]) ? is_hwlp_Q[0] : in_is_hwlp_i;

  // this valid signal must not depend on signals from outside!
  always_comb
  begin
    out_valid_stored_o = 1'b1;

    if (out_addr_o[1] && (~is_hwlp_Q[1])) begin
      if (unaligned_is_compressed_st)
        out_valid_stored_o = 1'b1;
      else
        out_valid_stored_o = valid_Q[1];
    end else begin
      out_valid_stored_o = valid_Q[0];
    end
  end


  //////////////////////////////////////////////////////////////////////////////
  // input port
  //////////////////////////////////////////////////////////////////////////////

  // we accept data as long as our fifo is not full
  // we don't care about clear here as the data will be received one cycle
  // later anyway
  assign in_ready_o = ~valid_Q[DEPTH-2];


  //////////////////////////////////////////////////////////////////////////////
  // FIFO management
  //////////////////////////////////////////////////////////////////////////////

  int j;
  always_comb
  begin
    addr_int    = addr_Q;
    rdata_int   = rdata_Q;
    valid_int   = valid_Q;
    is_hwlp_int = is_hwlp_Q;

    if (in_valid_i) begin
      for(j = 0; j < DEPTH; j++) begin
        if (~valid_Q[j]) begin
          addr_int[j]  = in_addr_i;
          rdata_int[j] = in_rdata_i;
          valid_int[j] = 1'b1;

          break;
        end
      end

      // replace 2nd entry
      if (in_replace2_i) begin
        if (valid_Q[0]) begin
          addr_int[1]          = in_addr_i;

          // if we replace the 2nd entry, let's cache the output word in case we
          // still need it and it would span two words in the FIFO
          rdata_int[0]         = out_rdata_o;
          rdata_int[1]         = in_rdata_i;
          valid_int[1]         = 1'b1;
          valid_int[2:DEPTH-1] = '0;

          // hardware loop incoming?
          is_hwlp_int[1] = in_is_hwlp_i;
        end else begin
          is_hwlp_int[0] = in_is_hwlp_i;
        end
      end
    end
  end

  assign addr_next = {addr_int[0][31:2], 2'b00} + 32'h4;

  // move everything by one step
  always_comb
  begin
    addr_n     = addr_int;
    rdata_n    = rdata_int;
    valid_n    = valid_int;
    is_hwlp_n  = is_hwlp_int;

    if (out_ready_i && out_valid_o) begin
      is_hwlp_n = {is_hwlp_int[1], 1'b0};

      if (is_hwlp_int[1]) begin
        addr_n[0] = addr_int[1][31:0];
        rdata_n   = {rdata_int[1:DEPTH-1], 32'b0};
        valid_n   = {valid_int[1:DEPTH-1], 1'b0};
      end else begin
        if (addr_int[0][1]) begin
          // unaligned case
          if (unaligned_is_compressed) begin
            addr_n[0] = {addr_next[31:2], 2'b00};
          end else begin
            addr_n[0] = {addr_next[31:2], 2'b10};
          end

          rdata_n  = {rdata_int[1:DEPTH-1], 32'b0};
          valid_n  = {valid_int[1:DEPTH-1], 1'b0};
        end else begin
          // aligned case
          if (aligned_is_compressed) begin
            // just increase address, do not move to next entry in FIFO
            addr_n[0] = {addr_int[0][31:2], 2'b10};
          end else begin
            // move to next entry in FIFO
            addr_n[0] = {addr_next[31:2], 2'b00};
            rdata_n   = {rdata_int[1:DEPTH-1], 32'b0};
            valid_n   = {valid_int[1:DEPTH-1], 1'b0};
          end
        end
      end
    end
  end

  //////////////////////////////////////////////////////////////////////////////
  // registers
  //////////////////////////////////////////////////////////////////////////////

  always_ff @(posedge clk, negedge rst_n)
  begin
    if(rst_n == 1'b0)
    begin
      addr_Q    <= '{default: '0};
      rdata_Q   <= '{default: '0};
      valid_Q   <= '0;
      is_hwlp_Q <= '0;
    end
    else
    begin
      // on a clear signal from outside we invalidate the content of the FIFO
      // completely and start from an empty state
      if (clear_i) begin
        valid_Q    <= '0;
        is_hwlp_Q  <= '0;
      end else begin
        addr_Q    <= addr_n;
        rdata_Q   <= rdata_n;
        valid_Q   <= valid_n;
        is_hwlp_Q <= is_hwlp_n;
      end
    end
  end

`ifndef verilator
  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------

  // check for FIFO overflows
  assert property (
    @(posedge clk) (in_valid_i) |-> ((valid_Q[DEPTH-1] == 1'b0) || (clear_i == 1'b1) || (in_replace2_i == 1'b1)) );
`endif

endmodule


module riscv_prefetch_buffer
(
  input  logic        clk,
  input  logic        rst_n,

  input  logic        req_i,

  input  logic        branch_i,
  input  logic [31:0] addr_i,

  input  logic        hwloop_i,
  input  logic [31:0] hwloop_target_i,

  input  logic        ready_i,
  output logic        valid_o,
  output logic [31:0] rdata_o,
  output logic [31:0] addr_o,
  output logic        is_hwlp_o, // is set when the currently served data is from a hwloop

  // goes to instruction memory / instruction cache
  output logic        instr_req_o,
  input  logic        instr_gnt_i,
  output logic [31:0] instr_addr_o,
  input  logic [31:0] instr_rdata_i,
  input  logic        instr_rvalid_i,

  // Prefetch Buffer Status
  output logic        busy_o
);

  enum logic [1:0] {IDLE, WAIT_GNT, WAIT_RVALID, WAIT_ABORTED } CS, NS;
  enum logic [1:0] {HWLP_NONE, HWLP_IN, HWLP_FETCHING, HWLP_DONE } hwlp_CS, hwlp_NS;

  logic [31:0] instr_addr_q, fetch_addr;
  logic        fetch_is_hwlp;
  logic        addr_valid;

  logic        fifo_valid;
  logic        fifo_ready;
  logic        fifo_clear;
  logic        fifo_hwlp;

  logic        valid_stored;
  logic        hwlp_masked;


  //////////////////////////////////////////////////////////////////////////////
  // prefetch buffer status
  //////////////////////////////////////////////////////////////////////////////

  assign busy_o = (CS != IDLE) || instr_req_o;

  //////////////////////////////////////////////////////////////////////////////
  // fetch fifo
  // consumes addresses and rdata
  //////////////////////////////////////////////////////////////////////////////

  riscv_fetch_fifo fifo_i
  (
    .clk                   ( clk               ),
    .rst_n                 ( rst_n             ),

    .clear_i               ( fifo_clear        ),

    .in_addr_i             ( instr_addr_q      ),
    .in_rdata_i            ( instr_rdata_i     ),
    .in_valid_i            ( fifo_valid        ),
    .in_ready_o            ( fifo_ready        ),

    .in_replace2_i         ( fifo_hwlp         ),
    .in_is_hwlp_i          ( fifo_hwlp         ),

    .out_valid_o           ( valid_o           ),
    .out_ready_i           ( ready_i           ),
    .out_rdata_o           ( rdata_o           ),
    .out_addr_o            ( addr_o            ),

    .out_valid_stored_o    ( valid_stored      ),
    .out_is_hwlp_o         ( is_hwlp_o         )
  );


  //////////////////////////////////////////////////////////////////////////////
  // fetch addr
  //////////////////////////////////////////////////////////////////////////////

  assign fetch_addr = {instr_addr_q[31:2], 2'b00} + 32'd4;

  always_comb
  begin
    hwlp_NS     = hwlp_CS;
    fifo_hwlp   = 1'b0;
    hwlp_masked = 1'b0;
    fifo_clear  = 1'b0;

    unique case (hwlp_CS)
      HWLP_NONE: begin
        if (hwloop_i) begin
          hwlp_masked = 1'b1;

          if (fetch_is_hwlp)
            hwlp_NS = HWLP_FETCHING;
          else
            hwlp_NS = HWLP_IN;

          if (ready_i)
            fifo_clear = 1'b1;
        end
      end

      HWLP_IN: begin
        hwlp_masked = 1'b1;

        if (fetch_is_hwlp)
          hwlp_NS = HWLP_FETCHING;

        if (ready_i)
          fifo_clear = 1'b1;
      end

      // just waiting for rvalid really
      HWLP_FETCHING: begin
        fifo_hwlp = 1'b1;

        if (instr_rvalid_i & (CS != WAIT_ABORTED)) begin
          if (valid_o & is_hwlp_o)
            hwlp_NS = HWLP_NONE;
          else
            hwlp_NS = HWLP_DONE;
        end else begin
          if (ready_i)
            fifo_clear = 1'b1;
        end
      end

      HWLP_DONE: begin
        if (valid_o & is_hwlp_o)
          hwlp_NS = HWLP_NONE;
      end

      default: begin
        hwlp_NS = HWLP_NONE;
      end
    endcase

    if (branch_i) begin
      hwlp_NS    = HWLP_NONE;
      fifo_clear = 1'b1;
    end
  end

  //////////////////////////////////////////////////////////////////////////////
  // instruction fetch FSM
  // deals with instruction memory / instruction cache
  //////////////////////////////////////////////////////////////////////////////

  always_comb
  begin
    instr_req_o   = 1'b0;
    instr_addr_o  = fetch_addr;
    fifo_valid    = 1'b0;
    addr_valid    = 1'b0;
    fetch_is_hwlp = 1'b0;
    NS            = CS;

    unique case(CS)
      // default state, not waiting for requested data
      IDLE:
      begin
        instr_addr_o = fetch_addr;
        instr_req_o  = 1'b0;

        if (branch_i)
          instr_addr_o = addr_i;
        else if(hwlp_masked & valid_stored)
          instr_addr_o = hwloop_target_i;

        if (req_i & (fifo_ready | branch_i | (hwlp_masked & valid_stored))) begin
          instr_req_o = 1'b1;
          addr_valid  = 1'b1;

          if (hwlp_masked & valid_stored) begin
            fetch_is_hwlp = 1'b1;
          end

          if(instr_gnt_i) //~>  granted request
            NS = WAIT_RVALID;
          else begin //~> got a request but no grant
            NS = WAIT_GNT;
          end
        end
      end // case: IDLE

      // we sent a request but did not yet get a grant
      WAIT_GNT:
      begin
        instr_addr_o = instr_addr_q;
        instr_req_o  = 1'b1;

        if (branch_i) begin
          instr_addr_o = addr_i;
          addr_valid   = 1'b1;
        end else if (hwlp_masked & valid_stored) begin
          instr_addr_o  = hwloop_target_i;
          addr_valid    = 1'b1;
          fetch_is_hwlp = 1'b1;
        end

        if(instr_gnt_i)
          NS = WAIT_RVALID;
        else
          NS = WAIT_GNT;
      end // case: WAIT_GNT

      // we wait for rvalid, after that we are ready to serve a new request
      WAIT_RVALID: begin
        instr_addr_o = fetch_addr;

        if (branch_i)
          instr_addr_o  = addr_i;
        else if (hwlp_masked)
          instr_addr_o  = hwloop_target_i;

        if (req_i & (fifo_ready | branch_i | hwlp_masked)) begin
          // prepare for next request

          if (instr_rvalid_i) begin
            instr_req_o = 1'b1;
            fifo_valid  = 1'b1;
            addr_valid  = 1'b1;

            if (hwlp_masked) begin
              fetch_is_hwlp = 1'b1;
            end

            if (instr_gnt_i) begin
              NS = WAIT_RVALID;
            end else begin
              NS = WAIT_GNT;
            end
          end else begin
            // we are requested to abort our current request
            // we didn't get an rvalid yet, so wait for it
            if (branch_i) begin
              addr_valid = 1'b1;
              NS         = WAIT_ABORTED;
            end else if (hwlp_masked & valid_o) begin
              addr_valid    = 1'b1;
              fetch_is_hwlp = 1'b1;
              NS            = WAIT_ABORTED;
            end
          end
        end else begin
          // just wait for rvalid and go back to IDLE, no new request

          if (instr_rvalid_i) begin
            fifo_valid = 1'b1;
            NS         = IDLE;
          end
        end
      end // case: WAIT_RVALID

      // our last request was aborted, but we didn't yet get a rvalid and
      // there was no new request sent yet
      // we assume that req_i is set to high
      WAIT_ABORTED: begin
        instr_addr_o = instr_addr_q;

        if (branch_i) begin
          instr_addr_o = addr_i;
          addr_valid   = 1'b1;
        end

        if (instr_rvalid_i) begin
          instr_req_o  = 1'b1;
          // no need to send address, already done in WAIT_RVALID

          if (instr_gnt_i) begin
            NS = WAIT_RVALID;
          end else begin
            NS = WAIT_GNT;
          end
        end
      end

      default:
      begin
        NS          = IDLE;
        instr_req_o = 1'b0;
      end
    endcase
  end

  //////////////////////////////////////////////////////////////////////////////
  // registers
  //////////////////////////////////////////////////////////////////////////////

  always_ff @(posedge clk, negedge rst_n)
  begin
    if(rst_n == 1'b0)
    begin
      CS              <= IDLE;
      hwlp_CS         <= HWLP_NONE;
      instr_addr_q    <= '0;
    end
    else
    begin
      CS              <= NS;
      hwlp_CS         <= hwlp_NS;

      if (addr_valid) begin
        instr_addr_q    <= instr_addr_o;
      end
    end
  end

endmodule

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the âLicenseâ?); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an âAS ISâ? BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Renzo Andri - andrire@student.ethz.ch                      //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Andreas Traber - atraber@student.ethz.ch                   //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
// Design Name:    Instruction Decode Stage                                   //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Decode stage of the core. It decodes the instructions      //
//                 and hosts the register file.                               //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


import riscv_defines::*;

// Source/Destination register instruction index
`define REG_S1 19:15
`define REG_S2 24:20
`define REG_S3 29:25
`define REG_D  11:07

module riscv_id_stage
#(
  parameter N_HWLP      = 2,
  parameter N_HWLP_BITS = $clog2(N_HWLP)
)
(
    input  logic        clk,
    input  logic        rst_n,

    input  logic        test_en_i,

    input  logic        fetch_enable_i,
    output logic        ctrl_busy_o,
    output logic        is_decoding_o,

    // Interface to IF stage
    input  logic [N_HWLP-1:0] hwlp_dec_cnt_i,
    input  logic              is_hwlp_i,
    input  logic              instr_valid_i,
    input  logic       [31:0] instr_rdata_i,      // comes from pipeline of IF stage
    output logic              instr_req_o,


    // Jumps and branches
    output logic        branch_in_ex_o,
    input  logic        branch_decision_i,
    output logic [31:0] jump_target_o,

    // IF and ID stage signals
    output logic        clear_instr_valid_o,
    output logic        pc_set_o,
    output logic [2:0]  pc_mux_o,
    output logic [1:0]  exc_pc_mux_o,
    output logic [4:0]  exc_vec_pc_mux_o,

    input  logic        illegal_c_insn_i,
    input  logic        is_compressed_i,

    input  logic [31:0] pc_if_i,
    input  logic [31:0] pc_id_i,

    // Stalls
    output logic        halt_if_o,      // controller requests a halt of the IF stage

    output logic        id_ready_o,     // ID stage is ready for the next instruction
    input  logic        ex_ready_i,     // EX stage is ready for the next instruction

    input  logic        if_ready_i,     // IF stage is done
    input  logic        if_valid_i,     // IF stage is done
    output logic        id_valid_o,     // ID stage is done
    input  logic        ex_valid_i,     // EX stage is done
    input  logic        wb_valid_i,     // WB stage is done

    // Pipeline ID/EX
    output logic [31:0] pc_ex_o,

    output logic [31:0] alu_operand_a_ex_o,
    output logic [31:0] alu_operand_b_ex_o,
    output logic [31:0] alu_operand_c_ex_o,
    output logic [ 4:0] bmask_a_ex_o,
    output logic [ 4:0] bmask_b_ex_o,
    output logic [ 1:0] imm_vec_ext_ex_o,
    output logic [ 1:0] alu_vec_mode_ex_o,

    output logic [4:0]  regfile_waddr_ex_o,
    output logic        regfile_we_ex_o,

    output logic [4:0]  regfile_alu_waddr_ex_o,
    output logic        regfile_alu_we_ex_o,

    // ALU
    output logic [ALU_OP_WIDTH-1:0] alu_operator_ex_o,


    // MUL
    output logic [ 2:0] mult_operator_ex_o,
    output logic [31:0] mult_operand_a_ex_o,
    output logic [31:0] mult_operand_b_ex_o,
    output logic [31:0] mult_operand_c_ex_o,
    output logic        mult_en_ex_o,
    output logic        mult_sel_subword_ex_o,
    output logic [ 1:0] mult_signed_mode_ex_o,
    output logic [ 4:0] mult_imm_ex_o,

    output logic [31:0] mult_dot_op_a_ex_o,
    output logic [31:0] mult_dot_op_b_ex_o,
    output logic [31:0] mult_dot_op_c_ex_o,
    output logic [ 1:0] mult_dot_signed_ex_o,

    // CSR ID/EX
    output logic        csr_access_ex_o,
    output logic [1:0]  csr_op_ex_o,

    // hwloop signals
    output logic [N_HWLP-1:0] [31:0] hwlp_start_o,
    output logic [N_HWLP-1:0] [31:0] hwlp_end_o,
    output logic [N_HWLP-1:0] [31:0] hwlp_cnt_o,

    // hwloop signals from CS register
    input  logic   [N_HWLP_BITS-1:0] csr_hwlp_regid_i,
    input  logic               [2:0] csr_hwlp_we_i,
    input  logic              [31:0] csr_hwlp_data_i,

    // Interface to load store unit
    output logic        data_req_ex_o,
    output logic        data_we_ex_o,
    output logic [1:0]  data_type_ex_o,
    output logic        data_sign_ext_ex_o,
    output logic [1:0]  data_reg_offset_ex_o,
    output logic        data_load_event_ex_o,

    output logic        data_misaligned_ex_o,

    output logic        prepost_useincr_ex_o,
    input  logic        data_misaligned_i,

    // Interrupt signals
    input  logic [31:0] irq_i,
    input  logic        irq_enable_i,

    output logic [5:0]  exc_cause_o,
    output logic        save_exc_cause_o,

    output logic        exc_save_if_o,
    output logic        exc_save_id_o,
    output logic        exc_restore_id_o,

    input  logic        lsu_load_err_i,
    input  logic        lsu_store_err_i,

    // Debug Unit Signals
    input  logic [DBG_SETS_W-1:0] dbg_settings_i,
    input  logic        dbg_req_i,
    output logic        dbg_ack_o,
    input  logic        dbg_stall_i,
    output logic        dbg_trap_o,

    input  logic        dbg_reg_rreq_i,
    input  logic [ 4:0] dbg_reg_raddr_i,
    output logic [31:0] dbg_reg_rdata_o,

    input  logic        dbg_reg_wreq_i,
    input  logic [ 4:0] dbg_reg_waddr_i,
    input  logic [31:0] dbg_reg_wdata_i,

    input  logic        dbg_jump_req_i,

    // Forward Signals
    input  logic [4:0]  regfile_waddr_wb_i,
    input  logic        regfile_we_wb_i,
    input  logic [31:0] regfile_wdata_wb_i, // From wb_stage: selects data from data memory, ex_stage result and sp rdata

    input  logic [4:0]  regfile_alu_waddr_fw_i,
    input  logic        regfile_alu_we_fw_i,
    input  logic [31:0] regfile_alu_wdata_fw_i,

    // from ALU
    input  logic        mult_multicycle_i,    // when we need multiple cycles in the multiplier and use op c as storage

    // Performance Counters
    output logic        perf_jump_o,          // we are executing a jump instruction
    output logic        perf_jr_stall_o,      // jump-register-hazard
    output logic        perf_ld_stall_o,       // load-use-hazard
     // HDC opcode from decoder output signal
    output logic        start
);
  
  // Instruction
  logic [31:0] instr;

  // Decoder/Controller ID stage internal signals
  logic        deassert_we;

  logic        illegal_insn_dec;
  logic        ebrk_insn;
  logic        eret_insn_dec;
  logic        ecall_insn_dec;
  logic        pipe_flush_dec;

  logic        rega_used_dec;
  logic        regb_used_dec;
  logic        regc_used_dec;
  logic        bmask_needed_dec;

  logic        branch_taken_ex;
  logic [1:0]  jump_in_id;
  logic [1:0]  jump_in_dec;

  logic        misaligned_stall;
  logic        jr_stall;
  logic        load_stall;

  logic        halt_id;


  // Immediate decoding and sign extension
  logic [31:0] imm_i_type;
  logic [31:0] imm_iz_type;
  logic [31:0] imm_s_type;
  logic [31:0] imm_sb_type;
  logic [31:0] imm_u_type;
  logic [31:0] imm_uj_type;
  logic [31:0] imm_z_type;
  logic [31:0] imm_s2_type;
  logic [31:0] imm_bi_type;
  logic [31:0] imm_s3_type;
  logic [31:0] imm_vs_type;
  logic [31:0] imm_vu_type;
  logic [31:0] imm_shuffleb_type;
  logic [31:0] imm_shuffleh_type;
  logic [31:0] imm_shuffle_type;
  logic [31:0] imm_clip_type;

  logic [31:0] imm_a;       // contains the immediate for operand b
  logic [31:0] imm_b;       // contains the immediate for operand b

  logic [31:0] jump_target;       // calculated jump target (-> EX -> IF)



  // Signals running between controller and exception controller
  logic        exc_req, exc_ack;  // handshake

  // Register file interface
  logic [4:0]  regfile_addr_ra_id;
  logic [4:0]  regfile_addr_rb_id;
  logic [4:0]  regfile_addr_rc_id;

  logic [4:0]  regfile_waddr_id;
  logic [4:0]  regfile_alu_waddr_id;
  logic        regfile_alu_we_id;

  logic [31:0] regfile_data_ra_id;
  logic [31:0] regfile_data_rb_id;
  logic [31:0] regfile_data_rc_id;

  // ALU Control
  logic [ALU_OP_WIDTH-1:0] alu_operator;
  logic [1:0]  alu_op_a_mux_sel;
  logic [1:0]  alu_op_b_mux_sel;
  logic [1:0]  alu_op_c_mux_sel;
  logic [1:0]  regc_mux;

  logic [0:0]  imm_a_mux_sel;
  logic [3:0]  imm_b_mux_sel;
  logic [1:0]  jump_target_mux_sel;

  // Multiplier Control
  logic [2:0]  mult_operator;    // multiplication operation selection
  logic        mult_en;          // multiplication is used instead of ALU
  logic        mult_int_en;      // use integer multiplier
  logic        mult_sel_subword; // Select a subword when doing multiplications
  logic [1:0]  mult_signed_mode; // Signed mode multiplication at the output of the controller, and before the pipe registers
  logic        mult_dot_en;      // use dot product
  logic [1:0]  mult_dot_signed;  // Signed mode dot products (can be mixed types)

  // Register Write Control
  logic        regfile_we_id;
  logic        regfile_alu_waddr_mux_sel;

  // Data Memory Control
  logic        data_we_id;
  logic [1:0]  data_type_id;
  logic        data_sign_ext_id;
  logic [1:0]  data_reg_offset_id;
  logic        data_req_id;
  logic        data_load_event_id;

  // hwloop signals
  logic [N_HWLP_BITS-1:0] hwloop_regid, hwloop_regid_int;
  logic             [2:0] hwloop_we, hwloop_we_int;
  logic                   hwloop_target_mux_sel;
  logic                   hwloop_start_mux_sel;
  logic                   hwloop_cnt_mux_sel;

  logic            [31:0] hwloop_target;
  logic            [31:0] hwloop_start, hwloop_start_int;
  logic            [31:0] hwloop_end;
  logic            [31:0] hwloop_cnt, hwloop_cnt_int;

  logic                   hwloop_valid;

  // CSR control
  logic        csr_access;
  logic [1:0]  csr_op;

  logic        prepost_useincr;

  // Forwarding
  logic [1:0]  operand_a_fw_mux_sel;
  logic [1:0]  operand_b_fw_mux_sel;
  logic [1:0]  operand_c_fw_mux_sel;
  logic [31:0] operand_a_fw_id;
  logic [31:0] operand_b_fw_id;
  logic [31:0] operand_c_fw_id;

  logic [31:0] operand_b, operand_b_vec;

  logic [31:0] alu_operand_a;
  logic [31:0] alu_operand_b;
  logic [31:0] alu_operand_c;

  // Immediates for ID
  logic [0:0]  bmask_a_mux;
  logic [1:0]  bmask_b_mux;
  logic [0:0]  mult_imm_mux;

  logic [ 4:0] bmask_a_id;
  logic [ 4:0] bmask_b_id;
  logic [ 1:0] imm_vec_ext_id;
  logic [ 4:0] mult_imm_id;

  logic [ 1:0] alu_vec_mode;
  logic        scalar_replication;

  // Forwarding detection signals
  logic        reg_d_ex_is_reg_a_id;
  logic        reg_d_ex_is_reg_b_id;
  logic        reg_d_ex_is_reg_c_id;
  logic        reg_d_wb_is_reg_a_id;
  logic        reg_d_wb_is_reg_b_id;
  logic        reg_d_wb_is_reg_c_id;
  logic        reg_d_alu_is_reg_a_id;
  logic        reg_d_alu_is_reg_b_id;
  logic        reg_d_alu_is_reg_c_id;


  assign instr = instr_rdata_i;

  // immediate extraction and sign extension
  assign imm_i_type  = { {20 {instr[31]}}, instr[31:20] };
  assign imm_iz_type = {            20'b0, instr[31:20] };
  assign imm_s_type  = { {20 {instr[31]}}, instr[31:25], instr[11:7] };
  assign imm_sb_type = { {19 {instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0 };
  assign imm_u_type  = { instr[31:12], 12'b0 };
  assign imm_uj_type = { {12 {instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0 };

  // immediate for CSR manipulatin (zero extended)
  assign imm_z_type  = { 27'b0, instr[`REG_S1] };

  assign imm_s2_type = { 27'b0, instr[24:20] };
  assign imm_bi_type = { {27{instr[24]}}, instr[24:20] };
  assign imm_s3_type = { 27'b0, instr[29:25] };
  assign imm_vs_type = { {26 {instr[24]}}, instr[24:20], instr[25] };
  assign imm_vu_type = { 26'b0, instr[24:20], instr[25] };

  // same format as rS2 for shuffle needs, expands immediate
  assign imm_shuffleb_type = {6'b0, instr[28:27], 6'b0, instr[24:23], 6'b0, instr[22:21], 6'b0, instr[20], instr[25]};
  assign imm_shuffleh_type = {15'h0, instr[20], 15'h0, instr[25]};

  // clipping immediate, uses a small barrel shifter to pre-process the
  // immediate and an adder to subtract 1
  // The end result is a mask that has 1's set in the lower part
  // TODO: check if this can be shared with the bit-manipulation unit
  assign imm_clip_type    = (32'h1 << instr[24:20]) - 1;

  //---------------------------------------------------------------------------
  // source register selection
  //---------------------------------------------------------------------------
  assign regfile_addr_ra_id = instr[`REG_S1];
  assign regfile_addr_rb_id = instr[`REG_S2];

  // register C mux
  always_comb
  begin
    unique case (regc_mux)
      REGC_ZERO:  regfile_addr_rc_id = '0;
      REGC_RD:    regfile_addr_rc_id = instr[`REG_D];
      REGC_S1:    regfile_addr_rc_id = instr[`REG_S1];
      default:     regfile_addr_rc_id = '0;
    endcase
  end

  //---------------------------------------------------------------------------
  // destination registers
  //---------------------------------------------------------------------------
  assign regfile_waddr_id = instr[`REG_D];

  // Second Register Write Adress Selection
  // Used for prepost load/store and multiplier
  assign regfile_alu_waddr_id = regfile_alu_waddr_mux_sel ?
                                regfile_waddr_id : regfile_addr_ra_id;

  // Forwarding control signals
  assign reg_d_ex_is_reg_a_id  = (regfile_waddr_ex_o     == regfile_addr_ra_id) && (rega_used_dec == 1'b1) && (regfile_addr_ra_id != '0);
  assign reg_d_ex_is_reg_b_id  = (regfile_waddr_ex_o     == regfile_addr_rb_id) && (regb_used_dec == 1'b1) && (regfile_addr_rb_id != '0);
  assign reg_d_ex_is_reg_c_id  = (regfile_waddr_ex_o     == regfile_addr_rc_id) && (regc_used_dec == 1'b1) && (regfile_addr_rc_id != '0);
  assign reg_d_wb_is_reg_a_id  = (regfile_waddr_wb_i     == regfile_addr_ra_id) && (rega_used_dec == 1'b1) && (regfile_addr_ra_id != '0);
  assign reg_d_wb_is_reg_b_id  = (regfile_waddr_wb_i     == regfile_addr_rb_id) && (regb_used_dec == 1'b1) && (regfile_addr_rb_id != '0);
  assign reg_d_wb_is_reg_c_id  = (regfile_waddr_wb_i     == regfile_addr_rc_id) && (regc_used_dec == 1'b1) && (regfile_addr_rc_id != '0);
  assign reg_d_alu_is_reg_a_id = (regfile_alu_waddr_fw_i == regfile_addr_ra_id) && (rega_used_dec == 1'b1) && (regfile_addr_ra_id != '0);
  assign reg_d_alu_is_reg_b_id = (regfile_alu_waddr_fw_i == regfile_addr_rb_id) && (regb_used_dec == 1'b1) && (regfile_addr_rb_id != '0);
  assign reg_d_alu_is_reg_c_id = (regfile_alu_waddr_fw_i == regfile_addr_rc_id) && (regc_used_dec == 1'b1) && (regfile_addr_rc_id != '0);



  // kill instruction in the IF/ID stage by setting the instr_valid_id control
  // signal to 0 for instructions that are done
  assign clear_instr_valid_o = id_ready_o | halt_id;

  assign branch_taken_ex = branch_in_ex_o & branch_decision_i;


  assign mult_en = mult_int_en | mult_dot_en;

  ///////////////////////////////////////////////
  //  _   ___        ___     ___   ___  ____   //
  // | | | \ \      / / |   / _ \ / _ \|  _ \  //
  // | |_| |\ \ /\ / /| |  | | | | | | | |_) | //
  // |  _  | \ V  V / | |__| |_| | |_| |  __/  //
  // |_| |_|  \_/\_/  |_____\___/ \___/|_|     //
  //                                           //
  ///////////////////////////////////////////////

  // hwloop register id
  assign hwloop_regid_int = instr[7];   // rd contains hwloop register id

  // hwloop target mux
  always_comb
  begin
    case (hwloop_target_mux_sel)
      1'b0: hwloop_target = pc_id_i + {imm_iz_type[30:0], 1'b0};
      1'b1: hwloop_target = pc_id_i + {imm_z_type[30:0], 1'b0};
    endcase
  end

  // hwloop start mux
  always_comb
  begin
    case (hwloop_start_mux_sel)
      1'b0: hwloop_start_int = hwloop_target;   // for PC + I imm
      1'b1: hwloop_start_int = pc_if_i;         // for next PC
    endcase
  end


  // hwloop cnt mux
  always_comb
  begin : hwloop_cnt_mux
    case (hwloop_cnt_mux_sel)
      1'b0: hwloop_cnt_int = imm_iz_type;
      1'b1: hwloop_cnt_int = operand_a_fw_id;
    endcase;
  end

  // multiplex between access from instructions and access via CSR registers
  assign hwloop_start = hwloop_we_int[0] ? hwloop_start_int : csr_hwlp_data_i;
  assign hwloop_end   = hwloop_we_int[1] ? hwloop_target    : csr_hwlp_data_i;
  assign hwloop_cnt   = hwloop_we_int[2] ? hwloop_cnt_int   : csr_hwlp_data_i;
  assign hwloop_regid = (|hwloop_we_int) ? hwloop_regid_int : csr_hwlp_regid_i;
  assign hwloop_we    = (|hwloop_we_int) ? hwloop_we_int    : csr_hwlp_we_i;


  //////////////////////////////////////////////////////////////////
  //      _                         _____                    _    //
  //     | |_   _ _ __ ___  _ __   |_   _|_ _ _ __ __ _  ___| |_  //
  //  _  | | | | | '_ ` _ \| '_ \    | |/ _` | '__/ _` |/ _ \ __| //
  // | |_| | |_| | | | | | | |_) |   | | (_| | | | (_| |  __/ |_  //
  //  \___/ \__,_|_| |_| |_| .__/    |_|\__,_|_|  \__, |\___|\__| //
  //                       |_|                    |___/           //
  //////////////////////////////////////////////////////////////////

  always_comb
  begin : jump_target_mux
    unique case (jump_target_mux_sel)
      JT_JAL:  jump_target = pc_id_i + imm_uj_type;
      JT_COND: jump_target = pc_id_i + imm_sb_type;

      // JALR: Cannot forward RS1, since the path is too long
      JT_JALR: jump_target = regfile_data_ra_id + imm_i_type;
      default:  jump_target = regfile_data_ra_id + imm_i_type;
    endcase
  end

  assign jump_target_o = jump_target;


  ////////////////////////////////////////////////////////
  //   ___                                 _      _     //
  //  / _ \ _ __   ___ _ __ __ _ _ __   __| |    / \    //
  // | | | | '_ \ / _ \ '__/ _` | '_ \ / _` |   / _ \   //
  // | |_| | |_) |  __/ | | (_| | | | | (_| |  / ___ \  //
  //  \___/| .__/ \___|_|  \__,_|_| |_|\__,_| /_/   \_\ //
  //       |_|                                          //
  ////////////////////////////////////////////////////////

  // ALU_Op_a Mux
  always_comb
  begin : alu_operand_a_mux
    case (alu_op_a_mux_sel)
      OP_A_REGA_OR_FWD:  alu_operand_a = operand_a_fw_id;
      OP_A_REGB_OR_FWD:  alu_operand_a = operand_b_fw_id;
      OP_A_CURRPC:       alu_operand_a = pc_id_i;
      OP_A_IMM:          alu_operand_a = imm_a;
      default:            alu_operand_a = operand_a_fw_id;
    endcase; // case (alu_op_a_mux_sel)
  end

  always_comb
  begin : immediate_a_mux
    unique case (imm_a_mux_sel)
      IMMA_Z:      imm_a = imm_z_type;
      IMMA_ZERO:   imm_a = '0;
      default:      imm_a = '0;
    endcase
  end

  // Operand a forwarding mux
  always_comb
  begin : operand_a_fw_mux
    case (operand_a_fw_mux_sel)
      SEL_FW_EX:    operand_a_fw_id = regfile_alu_wdata_fw_i;
      SEL_FW_WB:    operand_a_fw_id = regfile_wdata_wb_i;
      SEL_REGFILE:  operand_a_fw_id = regfile_data_ra_id;
      default:       operand_a_fw_id = regfile_data_ra_id;
    endcase; // case (operand_a_fw_mux_sel)
  end

  //////////////////////////////////////////////////////
  //   ___                                 _   ____   //
  //  / _ \ _ __   ___ _ __ __ _ _ __   __| | | __ )  //
  // | | | | '_ \ / _ \ '__/ _` | '_ \ / _` | |  _ \  //
  // | |_| | |_) |  __/ | | (_| | | | | (_| | | |_) | //
  //  \___/| .__/ \___|_|  \__,_|_| |_|\__,_| |____/  //
  //       |_|                                        //
  //////////////////////////////////////////////////////

  // Immediate Mux for operand B
  // TODO: check if sign-extension stuff works well here, maybe able to save
  // some area here
  always_comb
  begin : immediate_b_mux
    unique case (imm_b_mux_sel)
      IMMB_I:      imm_b = imm_i_type;
      IMMB_S:      imm_b = imm_s_type;
      IMMB_U:      imm_b = imm_u_type;
      IMMB_PCINCR: imm_b = (is_compressed_i && (~data_misaligned_i)) ? 32'h2 : 32'h4;
      IMMB_S2:     imm_b = imm_s2_type;
      IMMB_BI:     imm_b = imm_bi_type;
      IMMB_S3:     imm_b = imm_s3_type;
      IMMB_VS:     imm_b = imm_vs_type;
      IMMB_VU:     imm_b = imm_vu_type;
      IMMB_SHUF:   imm_b = imm_shuffle_type;
      IMMB_CLIP:   imm_b = {1'b0, imm_clip_type[31:1]};
      default:      imm_b = imm_i_type;
    endcase
  end

  // ALU_Op_b Mux
  always_comb
  begin : alu_operand_b_mux
    case (alu_op_b_mux_sel)
      OP_B_REGB_OR_FWD:  operand_b = operand_b_fw_id;
      OP_B_REGC_OR_FWD:  operand_b = operand_c_fw_id;
      OP_B_IMM:          operand_b = imm_b;
      default:           operand_b = operand_b_fw_id;
    endcase // case (alu_op_b_mux_sel)
  end


  // scalar replication for operand B and shuffle type
  always_comb
  begin
    if (alu_vec_mode == VEC_MODE8) begin
      operand_b_vec    = {4{operand_b[7:0]}};
      imm_shuffle_type = imm_shuffleb_type;
    end else begin
      operand_b_vec    = {2{operand_b[15:0]}};
      imm_shuffle_type = imm_shuffleh_type;
    end
  end

  // choose normal or scalar replicated version of operand b
  assign alu_operand_b = (scalar_replication == 1'b1) ? operand_b_vec : operand_b;


  // Operand b forwarding mux
  always_comb
  begin : operand_b_fw_mux
    case (operand_b_fw_mux_sel)
      SEL_FW_EX:    operand_b_fw_id = regfile_alu_wdata_fw_i;
      SEL_FW_WB:    operand_b_fw_id = regfile_wdata_wb_i;
      SEL_REGFILE:  operand_b_fw_id = regfile_data_rb_id;
      default:       operand_b_fw_id = regfile_data_rb_id;
    endcase; // case (operand_b_fw_mux_sel)
  end


  //////////////////////////////////////////////////////
  //   ___                                 _    ____  //
  //  / _ \ _ __   ___ _ __ __ _ _ __   __| |  / ___| //
  // | | | | '_ \ / _ \ '__/ _` | '_ \ / _` | | |     //
  // | |_| | |_) |  __/ | | (_| | | | | (_| | | |___  //
  //  \___/| .__/ \___|_|  \__,_|_| |_|\__,_|  \____| //
  //       |_|                                        //
  //////////////////////////////////////////////////////

  // ALU OP C Mux
  always_comb
  begin : alu_operand_c_mux
    case (alu_op_c_mux_sel)
      OP_C_REGC_OR_FWD:  alu_operand_c = operand_c_fw_id;
      OP_C_REGB_OR_FWD:  alu_operand_c = operand_b_fw_id;
      OP_C_JT:           alu_operand_c = jump_target;
      default:            alu_operand_c = operand_c_fw_id;
    endcase // case (alu_op_c_mux_sel)
  end

  // Operand c forwarding mux
  always_comb
  begin : operand_c_fw_mux
    case (operand_c_fw_mux_sel)
      SEL_FW_EX:    operand_c_fw_id = regfile_alu_wdata_fw_i;
      SEL_FW_WB:    operand_c_fw_id = regfile_wdata_wb_i;
      SEL_REGFILE:  operand_c_fw_id = regfile_data_rc_id;
      default:       operand_c_fw_id = regfile_data_rc_id;
    endcase; // case (operand_c_fw_mux_sel)
  end


  ///////////////////////////////////////////////////////////////////////////
  //  ___                              _ _       _              ___ ____   //
  // |_ _|_ __ ___  _ __ ___   ___  __| (_) __ _| |_ ___  ___  |_ _|  _ \  //
  //  | || '_ ` _ \| '_ ` _ \ / _ \/ _` | |/ _` | __/ _ \/ __|  | || | | | //
  //  | || | | | | | | | | | |  __/ (_| | | (_| | ||  __/\__ \  | || |_| | //
  // |___|_| |_| |_|_| |_| |_|\___|\__,_|_|\__,_|\__\___||___/ |___|____/  //
  //                                                                       //
  ///////////////////////////////////////////////////////////////////////////

  always_comb
  begin
    unique case (bmask_a_mux)
      BMASK_A_ZERO: bmask_a_id = '0;
      BMASK_A_S3:   bmask_a_id = imm_s3_type[4:0];
      default:       bmask_a_id = '0;
    endcase
  end
  always_comb
  begin
    unique case (bmask_b_mux)
      BMASK_B_ZERO: bmask_b_id = '0;
      BMASK_B_ONE:  bmask_b_id = 5'd1;
      BMASK_B_S2:   bmask_b_id = imm_s2_type[4:0];
      BMASK_B_S3:   bmask_b_id = imm_s3_type[4:0];
      default:       bmask_b_id = '0;
    endcase
  end

  assign imm_vec_ext_id = imm_vu_type[1:0];


  always_comb
  begin
    unique case (mult_imm_mux)
      MIMM_ZERO: mult_imm_id = '0;
      MIMM_S3:   mult_imm_id = imm_s3_type[4:0];
      default:    mult_imm_id = '0;
    endcase
  end

  /////////////////////////////////////////////////////////
  //  ____  _____ ____ ___ ____ _____ _____ ____  ____   //
  // |  _ \| ____/ ___|_ _/ ___|_   _| ____|  _ \/ ___|  //
  // | |_) |  _|| |  _ | |\___ \ | | |  _| | |_) \___ \  //
  // |  _ <| |__| |_| || | ___) || | | |___|  _ < ___) | //
  // |_| \_\_____\____|___|____/ |_| |_____|_| \_\____/  //
  //                                                     //
  /////////////////////////////////////////////////////////
  riscv_register_file  registers_i
  (
    .clk          ( clk                ),
    .rst_n        ( rst_n              ),

    .test_en_i    ( test_en_i          ),

    // Read port a
    .raddr_a_i    ( regfile_addr_ra_id ),
    .rdata_a_o    ( regfile_data_ra_id ),

    // Read port b
    .raddr_b_i    ( regfile_addr_rb_id ),
    .rdata_b_o    ( regfile_data_rb_id ),

    // Read port c
    .raddr_c_i    ( (dbg_reg_rreq_i == 1'b0) ? regfile_addr_rc_id : dbg_reg_raddr_i ),
    .rdata_c_o    ( regfile_data_rc_id ),

    // Write port a
    .waddr_a_i    ( regfile_waddr_wb_i ),
    .wdata_a_i    ( regfile_wdata_wb_i ),
    .we_a_i       ( regfile_we_wb_i    ),

    // Write port b
    .waddr_b_i    ( (dbg_reg_wreq_i == 1'b0) ? regfile_alu_waddr_fw_i : dbg_reg_waddr_i  ),
    .wdata_b_i    ( (dbg_reg_wreq_i == 1'b0) ? regfile_alu_wdata_fw_i : dbg_reg_wdata_i ),
    .we_b_i       ( (dbg_reg_wreq_i == 1'b0) ? regfile_alu_we_fw_i    : 1'b1            )
  );

  assign dbg_reg_rdata_o = regfile_data_rc_id;


  ///////////////////////////////////////////////
  //  ____  _____ ____ ___  ____  _____ ____   //
  // |  _ \| ____/ ___/ _ \|  _ \| ____|  _ \  //
  // | | | |  _|| |  | | | | | | |  _| | |_) | //
  // | |_| | |__| |__| |_| | |_| | |___|  _ <  //
  // |____/|_____\____\___/|____/|_____|_| \_\ //
  //                                           //
  ///////////////////////////////////////////////

  riscv_decoder decoder_i
  (
    // controller related signals
    .deassert_we_i                   ( deassert_we               ),
    .data_misaligned_i               ( data_misaligned_i         ),
    .mult_multicycle_i               ( mult_multicycle_i         ),

    .illegal_insn_o                  ( illegal_insn_dec          ),
    .ebrk_insn_o                     ( ebrk_insn                 ),
    .eret_insn_o                     ( eret_insn_dec             ),
    .ecall_insn_o                    ( ecall_insn_dec            ),
    .pipe_flush_o                    ( pipe_flush_dec            ),

    .rega_used_o                     ( rega_used_dec             ),
    .regb_used_o                     ( regb_used_dec             ),
    .regc_used_o                     ( regc_used_dec             ),

    .bmask_needed_o                  ( bmask_needed_dec          ),
    .bmask_a_mux_o                   ( bmask_a_mux               ),
    .bmask_b_mux_o                   ( bmask_b_mux               ),

    // from IF/ID pipeline
    .instr_rdata_i                   ( instr                     ),
    .illegal_c_insn_i                ( illegal_c_insn_i          ),

    // ALU signals
    .alu_operator_o                  ( alu_operator              ),
    .alu_op_a_mux_sel_o              ( alu_op_a_mux_sel          ),
    .alu_op_b_mux_sel_o              ( alu_op_b_mux_sel          ),
    .alu_op_c_mux_sel_o              ( alu_op_c_mux_sel          ),
    .alu_vec_mode_o                  ( alu_vec_mode              ),
    .scalar_replication_o            ( scalar_replication        ),
    .imm_a_mux_sel_o                 ( imm_a_mux_sel             ),
    .imm_b_mux_sel_o                 ( imm_b_mux_sel             ),
    .regc_mux_o                      ( regc_mux                  ),

    // MUL signals
    .mult_operator_o                 ( mult_operator             ),
    .mult_int_en_o                   ( mult_int_en               ),
    .mult_sel_subword_o              ( mult_sel_subword          ),
    .mult_signed_mode_o              ( mult_signed_mode          ),
    .mult_imm_mux_o                  ( mult_imm_mux              ),
    .mult_dot_en_o                   ( mult_dot_en               ),
    .mult_dot_signed_o               ( mult_dot_signed           ),

    // Register file control signals
    .regfile_mem_we_o                ( regfile_we_id             ),
    .regfile_alu_we_o                ( regfile_alu_we_id         ),
    .regfile_alu_waddr_sel_o         ( regfile_alu_waddr_mux_sel ),

    // CSR control signals
    .csr_access_o                    ( csr_access                ),
    .csr_op_o                        ( csr_op                    ),

    // Data bus interface
    .data_req_o                      ( data_req_id               ),
    .data_we_o                       ( data_we_id                ),
    .prepost_useincr_o               ( prepost_useincr           ),
    .data_type_o                     ( data_type_id              ),
    .data_sign_extension_o           ( data_sign_ext_id          ),
    .data_reg_offset_o               ( data_reg_offset_id        ),
    .data_load_event_o               ( data_load_event_id        ),

    // hwloop signals
    .hwloop_we_o                     ( hwloop_we_int             ),
    .hwloop_target_mux_sel_o         ( hwloop_target_mux_sel     ),
    .hwloop_start_mux_sel_o          ( hwloop_start_mux_sel      ),
    .hwloop_cnt_mux_sel_o            ( hwloop_cnt_mux_sel        ),

    // jump/branches
    .jump_in_dec_o                   ( jump_in_dec               ),
    .jump_in_id_o                    ( jump_in_id                ),
    .jump_target_mux_sel_o           ( jump_target_mux_sel       ),
    
    // HDC signal
    .start                           ( start                     )
  );

  ////////////////////////////////////////////////////////////////////
  //    ____ ___  _   _ _____ ____   ___  _     _     _____ ____    //
  //   / ___/ _ \| \ | |_   _|  _ \ / _ \| |   | |   | ____|  _ \   //
  //  | |  | | | |  \| | | | | |_) | | | | |   | |   |  _| | |_) |  //
  //  | |__| |_| | |\  | | | |  _ <| |_| | |___| |___| |___|  _ <   //
  //   \____\___/|_| \_| |_| |_| \_\\___/|_____|_____|_____|_| \_\  //
  //                                                                //
  ////////////////////////////////////////////////////////////////////

  riscv_controller controller_i
  (
    .clk                            ( clk                    ),
    .rst_n                          ( rst_n                  ),

    .fetch_enable_i                 ( fetch_enable_i         ),
    .ctrl_busy_o                    ( ctrl_busy_o            ),
    .is_decoding_o                  ( is_decoding_o          ),

    // decoder related signals
    .deassert_we_o                  ( deassert_we            ),
    .illegal_insn_i                 ( illegal_insn_dec       ),
    .eret_insn_i                    ( eret_insn_dec          ),
    .pipe_flush_i                   ( pipe_flush_dec         ),

    .rega_used_i                    ( rega_used_dec          ),
    .regb_used_i                    ( regb_used_dec          ),
    .regc_used_i                    ( regc_used_dec          ),

    // from IF/ID pipeline
    .instr_valid_i                  ( instr_valid_i          ),
    .instr_rdata_i                  ( instr                  ),

    // from prefetcher
    .instr_req_o                    ( instr_req_o            ),

    // to prefetcher
    .pc_set_o                       ( pc_set_o               ),
    .pc_mux_o                       ( pc_mux_o               ),

    // LSU
    .data_req_ex_i                  ( data_req_ex_o          ),
    .data_misaligned_i              ( data_misaligned_i      ),
    .data_load_event_i              ( data_load_event_ex_o   ),

    // ALU
    .mult_multicycle_i              ( mult_multicycle_i      ),

    // jump/branch control
    .branch_taken_ex_i              ( branch_taken_ex        ),
    .jump_in_id_i                   ( jump_in_id             ),
    .jump_in_dec_i                  ( jump_in_dec            ),

    // Exception Controller Signals
    .exc_req_i                      ( exc_req                ),
    .exc_ack_o                      ( exc_ack                ),

    .exc_save_if_o                  ( exc_save_if_o          ),
    .exc_save_id_o                  ( exc_save_id_o          ),
    .exc_restore_id_o               ( exc_restore_id_o       ),

    // Debug Unit Signals
    .dbg_req_i                      ( dbg_req_i              ),
    .dbg_ack_o                      ( dbg_ack_o              ),
    .dbg_stall_i                    ( dbg_stall_i            ),
    .dbg_jump_req_i                 ( dbg_jump_req_i         ),

    // Forwarding signals from regfile
    .regfile_waddr_ex_i             ( regfile_waddr_ex_o     ), // Write address for register file from ex-wb- pipeline registers
    .regfile_we_ex_i                ( regfile_we_ex_o        ),
    .regfile_waddr_wb_i             ( regfile_waddr_wb_i     ), // Write address for register file from ex-wb- pipeline registers
    .regfile_we_wb_i                ( regfile_we_wb_i        ),

    // regfile port 2
    .regfile_alu_waddr_fw_i         ( regfile_alu_waddr_fw_i ),
    .regfile_alu_we_fw_i            ( regfile_alu_we_fw_i    ),

    // Forwarding detection signals
    .reg_d_ex_is_reg_a_i            ( reg_d_ex_is_reg_a_id   ),
    .reg_d_ex_is_reg_b_i            ( reg_d_ex_is_reg_b_id   ),
    .reg_d_ex_is_reg_c_i            ( reg_d_ex_is_reg_c_id   ),
    .reg_d_wb_is_reg_a_i            ( reg_d_wb_is_reg_a_id   ),
    .reg_d_wb_is_reg_b_i            ( reg_d_wb_is_reg_b_id   ),
    .reg_d_wb_is_reg_c_i            ( reg_d_wb_is_reg_c_id   ),
    .reg_d_alu_is_reg_a_i           ( reg_d_alu_is_reg_a_id  ),
    .reg_d_alu_is_reg_b_i           ( reg_d_alu_is_reg_b_id  ),
    .reg_d_alu_is_reg_c_i           ( reg_d_alu_is_reg_c_id  ),

    // Forwarding signals
    .operand_a_fw_mux_sel_o         ( operand_a_fw_mux_sel   ),
    .operand_b_fw_mux_sel_o         ( operand_b_fw_mux_sel   ),
    .operand_c_fw_mux_sel_o         ( operand_c_fw_mux_sel   ),

    // Stall signals
    .halt_if_o                      ( halt_if_o              ),
    .halt_id_o                      ( halt_id                ),

    .misaligned_stall_o             ( misaligned_stall       ),
    .jr_stall_o                     ( jr_stall               ),
    .load_stall_o                   ( load_stall             ),

    .id_ready_i                     ( id_ready_o             ),

    .if_valid_i                     ( if_valid_i             ),
    .ex_valid_i                     ( ex_valid_i             ),
    .wb_valid_i                     ( wb_valid_i             ),

    // Performance Counters
    .perf_jump_o                    ( perf_jump_o            ),
    .perf_jr_stall_o                ( perf_jr_stall_o        ),
    .perf_ld_stall_o                ( perf_ld_stall_o        )
  );

  ///////////////////////////////////////////////////////////////////////
  //  _____               ____            _             _ _            //
  // | ____|_  _____     / ___|___  _ __ | |_ _ __ ___ | | | ___ _ __  //
  // |  _| \ \/ / __|   | |   / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__| //
  // | |___ >  < (__ _  | |__| (_) | | | | |_| | | (_) | | |  __/ |    //
  // |_____/_/\_\___(_)  \____\___/|_| |_|\__|_|  \___/|_|_|\___|_|    //
  //                                                                   //
  ///////////////////////////////////////////////////////////////////////

  riscv_exc_controller exc_controller_i
  (
    .clk                  ( clk              ),
    .rst_n                ( rst_n            ),

    // to controller
    .req_o                ( exc_req          ),
    .ack_i                ( exc_ack          ),

    .trap_o               ( dbg_trap_o       ),

    // to IF stage
    .pc_mux_o             ( exc_pc_mux_o     ),
    .vec_pc_mux_o         ( exc_vec_pc_mux_o ),

    // Interrupt signals
    .irq_i                ( irq_i            ),
    .irq_enable_i         ( irq_enable_i     ),

    .ebrk_insn_i          ( is_decoding_o & ebrk_insn        ),
    .illegal_insn_i       ( is_decoding_o & illegal_insn_dec ),
    .ecall_insn_i         ( is_decoding_o & ecall_insn_dec   ),
    .eret_insn_i          ( is_decoding_o & eret_insn_dec    ),

    .lsu_load_err_i       ( lsu_load_err_i   ),
    .lsu_store_err_i      ( lsu_store_err_i  ),

    .cause_o              ( exc_cause_o      ),
    .save_cause_o         ( save_exc_cause_o ),

    .dbg_settings_i       ( dbg_settings_i   )
  );


  //////////////////////////////////////////////////////////////////////////
  //          ____ ___  _   _ _____ ____   ___  _     _     _____ ____    //
  //         / ___/ _ \| \ | |_   _|  _ \ / _ \| |   | |   | ____|  _ \   //
  // HWLOOP-| |  | | | |  \| | | | | |_) | | | | |   | |   |  _| | |_) |  //
  //        | |__| |_| | |\  | | | |  _ <| |_| | |___| |___| |___|  _ <   //
  //         \____\___/|_| \_| |_| |_| \_\\___/|_____|_____|_____|_| \_\  //
  //                                                                      //
  //////////////////////////////////////////////////////////////////////////

  riscv_hwloop_regs
  #(
    .N_REGS ( N_HWLP )
  )
  hwloop_regs_i
  (
    .clk                   ( clk                       ),
    .rst_n                 ( rst_n                     ),

    // from ID
    .hwlp_start_data_i     ( hwloop_start              ),
    .hwlp_end_data_i       ( hwloop_end                ),
    .hwlp_cnt_data_i       ( hwloop_cnt                ),
    .hwlp_we_i             ( hwloop_we                 ),
    .hwlp_regid_i          ( hwloop_regid              ),

    // from controller
    .valid_i               ( hwloop_valid              ),

    // to hwloop controller
    .hwlp_start_addr_o     ( hwlp_start_o              ),
    .hwlp_end_addr_o       ( hwlp_end_o                ),
    .hwlp_counter_o        ( hwlp_cnt_o                ),

    // from hwloop controller
    .hwlp_dec_cnt_i        ( hwlp_dec_cnt_i            )
  );

  assign hwloop_valid = instr_valid_i & clear_instr_valid_o & is_hwlp_i;


  /////////////////////////////////////////////////////////////////////////////////
  //   ___ ____        _______  __  ____ ___ ____  _____ _     ___ _   _ _____   //
  //  |_ _|  _ \      | ____\ \/ / |  _ \_ _|  _ \| ____| |   |_ _| \ | | ____|  //
  //   | || | | |_____|  _|  \  /  | |_) | || |_) |  _| | |    | ||  \| |  _|    //
  //   | || |_| |_____| |___ /  \  |  __/| ||  __/| |___| |___ | || |\  | |___   //
  //  |___|____/      |_____/_/\_\ |_|  |___|_|   |_____|_____|___|_| \_|_____|  //
  //                                                                             //
  /////////////////////////////////////////////////////////////////////////////////

  always_ff @(posedge clk, negedge rst_n)
  begin : ID_EX_PIPE_REGISTERS
    if (rst_n == 1'b0)
    begin
      alu_operator_ex_o           <= ALU_SLTU;
      alu_operand_a_ex_o          <= '0;
      alu_operand_b_ex_o          <= '0;
      alu_operand_c_ex_o          <= '0;
      bmask_a_ex_o                <= '0;
      bmask_b_ex_o                <= '0;
      imm_vec_ext_ex_o            <= '0;
      alu_vec_mode_ex_o           <= '0;

      mult_operator_ex_o          <= '0;
      mult_operand_a_ex_o         <= '0;
      mult_operand_b_ex_o         <= '0;
      mult_operand_c_ex_o         <= '0;
      mult_en_ex_o                <= 1'b0;
      mult_sel_subword_ex_o       <= 1'b0;
      mult_signed_mode_ex_o       <= 2'b00;
      mult_imm_ex_o               <= '0;

      mult_dot_op_a_ex_o          <= '0;
      mult_dot_op_b_ex_o          <= '0;
      mult_dot_op_c_ex_o          <= '0;
      mult_dot_signed_ex_o        <= '0;

      regfile_waddr_ex_o          <= 5'b0;
      regfile_we_ex_o             <= 1'b0;

      regfile_alu_waddr_ex_o      <= 5'b0;
      regfile_alu_we_ex_o         <= 1'b0;
      prepost_useincr_ex_o        <= 1'b0;

      csr_access_ex_o             <= 1'b0;
      csr_op_ex_o                 <= CSR_OP_NONE;

      data_we_ex_o                <= 1'b0;
      data_type_ex_o              <= 2'b0;
      data_sign_ext_ex_o          <= 1'b0;
      data_reg_offset_ex_o        <= 2'b0;
      data_req_ex_o               <= 1'b0;
      data_load_event_ex_o        <= 1'b0;

      data_misaligned_ex_o        <= 1'b0;

      pc_ex_o                     <= '0;

      branch_in_ex_o              <= 1'b0;

    end
    else if (data_misaligned_i) begin
      // misaligned data access case
      if (ex_ready_i)
      begin // misaligned access case, only unstall alu operands

        // if we are using post increments, then we have to use the
        // original value of the register for the second memory access
        // => keep it stalled
        if (prepost_useincr_ex_o == 1'b1)
        begin
          alu_operand_a_ex_o        <= alu_operand_a;
        end

        alu_operand_b_ex_o          <= alu_operand_b;
        regfile_alu_we_ex_o         <= regfile_alu_we_id;
        prepost_useincr_ex_o        <= prepost_useincr;

        data_misaligned_ex_o        <= 1'b1;
      end
    end else if (mult_multicycle_i) begin
      mult_operand_c_ex_o <= alu_operand_c;
    end
    else begin
      // normal pipeline unstall case

      if (id_valid_o)
      begin // unstall the whole pipeline
        if (~mult_en)
        begin // only change those registers when we actually need to
          alu_operator_ex_o         <= alu_operator;
          alu_operand_a_ex_o        <= alu_operand_a;
          alu_operand_b_ex_o        <= alu_operand_b;
          alu_operand_c_ex_o        <= alu_operand_c;
          bmask_a_ex_o              <= bmask_a_id;
          bmask_b_ex_o              <= bmask_b_id;
          imm_vec_ext_ex_o          <= imm_vec_ext_id;
          alu_vec_mode_ex_o         <= alu_vec_mode;
        end

        mult_en_ex_o                <= mult_en;
        if (mult_int_en) begin  // when we are multiplying we don't need the ALU
          mult_operator_ex_o        <= mult_operator;
          mult_sel_subword_ex_o     <= mult_sel_subword;
          mult_signed_mode_ex_o     <= mult_signed_mode;
          mult_operand_a_ex_o       <= alu_operand_a;
          mult_operand_b_ex_o       <= alu_operand_b;
          mult_operand_c_ex_o       <= alu_operand_c;
          mult_imm_ex_o             <= mult_imm_id;
        end
        if (mult_dot_en) begin
          mult_operator_ex_o        <= mult_operator;
          mult_dot_signed_ex_o      <= mult_dot_signed;
          mult_dot_op_a_ex_o        <= alu_operand_a;
          mult_dot_op_b_ex_o        <= alu_operand_b;
          mult_dot_op_c_ex_o        <= alu_operand_c;
        end

        regfile_we_ex_o             <= regfile_we_id;
        if (regfile_we_id) begin
          regfile_waddr_ex_o        <= regfile_waddr_id;
        end

        regfile_alu_we_ex_o         <= regfile_alu_we_id;
        if (regfile_alu_we_id) begin
          regfile_alu_waddr_ex_o    <= regfile_alu_waddr_id;
        end

        prepost_useincr_ex_o        <= prepost_useincr;

        csr_access_ex_o             <= csr_access;
        csr_op_ex_o                 <= csr_op;

        data_req_ex_o               <= data_req_id;
        if (data_req_id)
        begin // only needed for LSU when there is an active request
          data_we_ex_o              <= data_we_id;
          data_type_ex_o            <= data_type_id;
          data_sign_ext_ex_o        <= data_sign_ext_id;
          data_reg_offset_ex_o      <= data_reg_offset_id;
          data_load_event_ex_o      <= data_load_event_id;;
        end else begin
          data_load_event_ex_o      <= 1'b0;
        end

        data_misaligned_ex_o        <= 1'b0;

        if ((jump_in_id == BRANCH_COND) || data_load_event_id) begin
          pc_ex_o                   <= pc_id_i;
        end

        branch_in_ex_o              <= jump_in_id == BRANCH_COND;
      end else if(ex_ready_i) begin
        // EX stage is ready but we don't have a new instruction for it,
        // so we set all write enables to 0, but unstall the pipe

        regfile_we_ex_o             <= 1'b0;

        regfile_alu_we_ex_o         <= 1'b0;

        csr_op_ex_o                 <= CSR_OP_NONE;

        data_req_ex_o               <= 1'b0;
        data_load_event_ex_o        <= 1'b0;

        data_misaligned_ex_o        <= 1'b0;

        branch_in_ex_o              <= 1'b0;
      end
    end
  end


  // stall control
  assign id_ready_o = ((~misaligned_stall) & (~jr_stall) & (~load_stall) & ex_ready_i);
  assign id_valid_o = (~halt_id) & id_ready_o;


`ifndef verilator
  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------

  // make sure that branch decision is valid when jumping
  assert property (
    @(posedge clk) (branch_in_ex_o) |-> (branch_decision_i !== 1'bx) ) else $display("Branch decision is X");

  // the instruction delivered to the ID stage should always be valid
  assert property (
    @(posedge clk) (instr_valid_i & (~illegal_c_insn_i)) |-> (!$isunknown(instr_rdata_i)) ) else $display("Instruction is valid, but has at least one X");
`endif

endmodule

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Francesco Conti - f.conti@unibo.it                         //
//                                                                            //
// Design Name:    RISC-V register file                                       //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Register file with 31x 32 bit wide registers. Register 0   //
//                 is fixed to 0. This register file is based on flip-flops.  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module riscv_register_file
#(
    parameter ADDR_WIDTH    = 5,
    parameter DATA_WIDTH    = 32
)
(
    // Clock and Reset
    input  logic         clk,
    input  logic         rst_n,

    input  logic                   test_en_i,

    //Read port R1
    input  logic [ADDR_WIDTH-1:0]  raddr_a_i,
    output logic [DATA_WIDTH-1:0]  rdata_a_o,

    //Read port R2
    input  logic [ADDR_WIDTH-1:0]  raddr_b_i,
    output logic [DATA_WIDTH-1:0]  rdata_b_o,

    //Read port R3
    input  logic [ADDR_WIDTH-1:0]  raddr_c_i,
    output logic [DATA_WIDTH-1:0]  rdata_c_o,

    // Write port W1
    input logic [ADDR_WIDTH-1:0]   waddr_a_i,
    input logic [DATA_WIDTH-1:0]   wdata_a_i,
    input logic                    we_a_i,

    // Write port W2
    input logic [ADDR_WIDTH-1:0]   waddr_b_i,
    input logic [DATA_WIDTH-1:0]   wdata_b_i,
    input logic                    we_b_i
);

  localparam    NUM_WORDS = 2**ADDR_WIDTH;

  logic [NUM_WORDS-1:0][DATA_WIDTH-1:0] rf_reg;
  logic [NUM_WORDS-1:0]                 we_a_dec;
  logic [NUM_WORDS-1:0]                 we_b_dec;

  always_comb
  begin : we_a_decoder
    for (int i = 0; i < NUM_WORDS; i++) begin
      if (waddr_a_i == i)
        we_a_dec[i] = we_a_i;
      else
        we_a_dec[i] = 1'b0;
    end
  end

  always_comb
  begin : we_b_decoder
    for (int i=0; i<NUM_WORDS; i++) begin
      if (waddr_b_i == i)
        we_b_dec[i] = we_b_i;
      else
        we_b_dec[i] = 1'b0;
    end
  end

  genvar i;
  generate

    // loop from 1 to NUM_WORDS-1 as R0 is nil
    for (i = 1; i < NUM_WORDS; i++)
    begin : rf_gen

      always_ff @(posedge clk, negedge rst_n)
      begin : register_write_behavioral
        if (rst_n==1'b0) begin
          rf_reg[i] <= 'b0;
        end else begin
          if(we_b_dec[i] == 1'b1)
            rf_reg[i] <= wdata_b_i;
          else if(we_a_dec[i] == 1'b1)
            rf_reg[i] <= wdata_a_i;
        end
      end

    end

`ifndef verilator
    // R0 is nil
    assign rf_reg[0] = '0;
`endif

  endgenerate

`ifdef verilator
   // R0 is nil
   always_ff @(posedge clk, negedge rst_n)
     rf_reg[0] <= '0;
`endif

  assign rdata_a_o = rf_reg[raddr_a_i];
  assign rdata_b_o = rf_reg[raddr_b_i];
  assign rdata_c_o = rf_reg[raddr_c_i];

endmodule

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the âLicenseâ?); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an âAS ISâ? BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer        Andreas Traber - atraber@iis.ee.ethz.ch                    //
//                                                                            //
// Additional contributions by:                                               //
//                 Matthias Baer - baermatt@student.ethz.ch                   //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
// Design Name:    Decoder                                                    //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Decoder                                                    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

import riscv_defines::*;

module riscv_decoder
(
  // singals running to/from controller
  input  logic        deassert_we_i,           // deassert we, we are stalled or not active
  input  logic        data_misaligned_i,       // misaligned data load/store in progress
  input  logic        mult_multicycle_i,       // multiplier taking multiple cycles, using op c as storage

  output logic        illegal_insn_o,          // illegal instruction encountered
  output logic        ebrk_insn_o,             // trap instruction encountered
  output logic        eret_insn_o,             // return from exception instruction encountered. Note ERET is now MRET.
  output logic        ecall_insn_o,            // environment call (syscall) instruction encountered
  output logic        pipe_flush_o,            // pipeline flush is requested

  output logic        rega_used_o,             // rs1 is used by current instruction
  output logic        regb_used_o,             // rs2 is used by current instruction
  output logic        regc_used_o,             // rs3 is used by current instruction

  output logic        bmask_needed_o,          // registers for bit manipulation mask is needed
  output logic [ 0:0] bmask_a_mux_o,           // bit manipulation mask a mux
  output logic [ 1:0] bmask_b_mux_o,           // bit manipulation mask a mux

  // from IF/ID pipeline
  input  logic [31:0] instr_rdata_i,           // instruction read from instr memory/cache
  input  logic        illegal_c_insn_i,        // compressed instruction decode failed

  // ALU signals
  output logic [ALU_OP_WIDTH-1:0] alu_operator_o, // ALU operation selection
  output logic [1:0]  alu_op_a_mux_sel_o,      // operand a selection: reg value, PC, immediate or zero
  output logic [1:0]  alu_op_b_mux_sel_o,      // operand b selection: reg value or immediate
  output logic [1:0]  alu_op_c_mux_sel_o,      // operand c selection: reg value or jump target
  output logic [1:0]  alu_vec_mode_o,          // selects between 32 bit, 16 bit and 8 bit vectorial modes
  output logic        scalar_replication_o,    // scalar replication enable
  output logic [0:0]  imm_a_mux_sel_o,         // immediate selection for operand a
  output logic [3:0]  imm_b_mux_sel_o,         // immediate selection for operand b
  output logic [1:0]  regc_mux_o,              // register c selection: S3, RD or 0

  // MUL related control signals
  output logic [2:0]  mult_operator_o,         // Multiplication operation selection
  output logic        mult_int_en_o,           // perform integer multiplication
  output logic        mult_dot_en_o,           // perform dot multiplication
  output logic [0:0]  mult_imm_mux_o,          // Multiplication immediate mux selector
  output logic        mult_sel_subword_o,      // Select subwords for 16x16 bit of multiplier
  output logic [1:0]  mult_signed_mode_o,      // Multiplication in signed mode
  output logic [1:0]  mult_dot_signed_o,       // Dot product in signed mode

  // register file related signals
  output logic        regfile_mem_we_o,        // write enable for regfile
  output logic        regfile_alu_we_o,        // write enable for 2nd regfile port
  output logic        regfile_alu_waddr_sel_o, // Select register write address for ALU/MUL operations

  // CSR manipulation
  output logic        csr_access_o,            // access to CSR
  output logic [1:0]  csr_op_o,                // operation to perform on CSR

  // LD/ST unit signals
  output logic        data_req_o,              // start transaction to data memory
  output logic        data_we_o,               // data memory write enable
  output logic        prepost_useincr_o,       // when not active bypass the alu result for address calculation
  output logic [1:0]  data_type_o,             // data type on data memory: byte, half word or word
  output logic        data_sign_extension_o,   // sign extension on read data from data memory
  output logic [1:0]  data_reg_offset_o,       // offset in byte inside register for stores
  output logic        data_load_event_o,       // data request is in the special event range

  // hwloop signals
  output logic [2:0]  hwloop_we_o,             // write enable for hwloop regs
  output logic        hwloop_target_mux_sel_o, // selects immediate for hwloop target
  output logic        hwloop_start_mux_sel_o,  // selects hwloop start address input
  output logic        hwloop_cnt_mux_sel_o,    // selects hwloop counter input

  // jump/branches
  output logic [1:0]  jump_in_dec_o,           // jump_in_id without deassert
  output logic [1:0]  jump_in_id_o,            // jump is being calculated in ALU
  output logic [1:0]  jump_target_mux_sel_o,   // jump target selection
  
  //HD computing control
  output logic        start                    //enable contol to start the HD computing co processor
);

  // write enable/request control
  logic       regfile_mem_we;
  logic       regfile_alu_we;
  logic       data_req;
  logic [2:0] hwloop_we;

  logic       ebrk_insn;
  logic       mret_insn;
  logic       pipe_flush;

  logic [1:0] jump_in_id;

  logic [1:0] csr_op;


  /////////////////////////////////////////////
  //   ____                     _            //
  //  |  _ \  ___  ___ ___   __| | ___ _ __  //
  //  | | | |/ _ \/ __/ _ \ / _` |/ _ \ '__| //
  //  | |_| |  __/ (_| (_) | (_| |  __/ |    //
  //  |____/ \___|\___\___/ \__,_|\___|_|    //
  //                                         //
  /////////////////////////////////////////////

  always_comb
  begin
    jump_in_id                  = BRANCH_NONE;
    jump_target_mux_sel_o       = JT_JAL;

    alu_operator_o              = ALU_SLTU;
    alu_op_a_mux_sel_o          = OP_A_REGA_OR_FWD;
    alu_op_b_mux_sel_o          = OP_B_REGB_OR_FWD;
    alu_op_c_mux_sel_o          = OP_C_REGC_OR_FWD;
    alu_vec_mode_o              = VEC_MODE32;
    scalar_replication_o        = 1'b0;
    regc_mux_o                  = REGC_ZERO;
    imm_a_mux_sel_o             = IMMA_ZERO;
    imm_b_mux_sel_o             = IMMB_I;

    mult_operator_o             = MUL_I;
    mult_int_en_o               = 1'b0;
    mult_dot_en_o               = 1'b0;
    mult_imm_mux_o              = MIMM_ZERO;
    mult_signed_mode_o          = 2'b00;
    mult_sel_subword_o          = 1'b0;
    mult_dot_signed_o           = 2'b00;

    regfile_mem_we              = 1'b0;
    regfile_alu_we              = 1'b0;
    regfile_alu_waddr_sel_o     = 1'b1;

    prepost_useincr_o           = 1'b1;

    hwloop_we                   = 3'b0;
    hwloop_target_mux_sel_o     = 1'b0;
    hwloop_start_mux_sel_o      = 1'b0;
    hwloop_cnt_mux_sel_o        = 1'b0;

    csr_access_o                = 1'b0;
    csr_op                      = CSR_OP_NONE;

    data_we_o                   = 1'b0;
    data_type_o                 = 2'b00;
    data_sign_extension_o       = 1'b0;
    data_reg_offset_o           = 2'b00;
    data_req                    = 1'b0;
    data_load_event_o           = 1'b0;

    illegal_insn_o              = 1'b0;
    ebrk_insn                   = 1'b0;
    mret_insn                   = 1'b0;
    ecall_insn_o                = 1'b0;
    pipe_flush                  = 1'b0;

    rega_used_o                 = 1'b0;
    regb_used_o                 = 1'b0;
    regc_used_o                 = 1'b0;
    bmask_needed_o              = 1'b1; // TODO: only use when necessary
    bmask_a_mux_o               = BMASK_A_ZERO;
    bmask_b_mux_o               = BMASK_B_ZERO;

    start                       = 1'b0;
    unique case (instr_rdata_i[6:0])
    
    //////////////////////////////////////
    //// HD Computing
    ///////////////////////////////////////
    
      OPCODE_HDC: begin
        start     = 1'b1;
      end
      
      /////////////////////////////////
      
      7'h0f: begin //for now fence is a no-op
	 illegal_insn_o = 1'b0;

      end
      //////////////////////////////////////
      //      _ _   _ __  __ ____  ____   //
      //     | | | | |  \/  |  _ \/ ___|  //
      //  _  | | | | | |\/| | |_) \___ \  //
      // | |_| | |_| | |  | |  __/ ___) | //
      //  \___/ \___/|_|  |_|_|   |____/  //
      //                                  //
      //////////////////////////////////////

      OPCODE_JAL: begin   // Jump and Link
        jump_target_mux_sel_o = JT_JAL;
        jump_in_id            = BRANCH_JAL;
        // Calculate and store PC+4
        alu_op_a_mux_sel_o  = OP_A_CURRPC;
        alu_op_b_mux_sel_o  = OP_B_IMM;
        imm_b_mux_sel_o     = IMMB_PCINCR;
        alu_operator_o      = ALU_ADD;
        regfile_alu_we      = 1'b1;
        // Calculate jump target (= PC + UJ imm)
      end

      OPCODE_JALR: begin  // Jump and Link Register
        jump_target_mux_sel_o = JT_JALR;
        jump_in_id            = BRANCH_JALR;
        // Calculate and store PC+4
        alu_op_a_mux_sel_o  = OP_A_CURRPC;
        alu_op_b_mux_sel_o  = OP_B_IMM;
        imm_b_mux_sel_o     = IMMB_PCINCR;
        alu_operator_o      = ALU_ADD;
        regfile_alu_we      = 1'b1;
        // Calculate jump target (= RS1 + I imm)
        rega_used_o         = 1'b1;

        if (instr_rdata_i[14:12] != 3'b0) begin
          jump_in_id       = BRANCH_NONE;
          regfile_alu_we   = 1'b0;
          illegal_insn_o   = 1'b1;
        end
      end

      OPCODE_BRANCH: begin // Branch
        jump_target_mux_sel_o = JT_COND;
        jump_in_id            = BRANCH_COND;
        alu_op_c_mux_sel_o    = OP_C_JT;
        rega_used_o           = 1'b1;
        regb_used_o           = 1'b1;

        unique case (instr_rdata_i[14:12])
          3'b000: alu_operator_o = ALU_EQ;
          3'b001: alu_operator_o = ALU_NE;
          3'b100: alu_operator_o = ALU_LTS;
          3'b101: alu_operator_o = ALU_GES;
          3'b110: alu_operator_o = ALU_LTU;
          3'b111: alu_operator_o = ALU_GEU;
          3'b010: begin
            alu_operator_o      = ALU_EQ;
            regb_used_o         = 1'b0;
            alu_op_b_mux_sel_o  = OP_B_IMM;
            imm_b_mux_sel_o     = IMMB_BI;
          end
          3'b011: begin
            alu_operator_o      = ALU_NE;
            regb_used_o         = 1'b0;
            alu_op_b_mux_sel_o  = OP_B_IMM;
            imm_b_mux_sel_o     = IMMB_BI;
          end
          default: begin
            illegal_insn_o = 1'b1;
          end
        endcase
      end


      //////////////////////////////////
      //  _     ____    ______ _____  //
      // | |   |  _ \  / / ___|_   _| //
      // | |   | | | |/ /\___ \ | |   //
      // | |___| |_| / /  ___) || |   //
      // |_____|____/_/  |____/ |_|   //
      //                              //
      //////////////////////////////////

      OPCODE_STORE,
      OPCODE_STORE_POST: begin
        data_req       = 1'b1;
        data_we_o      = 1'b1;
        rega_used_o    = 1'b1;
        regb_used_o    = 1'b1;
        alu_operator_o = ALU_ADD;

        // pass write data through ALU operand c
        alu_op_c_mux_sel_o = OP_C_REGB_OR_FWD;

        // post-increment setup
        if (instr_rdata_i[6:0] == OPCODE_STORE_POST) begin
          prepost_useincr_o       = 1'b0;
          regfile_alu_waddr_sel_o = 1'b0;
          regfile_alu_we          = 1'b1;
        end

        if (instr_rdata_i[14] == 1'b0) begin
          // offset from immediate
          imm_b_mux_sel_o     = IMMB_S;
          alu_op_b_mux_sel_o  = OP_B_IMM;
        end else begin
          // offset from register
          regc_used_o        = 1'b1;
          alu_op_b_mux_sel_o = OP_B_REGC_OR_FWD;
          regc_mux_o         = REGC_RD;
        end

        // store size
        unique case (instr_rdata_i[13:12])
          2'b00: data_type_o = 2'b10; // SB
          2'b01: data_type_o = 2'b01; // SH
          2'b10: data_type_o = 2'b00; // SW
          default: begin
            data_req       = 1'b0;
            data_we_o      = 1'b0;
            illegal_insn_o = 1'b1;
          end
        endcase
      end

      OPCODE_LOAD,
      OPCODE_LOAD_POST: begin
        data_req        = 1'b1;
        regfile_mem_we  = 1'b1;
        rega_used_o     = 1'b1;
        data_type_o     = 2'b00;

        // offset from immediate
        alu_operator_o      = ALU_ADD;
        alu_op_b_mux_sel_o  = OP_B_IMM;
        imm_b_mux_sel_o     = IMMB_I;

        // post-increment setup
        if (instr_rdata_i[6:0] == OPCODE_LOAD_POST) begin
          prepost_useincr_o       = 1'b0;
          regfile_alu_waddr_sel_o = 1'b0;
          regfile_alu_we          = 1'b1;
        end

        // sign/zero extension
        data_sign_extension_o = ~instr_rdata_i[14];

        // load size
        unique case (instr_rdata_i[13:12])
          2'b00:   data_type_o = 2'b10; // LB
          2'b01:   data_type_o = 2'b01; // LH
          2'b10:   data_type_o = 2'b00; // LW
          default: data_type_o = 2'b00; // illegal or reg-reg
        endcase

        // reg-reg load (different encoding)
        if (instr_rdata_i[14:12] == 3'b111) begin
          // offset from RS2
          regb_used_o        = 1'b1;
          alu_op_b_mux_sel_o = OP_B_REGB_OR_FWD;

          // sign/zero extension
          data_sign_extension_o = ~instr_rdata_i[30];

          // load size
          unique case (instr_rdata_i[31:25])
            7'b0000_000,
            7'b0100_000: data_type_o = 2'b10; // LB, LBU
            7'b0001_000,
            7'b0101_000: data_type_o = 2'b01; // LH, LHU
            7'b0010_000: data_type_o = 2'b00; // LW
            default: begin
              illegal_insn_o = 1'b1;
            end
          endcase
        end

        // special p.elw (event load)
        if (instr_rdata_i[14:12] == 3'b110)
          data_load_event_o = 1'b1;

        if (instr_rdata_i[14:12] == 3'b011) begin
          // LD -> RV64 only
          illegal_insn_o = 1'b1;
        end
      end


      //////////////////////////
      //     _    _    _   _  //
      //    / \  | |  | | | | //
      //   / _ \ | |  | | | | //
      //  / ___ \| |__| |_| | //
      // /_/   \_\_____\___/  //
      //                      //
      //////////////////////////

      OPCODE_LUI: begin  // Load Upper Immediate
        alu_op_a_mux_sel_o  = OP_A_IMM;
        alu_op_b_mux_sel_o  = OP_B_IMM;
        imm_a_mux_sel_o     = IMMA_ZERO;
        imm_b_mux_sel_o     = IMMB_U;
        alu_operator_o      = ALU_ADD;
        regfile_alu_we      = 1'b1;
      end

      OPCODE_AUIPC: begin  // Add Upper Immediate to PC
        alu_op_a_mux_sel_o  = OP_A_CURRPC;
        alu_op_b_mux_sel_o  = OP_B_IMM;
        imm_b_mux_sel_o     = IMMB_U;
        alu_operator_o      = ALU_ADD;
        regfile_alu_we      = 1'b1;
      end

      OPCODE_OPIMM: begin // Register-Immediate ALU Operations
        alu_op_b_mux_sel_o  = OP_B_IMM;
        imm_b_mux_sel_o     = IMMB_I;
        regfile_alu_we      = 1'b1;
        rega_used_o         = 1'b1;

        unique case (instr_rdata_i[14:12])
          3'b000: alu_operator_o = ALU_ADD;  // Add Immediate
          3'b010: alu_operator_o = ALU_SLTS; // Set to one if Lower Than Immediate
          3'b011: alu_operator_o = ALU_SLTU; // Set to one if Lower Than Immediate Unsigned
          3'b100: alu_operator_o = ALU_XOR;  // Exclusive Or with Immediate
          3'b110: alu_operator_o = ALU_OR;   // Or with Immediate
          3'b111: alu_operator_o = ALU_AND;  // And with Immediate

          3'b001: begin
            alu_operator_o = ALU_SLL;  // Shift Left Logical by Immediate
            if (instr_rdata_i[31:25] != 7'b0)
              illegal_insn_o = 1'b1;
          end

          3'b101: begin
            if (instr_rdata_i[31:25] == 7'b0)
              alu_operator_o = ALU_SRL;  // Shift Right Logical by Immediate
            else if (instr_rdata_i[31:25] == 7'b010_0000)
              alu_operator_o = ALU_SRA;  // Shift Right Arithmetically by Immediate
            else
              illegal_insn_o = 1'b1;
          end

          default: illegal_insn_o = 1'b1;
        endcase
      end

      OPCODE_OP: begin  // Register-Register ALU operation
        regfile_alu_we = 1'b1;
        rega_used_o    = 1'b1;

        if (instr_rdata_i[31]) begin
          // bit-manipulation instructions
          alu_op_b_mux_sel_o  = OP_B_IMM;
          bmask_needed_o      = 1'b1;
          bmask_a_mux_o       = BMASK_A_S3;
          bmask_b_mux_o       = BMASK_B_S2;

          unique case (instr_rdata_i[14:12])
            3'b000: begin
              alu_operator_o  = ALU_BEXT;
              imm_b_mux_sel_o = IMMB_S2;
              bmask_b_mux_o   = BMASK_B_ZERO;
            end
            3'b001: begin
              alu_operator_o  = ALU_BEXTU;
              imm_b_mux_sel_o = IMMB_S2;
              bmask_b_mux_o   = BMASK_B_ZERO;
            end

            3'b010: begin
              alu_operator_o      = ALU_BINS;
              imm_b_mux_sel_o     = IMMB_S2;
              regc_used_o         = 1'b1;
              regc_mux_o          = REGC_RD;
            end

            3'b011: begin alu_operator_o = ALU_BCLR; end
            3'b100: begin alu_operator_o = ALU_BSET; end

            default: illegal_insn_o = 1'b1;
          endcase
        end
        else
        begin // non bit-manipulation instructions

          if (~instr_rdata_i[28])
            regb_used_o = 1'b1;

          unique case ({instr_rdata_i[30:25], instr_rdata_i[14:12]})
            // RV32I ALU operations
            {6'b00_0000, 3'b000}: alu_operator_o = ALU_ADD;   // Add
            {6'b10_0000, 3'b000}: alu_operator_o = ALU_SUB;   // Sub
            {6'b00_0000, 3'b010}: alu_operator_o = ALU_SLTS;  // Set Lower Than
            {6'b00_0000, 3'b011}: alu_operator_o = ALU_SLTU;  // Set Lower Than Unsigned
            {6'b00_0000, 3'b100}: alu_operator_o = ALU_XOR;   // Xor
            {6'b00_0000, 3'b110}: alu_operator_o = ALU_OR;    // Or
            {6'b00_0000, 3'b111}: alu_operator_o = ALU_AND;   // And
            {6'b00_0000, 3'b001}: alu_operator_o = ALU_SLL;   // Shift Left Logical
            {6'b00_0000, 3'b101}: alu_operator_o = ALU_SRL;   // Shift Right Logical
            {6'b10_0000, 3'b101}: alu_operator_o = ALU_SRA;   // Shift Right Arithmetic

            // supported RV32M instructions
            {6'b00_0001, 3'b000}: begin // mul
              mult_int_en_o   = 1'b1;
              mult_operator_o = MUL_MAC32;
              regc_mux_o      = REGC_ZERO;
            end
            {6'b00_0001, 3'b001}: begin // mulh
              regc_used_o        = 1'b1;
              regc_mux_o         = REGC_ZERO;
              mult_signed_mode_o = 2'b11;
              mult_int_en_o      = 1'b1;
              mult_operator_o    = MUL_H;
            end
            {6'b00_0001, 3'b010}: begin // mulhsu
              regc_used_o        = 1'b1;
              regc_mux_o         = REGC_ZERO;
              mult_signed_mode_o = 2'b01;
              mult_int_en_o      = 1'b1;
              mult_operator_o    = MUL_H;
            end
            {6'b00_0001, 3'b011}: begin // mulhu
              regc_used_o        = 1'b1;
              regc_mux_o         = REGC_ZERO;
              mult_signed_mode_o = 2'b00;
              mult_int_en_o      = 1'b1;
              mult_operator_o    = MUL_H;
            end
            {6'b00_0001, 3'b100}: begin // div
              alu_op_a_mux_sel_o = OP_A_REGB_OR_FWD;
              alu_op_b_mux_sel_o = OP_B_REGC_OR_FWD;
              regc_mux_o         = REGC_S1;
              regc_used_o        = 1'b1;
              regb_used_o        = 1'b1;
              rega_used_o        = 1'b0;
              alu_operator_o     = ALU_DIV;
            end
            {6'b00_0001, 3'b101}: begin // divu
              alu_op_a_mux_sel_o = OP_A_REGB_OR_FWD;
              alu_op_b_mux_sel_o = OP_B_REGC_OR_FWD;
              regc_mux_o         = REGC_S1;
              regc_used_o        = 1'b1;
              regb_used_o        = 1'b1;
              rega_used_o        = 1'b0;
              alu_operator_o = ALU_DIVU;
            end
            {6'b00_0001, 3'b110}: begin // rem
              alu_op_a_mux_sel_o = OP_A_REGB_OR_FWD;
              alu_op_b_mux_sel_o = OP_B_REGC_OR_FWD;
              regc_mux_o         = REGC_S1;
              regc_used_o        = 1'b1;
              regb_used_o        = 1'b1;
              rega_used_o        = 1'b0;
              alu_operator_o = ALU_REM;
            end
            {6'b00_0001, 3'b111}: begin // remu
              alu_op_a_mux_sel_o = OP_A_REGB_OR_FWD;
              alu_op_b_mux_sel_o = OP_B_REGC_OR_FWD;
              regc_mux_o         = REGC_S1;
              regc_used_o        = 1'b1;
              regb_used_o        = 1'b1;
              rega_used_o        = 1'b0;
              alu_operator_o = ALU_REMU;
            end

            // PULP specific instructions
            {6'b10_0001, 3'b000}: begin // p.mac
              regc_used_o     = 1'b1;
              regc_mux_o      = REGC_RD;
              mult_int_en_o   = 1'b1;
              mult_operator_o = MUL_MAC32;
            end
            {6'b10_0001, 3'b001}: begin // p.msu
              regc_used_o     = 1'b1;
              regc_mux_o      = REGC_RD;
              mult_int_en_o   = 1'b1;
              mult_operator_o = MUL_MSU32;
            end

            {6'b00_0010, 3'b010}: alu_operator_o = ALU_SLETS; // Set Lower Equal Than
            {6'b00_0010, 3'b011}: alu_operator_o = ALU_SLETU; // Set Lower Equal Than Unsigned
            {6'b00_0010, 3'b100}: alu_operator_o = ALU_MIN;   // Min
            {6'b00_0010, 3'b101}: alu_operator_o = ALU_MINU;  // Min Unsigned
            {6'b00_0010, 3'b110}: alu_operator_o = ALU_MAX;   // Max
            {6'b00_0010, 3'b111}: alu_operator_o = ALU_MAXU;  // Max Unsigned

            {6'b00_0100, 3'b101}: alu_operator_o = ALU_ROR;   // Rotate Right

            // PULP specific instructions using only one source register
            {6'b00_1000, 3'b000}: alu_operator_o = ALU_FF1;   // Find First 1
            {6'b00_1000, 3'b001}: alu_operator_o = ALU_FL1;   // Find Last 1
            {6'b00_1000, 3'b010}: alu_operator_o = ALU_CLB;   // Count Leading Bits
            {6'b00_1000, 3'b011}: alu_operator_o = ALU_CNT;   // Count set bits (popcount)
            {6'b00_1000, 3'b100}: begin alu_operator_o = ALU_EXTS; alu_vec_mode_o = VEC_MODE16; end // Sign-extend Half-word
            {6'b00_1000, 3'b101}: begin alu_operator_o = ALU_EXT;  alu_vec_mode_o = VEC_MODE16; end // Zero-extend Half-word
            {6'b00_1000, 3'b110}: begin alu_operator_o = ALU_EXTS; alu_vec_mode_o = VEC_MODE8;  end // Sign-extend Byte
            {6'b00_1000, 3'b111}: begin alu_operator_o = ALU_EXT;  alu_vec_mode_o = VEC_MODE8;  end // Zero-extend Byte

            {6'b00_0010, 3'b000}: alu_operator_o = ALU_ABS;   // p.abs

            {6'b00_1010, 3'b001}: begin // p.clip
              alu_operator_o     = ALU_CLIP;
              alu_op_b_mux_sel_o = OP_A_IMM;
              imm_b_mux_sel_o    = IMMB_CLIP;
            end

            {6'b00_1010, 3'b010}: begin // p.clipu
              alu_operator_o     = ALU_CLIPU;
              alu_op_b_mux_sel_o = OP_A_IMM;
              imm_b_mux_sel_o    = IMMB_CLIP;
            end

            default: begin
              illegal_insn_o = 1'b1;
            end
          endcase
        end
      end

      OPCODE_PULP_OP: begin  // PULP specific ALU instructions with three source operands
        regfile_alu_we = 1'b1;
        rega_used_o    = 1'b1;
        regb_used_o    = 1'b1;

        case (instr_rdata_i[13:12])
          2'b00: begin // multiply with subword selection
            mult_sel_subword_o = instr_rdata_i[30];
            mult_signed_mode_o = {2{instr_rdata_i[31]}};

            mult_imm_mux_o = MIMM_S3;
            regc_mux_o     = REGC_ZERO;
            mult_int_en_o  = 1'b1;

            if (instr_rdata_i[14])
              mult_operator_o = MUL_IR;
            else
              mult_operator_o = MUL_I;
          end

          2'b01: begin // MAC with subword selection
            mult_sel_subword_o = instr_rdata_i[30];
            mult_signed_mode_o = {2{instr_rdata_i[31]}};

            regc_used_o     = 1'b1;
            regc_mux_o      = REGC_RD;
            mult_imm_mux_o  = MIMM_S3;
            mult_int_en_o   = 1'b1;

            if (instr_rdata_i[14])
              mult_operator_o = MUL_IR;
            else
              mult_operator_o = MUL_I;
          end

          2'b10: begin // add with normalization and rounding
            // decide between using unsigned and rounding, and combinations
            // thereof
            case ({instr_rdata_i[31],instr_rdata_i[14]})
              2'b00: alu_operator_o = ALU_ADD;
              2'b01: alu_operator_o = ALU_ADDR;
              2'b10: alu_operator_o = ALU_ADDU;
              2'b11: alu_operator_o = ALU_ADDUR;
            endcase

            bmask_a_mux_o = BMASK_A_ZERO;
            bmask_b_mux_o = BMASK_B_S3;
          end

          2'b11: begin // sub with normalization and rounding
            // decide between using unsigned and rounding, and combinations
            // thereof
            case ({instr_rdata_i[31],instr_rdata_i[14]})
              2'b00: alu_operator_o = ALU_SUB;
              2'b01: alu_operator_o = ALU_SUBR;
              2'b10: alu_operator_o = ALU_SUBU;
              2'b11: alu_operator_o = ALU_SUBUR;
            endcase

            bmask_a_mux_o = BMASK_A_ZERO;
            bmask_b_mux_o = BMASK_B_S3;
          end

          default: begin
            regfile_alu_we = 1'b0;
            illegal_insn_o = 1'b1;
          end
        endcase
      end

      OPCODE_VECOP: begin
        regfile_alu_we      = 1'b1;
        rega_used_o         = 1'b1;
        imm_b_mux_sel_o     = IMMB_VS;

        // vector size
        if (instr_rdata_i[12]) begin
          alu_vec_mode_o  = VEC_MODE8;
          mult_operator_o = MUL_DOT8;
        end else begin
          alu_vec_mode_o = VEC_MODE16;
          mult_operator_o = MUL_DOT16;
        end

        // distinguish normal vector, sc and sci modes
        if (instr_rdata_i[14]) begin
          scalar_replication_o = 1'b1;

          if (instr_rdata_i[13]) begin
            // immediate scalar replication, .sci
            alu_op_b_mux_sel_o = OP_B_IMM;
          end else begin
            // register scalar replication, .sc
            regb_used_o = 1'b1;
          end
        end else begin
          // normal register use
          regb_used_o = 1'b1;
        end

        // now decode the instruction
        unique case (instr_rdata_i[31:26])
          6'b00000_0: begin alu_operator_o = ALU_ADD;  imm_b_mux_sel_o = IMMB_VS; end // pv.add
          6'b00001_0: begin alu_operator_o = ALU_SUB;  imm_b_mux_sel_o = IMMB_VS; end // pv.sub
          6'b00010_0: begin alu_operator_o = ALU_ADD;  imm_b_mux_sel_o = IMMB_VS; bmask_b_mux_o = BMASK_B_ONE; end // pv.avg
          6'b00011_0: begin alu_operator_o = ALU_ADDU; imm_b_mux_sel_o = IMMB_VU; bmask_b_mux_o = BMASK_B_ONE; end // pv.avgu
          6'b00100_0: begin alu_operator_o = ALU_MIN;  imm_b_mux_sel_o = IMMB_VS; end // pv.min
          6'b00101_0: begin alu_operator_o = ALU_MINU; imm_b_mux_sel_o = IMMB_VU; end // pv.minu
          6'b00110_0: begin alu_operator_o = ALU_MAX;  imm_b_mux_sel_o = IMMB_VS; end // pv.max
          6'b00111_0: begin alu_operator_o = ALU_MAXU; imm_b_mux_sel_o = IMMB_VU; end // pv.maxu
          6'b01000_0: begin alu_operator_o = ALU_SRL;  imm_b_mux_sel_o = IMMB_VS; end // pv.srl
          6'b01001_0: begin alu_operator_o = ALU_SRA;  imm_b_mux_sel_o = IMMB_VS; end // pv.sra
          6'b01010_0: begin alu_operator_o = ALU_SLL;  imm_b_mux_sel_o = IMMB_VS; end // pv.sll
          6'b01011_0: begin alu_operator_o = ALU_OR;   imm_b_mux_sel_o = IMMB_VS; end // pv.or
          6'b01100_0: begin alu_operator_o = ALU_XOR;  imm_b_mux_sel_o = IMMB_VS; end // pv.xor
          6'b01101_0: begin alu_operator_o = ALU_AND;  imm_b_mux_sel_o = IMMB_VS; end // pv.and
          6'b01110_0: begin alu_operator_o = ALU_ABS;  imm_b_mux_sel_o = IMMB_VS; end // pv.abs

          // shuffle/pack
          6'b11101_0,       // pv.shuffleI1
          6'b11110_0,       // pv.shuffleI2
          6'b11111_0,       // pv.shuffleI3
          6'b11000_0: begin // pv.shuffle, pv.shuffleI0
            alu_operator_o       = ALU_SHUF;
            imm_b_mux_sel_o      = IMMB_SHUF;
            regb_used_o          = 1'b1;
            scalar_replication_o = 1'b0;
          end
          6'b11001_0: begin // pv.shuffle2
            alu_operator_o       = ALU_SHUF2;
            regb_used_o          = 1'b1;
            regc_used_o          = 1'b1;
            regc_mux_o           = REGC_RD;
            scalar_replication_o = 1'b0;
          end
          6'b11010_0: begin // pv.pack
            alu_operator_o = ALU_PCKLO;
            regb_used_o    = 1'b1;
          end
          6'b11011_0: begin // pv.packhi
            alu_operator_o = ALU_PCKHI;
            regb_used_o    = 1'b1;
            regc_used_o    = 1'b1;
            regc_mux_o     = REGC_RD;
          end
          6'b11100_0: begin // pv.packlo
            alu_operator_o = ALU_PCKLO;
            regb_used_o    = 1'b1;
            regc_used_o    = 1'b1;
            regc_mux_o     = REGC_RD;
          end

          6'b01111_0: begin // pv.extract
            alu_operator_o = ALU_EXTS;
          end

          6'b10010_0: begin // pv.extractu
            alu_operator_o = ALU_EXT;
          end

          6'b10110_0: begin // pv.insert
            alu_operator_o     = ALU_INS;
            regc_used_o        = 1'b1;
            regc_mux_o         = REGC_RD;
            alu_op_b_mux_sel_o = OP_B_REGC_OR_FWD;
          end

          6'b10000_0: begin // pv.dotup
            mult_dot_en_o     = 1'b1;
            mult_dot_signed_o = 2'b00;
          end
          6'b10001_0: begin // pv.dotusp
            mult_dot_en_o     = 1'b1;
            mult_dot_signed_o = 2'b01;
          end
          6'b10011_0: begin // pv.dotsp
            mult_dot_en_o     = 1'b1;
            mult_dot_signed_o = 2'b11;
          end
          6'b10100_0: begin // pv.sdotup
            mult_dot_en_o     = 1'b1;
            mult_dot_signed_o = 2'b00;
            regc_used_o       = 1'b1;
            regc_mux_o        = REGC_RD;
          end
          6'b10101_0: begin // pv.sdotusp
            mult_dot_en_o     = 1'b1;
            mult_dot_signed_o = 2'b01;
            regc_used_o       = 1'b1;
            regc_mux_o        = REGC_RD;
          end
          6'b10111_0: begin // pv.sdotsp
            mult_dot_en_o     = 1'b1;
            mult_dot_signed_o = 2'b11;
            regc_used_o       = 1'b1;
            regc_mux_o        = REGC_RD;
          end

          // comparisons, always have bit 26 set
          6'b00000_1: begin alu_operator_o = ALU_EQ;  imm_b_mux_sel_o     = IMMB_VS; end // pv.cmpeq
          6'b00001_1: begin alu_operator_o = ALU_NE;  imm_b_mux_sel_o     = IMMB_VS; end // pv.cmpne
          6'b00010_1: begin alu_operator_o = ALU_GTS; imm_b_mux_sel_o     = IMMB_VS; end // pv.cmpgt
          6'b00011_1: begin alu_operator_o = ALU_GES; imm_b_mux_sel_o     = IMMB_VS; end // pv.cmpge
          6'b00100_1: begin alu_operator_o = ALU_LTS; imm_b_mux_sel_o     = IMMB_VS; end // pv.cmplt
          6'b00101_1: begin alu_operator_o = ALU_LES; imm_b_mux_sel_o     = IMMB_VS; end // pv.cmple
          6'b00110_1: begin alu_operator_o = ALU_GTU; imm_b_mux_sel_o     = IMMB_VU; end // pv.cmpgtu
          6'b00111_1: begin alu_operator_o = ALU_GEU; imm_b_mux_sel_o     = IMMB_VU; end // pv.cmpgeu
          6'b01000_1: begin alu_operator_o = ALU_LTU; imm_b_mux_sel_o     = IMMB_VU; end // pv.cmpltu
          6'b01001_1: begin alu_operator_o = ALU_LEU; imm_b_mux_sel_o     = IMMB_VU; end // pv.cmpleu

          default: illegal_insn_o = 1'b1;
        endcase
      end


      ////////////////////////////////////////////////
      //  ____  ____  _____ ____ ___    _    _      //
      // / ___||  _ \| ____/ ___|_ _|  / \  | |     //
      // \___ \| |_) |  _|| |    | |  / _ \ | |     //
      //  ___) |  __/| |__| |___ | | / ___ \| |___  //
      // |____/|_|   |_____\____|___/_/   \_\_____| //
      //                                            //
      ////////////////////////////////////////////////

      OPCODE_SYSTEM: begin
        if (instr_rdata_i[14:12] == 3'b000)
        begin
          // non CSR related SYSTEM instructions
          unique case (instr_rdata_i[31:0])
            32'h00_00_00_73:  // ECALL
            begin
              // environment (system) call
              ecall_insn_o = 1'b1;
            end

            32'h00_10_00_73:  // ebreak
            begin
              // debugger trap
              ebrk_insn = 1'b1;
            end

            32'h30_20_00_73:  // mret
            begin
              mret_insn = 1'b1;
            end

            32'h10_20_00_73:  // wfi
            begin
              // flush pipeline
              pipe_flush = 1'b1;
            end

            default:
            begin
              illegal_insn_o = 1'b1;
            end
          endcase
        end
        else
        begin
          // instruction to read/modify CSR
          csr_access_o        = 1'b1;
          regfile_alu_we      = 1'b1;
          alu_op_b_mux_sel_o  = OP_B_IMM;
          imm_a_mux_sel_o     = IMMA_Z;
          imm_b_mux_sel_o     = IMMB_I;    // CSR address is encoded in I imm

          if (instr_rdata_i[14] == 1'b1) begin
            // rs1 field is used as immediate
            alu_op_a_mux_sel_o = OP_A_IMM;
          end else begin
            rega_used_o        = 1'b1;
            alu_op_a_mux_sel_o = OP_A_REGA_OR_FWD;
          end

          unique case (instr_rdata_i[13:12])
            2'b01:   csr_op   = CSR_OP_WRITE;
            2'b10:   csr_op   = CSR_OP_SET;
            2'b11:   csr_op   = CSR_OP_CLEAR;
            default: illegal_insn_o = 1'b1;
          endcase
        end

      end


      ///////////////////////////////////////////////
      //  _   ___        ___     ___   ___  ____   //
      // | | | \ \      / / |   / _ \ / _ \|  _ \  //
      // | |_| |\ \ /\ / /| |  | | | | | | | |_) | //
      // |  _  | \ V  V / | |__| |_| | |_| |  __/  //
      // |_| |_|  \_/\_/  |_____\___/ \___/|_|     //
      //                                           //
      ///////////////////////////////////////////////

      OPCODE_HWLOOP: begin
        hwloop_target_mux_sel_o = 1'b0;

        unique case (instr_rdata_i[14:12])
          3'b000: begin
            // lp.starti: set start address to PC + I-type immediate
            hwloop_we[0]           = 1'b1;
            hwloop_start_mux_sel_o = 1'b0;
          end

          3'b001: begin
            // lp.endi: set end address to PC + I-type immediate
            hwloop_we[1]         = 1'b1;
          end

          3'b010: begin
            // lp.count: initialize counter from rs1
            hwloop_we[2]         = 1'b1;
            hwloop_cnt_mux_sel_o = 1'b1;
            rega_used_o          = 1'b1;
          end

          3'b011: begin
            // lp.counti: initialize counter from I-type immediate
            hwloop_we[2]         = 1'b1;
            hwloop_cnt_mux_sel_o = 1'b0;
          end

          3'b100: begin
            // lp.setup: initialize counter from rs1, set start address to
            // next instruction and end address to PC + I-type immediate
            hwloop_we              = 3'b111;
            hwloop_start_mux_sel_o = 1'b1;
            hwloop_cnt_mux_sel_o   = 1'b1;
            rega_used_o            = 1'b1;
          end

          3'b101: begin
            // lp.setupi: initialize counter from immediate, set start address to
            // next instruction and end address to PC + I-type immediate
            hwloop_we               = 3'b111;
            hwloop_target_mux_sel_o = 1'b1;
            hwloop_start_mux_sel_o  = 1'b1;
            hwloop_cnt_mux_sel_o    = 1'b0;
          end

          default: begin
            illegal_insn_o = 1'b1;
          end
        endcase
      end

      default: begin
        illegal_insn_o = 1'b1;
      end
    endcase

    // make sure invalid compressed instruction causes an exception
    if (illegal_c_insn_i) begin
      illegal_insn_o = 1'b1;
    end

    // misaligned access was detected by the LSU
    // TODO: this section should eventually be moved out of the decoder
    if (data_misaligned_i == 1'b1)
    begin
      // only part of the pipeline is unstalled, make sure that the
      // correct operands are sent to the AGU
      alu_op_a_mux_sel_o  = OP_A_REGA_OR_FWD;
      alu_op_b_mux_sel_o  = OP_B_IMM;
      imm_b_mux_sel_o     = IMMB_PCINCR;

      // if prepost increments are used, we do not write back the
      // second address since the first calculated address was
      // the correct one
      regfile_alu_we = 1'b0;

      // if post increments are used, we must make sure that for
      // the second memory access we do use the adder
      prepost_useincr_o = 1'b1;
      // we do not want to replicate operand_b
      scalar_replication_o = 1'b0;
    end else if (mult_multicycle_i) begin
      alu_op_c_mux_sel_o = OP_C_REGC_OR_FWD;
    end
  end

  // deassert we signals (in case of stalls)
  assign regfile_mem_we_o  = (deassert_we_i) ? 1'b0          : regfile_mem_we;
  assign regfile_alu_we_o  = (deassert_we_i) ? 1'b0          : regfile_alu_we;
  assign data_req_o        = (deassert_we_i) ? 1'b0          : data_req;
  assign hwloop_we_o       = (deassert_we_i) ? 3'b0          : hwloop_we;
  assign csr_op_o          = (deassert_we_i) ? CSR_OP_NONE  : csr_op;
  assign jump_in_id_o      = (deassert_we_i) ? BRANCH_NONE  : jump_in_id;
  assign ebrk_insn_o       = (deassert_we_i) ? 1'b0          : ebrk_insn;
  assign eret_insn_o       = (deassert_we_i) ? 1'b0          : mret_insn;  // TODO: do not deassert? Note ERET is now MRET
  assign pipe_flush_o      = (deassert_we_i) ? 1'b0          : pipe_flush; // TODO: do not deassert?

  assign jump_in_dec_o     = jump_in_id;

endmodule // controller

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the âLicenseâ?); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an âAS ISâ? BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Matthias Baer - baermatt@student.ethz.ch                   //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Andreas Traber - atraber@student.ethz.ch                   //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
// Design Name:    Main controller                                            //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Main CPU controller of the processor                       //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

import riscv_defines::*;

module riscv_controller
(
  input  logic        clk,
  input  logic        rst_n,

  input  logic        fetch_enable_i,             // Start the decoding
  output logic        ctrl_busy_o,                // Core is busy processing instructions
  output logic        is_decoding_o,              // Core is in decoding state

  // decoder related signals
  output logic        deassert_we_o,              // deassert write enable for next instruction
  input  logic        illegal_insn_i,             // decoder encountered an invalid instruction
  input  logic        eret_insn_i,                // decoder encountered an eret instruction
  input  logic        pipe_flush_i,               // decoder wants to do a pipe flush

  input  logic        rega_used_i,                // register A is used
  input  logic        regb_used_i,                // register B is used
  input  logic        regc_used_i,                // register C is used

  // from IF/ID pipeline
  input  logic        instr_valid_i,              // instruction coming from IF/ID pipeline is valid
  input  logic [31:0] instr_rdata_i,              // Instruction read from instr memory/cache: (sampled in the if stage)

  // from prefetcher
  output logic        instr_req_o,                // Start fetching instructions

  // to prefetcher
  output logic        pc_set_o,                   // jump to address set by pc_mux
  output logic [2:0]  pc_mux_o,                   // Selector in the Fetch stage to select the rigth PC (normal, jump ...)

  // LSU
  input  logic        data_req_ex_i,              // data memory access is currently performed in EX stage
  input  logic        data_misaligned_i,
  input  logic        data_load_event_i,

  // from ALU
  input  logic        mult_multicycle_i,          // multiplier is taken multiple cycles and uses op c as storage

  // jump/branch signals
  input  logic        branch_taken_ex_i,          // branch taken signal from EX ALU
  input  logic [1:0]  jump_in_id_i,               // jump is being calculated in ALU
  input  logic [1:0]  jump_in_dec_i,              // jump is being calculated in ALU

  // Exception Controller Signals
  input  logic        exc_req_i,
  output logic        exc_ack_o,

  output logic        exc_save_if_o,
  output logic        exc_save_id_o,
  output logic        exc_restore_id_o,

  // Debug Signals
  input  logic        dbg_req_i,                  // a trap was hit, so we have to flush EX and WB
  output logic        dbg_ack_o,                  // we stopped and give control to debug now

  input  logic        dbg_stall_i,                // Pipeline stall is requested
  input  logic        dbg_jump_req_i,             // Change PC to value from debug unit

  // Forwarding signals from regfile
  input  logic [4:0]  regfile_waddr_ex_i,         // FW: write address from EX stage
  input  logic        regfile_we_ex_i,            // FW: write enable from  EX stage
  input  logic [4:0]  regfile_waddr_wb_i,         // FW: write address from WB stage
  input  logic        regfile_we_wb_i,            // FW: write enable from  WB stage
  input  logic [4:0]  regfile_alu_waddr_fw_i,     // FW: ALU/MUL write address from EX stage
  input  logic        regfile_alu_we_fw_i,        // FW: ALU/MUL write enable from  EX stage

  // forwarding signals
  output logic [1:0]  operand_a_fw_mux_sel_o,     // regfile ra data selector form ID stage
  output logic [1:0]  operand_b_fw_mux_sel_o,     // regfile rb data selector form ID stage
  output logic [1:0]  operand_c_fw_mux_sel_o,     // regfile rc data selector form ID stage

  // forwarding detection signals
  input logic         reg_d_ex_is_reg_a_i,
  input logic         reg_d_ex_is_reg_b_i,
  input logic         reg_d_ex_is_reg_c_i,
  input logic         reg_d_wb_is_reg_a_i,
  input logic         reg_d_wb_is_reg_b_i,
  input logic         reg_d_wb_is_reg_c_i,
  input logic         reg_d_alu_is_reg_a_i,
  input logic         reg_d_alu_is_reg_b_i,
  input logic         reg_d_alu_is_reg_c_i,


  // stall signals
  output logic        halt_if_o,
  output logic        halt_id_o,

  output logic        misaligned_stall_o,
  output logic        jr_stall_o,
  output logic        load_stall_o,

  input  logic        id_ready_i,                 // ID stage is ready

  input  logic        if_valid_i,                 // IF stage is done
  input  logic        ex_valid_i,                 // EX stage is done
  input  logic        wb_valid_i,                 // WB stage is done

  // Performance Counters
  output logic        perf_jump_o,                // we are executing a jump instruction   (j, jr, jal, jalr)
  output logic        perf_jr_stall_o,            // stall due to jump-register-hazard
  output logic        perf_ld_stall_o,             // stall due to load-use-hazard
  
  //TODO Output of HDC logic - predicted class
  input logic [3:0]   pred_class
);

  // FSM state encoding
  enum  logic [3:0] { RESET, BOOT_SET, SLEEP, FIRST_FETCH,
                      DECODE,
                      FLUSH_EX, FLUSH_WB,
                      DBG_SIGNAL, DBG_SIGNAL_SLEEP, DBG_WAIT, DBG_WAIT_BRANCH, DBG_WAIT_SLEEP } ctrl_fsm_cs, ctrl_fsm_ns;

  logic jump_done, jump_done_q;


`ifndef SYNTHESIS
  // synopsys translate_off
  // make sure we are called later so that we do not generate messages for
  // glitches
  always_ff @(negedge clk)
  begin
    // print warning in case of decoding errors
    if (is_decoding_o && illegal_insn_i) begin
      $display("%t: Illegal instruction (core %0d) at PC 0x%h:", $time, riscv_core.core_id_i,
               riscv_id_stage.pc_id_i);
    end
  end
  // synopsys translate_on
`endif


  ////////////////////////////////////////////////////////////////////////////////////////////
  //   ____ ___  ____  _____    ____ ___  _   _ _____ ____   ___  _     _     _____ ____    //
  //  / ___/ _ \|  _ \| ____|  / ___/ _ \| \ | |_   _|  _ \ / _ \| |   | |   | ____|  _ \   //
  // | |  | | | | |_) |  _|   | |  | | | |  \| | | | | |_) | | | | |   | |   |  _| | |_) |  //
  // | |__| |_| |  _ <| |___  | |__| |_| | |\  | | | |  _ <| |_| | |___| |___| |___|  _ <   //
  //  \____\___/|_| \_\_____|  \____\___/|_| \_| |_| |_| \_\\___/|_____|_____|_____|_| \_\  //
  //                                                                                        //
  ////////////////////////////////////////////////////////////////////////////////////////////
  always_comb
  begin
    // Default values
    instr_req_o      = 1'b1;

    exc_ack_o        = 1'b0;
    exc_save_if_o    = 1'b0;
    exc_save_id_o    = 1'b0;
    exc_restore_id_o = 1'b0;

    pc_mux_o         = PC_BOOT;
    pc_set_o         = 1'b0;
    jump_done        = jump_done_q;

    ctrl_fsm_ns      = ctrl_fsm_cs;

    ctrl_busy_o      = 1'b1;
    is_decoding_o    = 1'b0;

    halt_if_o        = 1'b0;
    halt_id_o        = 1'b0;
    dbg_ack_o        = 1'b0;

    unique case (ctrl_fsm_cs)
      // We were just reset, wait for fetch_enable
      RESET:
      begin
        ctrl_busy_o   = 1'b0;
        instr_req_o   = 1'b0;

        if (fetch_enable_i == 1'b1)
          ctrl_fsm_ns = BOOT_SET;
        else if (dbg_req_i) begin
          // just go to debug even when we did not yet get a fetch enable
          // this means that the NPC will not be set yet
          ctrl_fsm_ns = DBG_SIGNAL;
        end
      end

      // copy boot address to instr fetch address
      BOOT_SET:
      begin
        instr_req_o   = 1'b1;
        pc_mux_o      = PC_BOOT;
        pc_set_o      = 1'b1;

        ctrl_fsm_ns = FIRST_FETCH;
      end

      // instruction in if_stage is already valid
      SLEEP:
      begin
        // we begin execution when either fetch_enable is high or an
        // interrupt has arrived
        ctrl_busy_o   = 1'b0;
        instr_req_o   = 1'b0;
        halt_if_o = 1'b1;
        halt_id_o = 1'b1;

        if (dbg_req_i) begin
          // debug request, now we need to check if we should stay sleeping or
          // go to normal processing later
          if (fetch_enable_i || exc_req_i)
            ctrl_fsm_ns = DBG_SIGNAL;
          else
            ctrl_fsm_ns = DBG_SIGNAL_SLEEP;

        end else begin
          // no debug request incoming, normal execution flow
          if (fetch_enable_i || exc_req_i)
          begin
            ctrl_fsm_ns  = FIRST_FETCH;
          end
        end
      end

      FIRST_FETCH:
      begin
        // Stall because of IF miss
        if ((id_ready_i == 1'b1) && (dbg_stall_i == 1'b0))
        begin
          ctrl_fsm_ns = DECODE;
        end

        // handle exceptions
        if (exc_req_i) begin
          pc_mux_o     = PC_EXCEPTION;
          pc_set_o     = 1'b1;
          exc_ack_o    = 1'b1;

          // TODO: This assumes that the pipeline is always flushed before
          //       going to sleep.
          exc_save_if_o = 1'b1;
        end
      end

      DECODE:
      begin
        is_decoding_o = 1'b0;

        // decode and execute instructions only if the current conditional
        // branch in the EX stage is either not taken, or there is no
        // conditional branch in the EX stage
        if (instr_valid_i && (~branch_taken_ex_i))
        begin // now analyze the current instruction in the ID stage
          is_decoding_o = 1'b1;

          // handle unconditional jumps
          // we can jump directly since we know the address already
          // we don't need to worry about conditional branches here as they
          // will be evaluated in the EX stage
          if (jump_in_dec_i == BRANCH_JALR || jump_in_dec_i == BRANCH_JAL) begin
            pc_mux_o = PC_JUMP;

            // if there is a jr stall, wait for it to be gone
            if ((~jr_stall_o) && (~jump_done_q)) begin
              pc_set_o    = 1'b1;
              jump_done   = 1'b1;
            end

            // we don't have to change our current state here as the prefetch
            // buffer is automatically invalidated, thus the next instruction
            // that is served to the ID stage is the one of the jump target
          end else begin
            // handle exceptions
            if (exc_req_i) begin
              pc_mux_o      = PC_EXCEPTION;
              pc_set_o      = 1'b1;
              exc_ack_o     = 1'b1;

              halt_id_o     = 1'b1; // we don't want to propagate this instruction to EX
              exc_save_id_o = 1'b1;

              // we don't have to change our current state here as the prefetch
              // buffer is automatically invalidated, thus the next instruction
              // that is served to the ID stage is the one of the jump to the
              // exception handler
            end
          end

          if (eret_insn_i) begin
            pc_mux_o         = PC_ERET;
            exc_restore_id_o = 1'b1;

            if ((~jump_done_q)) begin
              pc_set_o    = 1'b1;
              jump_done   = 1'b1;
            end
          end

          // handle WFI instruction, flush pipeline and (potentially) go to
          // sleep
          // also handles eret when the core should go back to sleep
          if (pipe_flush_i || (eret_insn_i && (~fetch_enable_i)))
          begin
            halt_if_o = 1'b1;
            halt_id_o = 1'b1;

            ctrl_fsm_ns = FLUSH_EX;
          end
          else if (dbg_req_i)
          begin
            // take care of debug
            // branch conditional will be handled in next state
            // halt pipeline immediately
            halt_if_o = 1'b1;

            // make sure the current instruction has been executed
            // before changing state to non-decode
            if (id_ready_i) begin
              if (jump_in_id_i == BRANCH_COND)
                ctrl_fsm_ns = DBG_WAIT_BRANCH;
              else
                ctrl_fsm_ns = DBG_SIGNAL;
            end else if (data_load_event_i) begin
              // special case for p.elw
              // If there was a load event (which means p.elw), we go to debug
              // even though we are still blocked
              // we don't have to distuinguish between branch and non-branch,
              // since the p.elw sits in the EX stage
              ctrl_fsm_ns = DBG_SIGNAL;
            end
          end
        end

        // TODO: make sure this is not done multiple times in a row!!!
        //       maybe with an assertion?
        // handle conditional branches
        if (branch_taken_ex_i) begin
          // there is a branch in the EX stage that is taken
          pc_mux_o      = PC_BRANCH;
          pc_set_o      = 1'b1;

          is_decoding_o = 1'b0; // we are not decoding the current instruction in the ID stage

          // if we want to debug, flush the pipeline
          // the current_pc_if will take the value of the next instruction to
          // be executed (NPC)
          if (dbg_req_i)
          begin
            ctrl_fsm_ns = DBG_SIGNAL;
          end
        end
      end

      // a branch was in ID when a debug trap is hit
      DBG_WAIT_BRANCH:
      begin
        halt_if_o = 1'b1;

        if (branch_taken_ex_i) begin
          // there is a branch in the EX stage that is taken
          pc_mux_o = PC_BRANCH;
          pc_set_o = 1'b1;
        end

        ctrl_fsm_ns = DBG_SIGNAL;
      end

      // now we can signal to the debugger that our pipeline is empty and it
      // can examine our current state
      DBG_SIGNAL:
      begin
        dbg_ack_o  = 1'b1;
        halt_if_o  = 1'b1;

        ctrl_fsm_ns = DBG_WAIT;
      end

      DBG_SIGNAL_SLEEP:
      begin
        dbg_ack_o  = 1'b1;
        halt_if_o  = 1'b1;

        ctrl_fsm_ns = DBG_WAIT_SLEEP;
      end

      // The Debugger is active in this state
      // we wait until it is done and go back to SLEEP
      DBG_WAIT_SLEEP:
      begin
        halt_if_o = 1'b1;

        if (dbg_jump_req_i) begin
          pc_mux_o     = PC_DBG_NPC;
          pc_set_o     = 1'b1;
          ctrl_fsm_ns  = DBG_WAIT;
        end

        if (dbg_stall_i == 1'b0) begin
          ctrl_fsm_ns = SLEEP;
        end
      end

      // The Debugger is active in this state
      // we wait until it is done and go back to DECODE
      DBG_WAIT:
      begin
        halt_if_o = 1'b1;

        if (dbg_jump_req_i) begin
          pc_mux_o     = PC_DBG_NPC;
          pc_set_o     = 1'b1;
          ctrl_fsm_ns  = DBG_WAIT;
        end

        if (dbg_stall_i == 1'b0) begin
          ctrl_fsm_ns = DECODE;
        end
      end

      // flush the pipeline, insert NOP into EX stage
      FLUSH_EX:
      begin
        halt_if_o = 1'b1;
        halt_id_o = 1'b1;

        if (ex_valid_i)
          ctrl_fsm_ns = FLUSH_WB;
      end

      // flush the pipeline, insert NOP into EX and WB stage
      FLUSH_WB:
      begin
        halt_if_o = 1'b1;
        halt_id_o = 1'b1;

        if(fetch_enable_i) begin
          if (dbg_req_i) begin
            ctrl_fsm_ns = DBG_SIGNAL;
          end else begin
            ctrl_fsm_ns = DECODE;
            halt_if_o   = 1'b0;
          end
        end else begin
          if (dbg_req_i) begin
            ctrl_fsm_ns = DBG_SIGNAL_SLEEP;
          end else begin
            ctrl_fsm_ns = SLEEP;
          end
        end
      end

      default: begin
        instr_req_o = 1'b0;
        ctrl_fsm_ns = RESET;
      end
    endcase
  end

  /////////////////////////////////////////////////////////////
  //  ____  _        _ _    ____            _             _  //
  // / ___|| |_ __ _| | |  / ___|___  _ __ | |_ _ __ ___ | | //
  // \___ \| __/ _` | | | | |   / _ \| '_ \| __| '__/ _ \| | //
  //  ___) | || (_| | | | | |__| (_) | | | | |_| | | (_) | | //
  // |____/ \__\__,_|_|_|  \____\___/|_| |_|\__|_|  \___/|_| //
  //                                                         //
  /////////////////////////////////////////////////////////////
  always_comb
  begin
    load_stall_o   = 1'b0;
    jr_stall_o     = 1'b0;
    deassert_we_o  = 1'b0;

    // deassert WE when the core is not decoding instructions
    if (~is_decoding_o)
      deassert_we_o = 1'b1;

    // deassert WE in case of illegal instruction
    if (illegal_insn_i)
      deassert_we_o = 1'b1;

    // Stall because of load operation
    if ((data_req_ex_i == 1'b1) && (regfile_we_ex_i == 1'b1) &&
        ((reg_d_ex_is_reg_a_i == 1'b1) || (reg_d_ex_is_reg_b_i == 1'b1) || (reg_d_ex_is_reg_c_i == 1'b1)) )
    begin
      deassert_we_o   = 1'b1;
      load_stall_o    = 1'b1;
    end

    // Stall because of jr path
    // - always stall if a result is to be forwarded to the PC
    // we don't care about in which state the ctrl_fsm is as we deassert_we
    // anyway when we are not in DECODE
    if ((jump_in_dec_i == BRANCH_JALR) &&
        (((regfile_we_wb_i == 1'b1) && (reg_d_wb_is_reg_a_i == 1'b1)) ||
         ((regfile_we_ex_i == 1'b1) && (reg_d_ex_is_reg_a_i == 1'b1)) ||
         ((regfile_alu_we_fw_i == 1'b1) && (reg_d_alu_is_reg_a_i == 1'b1))) )
    begin
      jr_stall_o      = 1'b1;
      deassert_we_o     = 1'b1;
    end
  end

  // stall because of misaligned data access
  assign misaligned_stall_o = data_misaligned_i;


  // Forwarding control unit
  always_comb
  begin
    // default assignements
    operand_a_fw_mux_sel_o = SEL_REGFILE;
    operand_b_fw_mux_sel_o = SEL_REGFILE;
    operand_c_fw_mux_sel_o = SEL_REGFILE;

    // Forwarding WB -> ID
    if (regfile_we_wb_i == 1'b1)
    begin
      if (reg_d_wb_is_reg_a_i == 1'b1)
        operand_a_fw_mux_sel_o = SEL_FW_WB;
      if (reg_d_wb_is_reg_b_i == 1'b1)
        operand_b_fw_mux_sel_o = SEL_FW_WB;
      if (reg_d_wb_is_reg_c_i == 1'b1)
        operand_c_fw_mux_sel_o = SEL_FW_WB;
    end

    // Forwarding EX -> ID
    if (regfile_alu_we_fw_i == 1'b1)
    begin
     if (reg_d_alu_is_reg_a_i == 1'b1)
       operand_a_fw_mux_sel_o = SEL_FW_EX;
     if (reg_d_alu_is_reg_b_i == 1'b1)
       operand_b_fw_mux_sel_o = SEL_FW_EX;
     if (reg_d_alu_is_reg_c_i == 1'b1)
       operand_c_fw_mux_sel_o = SEL_FW_EX;
    end

    // for misaligned memory accesses
    if (data_misaligned_i)
    begin
      operand_a_fw_mux_sel_o  = SEL_FW_EX;
      operand_b_fw_mux_sel_o  = SEL_REGFILE;
    end else if (mult_multicycle_i) begin
      operand_c_fw_mux_sel_o  = SEL_FW_EX;
    end
  end

  // update registers
  always_ff @(posedge clk , negedge rst_n)
  begin : UPDATE_REGS
    if ( rst_n == 1'b0 )
    begin
      ctrl_fsm_cs <= RESET;
      jump_done_q <= 1'b0;
    end
    else
    begin
      ctrl_fsm_cs <= ctrl_fsm_ns;

      // clear when id is valid (no instruction incoming)
      jump_done_q <= jump_done & (~id_ready_i);
    end
  end

  // Performance Counters
  assign perf_jump_o      = (jump_in_id_i == BRANCH_JAL || jump_in_id_i == BRANCH_JALR);
  assign perf_jr_stall_o  = jr_stall_o;
  assign perf_ld_stall_o  = load_stall_o;


`ifndef verilator
  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------

  // make sure that taken branches do not happen back-to-back, as this is not
  // possible without branch prediction in the IF stage
  assert property (
    @(posedge clk) (branch_taken_ex_i) |=> (~branch_taken_ex_i) ) else $warning("Two branches back-to-back are taken");
`endif

endmodule // controller

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Andreas Traber - atraber@student.ethz.ch                   //
//                                                                            //
// Additional contributions by:                                               //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
// Design Name:    Exception Controller                                       //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Exception Controller of the pipelined processor            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

import riscv_defines::*;

module riscv_exc_controller
(
  input  logic        clk,
  input  logic        rst_n,

  // handshake signals to controller
  output logic        req_o,
  input  logic        ack_i,

  output logic        trap_o,

  // to IF stage
  output logic  [1:0] pc_mux_o,       // selects target PC for exception
  output logic  [4:0] vec_pc_mux_o,   // selects interrupt handler for vectorized interrupts

  // interrupt lines
  input  logic [31:0] irq_i,          // level-triggered interrupt inputs
  input  logic        irq_enable_i,   // interrupt enable bit from CSR

  // from decoder
  input  logic        ebrk_insn_i,    // ebrk instruction encountered (EBREAK)
  input  logic        illegal_insn_i, // illegal instruction encountered
  input  logic        ecall_insn_i,   // ecall instruction encountered
  input  logic        eret_insn_i,    // eret instruction encountered

  input  logic        lsu_load_err_i,
  input  logic        lsu_store_err_i,

  // to CSR
  output logic [5:0]  cause_o,
  output logic        save_cause_o,

  // from debug unit
  input  logic [DBG_SETS_W-1:0] dbg_settings_i
);


  enum logic [0:0] { IDLE, WAIT_CONTROLLER } exc_ctrl_cs, exc_ctrl_ns;

  logic req_int;
  logic [1:0] pc_mux_int, pc_mux_int_q;
  logic [5:0] cause_int, cause_int_q;

  integer i;


  // a trap towards the debug unit is generated when one of the
  // following conditions are true:
  // - ebreak instruction encountered
  // - single-stepping mode enabled
  // - illegal instruction exception and IIE bit is set
  // - IRQ and INTE bit is set and no exception is currently running
  // - Debuger requests halt
  assign trap_o =       (dbg_settings_i[DBG_SETS_SSTE])
                      | (ecall_insn_i            & dbg_settings_i[DBG_SETS_ECALL])
                      | (lsu_load_err_i          & dbg_settings_i[DBG_SETS_ELSU])
                      | (lsu_store_err_i         & dbg_settings_i[DBG_SETS_ELSU])
                      | (ebrk_insn_i             & dbg_settings_i[DBG_SETS_EBRK])
                      | (illegal_insn_i          & dbg_settings_i[DBG_SETS_EILL])
                      | (irq_enable_i & (|irq_i) & dbg_settings_i[DBG_SETS_IRQ]);

  // request for exception/interrupt
  assign req_int =   ecall_insn_i
                   | lsu_load_err_i
                   | lsu_store_err_i
                   | illegal_insn_i
                   | (irq_enable_i & (|irq_i));


  // Exception cause and ISR address selection
  always_comb
  begin
    cause_int  = 6'b0;
    pc_mux_int = 'x;

    if (irq_enable_i) begin
      // pc_mux_int is a critical signal, so try to get it as soon as possible
      if (|irq_i)
        pc_mux_int = EXC_PC_IRQ;

      for (i = 31; i >= 0; i--)
      begin
        if (irq_i[i]) begin
          cause_int[5]   = 1'b1;
          cause_int[4:0] = $unsigned(i);
        end
      end
    end

    if (ebrk_insn_i) begin
      cause_int  = 6'b0_00011;
    end

    if (ecall_insn_i) begin
      cause_int  = 6'b0_01011;
      pc_mux_int = EXC_PC_ECALL;
    end

    if (illegal_insn_i) begin
      cause_int  = 6'b0_00010;
      pc_mux_int = EXC_PC_ILLINSN;
    end

    if (lsu_load_err_i) begin
      cause_int  = 6'b0_00101;
      pc_mux_int = EXC_PC_LOAD;
    end

    if (lsu_store_err_i) begin
      cause_int  = 6'b0_00111;
      pc_mux_int = EXC_PC_STORE;
    end
  end

  always_ff @(posedge clk, negedge rst_n)
  begin
    if (rst_n == 1'b0) begin
      cause_int_q  <= '0;
      pc_mux_int_q <= '0;
    end else if (exc_ctrl_cs == IDLE && req_int) begin
      // save cause and ISR when new irq request is first sent to controller
      cause_int_q  <= cause_int;
      pc_mux_int_q <= pc_mux_int;
    end
  end


  // Exception cause and mux output (with bypass)
  assign cause_o      = ((exc_ctrl_cs == IDLE && req_int) || ebrk_insn_i) ? cause_int  : cause_int_q;
  assign pc_mux_o     = (exc_ctrl_cs == IDLE && req_int) ? pc_mux_int : pc_mux_int_q;

  // for vectorized IRQ PC mux
  assign vec_pc_mux_o = cause_o[4:0];


  // Exception controller FSM
  always_comb
  begin
    exc_ctrl_ns  = exc_ctrl_cs;
    req_o        = 1'b0;
    save_cause_o = 1'b0;

    unique case (exc_ctrl_cs)
      IDLE:
      begin
        req_o = req_int;

        if (req_int) begin
          exc_ctrl_ns = WAIT_CONTROLLER;

          if (ack_i) begin
            save_cause_o = 1'b1;
            exc_ctrl_ns  = IDLE;
          end
        end
      end

      WAIT_CONTROLLER:
      begin
        req_o = 1'b1;

        if (ack_i) begin
          save_cause_o = 1'b1;
          exc_ctrl_ns  = IDLE;
        end
      end

      default:
      begin
        exc_ctrl_ns = IDLE;
      end
    endcase
  end

  always_ff @(posedge clk, negedge rst_n)
  begin
    if (rst_n == 1'b0)
      exc_ctrl_cs <= IDLE;
    else
      exc_ctrl_cs <= exc_ctrl_ns;
  end


`ifndef SYNTHESIS
  // synopsys translate_off
  // evaluate at falling edge to avoid duplicates during glitches
  always_ff @(negedge clk)
  begin
    if (rst_n && req_o && ack_i)
      $display("%t: Entering exception routine.", $time);
  end
  // synopsys translate_on
`endif

endmodule

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Michael Gautschi - gautschi@iis.ee.ethz.ch                 //
//                                                                            //
// Design Name:    hwloop regs                                                //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Hardware loop registers                                    //
//                 a) store start/end address of N=4 hardware loops           //
//                 b) store init value of counter for each hardware loop      //
//                 c) decrement counter if hwloop taken                       //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module riscv_hwloop_regs
#(
  parameter N_REGS     = 2,
  parameter N_REG_BITS = $clog2(N_REGS)
)
(
  input  logic                     clk,
  input  logic                     rst_n,

  // from ex stage
  input  logic           [31:0]    hwlp_start_data_i,
  input  logic           [31:0]    hwlp_end_data_i,
  input  logic           [31:0]    hwlp_cnt_data_i,
  input  logic            [2:0]    hwlp_we_i,
  input  logic [N_REG_BITS-1:0]    hwlp_regid_i,         // selects the register set

  // from controller
  input  logic                     valid_i,

  // from hwloop controller
  input  logic [N_REGS-1:0]        hwlp_dec_cnt_i,

  // to hwloop controller
  output logic [N_REGS-1:0] [31:0] hwlp_start_addr_o,
  output logic [N_REGS-1:0] [31:0] hwlp_end_addr_o,
  output logic [N_REGS-1:0] [31:0] hwlp_counter_o
);


  logic [N_REGS-1:0] [31:0] hwlp_start_q;
  logic [N_REGS-1:0] [31:0] hwlp_end_q;
  logic [N_REGS-1:0] [31:0] hwlp_counter_q, hwlp_counter_n;

  int unsigned i;


  assign hwlp_start_addr_o = hwlp_start_q;
  assign hwlp_end_addr_o   = hwlp_end_q;
  assign hwlp_counter_o    = hwlp_counter_q;


  /////////////////////////////////////////////////////////////////////////////////
  // HWLOOP start-address register                                               //
  /////////////////////////////////////////////////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
  begin : HWLOOP_REGS_START
    if (rst_n == 1'b0)
    begin
      hwlp_start_q <= '{default: 32'b0};
    end
    else if (hwlp_we_i[0] == 1'b1)
    begin
      hwlp_start_q[hwlp_regid_i] <= hwlp_start_data_i;
    end
  end


  /////////////////////////////////////////////////////////////////////////////////
  // HWLOOP end-address register                                                 //
  /////////////////////////////////////////////////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
  begin : HWLOOP_REGS_END
    if (rst_n == 1'b0)
    begin
      hwlp_end_q <= '{default: 32'b0};
    end
    else if (hwlp_we_i[1] == 1'b1)
    begin
      hwlp_end_q[hwlp_regid_i] <= hwlp_end_data_i;
    end
  end


  /////////////////////////////////////////////////////////////////////////////////
  // HWLOOP counter register with decrement logic                                //
  /////////////////////////////////////////////////////////////////////////////////
  genvar k;
  for (k = 0; k < N_REGS; k++) begin
    assign hwlp_counter_n[k] = hwlp_counter_q[k] - 1;
  end

  always_ff @(posedge clk, negedge rst_n)
  begin : HWLOOP_REGS_COUNTER
    if (rst_n == 1'b0)
    begin
      hwlp_counter_q <= '{default: 32'b0};
    end
    else
    begin
      for (i = 0; i < N_REGS; i++)
      begin
        if ((hwlp_we_i[2] == 1'b1) && (i == hwlp_regid_i)) begin
          hwlp_counter_q[i] <= hwlp_cnt_data_i;
        end else begin
          if (hwlp_dec_cnt_i[i] && valid_i)
            hwlp_counter_q[i] <= hwlp_counter_n[i];
        end
      end
    end
  end

`ifndef verilator
  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------

  // do not decrement more than one counter at once
  assert property (
    @(posedge clk) (valid_i) |-> ($countones(hwlp_dec_cnt_i) <= 1) );
`endif

endmodule

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Renzo Andri - andrire@student.ethz.ch                      //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                 Andreas Traber - atraber@iis.ee.ethz.ch                    //
//                                                                            //
// Design Name:    Excecute stage                                             //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Execution stage: Hosts ALU and MAC unit                    //
//                 ALU: computes additions/subtractions/comparisons           //
//                 MAC:                                                       //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

import riscv_defines::*;


module riscv_ex_stage
(
  input  logic        clk,
  input  logic        rst_n,

  // ALU signals from ID stage
  input  logic [ALU_OP_WIDTH-1:0] alu_operator_i,
  input  logic [31:0] alu_operand_a_i,
  input  logic [31:0] alu_operand_b_i,
  input  logic [31:0] alu_operand_c_i,
  input  logic [ 4:0] bmask_a_i,
  input  logic [ 4:0] bmask_b_i,
  input  logic [ 1:0] imm_vec_ext_i,
  input  logic [ 1:0] alu_vec_mode_i,

  // Multiplier signals
  input  logic [ 2:0] mult_operator_i,
  input  logic [31:0] mult_operand_a_i,
  input  logic [31:0] mult_operand_b_i,
  input  logic [31:0] mult_operand_c_i,
  input  logic        mult_en_i,
  input  logic        mult_sel_subword_i,
  input  logic [ 1:0] mult_signed_mode_i,
  input  logic [ 4:0] mult_imm_i,

  input  logic [31:0] mult_dot_op_a_i,
  input  logic [31:0] mult_dot_op_b_i,
  input  logic [31:0] mult_dot_op_c_i,
  input  logic [ 1:0] mult_dot_signed_i,

  output logic        mult_multicycle_o,

  // input from ID stage
  input  logic        branch_in_ex_i,
  input  logic [4:0]  regfile_alu_waddr_i,
  input  logic        regfile_alu_we_i,

  // directly passed through to WB stage, not used in EX
  input  logic        regfile_we_i,
  input  logic [4:0]  regfile_waddr_i,

  // CSR access
  input  logic        csr_access_i,
  input  logic [31:0] csr_rdata_i,

  // Output of EX stage pipeline
  output logic [4:0]  regfile_waddr_wb_o,
  output logic        regfile_we_wb_o,

  // Forwarding ports : to ID stage
  output logic  [4:0] regfile_alu_waddr_fw_o,
  output logic        regfile_alu_we_fw_o,
  output logic [31:0] regfile_alu_wdata_fw_o,    // forward to RF and ID/EX pipe, ALU & MUL

  // To IF: Jump and branch target and decision
  output logic [31:0] jump_target_o,
  output logic        branch_decision_o,

  // Stall Control
  input  logic        lsu_ready_ex_i, // EX part of LSU is done

  output logic        ex_ready_o, // EX stage ready for new data
  output logic        ex_valid_o, // EX stage gets new data
  input  logic        wb_ready_i  // WB stage ready for new data
);


  logic [31:0] alu_result;
  logic [31:0] alu_csr_result;
  logic [31:0] mult_result;
  logic        alu_cmp_result;

  logic        alu_ready;
  logic        mult_ready;


  // EX stage result mux (ALU, MAC unit, CSR)
  assign alu_csr_result         = csr_access_i ? csr_rdata_i : alu_result;

  assign regfile_alu_wdata_fw_o = mult_en_i ? mult_result : alu_csr_result;


  assign regfile_alu_we_fw_o    = regfile_alu_we_i;
  assign regfile_alu_waddr_fw_o = regfile_alu_waddr_i;


  // branch handling
  assign branch_decision_o = alu_cmp_result;
  assign jump_target_o     = alu_operand_c_i;


  ////////////////////////////
  //     _    _    _   _    //
  //    / \  | |  | | | |   //
  //   / _ \ | |  | | | |   //
  //  / ___ \| |__| |_| |   //
  // /_/   \_\_____\___/    //
  //                        //
  ////////////////////////////

  riscv_alu alu_i
  (
    .clk                 ( clk             ),
    .rst_n               ( rst_n           ),

    .operator_i          ( alu_operator_i  ),
    .operand_a_i         ( alu_operand_a_i ),
    .operand_b_i         ( alu_operand_b_i ),
    .operand_c_i         ( alu_operand_c_i ),

    .vector_mode_i       ( alu_vec_mode_i  ),
    .bmask_a_i           ( bmask_a_i       ),
    .bmask_b_i           ( bmask_b_i       ),
    .imm_vec_ext_i       ( imm_vec_ext_i   ),

    .result_o            ( alu_result      ),
    .comparison_result_o ( alu_cmp_result  ),

    .ready_o             ( alu_ready       ),
    .ex_ready_i          ( ex_ready_o      )
  );


  ////////////////////////////////////////////////////////////////
  //  __  __ _   _ _   _____ ___ ____  _     ___ _____ ____     //
  // |  \/  | | | | | |_   _|_ _|  _ \| |   |_ _| ____|  _ \    //
  // | |\/| | | | | |   | |  | || |_) | |    | ||  _| | |_) |   //
  // | |  | | |_| | |___| |  | ||  __/| |___ | || |___|  _ <    //
  // |_|  |_|\___/|_____|_| |___|_|   |_____|___|_____|_| \_\   //
  //                                                            //
  ////////////////////////////////////////////////////////////////

  riscv_mult mult_i
  (
    .clk             ( clk                  ),
    .rst_n           ( rst_n                ),

    .enable_i        ( mult_en_i            ),
    .operator_i      ( mult_operator_i      ),

    .short_subword_i ( mult_sel_subword_i   ),
    .short_signed_i  ( mult_signed_mode_i   ),

    .op_a_i          ( mult_operand_a_i     ),
    .op_b_i          ( mult_operand_b_i     ),
    .op_c_i          ( mult_operand_c_i     ),
    .imm_i           ( mult_imm_i           ),

    .dot_op_a_i      ( mult_dot_op_a_i      ),
    .dot_op_b_i      ( mult_dot_op_b_i      ),
    .dot_op_c_i      ( mult_dot_op_c_i      ),
    .dot_signed_i    ( mult_dot_signed_i    ),

    .result_o        ( mult_result          ),

    .multicycle_o    ( mult_multicycle_o    ),
    .ready_o         ( mult_ready           ),
    .ex_ready_i      ( ex_ready_o           )
  );


  ///////////////////////////////////////
  // EX/WB Pipeline Register           //
  ///////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
  begin : EX_WB_Pipeline_Register
    if (~rst_n)
    begin
      regfile_waddr_wb_o   <= '0;
      regfile_we_wb_o      <= 1'b0;
    end
    else
    begin
      if (ex_valid_o) // wb_ready_i is implied
      begin
        regfile_we_wb_o    <= regfile_we_i;
        if (regfile_we_i) begin
          regfile_waddr_wb_o <= regfile_waddr_i;
        end
      end else if (wb_ready_i) begin
        // we are ready for a new instruction, but there is none available,
        // so we just flush the current one out of the pipe
        regfile_we_wb_o    <= 1'b0;
      end
    end
  end

  // As valid always goes to the right and ready to the left, and we are able
  // to finish branches without going to the WB stage, ex_valid does not
  // depend on ex_ready.
  assign ex_ready_o = (alu_ready & mult_ready & lsu_ready_ex_i & wb_ready_i) | branch_in_ex_i;
  assign ex_valid_o = (alu_ready & mult_ready & lsu_ready_ex_i & wb_ready_i);

endmodule

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Matthias Baer - baermatt@student.ethz.ch                   //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Andreas Traber - atraber@student.ethz.ch                   //
//                                                                            //
// Design Name:    ALU                                                        //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Arithmetic logic unit of the pipelined processor           //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

import riscv_defines::*;

module riscv_alu
(
  input  logic                     clk,
  input  logic                     rst_n,

  input  logic [ALU_OP_WIDTH-1:0] operator_i,
  input  logic [31:0]              operand_a_i,
  input  logic [31:0]              operand_b_i,
  input  logic [31:0]              operand_c_i,

  input  logic [ 1:0]              vector_mode_i,
  input  logic [ 4:0]              bmask_a_i,
  input  logic [ 4:0]              bmask_b_i,
  input  logic [ 1:0]              imm_vec_ext_i,

  output logic [31:0]              result_o,
  output logic                     comparison_result_o,

  output logic                     ready_o,
  input  logic                     ex_ready_i
);


  logic [31:0] operand_a_rev;
  logic [31:0] operand_a_neg;
  logic [31:0] operand_a_neg_rev;

  assign operand_a_neg = ~operand_a_i;

  // bit reverse operand_a for left shifts and bit counting
  generate
    genvar k;
    for(k = 0; k < 32; k++)
    begin
      assign operand_a_rev[k] = operand_a_i[31-k];
    end
  endgenerate

  // bit reverse operand_a_neg for left shifts and bit counting
  generate
    genvar m;
    for(m = 0; m < 32; m++)
    begin
      assign operand_a_neg_rev[m] = operand_a_neg[31-m];
    end
  endgenerate

  logic [31:0] operand_b_neg;

  assign operand_b_neg = ~operand_b_i;


  logic [5:0]  div_shift;
  logic        div_valid;
  logic [31:0] bmask;

  //////////////////////////////////////////////////////////////////////////////////////////
  //   ____            _   _ _   _                      _      _       _     _            //
  //  |  _ \ __ _ _ __| |_(_) |_(_) ___  _ __   ___  __| |    / \   __| | __| | ___ _ __  //
  //  | |_) / _` | '__| __| | __| |/ _ \| '_ \ / _ \/ _` |   / _ \ / _` |/ _` |/ _ \ '__| //
  //  |  __/ (_| | |  | |_| | |_| | (_) | | | |  __/ (_| |  / ___ \ (_| | (_| |  __/ |    //
  //  |_|   \__,_|_|   \__|_|\__|_|\___/|_| |_|\___|\__,_| /_/   \_\__,_|\__,_|\___|_|    //
  //                                                                                      //
  //////////////////////////////////////////////////////////////////////////////////////////

  logic        adder_op_b_negate;
  logic [31:0] adder_op_a, adder_op_b;
  logic [35:0] adder_in_a, adder_in_b;
  logic [31:0] adder_result;
  logic [35:0] adder_result_expanded;

  assign adder_op_b_negate = (operator_i == ALU_SUB) || (operator_i == ALU_SUBR) ||
                             (operator_i == ALU_SUBU) || (operator_i == ALU_SUBR);

  // prepare operand a
  assign adder_op_a = (operator_i == ALU_ABS) ? operand_a_neg : operand_a_i;

  // prepare operand b
  assign adder_op_b = adder_op_b_negate ? operand_b_neg : operand_b_i;

  // prepare carry
  always_comb
  begin
    adder_in_a[    0] = 1'b1;
    adder_in_a[ 8: 1] = adder_op_a[ 7: 0];
    adder_in_a[    9] = 1'b1;
    adder_in_a[17:10] = adder_op_a[15: 8];
    adder_in_a[   18] = 1'b1;
    adder_in_a[26:19] = adder_op_a[23:16];
    adder_in_a[   27] = 1'b1;
    adder_in_a[35:28] = adder_op_a[31:24];

    adder_in_b[    0] = 1'b0;
    adder_in_b[ 8: 1] = adder_op_b[ 7: 0];
    adder_in_b[    9] = 1'b0;
    adder_in_b[17:10] = adder_op_b[15: 8];
    adder_in_b[   18] = 1'b0;
    adder_in_b[26:19] = adder_op_b[23:16];
    adder_in_b[   27] = 1'b0;
    adder_in_b[35:28] = adder_op_b[31:24];

    if (adder_op_b_negate || (operator_i == ALU_ABS)) begin
      // special case for subtractions and absolute number calculations
      adder_in_b[0] = 1'b1;

      case (vector_mode_i)
        VEC_MODE16: begin
          adder_in_b[18] = 1'b1;
        end

        VEC_MODE8: begin
          adder_in_b[ 9] = 1'b1;
          adder_in_b[18] = 1'b1;
          adder_in_b[27] = 1'b1;
        end
      endcase

    end else begin
      // take care of partitioning the adder for the addition case
      case (vector_mode_i)
        VEC_MODE16: begin
          adder_in_a[18] = 1'b0;
        end

        VEC_MODE8: begin
          adder_in_a[ 9] = 1'b0;
          adder_in_a[18] = 1'b0;
          adder_in_a[27] = 1'b0;
        end
      endcase
    end
  end

  // actual adder
  assign adder_result_expanded = adder_in_a + adder_in_b;
  assign adder_result = {adder_result_expanded[35:28],
                         adder_result_expanded[26:19],
                         adder_result_expanded[17:10],
                         adder_result_expanded[8:1]};


  // normalization stage
  logic [31:0] adder_round_value;
  logic [31:0] adder_round_result;

  assign adder_round_value  = ((operator_i == ALU_ADDR) || (operator_i == ALU_SUBR) ||
                               (operator_i == ALU_ADDUR) || (operator_i == ALU_SUBUR)) ?
                                {1'b0, bmask[31:1]} : '0;
  assign adder_round_result = adder_result + adder_round_value;


  ////////////////////////////////////////
  //  ____  _   _ ___ _____ _____       //
  // / ___|| | | |_ _|  ___|_   _|      //
  // \___ \| |_| || || |_    | |        //
  //  ___) |  _  || ||  _|   | |        //
  // |____/|_| |_|___|_|     |_|        //
  //                                    //
  ////////////////////////////////////////

  logic        shift_left;         // should we shift left
  logic        shift_use_round;
  logic        shift_arithmetic;

  logic [31:0] shift_amt_left;     // amount of shift, if to the left
  logic [31:0] shift_amt;          // amount of shift, to the right
  logic [31:0] shift_amt_int;      // amount of shift, used for the actual shifters
  logic [31:0] shift_amt_norm;     // amount of shift, used for normalization
  logic [31:0] shift_op_a;         // input of the shifter
  logic [31:0] shift_result;
  logic [31:0] shift_right_result;
  logic [31:0] shift_left_result;

  // shifter is also used for preparing operand for divison
  assign shift_amt = div_valid ? div_shift : operand_b_i;

  // by reversing the bits of the input, we also have to reverse the order of shift amounts
  always_comb
  begin
    case(vector_mode_i)
      VEC_MODE16:
      begin
        shift_amt_left[15: 0] = shift_amt[31:16];
        shift_amt_left[31:16] = shift_amt[15: 0];
      end

      VEC_MODE8:
      begin
        shift_amt_left[ 7: 0] = shift_amt[31:24];
        shift_amt_left[15: 8] = shift_amt[23:16];
        shift_amt_left[23:16] = shift_amt[15: 8];
        shift_amt_left[31:24] = shift_amt[ 7: 0];
      end

      default: // VEC_MODE32
      begin
        shift_amt_left[31: 0] = shift_amt[31: 0];
      end
    endcase
  end

  // ALU_FL1 and ALU_CBL are used for the bit counting ops later
  assign shift_left = (operator_i == ALU_SLL) || (operator_i == ALU_BINS) ||
                      (operator_i == ALU_FL1) || (operator_i == ALU_CLB) ||
                      (operator_i == ALU_DIV) || (operator_i == ALU_DIVU) ||
                      (operator_i == ALU_REM) || (operator_i == ALU_REMU);

  assign shift_use_round = (operator_i == ALU_ADD)   || (operator_i == ALU_SUB)   ||
                           (operator_i == ALU_ADDR)  || (operator_i == ALU_SUBR)  ||
                           (operator_i == ALU_ADDU)  || (operator_i == ALU_SUBU)  ||
                           (operator_i == ALU_ADDUR) || (operator_i == ALU_SUBUR);

  assign shift_arithmetic = (operator_i == ALU_SRA)  ||
                            (operator_i == ALU_ADD)  || (operator_i == ALU_SUB)  ||
                            (operator_i == ALU_ADDR) || (operator_i == ALU_SUBR);

  // choose the bit reversed or the normal input for shift operand a
  assign shift_op_a    = shift_left ? operand_a_rev :
                          (shift_use_round ? adder_round_result : operand_a_i);
  assign shift_amt_int = shift_use_round ? shift_amt_norm :
                          (shift_left ? shift_amt_left : shift_amt);

  assign shift_amt_norm = {4{3'b000, bmask_b_i}};


  // right shifts, we let the synthesizer optimize this
  always_comb
  begin
    case(vector_mode_i)
      VEC_MODE16:
      begin
        if(shift_arithmetic)
        begin
          shift_right_result[31:16] = $unsigned( $signed(shift_op_a[31:16]) >>> shift_amt_int[19:16] );
          shift_right_result[15: 0] = $unsigned( $signed(shift_op_a[15: 0]) >>> shift_amt_int[ 3: 0] );
        end
        else
        begin
          shift_right_result[31:16] = shift_op_a[31:16]  >> shift_amt_int[19:16];
          shift_right_result[15: 0] = shift_op_a[15: 0]  >> shift_amt_int[ 3: 0];
        end
      end

      VEC_MODE8:
      begin
        if(shift_arithmetic)
        begin
          shift_right_result[31:24] = $unsigned( $signed(shift_op_a[31:24]) >>> shift_amt_int[26:24] );
          shift_right_result[23:16] = $unsigned( $signed(shift_op_a[23:16]) >>> shift_amt_int[18:16] );
          shift_right_result[15: 8] = $unsigned( $signed(shift_op_a[15: 8]) >>> shift_amt_int[10: 8] );
          shift_right_result[ 7: 0] = $unsigned( $signed(shift_op_a[ 7: 0]) >>> shift_amt_int[ 2: 0] );
        end
        else
        begin
          shift_right_result[31:24] = shift_op_a[31:24]  >> shift_amt_int[26:24];
          shift_right_result[23:16] = shift_op_a[23:16]  >> shift_amt_int[18:16];
          shift_right_result[15: 8] = shift_op_a[15: 8]  >> shift_amt_int[10: 8];
          shift_right_result[ 7: 0] = shift_op_a[ 7: 0]  >> shift_amt_int[ 2: 0];
        end
      end

      default: // VEC_MODE32
      begin
        if(shift_arithmetic)
          shift_right_result = $unsigned( $signed(shift_op_a) >>> shift_amt_int[4:0] );
        else if(operator_i == ALU_ROR)
          shift_right_result = {shift_op_a, shift_op_a}       >>  shift_amt_int[4:0];
        else
          shift_right_result = shift_op_a                     >>  shift_amt_int[4:0];
      end
    endcase; // case (vec_mode_i)
  end

  // bit reverse the shift_right_result for left shifts
  genvar       j;
  generate
    for(j = 0; j < 32; j++)
    begin
      assign shift_left_result[j] = shift_right_result[31-j];
    end
  endgenerate

  assign shift_result = shift_left ? shift_left_result : shift_right_result;


  //////////////////////////////////////////////////////////////////
  //   ____ ___  __  __ ____   _    ____  ___ ____   ___  _   _   //
  //  / ___/ _ \|  \/  |  _ \ / \  |  _ \|_ _/ ___| / _ \| \ | |  //
  // | |  | | | | |\/| | |_) / _ \ | |_) || |\___ \| | | |  \| |  //
  // | |__| |_| | |  | |  __/ ___ \|  _ < | | ___) | |_| | |\  |  //
  //  \____\___/|_|  |_|_| /_/   \_\_| \_\___|____/ \___/|_| \_|  //
  //                                                              //
  //////////////////////////////////////////////////////////////////

  logic [3:0] is_equal;
  logic [3:0] is_greater;  // handles both signed and unsigned forms

  // 8-bit vector comparisons, basic building blocks
  logic [3:0] cmp_signed;
  logic [3:0] is_equal_vec;
  logic [3:0] is_greater_vec;

  always_comb
  begin
    cmp_signed = 4'b0;

    unique case (operator_i)
      ALU_GTS,
      ALU_GES,
      ALU_LTS,
      ALU_LES,
      ALU_SLTS,
      ALU_SLETS,
      ALU_MIN,
      ALU_MAX,
      ALU_ABS,
      ALU_CLIP,
      ALU_CLIPU: begin
        case (vector_mode_i)
          VEC_MODE8:  cmp_signed[3:0] = 4'b1111;
          VEC_MODE16: cmp_signed[3:0] = 4'b1010;
          default:     cmp_signed[3:0] = 4'b1000;
        endcase
      end

      default:;
    endcase
  end

  // generate vector equal and greater than signals, cmp_signed decides if the
  // comparison is done signed or unsigned
  genvar i;
  generate
    for(i = 0; i < 4; i++)
    begin
      assign is_equal_vec[i]   = (operand_a_i[8*i+7:8*i] == operand_b_i[8*i+7:i*8]);
      assign is_greater_vec[i] = $signed({operand_a_i[8*i+7] & cmp_signed[i], operand_a_i[8*i+7:8*i]})
                                  >
                                 $signed({operand_b_i[8*i+7] & cmp_signed[i], operand_b_i[8*i+7:i*8]});
    end
  endgenerate

  // generate the real equal and greater than signals that take the vector
  // mode into account
  always_comb
  begin
    // 32-bit mode
    is_equal[3:0]   = {4{is_equal_vec[3] & is_equal_vec[2] & is_equal_vec[1] & is_equal_vec[0]}};
    is_greater[3:0] = {4{is_greater_vec[3] | (is_equal_vec[3] & (is_greater_vec[2]
                                            | (is_equal_vec[2] & (is_greater_vec[1]
                                             | (is_equal_vec[1] & (is_greater_vec[0]))))))}};

    case(vector_mode_i)
      VEC_MODE16:
      begin
        is_equal[1:0]   = {2{is_equal_vec[0]   & is_equal_vec[1]}};
        is_equal[3:2]   = {2{is_equal_vec[2]   & is_equal_vec[3]}};
        is_greater[1:0] = {2{is_greater_vec[1] | (is_equal_vec[1] & is_greater_vec[0])}};
        is_greater[3:2] = {2{is_greater_vec[3] | (is_equal_vec[3] & is_greater_vec[2])}};
      end

      VEC_MODE8:
      begin
        is_equal[3:0]   = is_equal_vec[3:0];
        is_greater[3:0] = is_greater_vec[3:0];
      end

      default:; // see default assignment
    endcase
  end

  // generate comparison result
  logic [3:0] cmp_result;

  always_comb
  begin
    cmp_result = is_equal;

    unique case (operator_i)
      ALU_EQ:            cmp_result = is_equal;
      ALU_NE:            cmp_result = ~is_equal;
      ALU_GTS, ALU_GTU:  cmp_result = is_greater;
      ALU_GES, ALU_GEU:  cmp_result = is_greater | is_equal;
      ALU_LTS, ALU_SLTS,
      ALU_LTU, ALU_SLTU: cmp_result = ~(is_greater | is_equal);
      ALU_SLETS,
      ALU_SLETU,
      ALU_LES, ALU_LEU:  cmp_result = ~is_greater;

      default: ;
    endcase
  end

  assign comparison_result_o = cmp_result[3];


  // min/max/abs handling
  logic [31:0] result_minmax;
  logic [ 3:0] sel_minmax;
  logic        do_min;
  logic [31:0] minmax_b;

  assign minmax_b = (operator_i == ALU_ABS) ? adder_result : operand_b_i;

  assign do_min   = (operator_i == ALU_MIN)  || (operator_i == ALU_MINU) ||
                    (operator_i == ALU_CLIP) || (operator_i == ALU_CLIPU);

  assign sel_minmax[3:0]      = is_greater ^ {4{do_min}};

  assign result_minmax[31:24] = (sel_minmax[3] == 1'b1) ? operand_a_i[31:24] : minmax_b[31:24];
  assign result_minmax[23:16] = (sel_minmax[2] == 1'b1) ? operand_a_i[23:16] : minmax_b[23:16];
  assign result_minmax[15: 8] = (sel_minmax[1] == 1'b1) ? operand_a_i[15: 8] : minmax_b[15: 8];
  assign result_minmax[ 7: 0] = (sel_minmax[0] == 1'b1) ? operand_a_i[ 7: 0] : minmax_b[ 7: 0];


  // clip
  logic [31:0] clip_result;        // result of clip and clip
  logic        clip_is_lower_neg;  // only signed comparison; used for clip
  logic        clip_is_lower_u;    // only signed comparison; used for clipu, checks for negative number

  assign clip_is_lower_neg = $signed(operand_a_i) < $signed(operand_b_neg);
  assign clip_is_lower_u   = (operator_i == ALU_CLIPU) && operand_a_i[31];

  assign clip_result       = clip_is_lower_u ? '0 : (clip_is_lower_neg ? operand_b_neg : result_minmax);

  //////////////////////////////////////////////////
  //  ____  _   _ _   _ _____ _____ _     _____   //
  // / ___|| | | | | | |  ___|  ___| |   | ____|  //
  // \___ \| |_| | | | | |_  | |_  | |   |  _|    //
  //  ___) |  _  | |_| |  _| |  _| | |___| |___   //
  // |____/|_| |_|\___/|_|   |_|   |_____|_____|  //
  //                                              //
  //////////////////////////////////////////////////

  logic [ 3: 0][1:0] shuffle_byte_sel; // select byte in register: 31:24, 23:16, 15:8, 7:0
  logic [ 3: 0]      shuffle_reg_sel;  // select register: rD/rS2 or rS1
  logic [ 1: 0]      shuffle_reg1_sel; // select register rD or rS2 for next stage
  logic [ 1: 0]      shuffle_reg0_sel;
  logic [ 3: 0]      shuffle_through;

  logic [31: 0]      shuffle_r1, shuffle_r0;
  logic [31: 0]      shuffle_r1_in, shuffle_r0_in;
  logic [31: 0]      shuffle_result;
  logic [31: 0]      pack_result;


  always_comb
  begin
    shuffle_reg_sel  = '0;
    shuffle_reg1_sel = 2'b01;
    shuffle_reg0_sel = 2'b10;
    shuffle_through  = '1;

    unique case(operator_i)
      ALU_EXT, ALU_EXTS: begin
        if (operator_i == ALU_EXTS)
          shuffle_reg1_sel = 2'b11;

        if (vector_mode_i == VEC_MODE8) begin
          shuffle_reg_sel[3:1] = 3'b111;
          shuffle_reg_sel[0]   = 1'b0;
        end else begin
          shuffle_reg_sel[3:2] = 2'b11;
          shuffle_reg_sel[1:0] = 2'b00;
        end
      end

      ALU_PCKLO: begin
        shuffle_reg1_sel = 2'b00;

        if (vector_mode_i == VEC_MODE8) begin
          shuffle_through = 4'b0011;
          shuffle_reg_sel = 4'b0001;
        end else begin
          shuffle_reg_sel = 4'b0011;
        end
      end

      ALU_PCKHI: begin
        shuffle_reg1_sel = 2'b00;

        shuffle_reg_sel = 4'b0100;
        shuffle_through = 4'b1100;
      end

      ALU_SHUF2: begin
        unique case (vector_mode_i)
          VEC_MODE8: begin
            shuffle_reg_sel[3] = ~operand_b_i[26];
            shuffle_reg_sel[2] = ~operand_b_i[18];
            shuffle_reg_sel[1] = ~operand_b_i[10];
            shuffle_reg_sel[0] = ~operand_b_i[ 2];
          end

          VEC_MODE16: begin
            shuffle_reg_sel[3] = ~operand_b_i[17];
            shuffle_reg_sel[2] = ~operand_b_i[17];
            shuffle_reg_sel[1] = ~operand_b_i[ 1];
            shuffle_reg_sel[0] = ~operand_b_i[ 1];
          end
          default:;
        endcase
      end

      ALU_INS: begin
        unique case (vector_mode_i)
          VEC_MODE8: begin
            shuffle_reg0_sel = 2'b00;
            unique case (imm_vec_ext_i)
              2'b00: begin
                shuffle_reg_sel[3:0] = 4'b1110;
              end
              2'b01: begin
                shuffle_reg_sel[3:0] = 4'b1101;
              end
              2'b10: begin
                shuffle_reg_sel[3:0] = 4'b1011;
              end
              2'b11: begin
                shuffle_reg_sel[3:0] = 4'b0111;
              end
              default:;
            endcase
          end
          VEC_MODE16: begin
            shuffle_reg0_sel = 2'b01;
            shuffle_reg_sel[3] = ~imm_vec_ext_i[ 0];
            shuffle_reg_sel[2] = ~imm_vec_ext_i[ 0];
            shuffle_reg_sel[1] =  imm_vec_ext_i[ 0];
            shuffle_reg_sel[0] =  imm_vec_ext_i[ 0];
          end
          default:;
        endcase
      end

      default:;
    endcase
  end

  always_comb
  begin
    shuffle_byte_sel = 'x;

    // byte selector
    unique case (operator_i)
      ALU_EXTS,
      ALU_EXT: begin
        unique case (vector_mode_i)
          VEC_MODE8: begin
            shuffle_byte_sel[3] = imm_vec_ext_i[1:0];
            shuffle_byte_sel[2] = imm_vec_ext_i[1:0];
            shuffle_byte_sel[1] = imm_vec_ext_i[1:0];
            shuffle_byte_sel[0] = imm_vec_ext_i[1:0];
          end

          VEC_MODE16: begin
            shuffle_byte_sel[3] = {imm_vec_ext_i[0], 1'b1};
            shuffle_byte_sel[2] = {imm_vec_ext_i[0], 1'b1};
            shuffle_byte_sel[1] = {imm_vec_ext_i[0], 1'b1};
            shuffle_byte_sel[0] = {imm_vec_ext_i[0], 1'b0};
          end

          default:;
        endcase
      end

      ALU_PCKLO,
      ALU_PCKHI: begin
        unique case (vector_mode_i)
          VEC_MODE8: begin
            shuffle_byte_sel[3] = 2'b00;
            shuffle_byte_sel[2] = 2'b00;
            shuffle_byte_sel[1] = 2'b00;
            shuffle_byte_sel[0] = 2'b00;
          end

          VEC_MODE16: begin
            shuffle_byte_sel[3] = 2'b01;
            shuffle_byte_sel[2] = 2'b00;
            shuffle_byte_sel[1] = 2'b01;
            shuffle_byte_sel[0] = 2'b00;
          end

          default:;
        endcase
      end

      ALU_SHUF2,
      ALU_SHUF: begin
        unique case (vector_mode_i)
          VEC_MODE8: begin
            shuffle_byte_sel[3] = operand_b_i[25:24];
            shuffle_byte_sel[2] = operand_b_i[17:16];
            shuffle_byte_sel[1] = operand_b_i[ 9: 8];
            shuffle_byte_sel[0] = operand_b_i[ 1: 0];
          end

          VEC_MODE16: begin
            shuffle_byte_sel[3] = {operand_b_i[16], 1'b1};
            shuffle_byte_sel[2] = {operand_b_i[16], 1'b0};
            shuffle_byte_sel[1] = {operand_b_i[ 0], 1'b1};
            shuffle_byte_sel[0] = {operand_b_i[ 0], 1'b0};
          end
          default:;
        endcase
      end

      ALU_INS: begin
        shuffle_byte_sel[3] = 2'b11;
        shuffle_byte_sel[2] = 2'b10;
        shuffle_byte_sel[1] = 2'b01;
        shuffle_byte_sel[0] = 2'b00;
      end

      default:;
    endcase
  end

  assign shuffle_r0_in = shuffle_reg0_sel[1] ?
                          operand_a_i :
                          (shuffle_reg0_sel[0] ? {2{operand_a_i[15:0]}} : {4{operand_a_i[7:0]}});

  assign shuffle_r1_in = shuffle_reg1_sel[1] ?
                                 {{8{operand_a_i[31]}}, {8{operand_a_i[23]}}, {8{operand_a_i[15]}}, {8{operand_a_i[7]}}} :
                                 (shuffle_reg1_sel[0] ? operand_c_i : operand_b_i);

  assign shuffle_r0[31:24] = shuffle_byte_sel[3][1] ?
                              (shuffle_byte_sel[3][0] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) :
                              (shuffle_byte_sel[3][0] ? shuffle_r0_in[15: 8] : shuffle_r0_in[ 7: 0]);
  assign shuffle_r0[23:16] = shuffle_byte_sel[2][1] ?
                              (shuffle_byte_sel[2][0] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) :
                              (shuffle_byte_sel[2][0] ? shuffle_r0_in[15: 8] : shuffle_r0_in[ 7: 0]);
  assign shuffle_r0[15: 8] = shuffle_byte_sel[1][1] ?
                              (shuffle_byte_sel[1][0] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) :
                              (shuffle_byte_sel[1][0] ? shuffle_r0_in[15: 8] : shuffle_r0_in[ 7: 0]);
  assign shuffle_r0[ 7: 0] = shuffle_byte_sel[0][1] ?
                              (shuffle_byte_sel[0][0] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) :
                              (shuffle_byte_sel[0][0] ? shuffle_r0_in[15: 8] : shuffle_r0_in[ 7: 0]);

  assign shuffle_r1[31:24] = shuffle_byte_sel[3][1] ?
                              (shuffle_byte_sel[3][0] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) :
                              (shuffle_byte_sel[3][0] ? shuffle_r1_in[15: 8] : shuffle_r1_in[ 7: 0]);
  assign shuffle_r1[23:16] = shuffle_byte_sel[2][1] ?
                              (shuffle_byte_sel[2][0] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) :
                              (shuffle_byte_sel[2][0] ? shuffle_r1_in[15: 8] : shuffle_r1_in[ 7: 0]);
  assign shuffle_r1[15: 8] = shuffle_byte_sel[1][1] ?
                              (shuffle_byte_sel[1][0] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) :
                              (shuffle_byte_sel[1][0] ? shuffle_r1_in[15: 8] : shuffle_r1_in[ 7: 0]);
  assign shuffle_r1[ 7: 0] = shuffle_byte_sel[0][1] ?
                              (shuffle_byte_sel[0][0] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) :
                              (shuffle_byte_sel[0][0] ? shuffle_r1_in[15: 8] : shuffle_r1_in[ 7: 0]);

  assign shuffle_result[31:24] = shuffle_reg_sel[3] ? shuffle_r1[31:24] : shuffle_r0[31:24];
  assign shuffle_result[23:16] = shuffle_reg_sel[2] ? shuffle_r1[23:16] : shuffle_r0[23:16];
  assign shuffle_result[15: 8] = shuffle_reg_sel[1] ? shuffle_r1[15: 8] : shuffle_r0[15: 8];
  assign shuffle_result[ 7: 0] = shuffle_reg_sel[0] ? shuffle_r1[ 7: 0] : shuffle_r0[ 7: 0];

  assign pack_result[31:24] = shuffle_through[3] ? shuffle_result[31:24] : operand_c_i[31:24];
  assign pack_result[23:16] = shuffle_through[2] ? shuffle_result[23:16] : operand_c_i[23:16];
  assign pack_result[15: 8] = shuffle_through[1] ? shuffle_result[15: 8] : operand_c_i[15: 8];
  assign pack_result[ 7: 0] = shuffle_through[0] ? shuffle_result[ 7: 0] : operand_c_i[ 7: 0];


  /////////////////////////////////////////////////////////////////////
  //   ____  _ _      ____                  _      ___               //
  //  | __ )(_) |_   / ___|___  _   _ _ __ | |_   / _ \ _ __  ___    //
  //  |  _ \| | __| | |   / _ \| | | | '_ \| __| | | | | '_ \/ __|   //
  //  | |_) | | |_  | |__| (_) | |_| | | | | |_  | |_| | |_) \__ \_  //
  //  |____/|_|\__|  \____\___/ \__,_|_| |_|\__|  \___/| .__/|___(_) //
  //                                                   |_|           //
  /////////////////////////////////////////////////////////////////////

  logic [31:0] ff_input;   // either op_a_i or its bit reversed version
  logic [5:0]  cnt_result; // population count
  logic [5:0]  clb_result; // count leading bits
  logic [4:0]  ff1_result; // holds the index of the first '1'
  logic        ff_no_one;  // if no ones are found
  logic [4:0]  fl1_result; // holds the index of the last '1'
  logic [5:0]  bitop_result; // result of all bitop operations muxed together

  alu_popcnt alu_popcnt_i
  (
    .in_i        ( operand_a_i ),
    .result_o    ( cnt_result  )
  );

  always_comb
  begin
    ff_input = 'x;

    case (operator_i)
      ALU_FF1: ff_input = operand_a_i;

      ALU_DIVU,
      ALU_REMU,
      ALU_FL1: ff_input = operand_a_rev;

      ALU_DIV,
      ALU_REM,
      ALU_CLB: begin
        if (operand_a_i[31])
          ff_input = operand_a_neg_rev;
        else
          ff_input = operand_a_rev;
      end
    endcase
  end

  alu_ff alu_ff_i
  (
    .in_i        ( ff_input   ),
    .first_one_o ( ff1_result ),
    .no_ones_o   ( ff_no_one  )
  );

  // special case if ff1_res is 0 (no 1 found), then we keep the 0
  // this is done in the result mux
  assign fl1_result  = 5'd31 - ff1_result;
  assign clb_result  = ff1_result - 5'd1;

  always_comb
  begin
    bitop_result = 'x;
    case (operator_i)
      ALU_FF1: bitop_result = ff_no_one ? 6'd32 : {1'b0, ff1_result};
      ALU_FL1: bitop_result = ff_no_one ? 6'd32 : {1'b0, fl1_result};
      ALU_CNT: bitop_result = cnt_result;
      ALU_CLB: begin
        if (ff_no_one) begin
          if (operand_a_i[31])
            bitop_result = 6'd31;
          else
            bitop_result = '0;
        end else begin
          bitop_result = clb_result;
        end
      end
      default:;
    endcase
  end


  ////////////////////////////////////////////////
  //  ____  _ _     __  __             _        //
  // | __ )(_) |_  |  \/  | __ _ _ __ (_)_ __   //
  // |  _ \| | __| | |\/| |/ _` | '_ \| | '_ \  //
  // | |_) | | |_  | |  | | (_| | | | | | |_) | //
  // |____/|_|\__| |_|  |_|\__,_|_| |_|_| .__/  //
  //                                    |_|     //
  ////////////////////////////////////////////////

  logic        extract_is_signed;
  logic        extract_sign;
  logic [31:0] bmask_first, bmask_inv;
  logic [31:0] bextins_and;
  logic [31:0] bextins_result, bclr_result, bset_result;


  // construct bit mask for insert/extract/bclr/bset
  // bmask looks like this 00..0011..1100..00
  assign bmask_first = {32'hFFFFFFFE} << bmask_a_i;
  assign bmask       = (~bmask_first) << bmask_b_i;
  assign bmask_inv   = ~bmask;

  assign bextins_and = (operator_i == ALU_BINS) ? operand_c_i : {32{extract_sign}};

  assign extract_is_signed = (operator_i == ALU_BEXT);
  assign extract_sign = extract_is_signed & shift_result[bmask_a_i];

  assign bextins_result = (bmask & shift_result) | (bextins_and & bmask_inv);

  assign bclr_result = operand_a_i & bmask_inv;
  assign bset_result = operand_a_i | bmask;

  ////////////////////////////////////////////////////
  //  ____ _____     __     __  ____  _____ __  __  //
  // |  _ \_ _\ \   / /    / / |  _ \| ____|  \/  | //
  // | | | | | \ \ / /    / /  | |_) |  _| | |\/| | //
  // | |_| | |  \ V /    / /   |  _ <| |___| |  | | //
  // |____/___|  \_/    /_/    |_| \_\_____|_|  |_| //
  //                                                //
  ////////////////////////////////////////////////////

  logic [31:0] result_div;

  logic        div_ready;
  logic        div_signed;
  logic        div_op_a_signed;
  logic        div_op_b_signed;
  logic [5:0]  div_shift_int;

  assign div_signed = operator_i[0];

  assign div_op_a_signed = operand_a_i[31] & div_signed;
  assign div_op_b_signed = operand_b_i[31] & div_signed;

  assign div_shift_int = ff_no_one ? 6'd31 : clb_result;
  assign div_shift = div_shift_int + (div_op_a_signed ? 6'd0 : 6'd1);

  assign div_valid = (operator_i == ALU_DIV) || (operator_i == ALU_DIVU) ||
                     (operator_i == ALU_REM) || (operator_i == ALU_REMU);


  // inputs A and B are swapped
  riscv_alu_div div_i
  (
    .Clk_CI       ( clk               ),
    .Rst_RBI      ( rst_n             ),

    // input IF
    .OpA_DI       ( operand_b_i       ),
    .OpB_DI       ( shift_left_result ),
    .OpBShift_DI  ( div_shift         ),
    .OpBIsZero_SI ( (cnt_result == 0) ),

    .OpBSign_SI   ( div_op_a_signed   ),
    .OpCode_SI    ( operator_i[1:0]   ),

    .Res_DO       ( result_div        ),

    // Hand-Shake
    .InVld_SI     ( div_valid         ),
    .OutRdy_SI    ( ex_ready_i        ),
    .OutVld_SO    ( div_ready         )
  );

  ////////////////////////////////////////////////////////
  //   ____                 _ _     __  __              //
  //  |  _ \ ___  ___ _   _| | |_  |  \/  |_   ___  __  //
  //  | |_) / _ \/ __| | | | | __| | |\/| | | | \ \/ /  //
  //  |  _ <  __/\__ \ |_| | | |_  | |  | | |_| |>  <   //
  //  |_| \_\___||___/\__,_|_|\__| |_|  |_|\__,_/_/\_\  //
  //                                                    //
  ////////////////////////////////////////////////////////

  always_comb
  begin
    result_o   = 'x;

    unique case (operator_i)
      // Standard Operations
      ALU_AND:  result_o = operand_a_i & operand_b_i;
      ALU_OR:   result_o = operand_a_i | operand_b_i;
      ALU_XOR:  result_o = operand_a_i ^ operand_b_i;

      // Shift Operations
      ALU_ADD, ALU_ADDR, ALU_ADDU, ALU_ADDUR,
      ALU_SUB, ALU_SUBR, ALU_SUBU, ALU_SUBUR,
      ALU_SLL,
      ALU_SRL, ALU_SRA,
      ALU_ROR:  result_o = shift_result;

      // bit manipulation instructions
      ALU_BINS,
      ALU_BEXT,
      ALU_BEXTU: result_o = bextins_result;

      ALU_BCLR:  result_o = bclr_result;
      ALU_BSET:  result_o = bset_result;

      // pack and shuffle operations
      ALU_SHUF,  ALU_SHUF2,
      ALU_PCKLO, ALU_PCKHI,
      ALU_EXT,   ALU_EXTS,
      ALU_INS: result_o = pack_result;

      // Min/Max/Abs/Ins
      ALU_MIN, ALU_MINU,
      ALU_MAX, ALU_MAXU,
      ALU_ABS: result_o = result_minmax;

      ALU_CLIP, ALU_CLIPU: result_o = clip_result;

      // Comparison Operations
      ALU_EQ,    ALU_NE,
      ALU_GTU,   ALU_GEU,
      ALU_LTU,   ALU_LEU,
      ALU_GTS,   ALU_GES,
      ALU_LTS,   ALU_LES: begin
          result_o[31:24] = {8{cmp_result[3]}};
          result_o[23:16] = {8{cmp_result[2]}};
          result_o[15: 8] = {8{cmp_result[1]}};
          result_o[ 7: 0] = {8{cmp_result[0]}};
       end
      ALU_SLTS,  ALU_SLTU,
      ALU_SLETS, ALU_SLETU: result_o = {31'b0, comparison_result_o};

      ALU_FF1, ALU_FL1, ALU_CLB, ALU_CNT: result_o = {26'h0, bitop_result[5:0]};

      // Division Unit Commands
      ALU_DIV, ALU_DIVU,
      ALU_REM, ALU_REMU: result_o = result_div;

      default: ; // default case to suppress unique warning
    endcase
  end

  assign ready_o = div_ready;

endmodule

module alu_ff
#(
  parameter LEN = 32
)
(
  input  logic [LEN-1:0]         in_i,

  output logic [$clog2(LEN)-1:0] first_one_o,
  output logic                   no_ones_o
);

  localparam NUM_LEVELS = $clog2(LEN);

  logic [LEN-1:0] [NUM_LEVELS-1:0]           index_lut;
  logic [2**NUM_LEVELS-1:0]                  sel_nodes;
  logic [2**NUM_LEVELS-1:0] [NUM_LEVELS-1:0] index_nodes;


  //////////////////////////////////////////////////////////////////////////////
  // generate tree structure
  //////////////////////////////////////////////////////////////////////////////

  generate
    genvar j;
    for (j = 0; j < LEN; j++) begin
      assign index_lut[j] = $unsigned(j);
    end
  endgenerate

  generate
    genvar k;
    genvar l;
    genvar level;
    for (level = 0; level < NUM_LEVELS; level++) begin
    //------------------------------------------------------------
    if (level < NUM_LEVELS-1) begin
      for (l = 0; l < 2**level; l++) begin
        assign sel_nodes[2**level-1+l]   = sel_nodes[2**(level+1)-1+l*2] | sel_nodes[2**(level+1)-1+l*2+1];
        assign index_nodes[2**level-1+l] = (sel_nodes[2**(level+1)-1+l*2] == 1'b1) ?
                                           index_nodes[2**(level+1)-1+l*2] : index_nodes[2**(level+1)-1+l*2+1];
      end
    end
    //------------------------------------------------------------
    if (level == NUM_LEVELS-1) begin
      for (k = 0; k < 2**level; k++) begin
        // if two successive indices are still in the vector...
        if (k * 2 < LEN) begin
          assign sel_nodes[2**level-1+k]   = in_i[k*2] | in_i[k*2+1];
          assign index_nodes[2**level-1+k] = (in_i[k*2] == 1'b1) ? index_lut[k*2] : index_lut[k*2+1];
        end
        // if only the first index is still in the vector...
        if (k * 2 == LEN) begin
          assign sel_nodes[2**level-1+k]   = in_i[k*2];
          assign index_nodes[2**level-1+k] = index_lut[k*2];
        end
        // if index is out of range
        if (k * 2 > LEN) begin
          assign sel_nodes[2**level-1+k]   = 1'b0;
          assign index_nodes[2**level-1+k] = '0;
        end
      end
    end
    //------------------------------------------------------------
    end
  endgenerate

  //////////////////////////////////////////////////////////////////////////////
  // connect output
  //////////////////////////////////////////////////////////////////////////////

  assign first_one_o = index_nodes[0];
  assign no_ones_o   = ~sel_nodes[0];

endmodule

// count the number of '1's in a word
module alu_popcnt
(
  input  logic [31:0]  in_i,
  output logic [5: 0]  result_o
);

  logic [15:0][1:0] cnt_l1;
  logic [ 7:0][2:0] cnt_l2;
  logic [ 3:0][3:0] cnt_l3;
  logic [ 1:0][4:0] cnt_l4;

  genvar      l, m, n, p;
  generate for(l = 0; l < 16; l++)
    begin
      assign cnt_l1[l] = {1'b0, in_i[2*l]} + {1'b0, in_i[2*l + 1]};
    end
  endgenerate

  generate for(m = 0; m < 8; m++)
    begin
      assign cnt_l2[m] = {1'b0, cnt_l1[2*m]} + {1'b0, cnt_l1[2*m + 1]};
    end
  endgenerate

  generate for(n = 0; n < 4; n++)
    begin
      assign cnt_l3[n] = {1'b0, cnt_l2[2*n]} + {1'b0, cnt_l2[2*n + 1]};
    end
  endgenerate

  generate for(p = 0; p < 2; p++)
    begin
      assign cnt_l4[p] = {1'b0, cnt_l3[2*p]} + {1'b0, cnt_l3[2*p + 1]};
    end
  endgenerate

  assign result_o = {1'b0, cnt_l4[0]} + {1'b0, cnt_l4[1]};

endmodule

// Copyright 2016 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

///////////////////////////////////////////////////////////////////////////////
// File       : Simple Serial Divider
// Ver        : 1.0
// Date       : 15.03.2016
///////////////////////////////////////////////////////////////////////////////
//
// Description: this is a simple serial divider for signed integers (int32).
//
///////////////////////////////////////////////////////////////////////////////
//
// Authors    : Michael Schaffner (schaffner@iis.ee.ethz.ch)
//              Andreas Traber    (atraber@iis.ee.ethz.ch)
//
///////////////////////////////////////////////////////////////////////////////

module riscv_alu_div
#(
   parameter C_WIDTH     = 32,
   parameter C_LOG_WIDTH = 6
)
(
    input  logic                    Clk_CI,
    input  logic                    Rst_RBI,
    // input IF
    input  logic [C_WIDTH-1:0]      OpA_DI,
    input  logic [C_WIDTH-1:0]      OpB_DI,
    input  logic [C_LOG_WIDTH-1:0]  OpBShift_DI,
    input  logic                    OpBIsZero_SI,
    //
    input  logic                    OpBSign_SI, // gate this to 0 in case of unsigned ops
    input  logic [1:0]              OpCode_SI,  // 0: udiv, 2: urem, 1: div, 3: rem
    // handshake
    input  logic                    InVld_SI,
    // output IF
    input  logic                    OutRdy_SI,
    output logic                    OutVld_SO,
    output logic [C_WIDTH-1:0]      Res_DO
  );

  ///////////////////////////////////////////////////////////////////////////////
  // signal declarations
  ///////////////////////////////////////////////////////////////////////////////

  logic [C_WIDTH-1:0] ResReg_DP, ResReg_DN;
  logic [C_WIDTH-1:0] ResReg_DP_rev;
  logic [C_WIDTH-1:0] AReg_DP, AReg_DN;
  logic [C_WIDTH-1:0] BReg_DP, BReg_DN;

  logic RemSel_SN, RemSel_SP;
  logic CompInv_SN, CompInv_SP;
  logic ResInv_SN, ResInv_SP;

  logic [C_WIDTH-1:0] AddMux_D;
  logic [C_WIDTH-1:0] AddOut_D;
  logic [C_WIDTH-1:0] AddTmp_D;
  logic [C_WIDTH-1:0] BMux_D;
  logic [C_WIDTH-1:0] OutMux_D;

  logic [C_LOG_WIDTH-1:0] Cnt_DP, Cnt_DN;
  logic CntZero_S;

  logic ARegEn_S, BRegEn_S, ResRegEn_S, ABComp_S, PmSel_S, LoadEn_S;

  enum logic [1:0] {IDLE, DIVIDE, FINISH} State_SN, State_SP;


  ///////////////////////////////////////////////////////////////////////////////
  // datapath
  ///////////////////////////////////////////////////////////////////////////////

  assign PmSel_S     = LoadEn_S & ~(OpCode_SI[0] & (OpA_DI[$high(OpA_DI)] ^ OpBSign_SI));

  // muxes
  assign AddMux_D    = (LoadEn_S) ? OpA_DI  : BReg_DP;

  // attention: logical shift in case of negative operand B!
  assign BMux_D      = (LoadEn_S) ? OpB_DI : {CompInv_SP, (BReg_DP[$high(BReg_DP):1])};

  assign ResReg_DP_rev = {<<{ResReg_DP}};
  assign OutMux_D    = (RemSel_SP) ? AReg_DP : ResReg_DP_rev;

  // invert if necessary
  assign Res_DO      = (ResInv_SP) ? -$signed(OutMux_D) : OutMux_D;

  // main comparator
  assign ABComp_S    = ((AReg_DP == BReg_DP) | ((AReg_DP > BReg_DP) ^ CompInv_SP)) & ((|AReg_DP) | OpBIsZero_SI);

  // main adder
  assign AddTmp_D    = (LoadEn_S) ? 0 : AReg_DP;
  assign AddOut_D    = (PmSel_S)  ? AddTmp_D + AddMux_D : AddTmp_D - $signed(AddMux_D);

  ///////////////////////////////////////////////////////////////////////////////
  // counter
  ///////////////////////////////////////////////////////////////////////////////

  assign Cnt_DN      = (LoadEn_S)   ? OpBShift_DI :
                       (~CntZero_S) ? Cnt_DP - 1  : Cnt_DP;

  assign CntZero_S   = ~(|Cnt_DP);

  ///////////////////////////////////////////////////////////////////////////////
  // FSM
  ///////////////////////////////////////////////////////////////////////////////

  always_comb
  begin : p_fsm
    // default
    State_SN       = State_SP;

    OutVld_SO      = 1'b0;

    LoadEn_S       = 1'b0;

    ARegEn_S       = 1'b0;
    BRegEn_S       = 1'b0;
    ResRegEn_S     = 1'b0;

    case (State_SP)
      /////////////////////////////////
      IDLE: begin
        OutVld_SO    = 1'b1;

        if(InVld_SI) begin
          OutVld_SO  = 1'b0;
          ARegEn_S   = 1'b1;
          BRegEn_S   = 1'b1;
          LoadEn_S   = 1'b1;
          State_SN   = DIVIDE;
        end
      end
      /////////////////////////////////
      DIVIDE: begin

        ARegEn_S     = ABComp_S;
        BRegEn_S     = 1'b1;
        ResRegEn_S   = 1'b1;

        // calculation finished
        // one more divide cycle (32nd divide cycle)
        if (CntZero_S) begin
          State_SN   = FINISH;
        end
      end
      /////////////////////////////////
      FINISH: begin
        OutVld_SO = 1'b1;

        if(OutRdy_SI) begin
          State_SN  = IDLE;
        end
      end
      /////////////////////////////////
      default : /* default */ ;
      /////////////////////////////////
    endcase
  end


  ///////////////////////////////////////////////////////////////////////////////
  // regs
  ///////////////////////////////////////////////////////////////////////////////

  // get flags
  assign RemSel_SN  = (LoadEn_S) ? OpCode_SI[1] : RemSel_SP;
  assign CompInv_SN = (LoadEn_S) ? OpBSign_SI   : CompInv_SP;
  assign ResInv_SN  = (LoadEn_S) ? (~OpBIsZero_SI | OpCode_SI[1]) & OpCode_SI[0] & (OpA_DI[$high(OpA_DI)] ^ OpBSign_SI) : ResInv_SP;

  assign AReg_DN   = (ARegEn_S)   ? AddOut_D : AReg_DP;
  assign BReg_DN   = (BRegEn_S)   ? BMux_D   : BReg_DP;
  assign ResReg_DN = (LoadEn_S)   ? '0       :
                     (ResRegEn_S) ? {ABComp_S, ResReg_DP[$high(ResReg_DP):1]} : ResReg_DP;

  always_ff @(posedge Clk_CI or negedge Rst_RBI) begin : p_regs
    if(~Rst_RBI) begin
       State_SP   <= IDLE;
       AReg_DP    <= '0;
       BReg_DP    <= '0;
       ResReg_DP  <= '0;
       Cnt_DP     <= '0;
       RemSel_SP  <= 1'b0;
       CompInv_SP <= 1'b0;
       ResInv_SP  <= 1'b0;
    end else begin
       State_SP   <= State_SN;
       AReg_DP    <= AReg_DN;
       BReg_DP    <= BReg_DN;
       ResReg_DP  <= ResReg_DN;
       Cnt_DP     <= Cnt_DN;
       RemSel_SP  <= RemSel_SN;
       CompInv_SP <= CompInv_SN;
       ResInv_SP  <= ResInv_SN;
    end
  end

  ///////////////////////////////////////////////////////////////////////////////
  // assertions
  ///////////////////////////////////////////////////////////////////////////////

`ifndef SYNTHESIS
  initial
  begin : p_assertions
    assert (C_LOG_WIDTH == $clog2(C_WIDTH+1)) else $error("C_LOG_WIDTH must be $clog2(C_WIDTH+1)");
  end
`endif

endmodule // serDiv

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Matthias Baer - baermatt@student.ethz.ch                   //
//                                                                            //
// Additional contributions by:                                               //
//                 Andreas Traber - atraber@student.ethz.ch                   //
//                                                                            //
// Design Name:    Subword multiplier and MAC                                 //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Advanced MAC unit for PULP.                                //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

import riscv_defines::*;


module riscv_mult
(
  input  logic        clk,
  input  logic        rst_n,

  input  logic        enable_i,
  input  logic [ 2:0] operator_i,

  // integer and short multiplier
  input  logic        short_subword_i,
  input  logic [ 1:0] short_signed_i,

  input  logic [31:0] op_a_i,
  input  logic [31:0] op_b_i,
  input  logic [31:0] op_c_i,

  input  logic [ 4:0] imm_i,


  // dot multiplier
  input  logic [ 1:0] dot_signed_i,
  input  logic [31:0] dot_op_a_i,
  input  logic [31:0] dot_op_b_i,
  input  logic [31:0] dot_op_c_i,

  output logic [31:0] result_o,

  output logic        multicycle_o,
  output logic        ready_o,
  input  logic        ex_ready_i
);

  ///////////////////////////////////////////////////////////////
  //  ___ _  _ _____ ___ ___ ___ ___   __  __ _   _ _  _____   //
  // |_ _| \| |_   _| __/ __| __| _ \ |  \/  | | | | ||_   _|  //
  //  | || . | | | | _| (_ | _||   / | |\/| | |_| | |__| |    //
  // |___|_|\_| |_| |___\___|___|_|_\ |_|  |_|\___/|____|_|    //
  //                                                           //
  ///////////////////////////////////////////////////////////////

  logic [16:0] short_op_a;
  logic [16:0] short_op_b;
  logic [32:0] short_op_c;
  logic [33:0] short_mac;
  logic        short_mac_msb;
  logic [31:0] short_round, short_round_tmp;
  logic [33:0] short_result;

  logic [ 4:0] short_imm;
  logic [ 1:0] short_subword;
  logic [ 1:0] short_signed;
  logic        short_shift_arith;
  logic        short_shift_ext;
  logic [ 4:0] mulh_imm;
  logic [ 1:0] mulh_subword;
  logic [ 1:0] mulh_signed;
  logic        mulh_shift_arith;
  logic        mulh_carry_q;
  logic        mulh_active;
  logic        mulh_save;
  logic        mulh_ready;
  enum logic [2:0] {IDLE, STEP0, STEP1, STEP2, FINISH} mulh_CS, mulh_NS;

  // prepare the rounding value
  assign short_round_tmp = (32'h00000001) << imm_i;
  assign short_round = (operator_i == MUL_IR) ? {1'b0, short_round_tmp[31:1]} : '0;

  // perform subword selection and sign extensions
  assign short_op_a[15:0] = short_subword[0] ? op_a_i[31:16] : op_a_i[15:0];
  assign short_op_b[15:0] = short_subword[1] ? op_b_i[31:16] : op_b_i[15:0];

  assign short_op_a[16] = short_signed[0] & short_op_a[15];
  assign short_op_b[16] = short_signed[1] & short_op_b[15];

  assign short_op_c     = mulh_active ? {mulh_carry_q, op_c_i} : {1'b0, op_c_i};

  assign short_mac = $signed(short_op_c) + $signed(short_op_a) * $signed(short_op_b) + $signed(short_round);
  assign short_mac_msb = mulh_active ? short_mac[32] : short_mac[31];

  assign short_result = $signed({short_shift_arith & short_mac_msb, short_shift_ext & short_mac_msb, short_mac[31:0]}) >>> short_imm;

  // choose between normal short multiplication operation and mulh operation
  assign short_imm         = mulh_active ? mulh_imm         : imm_i;
  assign short_subword     = mulh_active ? mulh_subword     : {2{short_subword_i}};
  assign short_signed      = mulh_active ? mulh_signed      : short_signed_i;
  assign short_shift_arith = mulh_active ? mulh_shift_arith : short_signed_i[0];
  assign short_shift_ext   = mulh_active ? 1'b1             : short_signed_i[0];

  always_comb
  begin
    mulh_NS          = mulh_CS;
    mulh_imm         = 5'd0;
    mulh_subword     = 2'b00;
    mulh_signed      = 2'b00;
    mulh_shift_arith = 1'b0;
    mulh_ready       = 1'b0;
    mulh_active      = 1'b1;
    mulh_save        = 1'b0;

    case (mulh_CS)
      IDLE: begin
        mulh_active = 1'b0;
        mulh_ready  = 1'b1;

        if ((operator_i == MUL_H) && enable_i) begin
          mulh_ready  = 1'b0;
          mulh_NS     = STEP0;
        end
      end

      STEP0: begin
        mulh_imm         = 5'd16;
        mulh_shift_arith = 1'b0;
        mulh_active      = 1'b1;
        mulh_save        = 1'b1;
        mulh_NS          = STEP1;
      end

      STEP1: begin
        mulh_signed  = {1'b0, short_signed_i[0]};

        mulh_subword = 2'b01;
        mulh_save    = 1'b1;
        mulh_NS      = STEP2;
      end

      STEP2: begin
        mulh_signed      = {short_signed_i[1], 1'b0};

        mulh_subword     = 2'b10;
        mulh_shift_arith = short_signed_i[0];
        mulh_imm         = 5'd16;
        mulh_save        = 1'b1;
        mulh_NS          = FINISH;
      end

      FINISH: begin
        mulh_signed = short_signed_i;

        mulh_subword = 2'b11;
        mulh_ready   = 1'b1;

        if (ex_ready_i)
          mulh_NS = IDLE;
      end
    endcase
  end

  always_ff @(posedge clk, negedge rst_n)
  begin
    if (~rst_n)
    begin
      mulh_CS      <= IDLE;
      mulh_carry_q <= 1'b0;
    end else begin
      mulh_CS      <= mulh_NS;

      if (mulh_save)
        mulh_carry_q <= short_result[32];
      else if (ex_ready_i) // clear carry when we are going to the next instruction
        mulh_carry_q <= 1'b0;
    end
  end

  // 32x32 = 32-bit multiplier
  logic [31:0] int_op_a_msu;
  logic [31:0] int_op_b_msu;
  logic [31:0] int_result;

  logic        int_is_msu;

  assign int_is_msu = (operator_i == MUL_MSU32); // TODO: think about using a separate signal here, could prevent some switching

  assign int_op_a_msu = op_a_i ^ {32{int_is_msu}};
  assign int_op_b_msu = op_b_i & {32{int_is_msu}};

  assign int_result = $signed(op_c_i) + $signed(int_op_b_msu) + $signed(int_op_a_msu) * $signed(op_b_i);

  ///////////////////////////////////////////////
  //  ___   ___ _____   __  __ _   _ _  _____  //
  // |   \ / _ \_   _| |  \/  | | | | ||_   _| //
  // | |) | (_) || |   | |\/| | |_| | |__| |   //
  // |___/ \___/ |_|   |_|  |_|\___/|____|_|   //
  //                                           //
  ///////////////////////////////////////////////

  logic [3:0][ 8:0] dot_char_op_a;
  logic [3:0][ 8:0] dot_char_op_b;
  logic [3:0][17:0] dot_char_mul;
  logic [31:0]      dot_char_result;

  logic [1:0][16:0] dot_short_op_a;
  logic [1:0][16:0] dot_short_op_b;
  logic [1:0][33:0] dot_short_mul;
  logic [31:0]      dot_short_result;


  assign dot_char_op_a[0] = {dot_signed_i[1] & dot_op_a_i[ 7], dot_op_a_i[ 7: 0]};
  assign dot_char_op_a[1] = {dot_signed_i[1] & dot_op_a_i[15], dot_op_a_i[15: 8]};
  assign dot_char_op_a[2] = {dot_signed_i[1] & dot_op_a_i[23], dot_op_a_i[23:16]};
  assign dot_char_op_a[3] = {dot_signed_i[1] & dot_op_a_i[31], dot_op_a_i[31:24]};

  assign dot_char_op_b[0] = {dot_signed_i[0] & dot_op_b_i[ 7], dot_op_b_i[ 7: 0]};
  assign dot_char_op_b[1] = {dot_signed_i[0] & dot_op_b_i[15], dot_op_b_i[15: 8]};
  assign dot_char_op_b[2] = {dot_signed_i[0] & dot_op_b_i[23], dot_op_b_i[23:16]};
  assign dot_char_op_b[3] = {dot_signed_i[0] & dot_op_b_i[31], dot_op_b_i[31:24]};

  assign dot_char_mul[0]  = $signed(dot_char_op_a[0]) * $signed(dot_char_op_b[0]);
  assign dot_char_mul[1]  = $signed(dot_char_op_a[1]) * $signed(dot_char_op_b[1]);
  assign dot_char_mul[2]  = $signed(dot_char_op_a[2]) * $signed(dot_char_op_b[2]);
  assign dot_char_mul[3]  = $signed(dot_char_op_a[3]) * $signed(dot_char_op_b[3]);

  assign dot_char_result  = $signed(dot_char_mul[0]) + $signed(dot_char_mul[1]) +
                            $signed(dot_char_mul[2]) + $signed(dot_char_mul[3]) +
                            $signed(dot_op_c_i);


  assign dot_short_op_a[0] = {dot_signed_i[1] & dot_op_a_i[15], dot_op_a_i[15: 0]};
  assign dot_short_op_a[1] = {dot_signed_i[1] & dot_op_a_i[31], dot_op_a_i[31:16]};

  assign dot_short_op_b[0] = {dot_signed_i[0] & dot_op_b_i[15], dot_op_b_i[15: 0]};
  assign dot_short_op_b[1] = {dot_signed_i[0] & dot_op_b_i[31], dot_op_b_i[31:16]};

  assign dot_short_mul[0]  = $signed(dot_short_op_a[0]) * $signed(dot_short_op_b[0]);
  assign dot_short_mul[1]  = $signed(dot_short_op_a[1]) * $signed(dot_short_op_b[1]);

  assign dot_short_result  = $signed(dot_short_mul[0][31:0]) + $signed(dot_short_mul[1][31:0]) + $signed(dot_op_c_i);


  ////////////////////////////////////////////////////////
  //   ____                 _ _     __  __              //
  //  |  _ \ ___  ___ _   _| | |_  |  \/  |_   ___  __  //
  //  | |_) / _ \/ __| | | | | __| | |\/| | | | \ \/ /  //
  //  |  _ <  __/\__ \ |_| | | |_  | |  | | |_| |>  <   //
  //  |_| \_\___||___/\__,_|_|\__| |_|  |_|\__,_/_/\_\  //
  //                                                    //
  ////////////////////////////////////////////////////////

  always_comb
  begin
    result_o   = 'x;

    unique case (operator_i)
      MUL_MAC32, MUL_MSU32: result_o = int_result[31:0];

      MUL_I, MUL_IR, MUL_H: result_o = short_result[31:0];

      MUL_DOT8:  result_o = dot_char_result[31:0];
      MUL_DOT16: result_o = dot_short_result[31:0];

      default: ; // default case to suppress unique warning
    endcase
  end

  assign multicycle_o = mulh_save;
  assign ready_o      = mulh_ready;

`ifndef verilator
  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------

  // check multiplication result for mulh
  assert property (
    @(posedge clk) ((mulh_CS == FINISH) && (operator_i == MUL_H) && (short_signed_i == 2'b11))
    |->
    (result_o == (($signed({{32{op_a_i[31]}}, op_a_i}) * $signed({{32{op_b_i[31]}}, op_b_i})) >>> 32) ) );

  // check multiplication result for mulhsu
  assert property (
    @(posedge clk) ((mulh_CS == FINISH) && (operator_i == MUL_H) && (short_signed_i == 2'b01))
    |->
    (result_o == (($signed({{32{op_a_i[31]}}, op_a_i}) * {32'b0, op_b_i}) >> 32) ) );

  // check multiplication result for mulhu
  assert property (
    @(posedge clk) ((mulh_CS == FINISH) && (operator_i == MUL_H) && (short_signed_i == 2'b00))
    |->
    (result_o == (({32'b0, op_a_i} * {32'b0, op_b_i}) >> 32) ) );
`endif

endmodule

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Igor Loi - igor.loi@unibo.it                               //
//                                                                            //
// Additional contributions by:                                               //
//                 Andreas Traber - atraber@iis.ee.ethz.ch                    //
//                                                                            //
// Design Name:    Load Store Unit                                            //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Load Store Unit, used to eliminate multiple access during  //
//                 processor stalls, and to align bytes and halfwords         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


module riscv_load_store_unit
(
    input  logic         clk,
    input  logic         rst_n,

    // output to data memory
    output logic         data_req_o,
    input  logic         data_gnt_i,
    input  logic         data_rvalid_i,
    input  logic         data_err_i,

    output logic [31:0]  data_addr_o,
    output logic         data_we_o,
    output logic [3:0]   data_be_o,
    output logic [31:0]  data_wdata_o,
    input  logic [31:0]  data_rdata_i,

    // signals from ex stage
    input  logic         data_we_ex_i,         // write enable                      -> from ex stage
    input  logic [1:0]   data_type_ex_i,       // Data type word, halfword, byte    -> from ex stage
    input  logic [31:0]  data_wdata_ex_i,      // data to write to memory           -> from ex stage
    input  logic [1:0]   data_reg_offset_ex_i, // offset inside register for stores -> from ex stage
    input  logic         data_sign_ext_ex_i,   // sign extension                    -> from ex stage

    output logic [31:0]  data_rdata_ex_o,      // requested data                    -> to ex stage
    input  logic         data_req_ex_i,        // data request                      -> from ex stage
    input  logic [31:0]  operand_a_ex_i,       // operand a from RF for address     -> from ex stage
    input  logic [31:0]  operand_b_ex_i,       // operand b from RF for address     -> from ex stage
    input  logic         addr_useincr_ex_i,    // use a + b or just a for address   -> from ex stage

    input  logic         data_misaligned_ex_i, // misaligned access in last ld/st   -> from ID/EX pipeline
    output logic         data_misaligned_o,    // misaligned access was detected    -> to controller

    // exception signals
    output logic         load_err_o,
    output logic         store_err_o,

    // stall signal
    output logic         lsu_ready_ex_o, // LSU ready for new data in EX stage
    output logic         lsu_ready_wb_o, // LSU ready for new data in WB stage

    input  logic         ex_valid_i,
    output logic         busy_o
);

  logic [31:0]  data_addr_int;

  // registers for data_rdata alignment and sign extension
  logic [1:0]   data_type_q;
  logic [1:0]   rdata_offset_q;
  logic         data_sign_ext_q;
  logic         data_we_q;

  logic [1:0]   wdata_offset;   // mux control for data to be written to memory

  logic [3:0]   data_be;
  logic [31:0]  data_wdata;

  logic         misaligned_st;   // high if we are currently performing the second part of a misaligned store


  enum logic [1:0]  { IDLE, WAIT_RVALID, WAIT_RVALID_EX_STALL, IDLE_EX_STALL } CS, NS;

  logic [31:0]  rdata_q;

  ///////////////////////////////// BE generation ////////////////////////////////
  always_comb
  begin
    case (data_type_ex_i) // Data type 00 Word, 01 Half word, 11,10 byte
      2'b00:
      begin // Writing a word
        if (misaligned_st == 1'b0)
        begin // non-misaligned case
          case (data_addr_int[1:0])
            2'b00: data_be = 4'b1111;
            2'b01: data_be = 4'b1110;
            2'b10: data_be = 4'b1100;
            2'b11: data_be = 4'b1000;
          endcase; // case (data_addr_int[1:0])
        end
        else
        begin // misaligned case
          case (data_addr_int[1:0])
            2'b00: data_be = 4'b0000; // this is not used, but included for completeness
            2'b01: data_be = 4'b0001;
            2'b10: data_be = 4'b0011;
            2'b11: data_be = 4'b0111;
          endcase; // case (data_addr_int[1:0])
        end
      end

      2'b01:
      begin // Writing a half word
        if (misaligned_st == 1'b0)
        begin // non-misaligned case
          case (data_addr_int[1:0])
            2'b00: data_be = 4'b0011;
            2'b01: data_be = 4'b0110;
            2'b10: data_be = 4'b1100;
            2'b11: data_be = 4'b1000;
          endcase; // case (data_addr_int[1:0])
        end
        else
        begin // misaligned case
          data_be = 4'b0001;
        end
      end

      2'b10,
      2'b11: begin // Writing a byte
        case (data_addr_int[1:0])
          2'b00: data_be = 4'b0001;
          2'b01: data_be = 4'b0010;
          2'b10: data_be = 4'b0100;
          2'b11: data_be = 4'b1000;
        endcase; // case (data_addr_int[1:0])
      end
    endcase; // case (data_type_ex_i)
  end

  // prepare data to be written to the memory
  // we handle misaligned accesses, half word and byte accesses and
  // register offsets here
  assign wdata_offset = data_addr_int[1:0] - data_reg_offset_ex_i[1:0];
  always_comb
  begin
    case (wdata_offset)
      2'b00: data_wdata = data_wdata_ex_i[31:0];
      2'b01: data_wdata = {data_wdata_ex_i[23:0], data_wdata_ex_i[31:24]};
      2'b10: data_wdata = {data_wdata_ex_i[15:0], data_wdata_ex_i[31:16]};
      2'b11: data_wdata = {data_wdata_ex_i[ 7:0], data_wdata_ex_i[31: 8]};
    endcase; // case (wdata_offset)
  end


  // FF for rdata alignment and sign-extension
  always_ff @(posedge clk, negedge rst_n)
  begin
    if(rst_n == 1'b0)
    begin
      data_type_q     <= '0;
      rdata_offset_q  <= '0;
      data_sign_ext_q <= '0;
      data_we_q       <= 1'b0;
    end
    else if (data_gnt_i == 1'b1) // request was granted, we wait for rvalid and can continue to WB
    begin
      data_type_q     <= data_type_ex_i;
      rdata_offset_q  <= data_addr_int[1:0];
      data_sign_ext_q <= data_sign_ext_ex_i;
      data_we_q       <= data_we_ex_i;
    end
  end


  ////////////////////////////////////////////////////////////////////////
  //  ____  _               _____      _                 _              //
  // / ___|(_) __ _ _ __   | ____|_  _| |_ ___ _ __  ___(_) ___  _ __   //
  // \___ \| |/ _` | '_ \  |  _| \ \/ / __/ _ \ '_ \/ __| |/ _ \| '_ \  //
  //  ___) | | (_| | | | | | |___ >  <| ||  __/ | | \__ \ | (_) | | | | //
  // |____/|_|\__, |_| |_| |_____/_/\_\\__\___|_| |_|___/_|\___/|_| |_| //
  //          |___/                                                     //
  ////////////////////////////////////////////////////////////////////////

  logic [31:0] data_rdata_ext;

  logic [31:0] rdata_w_ext; // sign extension for words, actually only misaligned assembly
  logic [31:0] rdata_h_ext; // sign extension for half words
  logic [31:0] rdata_b_ext; // sign extension for bytes

  // take care of misaligned words
  always_comb
  begin
    case (rdata_offset_q)
      2'b00: rdata_w_ext = data_rdata_i[31:0];
      2'b01: rdata_w_ext = {data_rdata_i[ 7:0], rdata_q[31:8]};
      2'b10: rdata_w_ext = {data_rdata_i[15:0], rdata_q[31:16]};
      2'b11: rdata_w_ext = {data_rdata_i[23:0], rdata_q[31:24]};
    endcase
  end

  // sign extension for half words
  always_comb
  begin
    case (rdata_offset_q)
      2'b00:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_h_ext = {16'h0000, data_rdata_i[15:0]};
        else
          rdata_h_ext = {{16{data_rdata_i[15]}}, data_rdata_i[15:0]};
      end

      2'b01:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_h_ext = {16'h0000, data_rdata_i[23:8]};
        else
          rdata_h_ext = {{16{data_rdata_i[23]}}, data_rdata_i[23:8]};
      end

      2'b10:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_h_ext = {16'h0000, data_rdata_i[31:16]};
        else
          rdata_h_ext = {{16{data_rdata_i[31]}}, data_rdata_i[31:16]};
      end

      2'b11:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_h_ext = {16'h0000, data_rdata_i[7:0], rdata_q[31:24]};
        else
          rdata_h_ext = {{16{data_rdata_i[7]}}, data_rdata_i[7:0], rdata_q[31:24]};
      end
    endcase // case (rdata_offset_q)
  end

  // sign extension for bytes
  always_comb
  begin
    case (rdata_offset_q)
      2'b00:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_b_ext = {24'h00_0000, data_rdata_i[7:0]};
        else
          rdata_b_ext = {{24{data_rdata_i[7]}}, data_rdata_i[7:0]};
      end

      2'b01: begin
        if (data_sign_ext_q == 1'b0)
          rdata_b_ext = {24'h00_0000, data_rdata_i[15:8]};
        else
          rdata_b_ext = {{24{data_rdata_i[15]}}, data_rdata_i[15:8]};
      end

      2'b10:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_b_ext = {24'h00_0000, data_rdata_i[23:16]};
        else
          rdata_b_ext = {{24{data_rdata_i[23]}}, data_rdata_i[23:16]};
      end

      2'b11:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_b_ext = {24'h00_0000, data_rdata_i[31:24]};
        else
          rdata_b_ext = {{24{data_rdata_i[31]}}, data_rdata_i[31:24]};
      end
    endcase // case (rdata_offset_q)
  end

  // select word, half word or byte sign extended version
  always_comb
  begin
    case (data_type_q)
      2'b00:       data_rdata_ext = rdata_w_ext;
      2'b01:       data_rdata_ext = rdata_h_ext;
      2'b10,2'b11: data_rdata_ext = rdata_b_ext;
    endcase //~case(rdata_type_q)
  end



  always_ff @(posedge clk, negedge rst_n)
  begin
    if(rst_n == 1'b0)
    begin
      CS            <= IDLE;
      rdata_q       <= '0;
    end
    else
    begin
      CS            <= NS;

      if (data_rvalid_i && (~data_we_q))
      begin
        // if we have detected a misaligned access, and we are
        // currently doing the first part of this access, then
        // store the data coming from memory in rdata_q.
        // In all other cases, rdata_q gets the value that we are
        // writing to the register file
        if ((data_misaligned_ex_i == 1'b1) || (data_misaligned_o == 1'b1))
          rdata_q  <= data_rdata_i;
        else
          rdata_q  <= data_rdata_ext;
      end
    end
  end

  // output to register file
  assign data_rdata_ex_o = (data_rvalid_i == 1'b1) ? data_rdata_ext : rdata_q;

  // output to data interface
  assign data_addr_o   = data_addr_int;
  assign data_wdata_o  = data_wdata;
  assign data_we_o     = data_we_ex_i;
  assign data_be_o     = data_be;

  assign misaligned_st = data_misaligned_ex_i;

  assign load_err_o    = data_gnt_i && data_err_i && data_we_o;
  assign store_err_o   = data_gnt_i && data_err_i && data_we_o;

  // FSM
  always_comb
  begin
    NS             = CS;

    data_req_o     = 1'b0;

    lsu_ready_ex_o = 1'b1;
    lsu_ready_wb_o = 1'b1;

    case(CS)
      // starts from not active and stays in IDLE until request was granted
      IDLE:
      begin
        data_req_o = data_req_ex_i;

        if(data_req_ex_i) begin
          lsu_ready_ex_o = 1'b0;

          if(data_gnt_i) begin
            lsu_ready_ex_o = 1'b1;

            if (ex_valid_i)
              NS = WAIT_RVALID;
            else
              NS = WAIT_RVALID_EX_STALL;
          end
        end
      end //~ IDLE

      // wait for rvalid in WB stage and send a new request if there is any
      WAIT_RVALID:
      begin
        lsu_ready_wb_o = 1'b0;

        if (data_rvalid_i) begin
          // we don't have to wait for anything here as we are the only stall
          // source for the WB stage
          lsu_ready_wb_o = 1'b1;

          data_req_o = data_req_ex_i;

          if (data_req_ex_i) begin
            lsu_ready_ex_o = 1'b0;

            if (data_gnt_i) begin
              lsu_ready_ex_o = 1'b1;

              if(ex_valid_i)
                NS = WAIT_RVALID;
              else
                NS = WAIT_RVALID_EX_STALL;
            end else begin
              NS = IDLE;
            end
          end else begin
            if (data_rvalid_i) begin
              // no request, so go to IDLE
              NS = IDLE;
            end
          end
        end
      end

      // wait for rvalid while still in EX stage
      // we end up here when there was an EX stall, so in this cycle we just
      // wait and don't send new requests
      WAIT_RVALID_EX_STALL:
      begin
        data_req_o = 1'b0;

        if (data_rvalid_i) begin
          if (ex_valid_i) begin
            // we are done and can go back to idle
            // the data is safely stored already
            NS = IDLE;
          end else begin
            // we have to wait until ex_stall is deasserted
            NS = IDLE_EX_STALL;
          end
        end else begin
          // we didn't yet receive the rvalid, so we check the ex_stall
          // signal. If we are no longer stalled we can change to the "normal"
          // WAIT_RVALID state
          if (ex_valid_i)
            NS = WAIT_RVALID;
        end
      end

      IDLE_EX_STALL:
      begin
        // wait for us to be unstalled and then change back to IDLE state
        if (ex_valid_i) begin
          NS = IDLE;
        end
      end

      default: begin
        NS = IDLE;
      end
    endcase
  end

  // check for misaligned accesses that need a second memory access
  // If one is detected, this is signaled with data_misaligned_o to
  // the controller which selectively stalls the pipeline
  always_comb
  begin
    data_misaligned_o = 1'b0;

    if((data_req_ex_i == 1'b1) && (data_misaligned_ex_i == 1'b0))
    begin
      case (data_type_ex_i)
        2'b00: // word
        begin
          if(data_addr_int[1:0] != 2'b00)
            data_misaligned_o = 1'b1;
        end
        2'b01: // half word
        begin
          if(data_addr_int[1:0] == 2'b11)
            data_misaligned_o = 1'b1;
        end
      endcase // case (data_type_ex_i)
    end
  end

  // generate address from operands
  assign data_addr_int = (addr_useincr_ex_i) ? (operand_a_ex_i + operand_b_ex_i) : operand_a_ex_i;

  assign busy_o = (CS == WAIT_RVALID) || (CS == WAIT_RVALID_EX_STALL) || (CS == IDLE_EX_STALL) || (data_req_o == 1'b1);


`ifndef verilator
  //////////////////////////////////////////////////////////////////////////////
  // Assertions
  //////////////////////////////////////////////////////////////////////////////

  // make sure there is no new request when the old one is not yet completely done
  // i.e. it should not be possible to get a grant without an rvalid for the
  // last request
  assert property (
    @(posedge clk) ((CS == WAIT_RVALID) && (data_gnt_i == 1'b1)) |-> (data_rvalid_i == 1'b1) );

  // there should be no rvalid when we are in IDLE
  assert property (
    @(posedge clk) (CS == IDLE) |-> (data_rvalid_i == 1'b0) );

  // assert that errors are only sent at the same time as grant
  assert property ( @(posedge clk) (data_err_i) |-> (data_gnt_i) );

  // assert that the address does not contain X when request is sent
  assert property ( @(posedge clk) (data_req_o) |-> (!$isunknown(data_addr_o)) );
`endif

endmodule

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the âLicenseâ?); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an âAS ISâ? BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
// Additional contributions by:                                               //
//                 Andreas Traber - atraber@iis.ee.ethz.ch                    //
//                                                                            //
// Design Name:    Control and Status Registers                               //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Control and Status Registers (CSRs) loosely following the  //
//                 RiscV draft priviledged instruction set spec (v1.7)        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

import riscv_defines::*;

`ifndef PULP_FPGA_EMUL
 `ifdef SYNTHESIS
  `define ASIC_SYNTHESIS
 `endif
`endif

module riscv_cs_registers
#(
  parameter N_HWLP       = 2,
  parameter N_HWLP_BITS  = $clog2(N_HWLP),
  parameter N_EXT_CNT    = 0
)
(
  // Clock and Reset
  input  logic        clk,
  input  logic        rst_n,

  // Core and Cluster ID
  input  logic  [3:0] core_id_i,
  input  logic  [5:0] cluster_id_i,

  // Interface to registers (SRAM like)
  input  logic        csr_access_i,
  input  logic [11:0] csr_addr_i,
  input  logic [31:0] csr_wdata_i,
  input  logic  [1:0] csr_op_i,
  output logic [31:0] csr_rdata_o,

  // Interrupts
  output logic        irq_enable_o,
  output logic [31:0] mepc_o,

  input  logic [31:0] pc_if_i,
  input  logic [31:0] pc_id_i,
  input  logic [31:0] pc_ex_i,
  input  logic        data_load_event_ex_i,
  input  logic        exc_save_if_i,
  input  logic        exc_save_id_i,
  input  logic        exc_restore_i,

  input  logic [5:0]  exc_cause_i,
  input  logic        save_exc_cause_i,

  // Hardware loops
  input  logic [N_HWLP-1:0] [31:0] hwlp_start_i,
  input  logic [N_HWLP-1:0] [31:0] hwlp_end_i,
  input  logic [N_HWLP-1:0] [31:0] hwlp_cnt_i,

  output logic [31:0]              hwlp_data_o,
  output logic [N_HWLP_BITS-1:0]   hwlp_regid_o,
  output logic [2:0]               hwlp_we_o,

  // Performance Counters
  input  logic                 id_valid_i,        // ID stage is done
  input  logic                 is_compressed_i,   // compressed instruction in ID
  input  logic                 is_decoding_i,     // controller is in DECODE state

  input  logic                 imiss_i,           // instruction fetch
  input  logic                 pc_set_i,          // pc was set to a new value
  input  logic                 jump_i,            // jump instruction seen   (j, jr, jal, jalr)
  input  logic                 branch_i,          // branch instruction seen (bf, bnf)
  input  logic                 branch_taken_i,    // branch was taken
  input  logic                 ld_stall_i,        // load use hazard
  input  logic                 jr_stall_i,        // jump register use hazard

  input  logic                 mem_load_i,        // load from memory in this cycle
  input  logic                 mem_store_i,       // store to memory in this cycle

  input  logic [N_EXT_CNT-1:0] ext_counters_i,
  
  //HDC predicted class vector
  
//  input logic [3:0]            hdc_predicted_class
  input logic [31:0]           y
);

  localparam N_PERF_COUNTERS = 11 + N_EXT_CNT;

`ifdef ASIC_SYNTHESIS
  localparam N_PERF_REGS     = 1;
`else
  localparam N_PERF_REGS     = N_PERF_COUNTERS;
`endif

  // Performance Counter Signals
  logic                          id_valid_q;
  logic [N_PERF_COUNTERS-1:0]    PCCR_in;  // input signals for each counter category
  logic [N_PERF_COUNTERS-1:0]    PCCR_inc, PCCR_inc_q; // should the counter be increased?

  logic [N_PERF_REGS-1:0] [31:0] PCCR_q, PCCR_n; // performance counters counter register
  logic [1:0]                    PCMR_n, PCMR_q; // mode register, controls saturation and global enable
  logic [N_PERF_COUNTERS-1:0]    PCER_n, PCER_q; // selected counter input

  logic [31:0]                   perf_rdata;
  logic [4:0]                    pccr_index;
  logic                          pccr_all_sel;
  logic                          is_pccr;
  logic                          is_pcer;
  logic                          is_pcmr;

  // CSR update logic
  logic [31:0] csr_wdata_int;
  logic [31:0] csr_rdata_int;
  logic        csr_we_int;

  // Interrupt control signals
  logic [31:0] mepc_q, mepc_n;
  logic [ 0:0] mestatus_q, mestatus_n;
  logic [ 0:0] mstatus_q, mstatus_n;
  logic [ 5:0] exc_cause, exc_cause_n;

  // HDC CSR address
  localparam int unsigned CSR_predicted_class = 12'h7c4;
  
  ////////////////////////////////////////////
  //   ____ ____  ____    ____              //
  //  / ___/ ___||  _ \  |  _ \ ___  __ _   //
  // | |   \___ \| |_) | | |_) / _ \/ _` |  //   
  // | |___ ___) |  _ <  |  _ <  __/ (_| |  //
  //  \____|____/|_| \_\ |_| \_\___|\__, |  //
  //                                |___/   //
  ////////////////////////////////////////////

  // read logic
  always_comb
  begin
    csr_rdata_int = 'x;

    case (csr_addr_i)
      // mstatus: always M-mode, contains IE bit
      12'h300: csr_rdata_int = {29'b0, 2'b11, mstatus_q};

      // mepc: exception program counter
      12'h341: csr_rdata_int = mepc_q;
      // mcause: exception cause
      12'h342: csr_rdata_int = {exc_cause[5], 26'b0, exc_cause[4:0]};

      // mcpuid: RV32IM and X
      12'hF00: csr_rdata_int = 32'h00_80_11_00;
      // mimpid: PULP, anonymous source (no allocated ID yet)
      12'hF01: csr_rdata_int = 32'h00_00_80_00;
      // mhartid: unique hardware thread id
      12'hF10: csr_rdata_int = {21'b0, cluster_id_i[5:0], 1'b0, core_id_i[3:0]};

      // hardware loops
      12'h7B0: csr_rdata_int = hwlp_start_i[0];
      12'h7B1: csr_rdata_int = hwlp_end_i[0];
      12'h7B2: csr_rdata_int = hwlp_cnt_i[0];
      12'h7B4: csr_rdata_int = hwlp_start_i[1];
      12'h7B5: csr_rdata_int = hwlp_end_i[1];
      12'h7B6: csr_rdata_int = hwlp_cnt_i[1];

      12'h7C0: csr_rdata_int = {29'b0, 2'b11, mestatus_q};
      
      //HDC data read
      
//      12'h7c4: csr_rdata_int = {27'b0, hdc_predicted_class};   ///todo
      
      12'h7c4: csr_rdata_int = y;
    endcase
  end


  // write logic
  always_comb
  begin
    mepc_n       = mepc_q;
    mestatus_n   = mestatus_q;
    mstatus_n    = mstatus_q;
    exc_cause_n  = exc_cause;
    hwlp_we_o    = '0;
    hwlp_regid_o = '0;

    case (csr_addr_i)
      // mstatus: IE bit
      12'h300: if (csr_we_int) mstatus_n = csr_wdata_int[0];

      // mepc: exception program counter
      12'h341: if (csr_we_int) mepc_n = csr_wdata_int;
      // mcause
      12'h342: if (csr_we_int) exc_cause_n = {csr_wdata_int[5], csr_wdata_int[4:0]};

      // hardware loops
      12'h7B0: if (csr_we_int) begin hwlp_we_o = 3'b001; hwlp_regid_o = 1'b0; end
      12'h7B1: if (csr_we_int) begin hwlp_we_o = 3'b010; hwlp_regid_o = 1'b0; end
      12'h7B2: if (csr_we_int) begin hwlp_we_o = 3'b100; hwlp_regid_o = 1'b0; end
      12'h7B4: if (csr_we_int) begin hwlp_we_o = 3'b001; hwlp_regid_o = 1'b1; end
      12'h7B5: if (csr_we_int) begin hwlp_we_o = 3'b010; hwlp_regid_o = 1'b1; end
      12'h7B6: if (csr_we_int) begin hwlp_we_o = 3'b100; hwlp_regid_o = 1'b1; end

      // mestatus: machine exception status
      12'h7C0: if (csr_we_int) mestatus_n = csr_wdata_int[0];
    endcase

    // exception controller gets priority over other writes
    if (exc_save_if_i || exc_save_id_i) begin
      mestatus_n = mstatus_q;
      mstatus_n  = 1'b0;

      if (data_load_event_ex_i) begin
        mepc_n = pc_ex_i;
      end else begin
        if (exc_save_if_i)
          mepc_n = pc_if_i;
        else
          mepc_n = pc_id_i;
      end
    end

    if (save_exc_cause_i)
      exc_cause_n = exc_cause_i;

    if (exc_restore_i) begin
      mstatus_n = mestatus_q;
    end
  end

  assign hwlp_data_o = csr_wdata_int;


  // CSR operation logic
  always_comb
  begin
    csr_wdata_int = csr_wdata_i;
    csr_we_int    = 1'b1;

    unique case (csr_op_i)
      CSR_OP_WRITE: csr_wdata_int = csr_wdata_i;
      CSR_OP_SET:   csr_wdata_int = csr_wdata_i | csr_rdata_o;
      CSR_OP_CLEAR: csr_wdata_int = (~csr_wdata_i) & csr_rdata_o;

      CSR_OP_NONE: begin
        csr_wdata_int = csr_wdata_i;
        csr_we_int    = 1'b0;
      end

      default:;
    endcase
  end


  // output mux
  always_comb
  begin
    csr_rdata_o = csr_rdata_int;

    // performance counters
    if (is_pccr || is_pcer || is_pcmr)
      csr_rdata_o = perf_rdata;
  end


  // directly output some registers
  assign irq_enable_o = mstatus_q[0];
  assign mepc_o       = mepc_q;


  // actual registers
  always_ff @(posedge clk, negedge rst_n)
  begin
    if (rst_n == 1'b0)
    begin
      mstatus_q  <= '0;
      mepc_q     <= '0;
      mestatus_q <= '0;
      exc_cause  <= '0;
    end
    else
    begin
      // update CSRs
      mstatus_q  <= mstatus_n;

      mepc_q     <= mepc_n;
      mestatus_q <= mestatus_n;

      exc_cause  <= exc_cause_n;
    end
  end

  /////////////////////////////////////////////////////////////////
  //   ____            __     ____                  _            //
  // |  _ \ ___ _ __ / _|   / ___|___  _   _ _ __ | |_ ___ _ __  //
  // | |_) / _ \ '__| |_   | |   / _ \| | | | '_ \| __/ _ \ '__| //
  // |  __/  __/ |  |  _|  | |__| (_) | |_| | | | | ||  __/ |    //
  // |_|   \___|_|  |_|(_)  \____\___/ \__,_|_| |_|\__\___|_|    //
  //                                                             //
  /////////////////////////////////////////////////////////////////

  assign PCCR_in[0]  = 1'b1;                          // cycle counter
  assign PCCR_in[1]  = id_valid_i & is_decoding_i;    // instruction counter
  assign PCCR_in[2]  = ld_stall_i & id_valid_q;       // nr of load use hazards
  assign PCCR_in[3]  = jr_stall_i & id_valid_q;       // nr of jump register hazards
  assign PCCR_in[4]  = imiss_i & (~pc_set_i);         // cycles waiting for instruction fetches, excluding jumps and branches
  assign PCCR_in[5]  = mem_load_i;                    // nr of loads
  assign PCCR_in[6]  = mem_store_i;                   // nr of stores
  assign PCCR_in[7]  = jump_i                     & id_valid_q; // nr of jumps (unconditional)
  assign PCCR_in[8]  = branch_i                   & id_valid_q; // nr of branches (conditional)
  assign PCCR_in[9]  = branch_i & branch_taken_i  & id_valid_q; // nr of taken branches (conditional)
  assign PCCR_in[10] = id_valid_i & is_decoding_i & is_compressed_i;  // compressed instruction counter

  // assign external performance counters
  generate
    genvar i;
    for(i = 0; i < N_EXT_CNT; i++)
    begin
      assign PCCR_in[N_PERF_COUNTERS - N_EXT_CNT + i] = ext_counters_i[i];
    end
  endgenerate

  // address decoder for performance counter registers
  always_comb
  begin
    is_pccr      = 1'b0;
    is_pcmr      = 1'b0;
    is_pcer      = 1'b0;
    pccr_all_sel = 1'b0;
    pccr_index   = '0;
    perf_rdata   = '0;

    // only perform csr access if we actually care about the read data
    if (csr_access_i) begin
      unique case (csr_addr_i)
        12'h7A0: begin
          is_pcer = 1'b1;
          perf_rdata[N_PERF_COUNTERS-1:0] = PCER_q;
        end
        12'h7A1: begin
          is_pcmr = 1'b1;
          perf_rdata[1:0] = PCMR_q;
        end
        12'h79F: begin
          is_pccr = 1'b1;
          pccr_all_sel = 1'b1;
        end
        default:;
      endcase

      // look for 780 to 79F, Performance Counter Counter Registers
      if (csr_addr_i[11:5] == 7'b0111100) begin
        is_pccr     = 1'b1;

        pccr_index = csr_addr_i[4:0];
`ifdef  ASIC_SYNTHESIS
        perf_rdata = PCCR_q[0];
`else
        perf_rdata = PCCR_q[csr_addr_i[4:0]];
`endif
      end
    end
  end


  // performance counter counter update logic
`ifdef ASIC_SYNTHESIS
  // for synthesis we just have one performance counter register
  assign PCCR_inc[0] = (|(PCCR_in & PCER_q)) & PCMR_q[0];

  always_comb
  begin
    PCCR_n[0]   = PCCR_q[0];

    if ((PCCR_inc_q[0] == 1'b1) && ((PCCR_q[0] != 32'hFFFFFFFF) || (PCMR_q[1] == 1'b0)))
      PCCR_n[0] = PCCR_q[0] + 1;

    if (is_pccr == 1'b1) begin
      unique case (csr_op_i)
        CSR_OP_NONE:   ;
        CSR_OP_WRITE:  PCCR_n[0] = csr_wdata_i;
        CSR_OP_SET:    PCCR_n[0] = csr_wdata_i | PCCR_q[0];
        CSR_OP_CLEAR:  PCCR_n[0] = csr_wdata_i & ~(PCCR_q[0]);
      endcase
    end
  end
`else
  always_comb
  begin
    for(int i = 0; i < N_PERF_COUNTERS; i++)
    begin : PERF_CNT_INC
      PCCR_inc[i] = PCCR_in[i] & PCER_q[i] & PCMR_q[0];

      PCCR_n[i]   = PCCR_q[i];

      if ((PCCR_inc_q[i] == 1'b1) && ((PCCR_q[i] != 32'hFFFFFFFF) || (PCMR_q[1] == 1'b0)))
        PCCR_n[i] = PCCR_q[i] + 1;

      if (is_pccr == 1'b1 && (pccr_all_sel == 1'b1 || pccr_index == i)) begin
        unique case (csr_op_i)
          CSR_OP_NONE:   ;
          CSR_OP_WRITE:  PCCR_n[i] = csr_wdata_i;
          CSR_OP_SET:    PCCR_n[i] = csr_wdata_i | PCCR_q[i];
          CSR_OP_CLEAR:  PCCR_n[i] = csr_wdata_i & ~(PCCR_q[i]);
        endcase
      end
    end
  end
`endif

  // update PCMR and PCER
  always_comb
  begin
    PCMR_n = PCMR_q;
    PCER_n = PCER_q;

    if (is_pcmr) begin
      unique case (csr_op_i)
        CSR_OP_NONE:   ;
        CSR_OP_WRITE:  PCMR_n = csr_wdata_i[1:0];
        CSR_OP_SET:    PCMR_n = csr_wdata_i[1:0] | PCMR_q;
        CSR_OP_CLEAR:  PCMR_n = csr_wdata_i[1:0] & ~(PCMR_q);
      endcase
    end

    if (is_pcer) begin
      unique case (csr_op_i)
        CSR_OP_NONE:   ;
        CSR_OP_WRITE:  PCER_n = csr_wdata_i[N_PERF_COUNTERS-1:0];
        CSR_OP_SET:    PCER_n = csr_wdata_i[N_PERF_COUNTERS-1:0] | PCER_q;
        CSR_OP_CLEAR:  PCER_n = csr_wdata_i[N_PERF_COUNTERS-1:0] & ~(PCER_q);
      endcase
    end
  end

  // Performance Counter Registers
  always_ff @(posedge clk, negedge rst_n)
  begin
    if (rst_n == 1'b0)
    begin
      id_valid_q <= 1'b0;

      PCER_q <= '0;
      PCMR_q <= 2'h3;

      for(int i = 0; i < N_PERF_REGS; i++)
      begin
        PCCR_q[i]     <= '0;
        PCCR_inc_q[i] <= '0;
      end
    end
    else
    begin
      id_valid_q <= id_valid_i;

      PCER_q <= PCER_n;
      PCMR_q <= PCMR_n;

      for(int i = 0; i < N_PERF_REGS; i++)
      begin
        PCCR_q[i]     <= PCCR_n[i];
        PCCR_inc_q[i] <= PCCR_inc[i];
      end

    end
  end

endmodule

// Copyright 2016 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Andreas Traber - atraber@iis.ee.ethz.ch                    //
//                                                                            //
// Design Name:    Debug Unit                                                 //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Debug controller                                           //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


import riscv_defines::*;

module riscv_debug_unit
(
  input logic         clk,
  input logic         rst_n,

  // Debug Interface
  input  logic        debug_req_i,
  output logic        debug_gnt_o,
  output logic        debug_rvalid_o,
  input  logic [14:0] debug_addr_i,
  input  logic        debug_we_i,
  input  logic [31:0] debug_wdata_i,
  output logic [31:0] debug_rdata_o,

  output logic        debug_halted_o,
  input  logic        debug_halt_i,
  input  logic        debug_resume_i,

  // signals to core
  output logic [DBG_SETS_W-1:0] settings_o,

  input  logic        trap_i,      // trap found, need to stop the core now
  input  logic [5:0]  exc_cause_i, // if it was a trap, then the exception controller knows more
  output logic        stall_o,     // after we got control, we control the stall signal
  output logic        dbg_req_o,
  input  logic        dbg_ack_i,

  // register file read port
  output logic        regfile_rreq_o,
  output logic [ 4:0] regfile_raddr_o,
  input  logic [31:0] regfile_rdata_i,

  // register file write port
  output logic        regfile_wreq_o,
  output logic [ 4:0] regfile_waddr_o,
  output logic [31:0] regfile_wdata_o,

  // CSR read/write port
  output logic        csr_req_o,
  output logic [11:0] csr_addr_o,
  output logic        csr_we_o,
  output logic [31:0] csr_wdata_o,
  input  logic [31:0] csr_rdata_i,

  // Signals for PPC & NPC register
  input  logic [31:0] pc_if_i,
  input  logic [31:0] pc_id_i,
  input  logic [31:0] pc_ex_i,

  input  logic        data_load_event_i,
  input  logic        instr_valid_id_i,

  input  logic        sleeping_i,

  input  logic        branch_in_ex_i,
  input  logic        branch_taken_i,

  output logic        jump_req_o,
  output logic [31:0] jump_addr_o
);

  enum logic [2:0] {RD_NONE, RD_CSR, RD_GPR, RD_DBGA, RD_DBGS} rdata_sel_q, rdata_sel_n;

  enum logic [0:0] {FIRST, SECOND} state_q, state_n;

  logic [DBG_SETS_W-1:0] settings_q, settings_n;

  // for timing critical register file access we need to keep those in FFs
  logic [14:0] addr_q;
  logic [31:0] wdata_q; // mainly for jumps
  logic        regfile_rreq_q, regfile_rreq_n;
  logic        jump_req_q, jump_req_n;

  // not timing critical
  logic        csr_req_q, csr_req_n;
  logic        regfile_wreq;


  enum logic [1:0] {RUNNING, HALT_REQ, HALT} stall_cs, stall_ns;
  logic [31:0] dbg_rdata;
  logic        dbg_resume;
  logic        dbg_halt;
  logic [5:0]  dbg_cause_q, dbg_cause_n;
  logic        dbg_ssth_q,  dbg_ssth_n;

  logic        ssth_clear;


  // ppc/npc tracking
  enum logic [1:0] {IFID, IFEX, IDEX} pc_tracking_fsm_cs, pc_tracking_fsm_ns;
  logic [31:0] ppc_int, npc_int;


  // address decoding, write and read controller
  always_comb
  begin
    rdata_sel_n    = RD_NONE;
    state_n        = FIRST;

    debug_gnt_o    = 1'b0;

    regfile_rreq_n = 1'b0;
    regfile_wreq   = 1'b0;
    csr_req_n      = 1'b0;
    csr_we_o       = 1'b0;
    jump_req_n     = 1'b0;

    dbg_resume     = 1'b0;
    dbg_halt       = 1'b0;
    settings_n     = settings_q;

    ssth_clear     = 1'b0;

    if (debug_req_i) begin
      if (debug_we_i) begin
        //----------------------------------------------------------------------------
        // write access
        //----------------------------------------------------------------------------
        if (debug_addr_i[14]) begin
          // CSR access
          if (state_q == FIRST) begin
            // only grant in second cycle, address and data have been latched by then
            debug_gnt_o = 1'b0;
            state_n     = SECOND;

            if (debug_halted_o) begin
              // access to internal registers are only allowed when the core is halted
              csr_req_n = 1'b1;
            end
          end else begin
            debug_gnt_o = 1'b1; // grant it even when invalid access to not block
            state_n     = FIRST;
            csr_we_o    = 1'b1;
          end
        end else begin
          // non-CSR access
          unique case (debug_addr_i[13:8])
            6'b00_0000: begin // Debug Registers, always accessible
              debug_gnt_o = 1'b1;

              unique case (debug_addr_i[6:2])
                5'b0_0000: begin // DBG_CTRL
                  if (debug_wdata_i[16]) begin
                    // HALT set
                    if (~debug_halted_o) begin
                      // not halt, so STOP
                      dbg_halt = 1'b1;
                    end
                  end else begin
                    // RESUME set
                    if (debug_halted_o) begin
                      dbg_resume = 1'b1;
                    end
                  end

                  settings_n[DBG_SETS_SSTE] = debug_wdata_i[0];
                end
                5'b0_0001: begin // DBG_HIT
                  ssth_clear = debug_wdata_i[0];
                end
                5'b0_0010: begin // DBG_IE
                  settings_n[DBG_SETS_ECALL] = debug_wdata_i[11];
                  settings_n[DBG_SETS_ELSU]  = debug_wdata_i[7] | debug_wdata_i[5];
                  settings_n[DBG_SETS_EBRK]  = debug_wdata_i[3];
                  settings_n[DBG_SETS_EILL]  = debug_wdata_i[2];
                end
                default:;
              endcase
            end

            6'b10_0000: begin // Debug Registers, only accessible when in debug
              debug_gnt_o = 1'b1; // grant it even when invalid access to not block

              if (debug_halted_o) begin
                unique case (debug_addr_i[6:2])
                  5'b0_0000: jump_req_n = 1'b1; // DNPC
                  default:;
                endcase
              end
            end

            6'b00_0100: begin // General-Purpose Registers
              debug_gnt_o = 1'b1; // grant it even when invalid access to not block

              if (debug_halted_o) begin
                regfile_wreq = 1'b1;
              end
            end

            default: debug_gnt_o = 1'b1; // grant it even when invalid access to not block
          endcase
        end
      end else begin
        //----------------------------------------------------------------------------
        // read access
        //----------------------------------------------------------------------------
        if (debug_addr_i[14]) begin
          debug_gnt_o = 1'b1; // grant it even when invalid access to not block

          // CSR access
          if (debug_halted_o) begin
            // access to internal registers are only allowed when the core is halted
            csr_req_n   = 1'b1;
            rdata_sel_n = RD_CSR;
          end
        end else begin
          // non-CSR access
          unique case (debug_addr_i[13:8])
            6'b00_0000: begin // Debug Registers, always accessible
              debug_gnt_o = 1'b1;

              rdata_sel_n = RD_DBGA;
            end

            6'b10_0000: begin // Debug Registers, only accessible when in debug
              debug_gnt_o = 1'b1; // grant it even when invalid access to not block

              rdata_sel_n = RD_DBGS;
            end

            6'b00_0100: begin // General-Purpose Registers
              debug_gnt_o = 1'b1; // grant it even when invalid access to not block

              if (debug_halted_o) begin
                regfile_rreq_n = 1'b1;
                rdata_sel_n    = RD_GPR;
              end
            end

            default: debug_gnt_o = 1'b1; // grant it even when invalid access to not block
          endcase
        end
      end
    end
  end

  //----------------------------------------------------------------------------
  // debug register read access
  //
  // Since those are combinational, we can do it in the cycle where we set
  // rvalid. The address has been latched into addr_q
  //----------------------------------------------------------------------------
  always_comb
  begin
    dbg_rdata = '0;

    case (rdata_sel_q)
      RD_DBGA: begin
        unique case (addr_q[6:2])
          5'h00: dbg_rdata[31:0] = {15'b0, debug_halted_o, 15'b0, settings_q[DBG_SETS_SSTE]}; // DBG_CTRL
          5'h01: dbg_rdata[31:0] = {15'b0, sleeping_i, 15'b0, dbg_ssth_q}; // DBG_HIT
          5'h02: begin // DBG_IE
            dbg_rdata[31:16] = '0;
            dbg_rdata[15:12] = '0;
            dbg_rdata[11]    = settings_q[DBG_SETS_ECALL];
            dbg_rdata[10: 8] = '0;
            dbg_rdata[ 7]    = settings_q[DBG_SETS_ELSU];
            dbg_rdata[ 6]    = 1'b0;
            dbg_rdata[ 5]    = settings_q[DBG_SETS_ELSU];
            dbg_rdata[ 4]    = 1'b0;
            dbg_rdata[ 3]    = settings_q[DBG_SETS_EBRK];
            dbg_rdata[ 2]    = settings_q[DBG_SETS_EILL];
            dbg_rdata[ 1: 0] = '0;
          end
          5'h03: dbg_rdata = {dbg_cause_q[5], 26'b0, dbg_cause_q[4:0]}; // DBG_CAUSE
          5'h10: dbg_rdata = '0; // DBG_BPCTRL0
          5'h12: dbg_rdata = '0; // DBG_BPCTRL1
          5'h14: dbg_rdata = '0; // DBG_BPCTRL2
          5'h16: dbg_rdata = '0; // DBG_BPCTRL3
          5'h18: dbg_rdata = '0; // DBG_BPCTRL4
          5'h1A: dbg_rdata = '0; // DBG_BPCTRL5
          5'h1C: dbg_rdata = '0; // DBG_BPCTRL6
          5'h1E: dbg_rdata = '0; // DBG_BPCTRL7
          default:;
        endcase
      end

      RD_DBGS: begin
        unique case (addr_q[2:2])
          1'b0: dbg_rdata = npc_int; // DBG_NPC
          1'b1: dbg_rdata = ppc_int; // DBG_PPC
          default:;
        endcase
      end

      default:;
    endcase
  end

  //----------------------------------------------------------------------------
  // read data mux
  //----------------------------------------------------------------------------
  always_comb
  begin
    debug_rdata_o = '0;

    case (rdata_sel_q)
      RD_CSR:  debug_rdata_o = csr_rdata_i;
      RD_GPR:  debug_rdata_o = regfile_rdata_i;
      RD_DBGA: debug_rdata_o = dbg_rdata;
      RD_DBGS: debug_rdata_o = dbg_rdata;
    endcase
  end

  //----------------------------------------------------------------------------
  // rvalid generation
  //----------------------------------------------------------------------------
  always_ff @(posedge clk, negedge rst_n)
  begin
    if (~rst_n) begin
      debug_rvalid_o <= 1'b0;
    end else begin
      debug_rvalid_o <= debug_gnt_o; // always give the rvalid one cycle after gnt
    end
  end

  //----------------------------------------------------------------------------
  // stall control
  //----------------------------------------------------------------------------
  always_comb
  begin
    stall_ns       = stall_cs;
    dbg_req_o      = 1'b0;
    stall_o        = 1'b0;
    debug_halted_o = 1'b0;
    dbg_cause_n    = dbg_cause_q;
    dbg_ssth_n     = dbg_ssth_q;

    case (stall_cs)
      RUNNING: begin
        dbg_ssth_n = 1'b0;

        if (dbg_halt | debug_halt_i | trap_i) begin
          dbg_req_o   = 1'b1;
          stall_ns    = HALT_REQ;

          if (trap_i) begin
            if (settings_q[DBG_SETS_SSTE])
              dbg_ssth_n = 1'b1;

            dbg_cause_n = exc_cause_i;
          end else begin
            dbg_cause_n = DBG_CAUSE_HALT;
          end
        end
      end

      HALT_REQ: begin
        dbg_req_o = 1'b1;

        if (dbg_ack_i)
          stall_ns = HALT;

        if (dbg_resume | debug_resume_i)
          stall_ns = RUNNING;
      end

      HALT: begin
        stall_o        = 1'b1;
        debug_halted_o = 1'b1;

        if (dbg_resume | debug_resume_i) begin
          stall_ns = RUNNING;
          stall_o  = 1'b0;
        end
      end
    endcase

    if (ssth_clear)
      dbg_ssth_n = 1'b0;
  end

  always_ff @(posedge clk, negedge rst_n)
  begin
    if (~rst_n) begin
      stall_cs    <= RUNNING;
      dbg_cause_q <= DBG_CAUSE_HALT;
      dbg_ssth_q  <= 1'b0;
    end else begin
      stall_cs    <= stall_ns;
      dbg_cause_q <= dbg_cause_n;
      dbg_ssth_q  <= dbg_ssth_n;
    end
  end

  //----------------------------------------------------------------------------
  // NPC/PPC selection
  //----------------------------------------------------------------------------
  always_comb
  begin
    pc_tracking_fsm_ns = pc_tracking_fsm_cs;

    ppc_int = pc_id_i;
    npc_int = pc_if_i;

    // PPC/NPC mux
    unique case (pc_tracking_fsm_cs)
      IFID: begin
        ppc_int = pc_id_i;
        npc_int = pc_if_i;
      end

      IFEX: begin
        ppc_int = pc_ex_i;
        npc_int = pc_if_i;
      end

      IDEX: begin
        ppc_int = pc_ex_i;
        npc_int = pc_id_i;

        if (jump_req_o)
          pc_tracking_fsm_ns = IFEX;
      end

      default: begin
        pc_tracking_fsm_ns = IFID;
      end
    endcase

    // set state if trap is encountered
    if (dbg_ack_i) begin
      pc_tracking_fsm_ns = IFID;

      if (branch_in_ex_i) begin
        if (branch_taken_i)
          pc_tracking_fsm_ns = IFEX;
        else
          pc_tracking_fsm_ns = IDEX;
      end else if (data_load_event_i) begin
        // for p.elw
        if (instr_valid_id_i)
           pc_tracking_fsm_ns = IDEX;
        else
           pc_tracking_fsm_ns = IFEX;
      end
    end
  end


  always_ff @(posedge clk, negedge rst_n)
  begin
    if (~rst_n) begin
      pc_tracking_fsm_cs <= IFID;

      addr_q             <= '0;
      wdata_q            <= '0;
      state_q            <= FIRST;
      rdata_sel_q        <= RD_NONE;
      regfile_rreq_q     <= 1'b0;
      csr_req_q          <= 1'b0;
      jump_req_q         <= 1'b0;

      settings_q         <= 1'b0;
    end else begin
      pc_tracking_fsm_cs <= pc_tracking_fsm_ns;

      // settings
      settings_q         <= settings_n;

      // for the actual interface
      if (debug_req_i) begin
        addr_q  <= debug_addr_i;
        wdata_q <= debug_wdata_i;
        state_q <= state_n;
      end

      if (debug_req_i | debug_rvalid_o) begin
        // wait for either req or rvalid to set those FFs
        // This makes sure that they are only triggered once when there is
        // only one request, and the FFs can be properly clock gated otherwise
        regfile_rreq_q <= regfile_rreq_n;
        csr_req_q      <= csr_req_n;
        jump_req_q     <= jump_req_n;
        rdata_sel_q    <= rdata_sel_n;
      end
    end
  end

  assign regfile_rreq_o  = regfile_rreq_q;
  assign regfile_raddr_o = addr_q[6:2];

  assign regfile_wreq_o  = regfile_wreq;
  assign regfile_waddr_o = debug_addr_i[6:2];
  assign regfile_wdata_o = debug_wdata_i;

  assign csr_req_o       = csr_req_q;
  assign csr_addr_o      = addr_q[13:2];
  assign csr_wdata_o     = wdata_q;

  assign jump_req_o      = jump_req_q;
  assign jump_addr_o     = wdata_q;

  assign settings_o      = settings_q;

`ifndef verilator
  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------

  // check that no registers are accessed when we are not in debug mode
  assert property (
    @(posedge clk) (debug_req_i) |-> ((debug_halted_o == 1'b1) ||
                                      ((debug_addr_i[14] != 1'b1) &&
                                       (debug_addr_i[13:7] != 5'b0_1001)  &&
                                       (debug_addr_i[13:7] != 5'b0_1000)) ) )
    else $warning("Trying to access internal debug registers while core is not stalled");

  // check that all accesses are word-aligned
  assert property (
    @(posedge clk) (debug_req_i) |-> (debug_addr_i[1:0] == 2'b00) );
`endif //  `ifndef verilator

endmodule // debug_unit

// RAM wrapper for RI5CY

// Copyright (C) 2017 Embecosm <www.embecosm.com>

// Contributor: Jeremy Bennett <jeremy.bennett@embecosm.com>

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along with
// this program.  If not, see <http://www.gnu.org/licenses/>.


// This maps the dp_ram module of pulpino to the instruction and data ports of
// the RI5CY processor core.

// 4MB of RAM, matching the PicoRV32 implementation.

module ram
  #(
    parameter ADDR_WIDTH = 10
  )(
    input  logic                   clk,

    input  logic                   instr_req_i,
    input  logic [ADDR_WIDTH-1:0]  instr_addr_i,
    output logic [127:0]           instr_rdata_o,
    output logic                   instr_rvalid_o,
    output logic                   instr_gnt_o,

    input  logic                   data_req_i,
    input  logic [ADDR_WIDTH-1:0]  data_addr_i,
    input  logic                   data_we_i,
    input  logic [3:0]             data_be_i,
    input  logic [31:0]            data_wdata_i,
    output logic [31:0]            data_rdata_o,
    output logic                   data_rvalid_o,
    output logic                   data_gnt_o,

    // HDC activation result (signed 16-bit)
    input  logic signed [15:0]     y_i
  );

   // -----------------------------------------------------------------
   //  The hijack: on any store, write the 16-bit HDC result
   //  (sign-extended into 32 bits) instead of the CPU's store data.
   //  Loads are unaffected (store-data is a don't-care on reads).
   // -----------------------------------------------------------------
   logic [31:0] wdata_mux;
   logic [31:0] y_packed;

   // Sign-extend the signed 16-bit result to fill the 32-bit word.
   assign y_packed = {{16{y_i[15]}}, y_i};

   // Enable the hijack so software can read the result back from memory.
   assign wdata_mux = data_we_i ? y_packed : data_wdata_i;
   // (For a plain pass-through instead, use: assign wdata_mux = data_wdata_i;)

   dp_ram
     #(
       .ADDR_WIDTH (ADDR_WIDTH)
       )
   dp_ram_i
     (
      .clk       ( clk           ),

      .en_a_i    ( instr_req_i   ),
      .addr_a_i  ( instr_addr_i  ),
      .wdata_a_i ( '0            ),
      .rdata_a_o ( instr_rdata_o ),
      .we_a_i    ( '0            ),
      .be_a_i    ( 4'b1111       ),

      .en_b_i    ( data_req_i    ),
      .addr_b_i  ( data_addr_i   ),
      .wdata_b_i ( wdata_mux     ),
      .rdata_b_o ( data_rdata_o  ),
      .we_b_i    ( data_we_i     ),
      .be_b_i    ( data_be_i     )
      );

   assign data_gnt_o  = data_req_i;
   assign instr_gnt_o = instr_req_i;

   always_ff @(posedge clk)
     begin
        data_rvalid_o  <= data_req_i;
        instr_rvalid_o <= instr_req_i;
     end

endmodule

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the âLicenseâ?); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an âAS ISâ? BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

module dp_ram
  #(
    parameter ADDR_WIDTH = 8
  )(
    input  logic clk,
    input  logic                   en_a_i,
    input  logic [ADDR_WIDTH-1:0]  addr_a_i,
    input  logic [31:0]            wdata_a_i,
    output logic [127:0]           rdata_a_o,
    input  logic                   we_a_i,
    input  logic [3:0]             be_a_i,
    input  logic                   en_b_i,
    input  logic [ADDR_WIDTH-1:0]  addr_b_i,
    input  logic [31:0]            wdata_b_i,
    output logic [31:0]            rdata_b_o,
    input  logic                   we_b_i,
    input  logic [3:0]             be_b_i
  );

  localparam bytes = 2**ADDR_WIDTH;


    logic [7:0] mem[bytes];
  logic [ADDR_WIDTH-1:0] addr_b_int;
  integer idx;

	initial begin
    for (idx = 0; idx < bytes; idx = idx + 1) mem[idx] = 8'h00;
end


  // ==== Port A / Port B logic (unchanged) ====
  always_comb addr_b_int = {addr_b_i[ADDR_WIDTH-1:2], 2'b0};

  always @(posedge clk) begin
    rdata_a_o[  0+: 8] <= mem[addr_a_i +  0];
    rdata_a_o[  8+: 8] <= mem[addr_a_i +  1];
    rdata_a_o[ 16+: 8] <= mem[addr_a_i +  2];
    rdata_a_o[ 24+: 8] <= mem[addr_a_i +  3];
    rdata_a_o[ 32+: 8] <= mem[addr_a_i +  4];
    rdata_a_o[ 40+: 8] <= mem[addr_a_i +  5];
    rdata_a_o[ 48+: 8] <= mem[addr_a_i +  6];
    rdata_a_o[ 56+: 8] <= mem[addr_a_i +  7];
    rdata_a_o[ 64+: 8] <= mem[addr_a_i +  8];
    rdata_a_o[ 72+: 8] <= mem[addr_a_i +  9];
    rdata_a_o[ 80+: 8] <= mem[addr_a_i + 10];
    rdata_a_o[ 88+: 8] <= mem[addr_a_i + 11];
    rdata_a_o[ 96+: 8] <= mem[addr_a_i + 12];
    rdata_a_o[104+: 8] <= mem[addr_a_i + 13];
    rdata_a_o[112+: 8] <= mem[addr_a_i + 14];
    rdata_a_o[120+: 8] <= mem[addr_a_i + 15];

    if (en_b_i) begin
      if (we_b_i) begin
        if (be_b_i[0]) mem[addr_b_int    ] <= wdata_b_i[ 0+:8];
        if (be_b_i[1]) mem[addr_b_int + 1] <= wdata_b_i[ 8+:8];
        if (be_b_i[2]) mem[addr_b_int + 2] <= wdata_b_i[16+:8];
        if (be_b_i[3]) mem[addr_b_int + 3] <= wdata_b_i[24+:8];
      end else begin
        rdata_b_o[ 7: 0] <= mem[addr_b_int    ];
        rdata_b_o[15: 8] <= mem[addr_b_int + 1];
        rdata_b_o[23:16] <= mem[addr_b_int + 2];
        rdata_b_o[31:24] <= mem[addr_b_int + 3];
      end
    end
  end

  function [7:0] readByte;
    input integer byte_addr;
    readByte = mem[byte_addr];
  endfunction

  task writeByte;
    input integer byte_addr;
    input [7:0] val;
    mem[byte_addr] = val;
  endtask

endmodule

// ==========================================================================
// HDC accelerator wrapper - drop-in replacement for mac_10_split.
// Captures ONE packed 32-bit word on the first `load` after `top_start`:
//   data_path[15:0]  -> x_in (signed Q6.10)
//   data_path[17:16] -> func_sel
// Latches operands stable, runs hdc_activation, returns y + top_done.
// ==========================================================================
module hdc_split #(
    parameter D          = 1760,   // D / BLOCK_SIZE must == 80
    parameter BLOCK_SIZE = 22,     // and D % CHUNK_SIZE(110) == 0
    parameter FUNC_LSB   = 16      // func_sel = data_path[FUNC_LSB +: 2]
)(
    input                    clk,
    input                    rst,        // active high (= !rstn_i)
    input                    top_start,
    input             [31:0] data_path,
    input                    load,       // = data_rvalid
    output reg               top_done,
    output reg signed [15:0] y
);

    localparam IDLE = 2'd0;
    localparam WAIT = 2'd1;   // wait for load, capture the packed word
    localparam RUN  = 2'd2;   // hdc_activation crunching
    localparam FIN  = 2'd3;   // hold done, wait for top_start to drop

    reg  [1:0]         state;
    reg                hdc_start;
    reg  signed [15:0] x_in_r;
    reg  [1:0]         func_sel_r;

    wire signed [15:0] y_out;
    wire               done;

    hdc_activation #(.D(D), .BLOCK_SIZE(BLOCK_SIZE)) hdc_i (
        .clk      (clk),
        .rst      (rst),
        .start    (hdc_start),
        .x_in     (x_in_r),      // held stable for the whole run
        .func_sel (func_sel_r),
        .y_out    (y_out),
        .done     (done)
    );

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state      <= IDLE;
            hdc_start  <= 1'b0;
            top_done   <= 1'b0;
            y          <= 16'sd0;
            x_in_r     <= 16'sd0;
            func_sel_r <= 2'b00;
        end else begin
            hdc_start <= 1'b0;   // default: start is a 1-cycle pulse
            top_done  <= 1'b0;

            case (state)
                IDLE: begin
                    if (top_start) state <= WAIT;
                end

                // Grab the first returned word after top_start
                WAIT: begin
                    if (load) begin
                        x_in_r     <= data_path[15:0];
                        func_sel_r <= data_path[FUNC_LSB +: 2];
                        hdc_start  <= 1'b1;   // fire once; operands now latched
                        state      <= RUN;
                    end
                end

                RUN: begin
                    if (done) begin
                        y        <= y_out;
                        top_done <= 1'b1;
                        state    <= FIN;
                    end
                end

                FIN: begin
                    if (!top_start) state <= IDLE;
                end
            endcase
        end
    end
endmodule

// ==========================================================================
// HDC ACTIVATION -- ULTRA-LOW AREA SYNTHESIS (GRANULAR CHUNKS & TIGHT BITS)
// Applies the Double-Base Sliding Window to 22-bit slices per clock cycle.
// VERSION: Old Division-Based Quantization (0.1 steps, [-4.0, 0.0] domain)
// ==========================================================================

module hdc_activation #(
parameter D = 1760,
parameter BLOCK_SIZE = 22,
parameter VOCAB_SIZE = 36,
parameter INPUT_STEPS = 41,
parameter CHUNK_SIZE = 22,             // GRANULAR CHUNKING: 5x smaller adders!
parameter NUM_CHUNKS = D / CHUNK_SIZE  // Takes 80 clock cycles to complete
)(
input wire clk,
input wire rst,
input wire start,
input wire signed [15:0] x_in,
input wire [1:0] func_sel,
output reg signed [15:0] y_out,
output reg done
);

// ==========================================================================
// TRADITIONAL DIVISION QUANTIZATION (0.1 Step Size over [-4.0, 0.0] Domain)
// 4.0 in Q6.10 = 4096. 0.1 step = 102.
// ==========================================================================
wire is_positive = (x_in >= 0);
wire signed [15:0] abs_x = is_positive ? x_in : -x_in;
wire out_of_bounds = (abs_x > 16'd4096);

// Nearest Rounding Division via Hardware Divider
// 1. Add rounding offset (Half of 102.4 is approx 51)
wire [15:0] val = abs_x + 16'd51;

// 2. Multiply by 5 using Shift-and-Add (val * 5 = val + val*4)
wire [15:0] val_times_5 = val + (val << 2);

// 3. Divide by 512 via Bit-Shift
wire [15:0] rounded_index = val_times_5 >> 9;
wire [5:0] raw_index = (rounded_index >= 16'd40) ? 6'd40 : rounded_index[5:0];
wire [5:0] input_index = 6'd40 - raw_index;

wire [6:0] active_func_idx = (func_sel == 2'b00) ? 7'd41 :
                             (func_sel == 2'b01) ? 7'd42 :
                             (func_sel == 2'b10) ? 7'd43 : 7'd41;

// ==========================================================================
// THE "DOUBLE BASE" SLIDING WINDOW
// ==========================================================================
localparam [D-1:0] BASE_VECTOR = {
    22'b1010110111110000001010, // LUT 0
    22'b0101011000000111110111,
    22'b1001001101101011000000,
    22'b1110100000001100010001,
    22'b0101100101111110111011,
    22'b0000101000011000100110,
    22'b0011111110101101101101,
    22'b0100110011101011011100,
    22'b1011000011010111101100,
    22'b1100010110101000100111,
    22'b0000110110010110110000,
    22'b1110111001110101110100,
    22'b0111011101001100001010,
    22'b0101111010010000001001,
    22'b0011001111011001010111,
    22'b1001011101010010111101,
    22'b1100101111000110001111,
    22'b1011001010100000110010,
    22'b0010001100110111001011,
    22'b1011010000101010001110,
    22'b1111010001110110000001,
    22'b0110001010111010000101,
    22'b0100000111100101101001,
    22'b0001000100110100010101,
    22'b1001110110100011111011,
    22'b1110100100010011111110,
    22'b1010011100001001111000,
    22'b0101010001011001100000,
    22'b0010111101000011000110,
    22'b1110111001011010101011,
    22'b0010001010101101011110,
    22'b1101100111001101010100,
    22'b0111100110011111000001,
    22'b1111100011011010110000,
    22'b1100000001000010101000,
    22'b0000111101100100100011,
    22'b0001101011101100010001,
    22'b1010010110100101010001,
    22'b1100011111011101110001,
    22'b1000110011010000010101,
    22'b0000000000001110001101,
    22'b1111101110000101101010,
    22'b1000100011001011000011,
    22'b1110001111111110111110,
    22'b0111100101000000100101,
    22'b1000110000111111011010,
    22'b1100001010011000011010,
    22'b0111111100110001011111,
    22'b0011101100110010101100,
    22'b1100001001101001001111,
    22'b1010100101101000111111,
    22'b1011011000010101000110,
    22'b0101010111010010000110,
    22'b1101111101010111010010,
    22'b1000001001100110010110,
    22'b1111100000111001000100,
    22'b0110110011110111101111,
    22'b1001111100100100011000,
    22'b0010101001010011110001,
    22'b0110000011010101010000,
    22'b0101100000100011110000,
    22'b1111011110110011100000,
    22'b0001011011110101111010,
    22'b1000001010110001111101,
    22'b0011110001011101011001,
    22'b0110010100101110100000,
    22'b1011010011001100110111,
    22'b0111000011101011100110,
    22'b0011110101100101110110,
    22'b1110101010110101000011,
    22'b0110010110000100111110,
    22'b0111001001010000111110,
    22'b0101011110101001010010,
    22'b0011111010111110000010,
    22'b1101011110011110000101,
    22'b0100001010011111101001,
    22'b0011011000000000100001,
    22'b1101111101001000011111,
    22'b1011100010100110011101,
    22'b0010101100101110111010  // LUT 79
};

localparam [2*D-1:0] DOUBLE_BASE = {BASE_VECTOR, BASE_VECTOR};

localparam [D-1:0] SUPER_VECTOR = 1760'b01100011111111011101011010001010010000000010100001100100110100110001010001100110000001000011011000011001110001100000111101001001000010000001111010110111101110101000110100001100111011100111001101100011101100011011000111001011001110100101110111110000011100001010111101101111110100111110000010000001101111011010111001011111110000101110100101111101001011100100000111000100100010101010101000101110010000010011001110101100110011011110000101000001110111110011001110100110101100011011111101110001110111010110100000000001010111010010010001010100111000100101111010000111001011101101111001011010111001001001101100011111000111000101100101000000111101100100000100010011100100000001001111011010011110111100010110110111101101101000110010000011010101111111010001010000010110110111111001000110111000011111001100110011100100101010111110111100100111011100101100110110001011010111110010011110100010011100010000011111001000001100110011111100001010000001000000011111110100000110100010111011101011110100011100110011100111100000011000111001001011111100010001000101010111011110001110001111100100000000000011111110000110011010101100011100000001010001011010101011110101110111111110001000010001100000001111110000001010011111001001011100100001111011110010010100010001110111110101000111101010011100100111010110101000100000001000000100001001101001110011010100110110111011011011010110110011000010000100011111100001010110001000000000100001110011000000010101001000110010111101100100100000111100010100001001000110100110010011110101101001011101010110101100010111001000111111000010110010001110000100111010100101011110101011111101001110100011100010111101011111001001110101100101100011001111001100011100001111110000011101001111101000011000001101111000100111111110000100000011010001101100011100010101;

// ==========================================================================
// STREAMING CHUNK GENERATION (22 BITS AT A TIME)
// Parameterized to scale beautifully. 80-cycle latency, but ultra-low area.
// ==========================================================================
reg [6:0] chunk_counter; // 7-bit counter safely holds up to 80

// Constant bit-slice offset calculation
wire [11:0] input_offset = 12'd1760 - (input_index * 12'd22) + (chunk_counter * CHUNK_SIZE);
wire [11:0] func_offset  = 12'd1760 - (active_func_idx * 12'd22) + (chunk_counter * CHUNK_SIZE);

wire [CHUNK_SIZE-1:0] input_chunk = DOUBLE_BASE[input_offset +: CHUNK_SIZE];
wire [CHUNK_SIZE-1:0] func_chunk  = DOUBLE_BASE[func_offset +: CHUNK_SIZE];
wire [CHUNK_SIZE-1:0] super_chunk = SUPER_VECTOR[chunk_counter * CHUNK_SIZE +: CHUNK_SIZE];

// Compute exact CHUNK_SIZE bits of the XOR datapath per clock cycle
wire [CHUNK_SIZE-1:0] active_noisy_chunk = super_chunk ^ (input_chunk ^ func_chunk);

// Constant offset dictionary generation
wire [CHUNK_SIZE-1:0] output_dict_chunks [0:35];
genvar g;
generate
    for (g = 0; g < 36; g = g + 1) begin : OUT_DICT_GEN
        // Synthesizer calculates this subtraction entirely at compile time
        localparam [11:0] DICT_START_OFFSET = 1760 - ((g + 44) * 22);
        wire [11:0] dict_offset = DICT_START_OFFSET + (chunk_counter * CHUNK_SIZE);
        
        // This maps to 36 tiny 22-bit multiplexers!
        assign output_dict_chunks[g] = DOUBLE_BASE[dict_offset +: CHUNK_SIZE];
    end
endgenerate

// ==========================================================================
// PURE COMBINATIONAL OUTPUT MULTIPLEXER
// ==========================================================================
function [15:0] get_output_value;
    input [5:0] idx;
    begin
        case (idx)
            6'd0: get_output_value = 16'b1111111010011010;
            6'd1: get_output_value = 16'b1111111010100100;
            6'd2: get_output_value = 16'b1111111010101110;
            6'd3: get_output_value = 16'b1111111010111000;
            6'd4: get_output_value = 16'b1111111011000011;
            6'd5: get_output_value = 16'b1111111011001101;
            6'd6: get_output_value = 16'b1111111011010111;
            6'd7: get_output_value = 16'b1111111011100001;
            6'd8: get_output_value = 16'b1111111011101100;
            6'd9: get_output_value = 16'b1111111011110110;
            6'd10: get_output_value = 16'b1111111100000000;
            6'd11: get_output_value = 16'b1111111100001010;
            6'd12: get_output_value = 16'b1111111100010100;
            6'd13: get_output_value = 16'b1111111100011111;
            6'd14: get_output_value = 16'b1111111100101001;
            6'd15: get_output_value = 16'b1111111100110011;
            6'd16: get_output_value = 16'b1111111100111101;
            6'd17: get_output_value = 16'b1111111101001000;
            6'd18: get_output_value = 16'b1111111101010010;
            6'd19: get_output_value = 16'b1111111101011100;
            6'd20: get_output_value = 16'b1111111101100110;
            6'd21: get_output_value = 16'b1111111101110001;
            6'd22: get_output_value = 16'b1111111101111011;
            6'd23: get_output_value = 16'b1111111110000101;
            6'd24: get_output_value = 16'b1111111110001111;
            6'd25: get_output_value = 16'b1111111110011010;
            6'd26: get_output_value = 16'b1111111110100100;
            6'd27: get_output_value = 16'b1111111110101110;
            6'd28: get_output_value = 16'b1111111110111000;
            6'd29: get_output_value = 16'b1111111111000011;
            6'd30: get_output_value = 16'b1111111111001101;
            6'd31: get_output_value = 16'b1111111111010111;
            6'd32: get_output_value = 16'b1111111111100001;
            6'd33: get_output_value = 16'b1111111111101100;
            6'd34: get_output_value = 16'b1111111111110110;
            6'd35: get_output_value = 16'b0000000000000000;
            default: get_output_value = 16'd0;
        endcase
    end
endfunction

// ==========================================================================
// POPCOUNT ACCUMULATION TREE (TIGHTENED BIT-WIDTHS)
// ==========================================================================

// TIGHTENING: 7-bit return safely captures any 22-bit popcount up to 127
function [6:0] popcount_chunk;
    input [CHUNK_SIZE-1:0] vec;
    integer k;
    begin
        popcount_chunk = 0;
        for (k = 0; k < CHUNK_SIZE; k = k + 1) begin
            popcount_chunk = popcount_chunk + vec[k];
        end
    end
endfunction

localparam STATE_IDLE   = 2'd0;
localparam STATE_CALC   = 2'd1;
localparam STATE_ARGMIN = 2'd2;
localparam STATE_DONE   = 2'd3;

reg [1:0] state;

// TIGHTENING: Max distance is 1760. 11 bits can hold 2047! (Saved 108 Flip Flops)
reg [10:0] hamming_distances [0:35];

integer i;
reg [10:0] current_min;
reg [5:0] calculated_best_idx;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state <= STATE_IDLE;
        done <= 0;
        y_out <= 0;
        chunk_counter <= 0;
    end else begin
        case (state)
            STATE_IDLE: begin
                done <= 0;
                chunk_counter <= 0;
                if (start) begin
                    for (i = 0; i < VOCAB_SIZE; i = i + 1) begin
                        hamming_distances[i] <= 0;
                    end
                    state <= STATE_CALC;
                end
            end

            STATE_CALC: begin
                for (i = 0; i < VOCAB_SIZE; i = i + 1) begin
                    hamming_distances[i] <= hamming_distances[i] + 
                        popcount_chunk(active_noisy_chunk ^ output_dict_chunks[i]);
                end
                
                if (chunk_counter == NUM_CHUNKS - 1) begin
                    state <= STATE_ARGMIN;
                end else begin
                    chunk_counter <= chunk_counter + 1;
                end
            end

            STATE_ARGMIN: begin
                current_min = 11'h7FF; // 2047 (Acts as infinity for 1760 max)
                calculated_best_idx = 0;
                for (i = 0; i < VOCAB_SIZE; i = i + 1) begin
                    if (hamming_distances[i] < current_min) begin
                        current_min = hamming_distances[i];
                        calculated_best_idx = i[5:0];
                    end
                end

                if (func_sel == 2'b10) begin
                    if (is_positive) y_out <= x_in;
                    else if (out_of_bounds) y_out <= 0;
                    else y_out <= get_output_value(calculated_best_idx);
                end else begin
                    if (out_of_bounds) y_out <= is_positive ? x_in : 16'd0;
                    else if (is_positive) y_out <= x_in + get_output_value(calculated_best_idx);
                    else y_out <= get_output_value(calculated_best_idx);
                end
                state <= STATE_DONE;
            end

            STATE_DONE: begin
                done <= 1;
                state <= STATE_IDLE;
            end
        endcase
    end
end


endmodule
