// Copyright lowRISC contributors.
// Copyright 2018 ETH Zurich and University of Bologna, see also CREDITS.md.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

/**
 * Execution stage
 *
 * Execution block: Hosts ALU and MUL/DIV unit
 */
module ibex_ex_block #(
  parameter ibex_pkg::rv32m_e RV32M           = ibex_pkg::RV32MFast,
  parameter ibex_pkg::rv32b_e RV32B           = ibex_pkg::RV32BNone,
  parameter bit               BranchTargetALU = 0
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  // ALU
  input  ibex_pkg::alu_op_e     alu_operator_i,
  input  logic [31:0]           alu_operand_a_i,
  input  logic [31:0]           alu_operand_b_i,
  input  logic                  alu_instr_first_cycle_i,

  // Branch Target ALU
  // All of these signals are unusued when BranchTargetALU == 0
  input  logic [31:0]           bt_a_operand_i,
  input  logic [31:0]           bt_b_operand_i,

  // Multiplier/Divider
  input  ibex_pkg::md_op_e      multdiv_operator_i,
  input  logic                  mult_en_i,             // dynamic enable signal, for FSM control
  input  logic                  div_en_i,              // dynamic enable signal, for FSM control
  input  logic                  mult_sel_i,            // static decoder output, for data muxes
  input  logic                  div_sel_i,             // static decoder output, for data muxes
  input  logic  [1:0]           multdiv_signed_mode_i,
  input  logic [31:0]           multdiv_operand_a_i,
  input  logic [31:0]           multdiv_operand_b_i,
  input  logic                  multdiv_ready_id_i,
  input  logic                  data_ind_timing_i,

  // intermediate val reg
  output logic [1:0]            imd_val_we_o,
  output logic [33:0]           imd_val_d_o[2],
  input  logic [33:0]           imd_val_q_i[2],

  // Outputs
  output logic [31:0]           alu_adder_result_ex_o, // to LSU
  output logic [31:0]           result_ex_o,
  output logic [31:0]           branch_target_o,       // to IF
  output logic                  branch_decision_o,     // to ID

  output logic                  ex_valid_o,             // EX has valid output


  // D* Lite custom ISA extensions
  input  logic                  custom_insn_i,         // custom instruction active
  input  logic                  custom_en_i,           // one-shot enable (first executing cycle)
  input  logic [2:0]            custom_funct3_i,       // funct3 field
  input  logic [6:0]            custom_funct7_i,       // funct7 field
  output logic [31:0]           custom_result_o,       // result to writeback

  // D* Lite CSR configuration inputs
  input  logic [31:0]           dstar_state_base_i,
  input  logic [31:0]           dstar_blocked_base_i,
  input  logic [31:0]           dstar_grid_width_i,
  input  logic [31:0]           dstar_grid_height_i,
  input  logic [31:0]           dstar_start_i,
  input  logic [31:0]           dstar_km_i,
  input  logic [31:0]           dstar_infinity_i,
  input  logic [31:0]           dstar_straight_cost_i,
  input  logic [31:0]           dstar_diagonal_cost_i,
  input  logic [31:0]           dstar_heuristic_type_i,
  input  logic [31:0]           dstar_connectivity_i,

  // D* Lite private data bus
  output logic                  dstar_data_req_o,
  output logic                  dstar_data_we_o,
  output logic [3:0]            dstar_data_be_o,
  output logic [31:0]           dstar_data_addr_o,
  output logic [31:0]           dstar_data_wdata_o,
  input  logic                  dstar_data_gnt_i,
  input  logic                  dstar_data_rvalid_i,
  input  logic [31:0]           dstar_data_rdata_i
);

  import ibex_pkg::*;

  logic [31:0] alu_result, multdiv_result;

  logic [32:0] multdiv_alu_operand_b, multdiv_alu_operand_a;

  logic [33:0] alu_adder_result_ext;
  logic        alu_cmp_result, alu_is_equal_result;
  logic        multdiv_valid;
  logic        multdiv_sel;
  logic [31:0] alu_imd_val_q[2];
  logic [31:0] alu_imd_val_d[2];
  logic [ 1:0] alu_imd_val_we;
  logic [33:0] multdiv_imd_val_d[2];
  logic [ 1:0] multdiv_imd_val_we;

  // D* Lite custom execute unit signals
  logic [31:0] custom_result;
  logic        custom_valid;
  logic        custom_sel;
  assign custom_sel = custom_insn_i;

  /*
    The multdiv_i output is never selected if RV32M=RV32MNone
    At synthesis time, all the combinational and sequential logic
    from the multdiv_i module are eliminated
  */
  if (RV32M != RV32MNone) begin : gen_multdiv_m
    assign multdiv_sel = mult_sel_i | div_sel_i;
  end else begin : gen_multdiv_no_m
    assign multdiv_sel = 1'b0;
  end

  // Intermediate Value Register Mux
  assign imd_val_d_o[0] = multdiv_sel ? multdiv_imd_val_d[0] : {2'b0, alu_imd_val_d[0]};
  assign imd_val_d_o[1] = multdiv_sel ? multdiv_imd_val_d[1] : {2'b0, alu_imd_val_d[1]};
  assign imd_val_we_o   = multdiv_sel ? multdiv_imd_val_we : alu_imd_val_we;

  assign alu_imd_val_q = '{imd_val_q_i[0][31:0], imd_val_q_i[1][31:0]};

  assign result_ex_o  = multdiv_sel ? multdiv_result :
                        custom_sel  ? custom_result  :
                        alu_result;

  // branch handling
  assign branch_decision_o  = alu_cmp_result;

  if (BranchTargetALU) begin : g_branch_target_alu
    logic [32:0] bt_alu_result;
    logic        unused_bt_carry;

    assign bt_alu_result   = bt_a_operand_i + bt_b_operand_i;

    assign unused_bt_carry = bt_alu_result[32];
    assign branch_target_o = bt_alu_result[31:0];
  end else begin : g_no_branch_target_alu
    // Unused bt_operand signals cause lint errors, this avoids them
    logic [31:0] unused_bt_a_operand, unused_bt_b_operand;

    assign unused_bt_a_operand = bt_a_operand_i;
    assign unused_bt_b_operand = bt_b_operand_i;

    assign branch_target_o = alu_adder_result_ex_o;
  end

  /////////
  // ALU //
  /////////

  ibex_alu #(
    .RV32B(RV32B)
  ) alu_i (
    .operator_i         (alu_operator_i),
    .operand_a_i        (alu_operand_a_i),
    .operand_b_i        (alu_operand_b_i),
    .instr_first_cycle_i(alu_instr_first_cycle_i),
    .imd_val_q_i        (alu_imd_val_q),
    .imd_val_we_o       (alu_imd_val_we),
    .imd_val_d_o        (alu_imd_val_d),
    .multdiv_operand_a_i(multdiv_alu_operand_a),
    .multdiv_operand_b_i(multdiv_alu_operand_b),
    .multdiv_sel_i      (multdiv_sel),
    .adder_result_o     (alu_adder_result_ex_o),
    .adder_result_ext_o (alu_adder_result_ext),
    .result_o           (alu_result),
    .comparison_result_o(alu_cmp_result),
    .is_equal_result_o  (alu_is_equal_result)
  );

  ////////////////
  // Multiplier //
  ////////////////

  if (RV32M == RV32MSlow) begin : gen_multdiv_slow
    ibex_multdiv_slow multdiv_i (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .mult_en_i         (mult_en_i),
      .div_en_i          (div_en_i),
      .mult_sel_i        (mult_sel_i),
      .div_sel_i         (div_sel_i),
      .operator_i        (multdiv_operator_i),
      .signed_mode_i     (multdiv_signed_mode_i),
      .op_a_i            (multdiv_operand_a_i),
      .op_b_i            (multdiv_operand_b_i),
      .alu_adder_ext_i   (alu_adder_result_ext),
      .alu_adder_i       (alu_adder_result_ex_o),
      .equal_to_zero_i   (alu_is_equal_result),
      .data_ind_timing_i (data_ind_timing_i),
      .valid_o           (multdiv_valid),
      .alu_operand_a_o   (multdiv_alu_operand_a),
      .alu_operand_b_o   (multdiv_alu_operand_b),
      .imd_val_q_i       (imd_val_q_i),
      .imd_val_d_o       (multdiv_imd_val_d),
      .imd_val_we_o      (multdiv_imd_val_we),
      .multdiv_ready_id_i(multdiv_ready_id_i),
      .multdiv_result_o  (multdiv_result)
    );
  end else if (RV32M == RV32MFast || RV32M == RV32MSingleCycle) begin : gen_multdiv_fast
    ibex_multdiv_fast #(
      .RV32M(RV32M)
    ) multdiv_i (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .mult_en_i         (mult_en_i),
      .div_en_i          (div_en_i),
      .mult_sel_i        (mult_sel_i),
      .div_sel_i         (div_sel_i),
      .operator_i        (multdiv_operator_i),
      .signed_mode_i     (multdiv_signed_mode_i),
      .op_a_i            (multdiv_operand_a_i),
      .op_b_i            (multdiv_operand_b_i),
      .alu_operand_a_o   (multdiv_alu_operand_a),
      .alu_operand_b_o   (multdiv_alu_operand_b),
      .alu_adder_ext_i   (alu_adder_result_ext),
      .alu_adder_i       (alu_adder_result_ex_o),
      .equal_to_zero_i   (alu_is_equal_result),
      .data_ind_timing_i (data_ind_timing_i),
      .imd_val_q_i       (imd_val_q_i),
      .imd_val_d_o       (multdiv_imd_val_d),
      .imd_val_we_o      (multdiv_imd_val_we),
      .multdiv_ready_id_i(multdiv_ready_id_i),
      .valid_o           (multdiv_valid),
      .multdiv_result_o  (multdiv_result)
    );
  end

  // Multiplier/divider may require multiple cycles. The ALU output is valid in the same cycle
  // unless the intermediate result register is being written (which indicates this isn't the
  // final cycle of ALU operation).
  assign ex_valid_o = multdiv_sel ? multdiv_valid  :
                      custom_sel  ? custom_valid   :
                      ~(|alu_imd_val_we);

`ifdef INC_ASSERT
  // This is intended to be accessed via hierarchal references so isn't output from this module nor
  // used in any logic in this module
  logic sva_multdiv_fsm_idle;

  if (RV32M == RV32MSlow) begin : gen_multdiv_sva_idle_slow
    assign sva_multdiv_fsm_idle = gen_multdiv_slow.multdiv_i.sva_fsm_idle;
  end else if (RV32M == RV32MFast || RV32M == RV32MSingleCycle) begin : gen_multdiv_sva_idle_fast
    assign sva_multdiv_fsm_idle = gen_multdiv_fast.multdiv_i.sva_fsm_idle;
  end else begin : gen_multdiv_sva_idle_none
    assign sva_multdiv_fsm_idle = 1'b1;
  end

  // Mark the sva_multdiv_fsm_idle as unused to avoid lint issues
  logic unused_sva_multdiv_fsm_idle;
  assign unused_sva_multdiv_fsm_idle = sva_multdiv_fsm_idle;
`endif

  // D* Lite custom execute unit
  dstar_ex_unit dstar_ex_unit_i (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    .en_i             (custom_en_i),
    .funct3_i         (custom_funct3_i),
    .funct7_i         (custom_funct7_i),
    .operand_a_i      (alu_operand_a_i),
    .operand_b_i      (alu_operand_b_i),
    .state_base_i     (dstar_state_base_i),
    .blocked_base_i   (dstar_blocked_base_i),
    .grid_width_i     (dstar_grid_width_i),
    .grid_height_i    (dstar_grid_height_i),
    .start_i          (dstar_start_i),
    .km_i             (dstar_km_i),
    .infinity_i       (dstar_infinity_i),
    .straight_cost_i  (dstar_straight_cost_i),
    .diagonal_cost_i  (dstar_diagonal_cost_i),
    .heuristic_type_i (dstar_heuristic_type_i),
    .connectivity_i   (dstar_connectivity_i),
    .data_req_o       (dstar_data_req_o),
    .data_we_o        (dstar_data_we_o),
    .data_be_o        (dstar_data_be_o),
    .data_addr_o      (dstar_data_addr_o),
    .data_wdata_o     (dstar_data_wdata_o),
    .data_gnt_i       (dstar_data_gnt_i),
    .data_rvalid_i    (dstar_data_rvalid_i),
    .data_rdata_i     (dstar_data_rdata_i),
    .result_o         (custom_result),
    .valid_o          (custom_valid)
  );

  assign custom_result_o = custom_result;

endmodule
