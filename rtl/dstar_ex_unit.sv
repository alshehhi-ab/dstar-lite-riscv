// Copyright 2026 Abdulla Alshehhi
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     https://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * D* Lite Hardware Execute Unit
 *
 * Implements CUSTOM-0 (opcode 0x0B) R-type instructions for ibex_ex_block.
 *
 * Instruction set (funct3 selects operation, funct7 selects variant):
 *   funct3=001  KCALC  rs1=state, rs2=x0  → rd = k1 (funct7[0]=0) or k2 (funct7[0]=1)
 *   funct3=010  GCOST  rs1=state, rs2=x0  → rd = g(state)
 *   funct3=011  RCOST  rs1=state, rs2=x0  → rd = rhs(state)
 *   funct3=100  GSET   rs1=state, rs2=val → g(state) = val       [rd=x0]
 *   funct3=101  RSET   rs1=state, rs2=val → rhs(state) = val     [rd=x0]
 *   funct3=110  ECOST  rs1=state_a, rs2=state_b → rd = c(a,b)
 *   funct3=111  HEUR   rs1=state_a, rs2=state_b → rd = h(a,b)   [SINGLE-CYCLE]
 *
 * State packing convention (from isa_extensions.h):
 *   bits [15:0]  = x coordinate (signed 16-bit)
 *   bits [31:16] = y coordinate (signed 16-bit)
 *
 * Memory layout (from dstar_lite.h / isa_extensions.h):
 *   StateValue[i].g   @ state_base  + i*8 + 0  (int32_t, word-aligned)
 *   StateValue[i].rhs @ state_base  + i*8 + 4  (int32_t, word-aligned)
 *   blocked[i]        @ blocked_base + i*1     (uint8_t, byte array)
 *   where i = y * grid_width + x
 *
 * Latency:
 *   HEUR  : 1 cycle  (combinational, uses live inputs)
 *   GCOST : 2 cycles (1 memory read)
 *   RCOST : 2 cycles (1 memory read)
 *   GSET  : 2 cycles (1 memory write)
 *   RSET  : 2 cycles (1 memory write)
 *   KCALC : 3 cycles (2 memory reads + combinational)
 *   ECOST : 3 cycles (straight: 2 blocked reads)
 *   ECOST : 5 cycles (diagonal: 4 blocked reads — 2 nodes + 2 corners)
 *
 * Stall requirement:
 *   en_i must be a one-shot pulse on the FIRST cycle of execution (from id_stage).
 *   The id_stage MUST stall while valid_o = 0 for non-HEUR instructions.
 *   See ibex_id_stage stall patch.
 *
 * Memory bus:
 *   data_req/gnt/rvalid follow the Ibex LSU protocol exactly.
 *   An arbiter in ibex_core must mux between the regular LSU and this unit.
 *   Assumption: grid size ≤ 8192×8192 so that (index << 3) + state_base fits 32 bits.
 */

module dstar_ex_unit (
    input  logic        clk_i,
    input  logic        rst_ni,

    // One-shot enable: asserted only on the FIRST cycle this instruction executes.
    // Drive as: custom_insn & instr_first_cycle & instr_executing  (from id_stage)
    input  logic        en_i,

    // Instruction encoding
    input  logic [2:0]  funct3_i,
    input  logic [6:0]  funct7_i,

    // Register operands (from ALU operand mux, forwarded)
    input  logic [31:0] operand_a_i,   // rs1 : packed state  or state_a
    input  logic [31:0] operand_b_i,   // rs2 : value (GSET/RSET) or state_b (ECOST/HEUR)

    // CSR configuration (wired from ibex_cs_registers output ports)
    input  logic [31:0] state_base_i,
    input  logic [31:0] blocked_base_i,
    input  logic [31:0] grid_width_i,
    input  logic [31:0] grid_height_i,
    input  logic [31:0] start_i,          // packed s_start {y[31:16], x[15:0]}
    input  logic [31:0] km_i,
    input  logic [31:0] infinity_i,
    input  logic [31:0] straight_cost_i,
    input  logic [31:0] diagonal_cost_i,
    input  logic [31:0] heuristic_type_i,
    input  logic [31:0] connectivity_i,

    // Result to register file (via ex_block result_ex_o)
    output logic [31:0] result_o,
    output logic        valid_o,

    // Data memory bus — routed through ibex_core arbiter, NOT the main Ibex LSU.
    // Protocol identical to ibex_load_store_unit ↔ core data port.
    output logic        data_req_o,
    output logic        data_we_o,
    output logic [3:0]  data_be_o,
    output logic [31:0] data_addr_o,
    output logic [31:0] data_wdata_o,
    input  logic        data_gnt_i,
    input  logic        data_rvalid_i,
    input  logic [31:0] data_rdata_i
);

  // ──────────────────────────────────────────────────────────────────────────
  // Instruction opcode constants
  // ──────────────────────────────────────────────────────────────────────────

  localparam logic [2:0] F3_KCALC = 3'd1;
  localparam logic [2:0] F3_GCOST = 3'd2;
  localparam logic [2:0] F3_RCOST = 3'd3;
  localparam logic [2:0] F3_GSET  = 3'd4;
  localparam logic [2:0] F3_RSET  = 3'd5;
  localparam logic [2:0] F3_ECOST = 3'd6;
  localparam logic [2:0] F3_HEUR  = 3'd7;

  localparam logic [1:0] HEUR_MANHATTAN  = 2'd0;
  localparam logic [1:0] HEUR_OCTILE     = 2'd1;
  localparam logic [1:0] HEUR_CHEBYSHEV  = 2'd2;

  // ──────────────────────────────────────────────────────────────────────────
  // FSM state encoding
  // ──────────────────────────────────────────────────────────────────────────

  typedef enum logic [1:0] {
    ST_IDLE,      // Waiting for en_i. valid_o=1 for HEUR (combinational bypass).
    ST_MEM_REQ,   // Memory request in progress — waiting for data_gnt_i.
    ST_MEM_WAIT,  // Read accepted — waiting for data_rvalid_i.
    ST_COMPUTE    // All memory steps done. Combinatorial result. valid_o=1.
  } state_e;

  state_e state_q, state_d;

  // ──────────────────────────────────────────────────────────────────────────
  // Latched instruction parameters
  // (Registered on en_i. Used in ST_MEM_REQ, ST_MEM_WAIT, ST_COMPUTE.)
  // ──────────────────────────────────────────────────────────────────────────

  logic [2:0]  funct3_q;
  logic [6:0]  funct7_q;
  logic [31:0] opb_q;            // rs2 verbatim (value for GSET/RSET)

  // Coordinates, sign-extended to 32 bits for arithmetic
  logic signed [31:0] ax_q, ay_q;   // state_a = rs1
  logic signed [31:0] bx_q, by_q;   // state_b = rs2

  // CSR snapshots
  logic [31:0] state_base_q, blocked_base_q;
  logic [31:0] grid_width_q, grid_height_q;
  logic [31:0] start_q, km_q, inf_q;
  logic [31:0] str_q, dia_q, htype_q, conn_q;

  // Pre-computed cell indices (registered at en_i time from live inputs + live grid_width).
  // Vivado will infer DSP48 for these multiplies.  Valid from the cycle AFTER en_i.
  logic signed [31:0] idx_a_q;    // ay * width + ax  (state_a)
  logic signed [31:0] idx_b_q;    // by * width + bx  (state_b)
  logic signed [31:0] idx_c1_q;   // ay * width + bx  (corner1: State{bx, ay})
  logic signed [31:0] idx_c2_q;   // by * width + ax  (corner2: State{ax, by})

  // Pre-computed geometry flags (registered at en_i time)
  logic a_in_bounds_q;
  logic b_in_bounds_q;
  logic ecost_is_straight_q;
  logic ecost_is_diagonal_q;
  logic ecost_neighbors_ok_q;    // geometry says they are reachable
  logic [2:0] num_steps_q;       // how many memory operations this instruction needs

  // ──────────────────────────────────────────────────────────────────────────
  // Step counter (which memory access we are currently on)
  // ──────────────────────────────────────────────────────────────────────────

  logic [2:0] step_q, step_d;

  // ──────────────────────────────────────────────────────────────────────────
  // Captured memory read data (up to 4 word reads)
  // ──────────────────────────────────────────────────────────────────────────

  logic [31:0] rdata_q [4];   // word data from each read
  logic [1:0]  blane_q [4];   // byte lane for blocked reads (lane = addr & 2'b11)

  // ──────────────────────────────────────────────────────────────────────────
  // COMBINATIONAL: Heuristic using LIVE inputs (single-cycle, no latching needed)
  // ──────────────────────────────────────────────────────────────────────────

  // Sign-extend live coordinates to 32 bits
  logic signed [31:0] live_ax, live_ay, live_bx, live_by;
  assign live_ax = {{16{operand_a_i[15]}}, operand_a_i[15:0]};
  assign live_ay = {{16{operand_a_i[31]}}, operand_a_i[31:16]};
  assign live_bx = {{16{operand_b_i[15]}}, operand_b_i[15:0]};
  assign live_by = {{16{operand_b_i[31]}}, operand_b_i[31:16]};

  logic [31:0] heur_dx_abs, heur_dy_abs, heur_dmin, heur_dmax;
  logic signed [31:0] heur_dx_s, heur_dy_s;
  assign heur_dx_s   = live_bx - live_ax;
  assign heur_dy_s   = live_by - live_ay;
  assign heur_dx_abs = heur_dx_s[31] ? (~heur_dx_s + 32'd1) : heur_dx_s;
  assign heur_dy_abs = heur_dy_s[31] ? (~heur_dy_s + 32'd1) : heur_dy_s;
  assign heur_dmin   = (heur_dx_abs < heur_dy_abs) ? heur_dx_abs : heur_dy_abs;
  assign heur_dmax   = (heur_dx_abs > heur_dy_abs) ? heur_dx_abs : heur_dy_abs;

  // Heuristic variants: uses live straight/diagonal costs
  logic [31:0] heur_manhattan, heur_octile, heur_chebyshev, heur_result;
  assign heur_manhattan  = straight_cost_i * (heur_dx_abs + heur_dy_abs);
  assign heur_octile     = straight_cost_i * (heur_dx_abs + heur_dy_abs) +
                           (diagonal_cost_i - (straight_cost_i << 1)) * heur_dmin;
  assign heur_chebyshev  = straight_cost_i * heur_dmax;

  always_comb begin
    case (funct7_i[1:0])
      HEUR_MANHATTAN:  heur_result = heur_manhattan;
      HEUR_OCTILE:     heur_result = heur_octile;
      HEUR_CHEBYSHEV:  heur_result = heur_chebyshev;
      default:         heur_result = heur_manhattan;
    endcase
  end

  // ──────────────────────────────────────────────────────────────────────────
  // COMBINATIONAL: Pre-compute metadata from live inputs (registered at en_i)
  // ──────────────────────────────────────────────────────────────────────────

  // Bounds checks using live inputs + live grid dimensions
  logic live_a_in_bounds, live_b_in_bounds;
  assign live_a_in_bounds = ($signed(live_ax) >= 0) && ($signed(live_ay) >= 0) &&
                             ($signed(live_ax) < $signed(grid_width_i)) &&
                             ($signed(live_ay) < $signed(grid_height_i));
  assign live_b_in_bounds = ($signed(live_bx) >= 0) && ($signed(live_by) >= 0) &&
                             ($signed(live_bx) < $signed(grid_width_i)) &&
                             ($signed(live_by) < $signed(grid_height_i));

  // ECOST geometry from live inputs
  logic [31:0] live_edx, live_edy;
  logic signed [31:0] live_edx_s, live_edy_s;
  assign live_edx_s      = live_bx - live_ax;
  assign live_edy_s      = live_by - live_ay;
  assign live_edx        = live_edx_s[31] ? (~live_edx_s + 32'd1) : live_edx_s;
  assign live_edy        = live_edy_s[31] ? (~live_edy_s + 32'd1) : live_edy_s;
  logic live_is_straight, live_is_diagonal, live_neighbors_ok;
  assign live_is_straight  = ((live_edx + live_edy) == 32'd1);
  assign live_is_diagonal  = ((live_edx == 32'd1) && (live_edy == 32'd1));
  assign live_neighbors_ok = live_is_straight ||
                             (live_is_diagonal && (connectivity_i >= 32'd8));

  // Number of memory steps required (0 = no memory, bypass to COMPUTE)
  logic [2:0] live_num_steps;
  always_comb begin
    case (funct3_i)
      F3_KCALC:  live_num_steps = 3'd2;  // 2 reads: g, rhs
      F3_GCOST:  live_num_steps = 3'd1;  // 1 read: g
      F3_RCOST:  live_num_steps = 3'd1;  // 1 read: rhs
      F3_GSET:   live_num_steps = 3'd1;  // 1 write: g
      F3_RSET:   live_num_steps = 3'd1;  // 1 write: rhs
      F3_ECOST: begin
        if (!live_a_in_bounds || !live_b_in_bounds || !live_neighbors_ok)
          live_num_steps = 3'd0;         // geometry rejects — skip to COMPUTE
        else if (live_is_diagonal)
          live_num_steps = 3'd4;         // 4 reads: blocked_a, blocked_b, corner1, corner2
        else
          live_num_steps = 3'd2;         // 2 reads: blocked_a, blocked_b
      end
      default:   live_num_steps = 3'd0;  // HEUR and unknown: no memory
    endcase
  end

  // Pre-compute byte lanes for blocked reads (blocked_base + idx) & 2'b11
  // These use live_blocked_base = blocked_base_i and live indices
  logic signed [31:0] live_idx_a, live_idx_b, live_idx_c1, live_idx_c2;
  assign live_idx_a  = $signed(live_ay) * $signed(grid_width_i) + $signed(live_ax);
  assign live_idx_b  = $signed(live_by) * $signed(grid_width_i) + $signed(live_bx);
  assign live_idx_c1 = $signed(live_ay) * $signed(grid_width_i) + $signed(live_bx);
  assign live_idx_c2 = $signed(live_by) * $signed(grid_width_i) + $signed(live_ax);

  logic [31:0] live_ba_full, live_bb_full, live_bc1_full, live_bc2_full;
  assign live_ba_full  = blocked_base_i + live_idx_a[31:0];
  assign live_bb_full  = blocked_base_i + live_idx_b[31:0];
  assign live_bc1_full = blocked_base_i + live_idx_c1[31:0];
  assign live_bc2_full = blocked_base_i + live_idx_c2[31:0];

  logic [1:0] live_blane_a, live_blane_b, live_blane_c1, live_blane_c2;
  assign live_blane_a  = live_ba_full[1:0];
  assign live_blane_b  = live_bb_full[1:0];
  assign live_blane_c1 = live_bc1_full[1:0];
  assign live_blane_c2 = live_bc2_full[1:0];

  // ──────────────────────────────────────────────────────────────────────────
  // Sequential: register all parameters on en_i pulse
  // ──────────────────────────────────────────────────────────────────────────

  integer i;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q             <= ST_IDLE;
      step_q              <= '0;
      funct3_q            <= '0;
      funct7_q            <= '0;
      opb_q               <= '0;
      ax_q                <= '0; ay_q <= '0;
      bx_q                <= '0; by_q <= '0;
      state_base_q        <= '0;
      blocked_base_q      <= '0;
      grid_width_q        <= '0;
      grid_height_q       <= '0;
      start_q             <= '0;
      km_q                <= '0;
      inf_q               <= 32'h1FFF_FFFF;  // INT32_MAX/4
      str_q               <= 32'd1000;
      dia_q               <= 32'd1414;
      htype_q             <= '0;
      conn_q              <= 32'd4;
      idx_a_q             <= '0; idx_b_q  <= '0;
      idx_c1_q            <= '0; idx_c2_q <= '0;
      a_in_bounds_q       <= 1'b0;
      b_in_bounds_q       <= 1'b0;
      ecost_is_straight_q <= 1'b0;
      ecost_is_diagonal_q <= 1'b0;
      ecost_neighbors_ok_q<= 1'b0;
      num_steps_q         <= '0;
      for (i = 0; i < 4; i = i + 1) begin
        rdata_q[i] <= '0;
        blane_q[i] <= '0;
      end
    end else begin
      state_q <= state_d;
      step_q  <= step_d;

      // Latch instruction parameters and pre-computed metadata on en_i
      if (en_i) begin
        funct3_q             <= funct3_i;
        funct7_q             <= funct7_i;
        opb_q                <= operand_b_i;
        ax_q                 <= {{16{operand_a_i[15]}}, operand_a_i[15:0]};
        ay_q                 <= {{16{operand_a_i[31]}}, operand_a_i[31:16]};
        bx_q                 <= {{16{operand_b_i[15]}}, operand_b_i[15:0]};
        by_q                 <= {{16{operand_b_i[31]}}, operand_b_i[31:16]};
        state_base_q         <= state_base_i;
        blocked_base_q       <= blocked_base_i;
        grid_width_q         <= grid_width_i;
        grid_height_q        <= grid_height_i;
        start_q              <= start_i;
        km_q                 <= km_i;
        inf_q                <= infinity_i;
        str_q                <= straight_cost_i;
        dia_q                <= diagonal_cost_i;
        htype_q              <= heuristic_type_i;
        conn_q               <= connectivity_i;
        // Pre-compute indices from live inputs (multiply resolves in this FF chain;
        // the registered result is valid in ST_MEM_REQ, the cycle after en_i)
        idx_a_q              <= live_idx_a;
        idx_b_q              <= live_idx_b;
        idx_c1_q             <= live_idx_c1;
        idx_c2_q             <= live_idx_c2;
        // Geometry flags
        a_in_bounds_q        <= live_a_in_bounds;
        b_in_bounds_q        <= live_b_in_bounds;
        ecost_is_straight_q  <= live_is_straight;
        ecost_is_diagonal_q  <= live_is_diagonal;
        ecost_neighbors_ok_q <= live_neighbors_ok;
        num_steps_q          <= live_num_steps;
        // Byte lanes for blocked reads
        blane_q[0] <= live_blane_a;
        blane_q[1] <= live_blane_b;
        blane_q[2] <= live_blane_c1;
        blane_q[3] <= live_blane_c2;
      end

      // Capture memory read data when response arrives
      if ((state_q == ST_MEM_WAIT) && data_rvalid_i) begin
        rdata_q[step_q] <= data_rdata_i;
      end
    end
  end

  // ──────────────────────────────────────────────────────────────────────────
  // Memory address and control for current step (combinational)
  // ──────────────────────────────────────────────────────────────────────────

  // State array addresses (word-aligned by construction: state_base should be
  // 4-byte aligned and index*8 is always 4-byte aligned)
  logic [31:0] addr_g_a, addr_rhs_a;
  assign addr_g_a   = state_base_q + {idx_a_q[28:0], 3'b000};  // idx * 8
  assign addr_rhs_a = addr_g_a + 32'd4;

  // Blocked array addresses (word-aligned for word reads, byte extracted after)
  logic [31:0] ba_raw, bb_raw, bc1_raw, bc2_raw;
  assign ba_raw  = blocked_base_q + idx_a_q[31:0];
  assign bb_raw  = blocked_base_q + idx_b_q[31:0];
  assign bc1_raw = blocked_base_q + idx_c1_q[31:0];
  assign bc2_raw = blocked_base_q + idx_c2_q[31:0];

  // Word-align by clearing the bottom 2 bits
  logic [31:0] ba_addr, bb_addr, bc1_addr, bc2_addr;
  assign ba_addr  = {ba_raw[31:2],  2'b00};
  assign bb_addr  = {bb_raw[31:2],  2'b00};
  assign bc1_addr = {bc1_raw[31:2], 2'b00};
  assign bc2_addr = {bc2_raw[31:2], 2'b00};

  // Current step address / we / be / wdata mux
  logic [31:0] cur_addr;
  logic        cur_we;
  logic [3:0]  cur_be;
  logic [31:0] cur_wdata;

  always_comb begin
    cur_addr  = '0;
    cur_we    = 1'b0;
    cur_be    = 4'b1111;
    cur_wdata = '0;

    case (funct3_q)
      F3_KCALC: begin
        cur_we = 1'b0;
        case (step_q)
          3'd0: cur_addr = addr_g_a;
          3'd1: cur_addr = addr_rhs_a;
          default: cur_addr = '0;
        endcase
      end
      F3_GCOST: begin
        cur_addr = addr_g_a;
        cur_we   = 1'b0;
      end
      F3_RCOST: begin
        cur_addr = addr_rhs_a;
        cur_we   = 1'b0;
      end
      F3_GSET: begin
        cur_addr  = addr_g_a;
        cur_we    = 1'b1;
        cur_be    = 4'b1111;
        cur_wdata = opb_q;
      end
      F3_RSET: begin
        cur_addr  = addr_rhs_a;
        cur_we    = 1'b1;
        cur_be    = 4'b1111;
        cur_wdata = opb_q;
      end
      F3_ECOST: begin
        cur_we = 1'b0;
        // Byte reads from the blocked array; word-align and select byte lane
        case (step_q)
          3'd0: begin
            cur_addr = ba_addr;
            cur_be   = 4'b0001 << blane_q[0];
          end
          3'd1: begin
            cur_addr = bb_addr;
            cur_be   = 4'b0001 << blane_q[1];
          end
          3'd2: begin
            cur_addr = bc1_addr;
            cur_be   = 4'b0001 << blane_q[2];
          end
          3'd3: begin
            cur_addr = bc2_addr;
            cur_be   = 4'b0001 << blane_q[3];
          end
          default: cur_addr = '0;
        endcase
      end
      default: begin
        cur_addr = '0;
        cur_we   = 1'b0;
      end
    endcase
  end

  // ──────────────────────────────────────────────────────────────────────────
  // FSM combinational next-state logic
  // ──────────────────────────────────────────────────────────────────────────

  always_comb begin
    state_d = state_q;
    step_d  = step_q;

    case (state_q)
      ST_IDLE: begin
        if (en_i) begin
          if (funct3_i == F3_HEUR) begin
            // HEUR is single-cycle combinational: pulse into COMPUTE so valid_o fires
            // in the SAME cycle as en_i (see valid_o assignment below).
            state_d = ST_IDLE;  // HEUR never leaves IDLE — valid_o driven combinationally
          end else if (live_num_steps == 3'd0) begin
            state_d = ST_COMPUTE;   // ECOST geometry rejected immediately
            step_d  = 3'd0;
          end else begin
            state_d = ST_MEM_REQ;
            step_d  = 3'd0;
          end
        end
      end

      ST_MEM_REQ: begin
        if (data_gnt_i) begin
          if (cur_we) begin
            // Write: no rdata response needed — advance to next step or finish
            if (step_q + 3'd1 == num_steps_q) begin
              state_d = ST_COMPUTE;
            end else begin
              step_d  = step_q + 3'd1;
              // remain in MEM_REQ for next write step (unusual, but handles sequences)
            end
          end else begin
            // Read: must wait for rdata
            state_d = ST_MEM_WAIT;
          end
        end
      end

      ST_MEM_WAIT: begin
        if (data_rvalid_i) begin
          if (step_q + 3'd1 == num_steps_q) begin
            state_d = ST_COMPUTE;
          end else begin
            step_d  = step_q + 3'd1;
            state_d = ST_MEM_REQ;
          end
        end
      end

      ST_COMPUTE: begin
        state_d = ST_IDLE;   // One cycle of valid output, then retire
      end

      default: state_d = ST_IDLE;
    endcase
  end

  // ──────────────────────────────────────────────────────────────────────────
  // Memory bus output registers
  // ──────────────────────────────────────────────────────────────────────────

  always_comb begin
    data_req_o   = 1'b0;
    data_we_o    = 1'b0;
    data_be_o    = 4'b1111;
    data_addr_o  = '0;
    data_wdata_o = '0;

    if (state_q == ST_MEM_REQ) begin
      data_req_o   = 1'b1;
      data_we_o    = cur_we;
      data_be_o    = cur_be;
      data_addr_o  = cur_addr;
      data_wdata_o = cur_wdata;
    end
  end

  // ──────────────────────────────────────────────────────────────────────────
  // KCALC: heuristic h(start, state) using latched values
  // ──────────────────────────────────────────────────────────────────────────

  // start is s_start from CSR; state is operand_a (latched as ax_q/ay_q)
  logic signed [31:0] ksx, ksy;
  assign ksx = {{16{start_q[15]}}, start_q[15:0]};
  assign ksy = {{16{start_q[31]}}, start_q[31:16]};

  logic [31:0] kcalc_dx, kcalc_dy, kcalc_dmin, kcalc_dmax;
  logic signed [31:0] kcalc_dx_s, kcalc_dy_s;
  assign kcalc_dx_s  = ax_q - ksx;   // dx = state.x - start.x
  assign kcalc_dy_s  = ay_q - ksy;
  assign kcalc_dx    = kcalc_dx_s[31] ? (~kcalc_dx_s + 32'd1) : kcalc_dx_s;
  assign kcalc_dy    = kcalc_dy_s[31] ? (~kcalc_dy_s + 32'd1) : kcalc_dy_s;
  assign kcalc_dmin  = (kcalc_dx < kcalc_dy) ? kcalc_dx : kcalc_dy;
  assign kcalc_dmax  = (kcalc_dx > kcalc_dy) ? kcalc_dx : kcalc_dy;

  logic [31:0] kcalc_h;
  always_comb begin
    case (htype_q[1:0])
      HEUR_MANHATTAN:  kcalc_h = str_q * (kcalc_dx + kcalc_dy);
      HEUR_OCTILE:     kcalc_h = str_q * (kcalc_dx + kcalc_dy) +
                                 (dia_q - (str_q << 1)) * kcalc_dmin;
      HEUR_CHEBYSHEV:  kcalc_h = str_q * kcalc_dmax;
      default:         kcalc_h = str_q * (kcalc_dx + kcalc_dy);
    endcase
  end

  // ──────────────────────────────────────────────────────────────────────────
  // Result computation (combinational, evaluated in ST_COMPUTE)
  // ──────────────────────────────────────────────────────────────────────────

  // Byte extraction helper: extract bool (blocked?) from word + byte lane
  // blocked = (rdata >> (lane*8)) & 0xFF != 0
  logic blk_a, blk_b, blk_c1, blk_c2;
  always_comb begin
    case (blane_q[0])
      2'd0: blk_a = rdata_q[0][ 7: 0] != 8'h00;
      2'd1: blk_a = rdata_q[0][15: 8] != 8'h00;
      2'd2: blk_a = rdata_q[0][23:16] != 8'h00;
      2'd3: blk_a = rdata_q[0][31:24] != 8'h00;
    endcase
    case (blane_q[1])
      2'd0: blk_b = rdata_q[1][ 7: 0] != 8'h00;
      2'd1: blk_b = rdata_q[1][15: 8] != 8'h00;
      2'd2: blk_b = rdata_q[1][23:16] != 8'h00;
      2'd3: blk_b = rdata_q[1][31:24] != 8'h00;
    endcase
    case (blane_q[2])
      2'd0: blk_c1 = rdata_q[2][ 7: 0] != 8'h00;
      2'd1: blk_c1 = rdata_q[2][15: 8] != 8'h00;
      2'd2: blk_c1 = rdata_q[2][23:16] != 8'h00;
      2'd3: blk_c1 = rdata_q[2][31:24] != 8'h00;
    endcase
    case (blane_q[3])
      2'd0: blk_c2 = rdata_q[3][ 7: 0] != 8'h00;
      2'd1: blk_c2 = rdata_q[3][15: 8] != 8'h00;
      2'd2: blk_c2 = rdata_q[3][23:16] != 8'h00;
      2'd3: blk_c2 = rdata_q[3][31:24] != 8'h00;
    endcase
  end

  logic [31:0] result_comb;
  
  logic [31:0] kcalc_g, kcalc_rhs, kcalc_k2, kcalc_k1;
  
  always_comb begin
    result_comb = '0;

    case (funct3_q)
      F3_GCOST: begin
        result_comb = a_in_bounds_q ? rdata_q[0] : inf_q;
      end

      F3_RCOST: begin
        result_comb = a_in_bounds_q ? rdata_q[0] : inf_q;
      end

      F3_GSET, F3_RSET: begin
        result_comb = '0;  // rd = x0; write-only
      end

      F3_KCALC: begin
        // rdata_q[0] = g(state), rdata_q[1] = rhs(state)
        kcalc_g   = a_in_bounds_q ? rdata_q[0] : inf_q;
        kcalc_rhs = a_in_bounds_q ? rdata_q[1] : inf_q;
        kcalc_k2  = (kcalc_g < kcalc_rhs) ? kcalc_g : kcalc_rhs;
        kcalc_k1  = kcalc_k2 + kcalc_h + km_q;
        result_comb = funct7_q[0] ? kcalc_k2 : kcalc_k1;
      end

      F3_ECOST: begin
        if (!a_in_bounds_q || !b_in_bounds_q || !ecost_neighbors_ok_q) begin
          // Geometry rejects (should not reach ST_COMPUTE via memory, but can
          // reach it via the num_steps==0 shortcut)
          result_comb = inf_q;
        end else if (blk_a || blk_b) begin
          result_comb = inf_q;
        end else if (ecost_is_diagonal_q) begin
          // Diagonal move: check both shared corner cells for blocked status
          // (isDiagonalMoveAllowed in grid_map.cpp)
          result_comb = (blk_c1 || blk_c2) ? inf_q : dia_q;
        end else begin
          // Straight move: not blocked
          result_comb = str_q;
        end
      end

      default: result_comb = '0;
    endcase
  end

  // ──────────────────────────────────────────────────────────────────────────
  // Output mux
  // ──────────────────────────────────────────────────────────────────────────

  // HEUR: purely combinational using LIVE inputs. valid_o=1 when en_i & funct3==HEUR.
  //       The pipeline sees valid_o=1 on the same cycle as en_i.
  //
  // All others: result available in ST_COMPUTE (one cycle after last memory response,
  //             or immediately after en_i if num_steps=0).
  //             valid_o=1 only in ST_COMPUTE.

  assign valid_o  = (funct3_i == F3_HEUR) ||
                    (state_q == ST_COMPUTE);

  assign result_o = (funct3_i == F3_HEUR) ? heur_result : result_comb;

endmodule