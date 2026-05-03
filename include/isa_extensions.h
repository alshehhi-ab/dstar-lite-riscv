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

#ifndef ISA_EXTENSIONS_H_
#define ISA_EXTENSIONS_H_

#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// D* Lite Custom ISA Extensions for RV32IMC (custom-0, opcode 0x0B)
//
//   All instructions are R-type, encoded as:
//   funct7 | rs2 | rs1 | funct3 | rd | opcode(0x0B)
//
// ── Instruction Encoding Table ───────────────────────────────────────────────
//
//   Mnemonic   funct7    funct3   rd    rs1         rs2         Operation
//   ────────── ───────── ──────── ───── ─────────── ─────────── ──────────────────────────────
//   KCMP       0000000   000      rd    key_a.k1    key_b.k1    rd = (key_a < key_b) ? 1 : 0
//   KCALC      0000000   001      rd    state       x0          rd = calculateKey(state)
//   GCOST      0000000   010      rd    state       x0          rd = g(state)
//   RCOST      0000000   011      rd    state       x0          rd = rhs(state)
//   GSET       0000000   100      x0    state       value       g(state) = value
//   RSET       0000000   101      x0    state       value       rhs(state) = value
//   ECOST      0000000   110      rd    state_a     state_b     rd = cost(state_a, state_b)
//   HEUR       0000000   111      rd    state_a     state_b     rd = manhattan(a, b)
//   HEUR       0000001   111      rd    state_a     state_b     rd = octile(a, b)
//   HEUR       0000010   111      rd    state_a     state_b     rd = chebyshev(a, b)
//
// ── State Packing Convention ─────────────────────────────────────────────────
//
//   States are packed into a single 32-bit register:
//     bits [15:0]  = x coordinate
//     bits [31:16] = y coordinate
//
//   Use pack_state(x, y) to pack, unpack_x() / unpack_y() to unpack.
//
// ── Control and Status Registers (CSRs) ──────────────────────────────────────
//
//   Custom user-mode CSRs in the range 0x800–0x8FF (256 available).
//   Set once at planner initialisation via dstar_csr_init().
//   Read by hardware on every custom instruction execution.
//
//   Address   Name                  Description
//   ───────── ───────────────────── ──────────────────────────────────────────
//   0x800     CSR_STATE_BASE        Base address of state_values table
//   0x801     CSR_BLOCKED_BASE      Base address of blocked table
//   0x802     CSR_GRID_WIDTH        Grid width (columns)
//   0x803     CSR_GRID_HEIGHT       Grid height (rows)
//   0x804     CSR_START             s_start packed as state
//   0x805     CSR_GOAL              s_goal packed as state
//   0x806     CSR_KM                Key modifier km
//   0x807     CSR_INFINITY          kInfinity constant
//   0x808     CSR_STRAIGHT_COST     kStraightCost constant
//   0x809     CSR_DIAGONAL_COST     kDiagonalCost constant
//   0x80A     CSR_HEURISTIC_TYPE    Default heuristic (0=Manhattan,1=Octile,2=Chebyshev)
//   0x80B     CSR_GRID_SIZE         Grid size = width * height (precomputed)
//   0x80C     CSR_CONNECTIVITY      Connectivity (4 or 8)
//   0x80D–0x8FF                     Reserved for future D* Lite extensions
//
// =============================================================================

// [1] CSR ADDRESSES

#define CSR_STATE_BASE      0x800
#define CSR_BLOCKED_BASE    0x801
#define CSR_GRID_WIDTH      0x802
#define CSR_GRID_HEIGHT     0x803
#define CSR_START           0x804
#define CSR_GOAL            0x805
#define CSR_KM              0x806
#define CSR_INFINITY        0x807
#define CSR_STRAIGHT_COST   0x808
#define CSR_DIAGONAL_COST   0x809
#define CSR_HEURISTIC_TYPE  0x80A
#define CSR_GRID_SIZE       0x80B
#define CSR_CONNECTIVITY    0x80C

// [2] CONSTANTS

#define HEURISTIC_MANHATTAN  0
#define HEURISTIC_OCTILE     1
#define HEURISTIC_CHEBYSHEV  2

#define CONNECTIVITY_FOUR    4
#define CONNECTIVITY_EIGHT   8


#ifdef DSTAR_USE_HW_ISA

// WRITE TO CSR
#define CSR_WRITE(csr, val) \
    __asm__ volatile ("csrw " #csr ", %0" : : "r"((int32_t)(val)) : "memory")

// READ FROM CSR
#define CSR_READ(csr, val) \
__asm__ volatile ("csrr %0, " #csr : "=r"((val)) : : )

// Set specific bits in a custom CSR (Bitwise OR)
#define CSR_SET(csr, mask) \
    __asm__ volatile ("csrs " #csr ", %0" : : "r"((uint32_t)(mask)) : "memory")

// Clear specific bits in a custom CSR (Bitwise AND NOT)
#define CSR_CLR(csr, mask) \
    __asm__ volatile ("csrc " #csr ", %0" : : "r"((uint32_t)(mask)) : "memory")

// RW CSR in one instruction
#define CSR_SWAP(csr, val, old) \
    __asm__ volatile ("csrrw %0, " #csr ", %1" : "=r"((old)) : "r"((int32_t)(val)) : "memory")


/**
 * @brief Initialise all D* Lite CSRs.
 *
 * Must be called once after constructing the DStarLite object and before
 * calling plan() or replan(). Sets all hardware control registers so that
 * GCOST, RCOST, GSET, RSET, ECOST, HEUR, KCALC, and KCMP can operate
 * without software-side parameter passing.
 *
 * @param state_base      Pointer to the state_values array
 * @param blocked_base    Pointer to the blocked array
 * @param width           Grid width
 * @param height          Grid height
 * @param start_x         Start state x coordinate
 * @param start_y         Start state y coordinate
 * @param goal_x          Goal state x coordinate
 * @param goal_y          Goal state y coordinate
 * @param km              Current key modifier value
 * @param infinity        kInfinity constant
 * @param straight_cost   kStraightCost constant
 * @param diagonal_cost   kDiagonalCost constant
 * @param heuristic_type  HEURISTIC_MANHATTAN, HEURISTIC_OCTILE, or HEURISTIC_CHEBYSHEV
 * @param connectivity    CONNECTIVITY_FOUR or CONNECTIVITY_EIGHT
 */
static inline void dstar_csr_init(
    const void*  state_base,
    const void*  blocked_base,
    int32_t      width,
    int32_t      height,
    int32_t      start_x,    
    int32_t      start_y,
    int32_t      goal_x,     
    int32_t      goal_y,
    int32_t      km,
    int32_t      infinity,
    int32_t      straight_cost,
    int32_t      diagonal_cost,
    int32_t      heuristic_type,
    int32_t      connectivity
) {
    CSR_WRITE(0x800, (int32_t)(uintptr_t)state_base);
    CSR_WRITE(0x801, (int32_t)(uintptr_t)blocked_base);
    CSR_WRITE(0x802, width);
    CSR_WRITE(0x803, height);
    CSR_WRITE(0x804, (int32_t)(((uint32_t)start_y << 16) | ((uint32_t)start_x & 0xFFFF)));
    CSR_WRITE(0x805, (int32_t)(((uint32_t)goal_y  << 16) | ((uint32_t)goal_x  & 0xFFFF)));
    CSR_WRITE(0x806, km);
    CSR_WRITE(0x807, infinity);
    CSR_WRITE(0x808, straight_cost);
    CSR_WRITE(0x809, diagonal_cost);
    CSR_WRITE(0x80A, heuristic_type);
    CSR_WRITE(0x80B, width * height);
    CSR_WRITE(0x80C, connectivity);
}

/**
 * @brief Update km CSR call whenever km changes (after updateStart).
 */
static inline void dstar_csr_set_km(int32_t km) {
    CSR_WRITE(0x806, km);
}
 
/**
 * @brief Update s_start CSR call whenever start changes (after updateStart).
 */
static inline void dstar_csr_set_start(int32_t x, int32_t y) {
    CSR_WRITE(0x804, (int32_t)(((uint32_t)y << 16) | ((uint32_t)x & 0xFFFF)));
}


// STATE PACKING / UNPACKING 

/**
 * @brief Pack a (x, y) state into a single 32-bit register.
 * bits [15:0] = x, bits [31:16] = y
 */
static inline int32_t pack_state(int32_t x, int32_t y) {
    return (int32_t)(((uint32_t)y << 16) | ((uint32_t)x & 0xFFFF));
}
 
static inline int32_t unpack_x(int32_t packed) {
    return (int32_t)(int16_t)((uint32_t)packed & 0xFFFF);
}
 
static inline int32_t unpack_y(int32_t packed) {
    return (int32_t)(int16_t)(((uint32_t)packed >> 16) & 0xFFFF);
}

// CUSTOM INSTRUCTIONS

// =============================================================================
// Custom Instructions
// =============================================================================
 
/**
 * @brief KCMP : Key comparison (Lexicographic).
 *
 * Returns 1 if key_a < key_b lexicographically, 0 otherwise.
 * key_a > key_b can be expressed as kcmp(k1_b, k2_b, k1_a, k2_a).
 *
 * Hardware compares k1 first; if equal, compares k2.
 * k2 values are passed in rs2 as packed (k2_a in [15:0], k2_b in [31:16]).
 *
 * @param k1_a  k1 of key_a
 * @param k2_a  k2 of key_a
 * @param k1_b  k1 of key_b
 * @param k2_b  k2 of key_b
 * @return      1 if key_a < key_b, 0 otherwise
 * 
 */

static inline int32_t kcmp(int32_t k1_a, int32_t k2_a,
                            int32_t k1_b, int32_t k2_b) {
    int32_t result;
    // Pack k2 values into rs2: k2_a in [15:0], k2_b in [31:16]
    int32_t k2_packed = (int32_t)(((uint32_t)k2_b << 16) | ((uint32_t)k2_a & 0xFFFF));
    __asm__ volatile (
        ".insn r 0x0B, 0x0, 0x00, %0, %1, %2\n\t"
        : "=r"(result)
        : "r"(k1_a), "r"(k2_packed)
        :
    );
    (void)k1_b; (void)k2_b;
    return result;


/**
 * @brief KCALC : Key calculation.
 *
 * Computes calculateKey(state) in a single instruction.
 * Internally performs: g=GCOST, rhs=RCOST, h=HEUR(state, s_start),
 * k2=min(g,rhs), k1=k2+h+km.
 * Reads s_start, km, and heuristic type from CSRs 0x804, 0x806, 0x80A.
 *
 * @param state_packed  Packed state (x in [15:0], y in [31:16])
 * @param k1_out        Output k1 component
 * @param k2_out        Output k2 component
 */
static inline void kcalc(int32_t state_packed,
                          int32_t* k1_out, int32_t* k2_out) {
    int32_t k1, k2;
    __asm__ volatile (
        ".insn r 0x0B, 0x1, 0x00, %0, %2, x0\n\t"
        ".insn r 0x0B, 0x1, 0x01, %1, %2, x0\n\t"
        : "=r"(k1), "=r"(k2)
        : "r"(state_packed)
        :
    );
    *k1_out = k1;
    *k2_out = k2;
}
 
/**
 * @brief GCOST : Get g value.
 *
 * Returns g(state) directly from the hardware state table.
 * Reads state_values base address and grid width from CSRs 0x800, 0x802.
 * No bounds check — assumes state is valid (called only on valid states).
 *
 * @param state_packed  Packed state (x in [15:0], y in [31:16])
 * @return              g value of the state, or kInfinity if out of bounds
 */
static inline int32_t gcost(int32_t state_packed) {
    int32_t result;
    __asm__ volatile (
        ".insn r 0x0B, 0x2, 0x00, %0, %1, x0\n\t"
        : "=r"(result)
        : "r"(state_packed)
        :
    );
    return result;
}
 
/**
 * @brief RCOST : Get rhs value.
 *
 * Returns rhs(state) directly from the hardware state table.
 * Reads state_values base address and grid width from CSRs 0x800, 0x802.
 * No bounds check — assumes state is valid.
 *
 * @param state_packed  Packed state (x in [15:0], y in [31:16])
 * @return              rhs value of the state, or kInfinity if out of bounds
 */
static inline int32_t rcost(int32_t state_packed) {
    int32_t result;
    __asm__ volatile (
        ".insn r 0x0B, 0x3, 0x00, %0, %1, x0\n\t"
        : "=r"(result)
        : "r"(state_packed)
        :
    );
    return result;
}
 
/**
 * @brief GSET : Set g value.
 *
 * Writes value to g(state) in the hardware state table.
 * Reads state_values base address and grid width from CSRs 0x800, 0x802.
 *
 * @param state_packed  Packed state (x in [15:0], y in [31:16])
 * @param value         Value to write
 */
static inline void gset(int32_t state_packed, int32_t value) {
    __asm__ volatile (
        ".insn r 0x0B, 0x4, 0x00, x0, %0, %1\n\t"
        :
        : "r"(state_packed), "r"(value)
        : "memory"
    );
}
 
/**
 * @brief RSET : Set rhs value.
 *
 * Writes value to rhs(state) in the hardware state table.
 * Reads state_values base address and grid width from CSRs 0x800, 0x802.
 *
 * @param state_packed  Packed state (x in [15:0], y in [31:16])
 * @param value         Value to write
 */
static inline void rset(int32_t state_packed, int32_t value) {
    __asm__ volatile (
        ".insn r 0x0B, 0x5, 0x00, x0, %0, %1\n\t"
        :
        : "r"(state_packed), "r"(value)
        : "memory"
    );
}

#else

// [DELETE LATER, ONLY USE AS FALLBACK ON x86 FOR VERIFICATION]

#include <cstdlib>

// These are set by dstar_csr_init() and used by the fallback implementations
static const void*  _sw_state_base     = nullptr;
static const void*  _sw_blocked_base   = nullptr;
static int32_t      _sw_width          = 0;
static int32_t      _sw_height         = 0;
static int32_t      _sw_start          = 0;
static int32_t      _sw_km             = 0;
static int32_t      _sw_infinity       = 0;
static int32_t      _sw_straight_cost  = 0;
static int32_t      _sw_diagonal_cost  = 0;
static int32_t      _sw_heuristic_type = 0;
static int32_t      _sw_connectivity   = 0;

struct _SwStateValue { int32_t g; int32_t rhs; };

static inline int32_t pack_state(int32_t x, int32_t y) {
    return (int32_t)(((uint32_t)y << 16) | ((uint32_t)x & 0xFFFF));
}

static inline int32_t unpack_x(int32_t packed) {
    return (int32_t)(int16_t)((uint32_t)packed & 0xFFFF);
}

static inline int32_t unpack_y(int32_t packed) {
    return (int32_t)(int16_t)(((uint32_t)packed >> 16) & 0xFFFF);
}

static inline void dstar_csr_init(
    const void* state_base, const void* blocked_base,
    int32_t width,          int32_t height,
    int32_t start_x,        int32_t start_y,
    int32_t goal_x,         int32_t goal_y,
    int32_t km,             int32_t infinity,
    int32_t straight_cost,  int32_t diagonal_cost,
    int32_t heuristic_type, int32_t connectivity
) {
    _sw_state_base     = state_base;
    _sw_blocked_base   = blocked_base;
    _sw_width          = width;
    _sw_height         = height;
    _sw_start          = pack_state(start_x, start_y);
    _sw_km             = km;
    _sw_infinity       = infinity;
    _sw_straight_cost  = straight_cost;
    _sw_diagonal_cost  = diagonal_cost;
    _sw_heuristic_type = heuristic_type;
    _sw_connectivity   = connectivity;
    (void)goal_x; (void)goal_y;
}

static inline void dstar_csr_set_km(int32_t km) {
    _sw_km = km;
}

static inline void dstar_csr_set_start(int32_t x, int32_t y) {
    _sw_start = pack_state(x, y);
}

static inline int32_t gcost(int32_t packed) {
    int32_t x = unpack_x(packed);
    int32_t y = unpack_y(packed);
    if (x < 0 || x >= _sw_width || y < 0 || y >= _sw_height)
        return _sw_infinity;
    const _SwStateValue* table = (const _SwStateValue*)_sw_state_base;
    return table[y * _sw_width + x].g;
}

static inline int32_t rcost(int32_t packed) {
    int32_t x = unpack_x(packed);
    int32_t y = unpack_y(packed);
    if (x < 0 || x >= _sw_width || y < 0 || y >= _sw_height)
        return _sw_infinity;
    const _SwStateValue* table = (const _SwStateValue*)_sw_state_base;
    return table[y * _sw_width + x].rhs;
}

static inline void gset(int32_t packed, int32_t value) {
    int32_t x = unpack_x(packed);
    int32_t y = unpack_y(packed);
    if (x < 0 || x >= _sw_width || y < 0 || y >= _sw_height) return;
    _SwStateValue* table = (_SwStateValue*)_sw_state_base;
    table[y * _sw_width + x].g = value;
}

static inline void rset(int32_t packed, int32_t value) {
    int32_t x = unpack_x(packed);
    int32_t y = unpack_y(packed);
    if (x < 0 || x >= _sw_width || y < 0 || y >= _sw_height) return;
    _SwStateValue* table = (_SwStateValue*)_sw_state_base;
    table[y * _sw_width + x].rhs = value;
}

static inline int32_t kcmp(int32_t k1_a, int32_t k2_a,
                            int32_t k1_b, int32_t k2_b) {
    if (k1_a != k1_b) return k1_a < k1_b ? 1 : 0;
    return k2_a < k2_b ? 1 : 0;
}

static inline int32_t heur_manhattan(int32_t a, int32_t b) {
    int32_t dx = abs(unpack_x(b) - unpack_x(a));
    int32_t dy = abs(unpack_y(b) - unpack_y(a));
    return _sw_straight_cost * (dx + dy);
}

static inline int32_t heur_octile(int32_t a, int32_t b) {
    int32_t dx = abs(unpack_x(b) - unpack_x(a));
    int32_t dy = abs(unpack_y(b) - unpack_y(a));
    int32_t mn = dx < dy ? dx : dy;
    return _sw_straight_cost * (dx + dy) + (_sw_diagonal_cost - 2 * _sw_straight_cost) * mn;
}

static inline int32_t heur_chebyshev(int32_t a, int32_t b) {
    int32_t dx = abs(unpack_x(b) - unpack_x(a));
    int32_t dy = abs(unpack_y(b) - unpack_y(a));
    return _sw_straight_cost * (dx > dy ? dx : dy);
}

static inline int32_t ecost(int32_t a, int32_t b) {
    int32_t ax = unpack_x(a), ay = unpack_y(a);
    int32_t bx = unpack_x(b), by = unpack_y(b);
    // bounds check
    if (ax < 0 || ax >= _sw_width  || ay < 0 || ay >= _sw_height) return _sw_infinity;
    if (bx < 0 || bx >= _sw_width  || by < 0 || by >= _sw_height) return _sw_infinity;
    // blocked check
    const uint8_t* bl = (const uint8_t*)_sw_blocked_base;
    if (bl[ay * _sw_width + ax] || bl[by * _sw_width + bx]) return _sw_infinity;
    // neighbour check
    int32_t dx = abs(bx - ax);
    int32_t dy = abs(by - ay);
    bool straight = (dx + dy) == 1;
    bool diagonal = (dx == 1) && (dy == 1);
    if (_sw_connectivity == CONNECTIVITY_FOUR && !straight) return _sw_infinity;
    if (!straight && !diagonal) return _sw_infinity;
    // diagonal move allowed check
    if (diagonal) {
        if (bl[by * _sw_width + ax] || bl[ay * _sw_width + bx]) return _sw_infinity;
        return _sw_diagonal_cost;
    }
    return _sw_straight_cost;
}

static inline void kcalc(int32_t packed, int32_t* k1_out, int32_t* k2_out) {
    int32_t g   = gcost(packed);
    int32_t rhs = rcost(packed);
    int32_t k2  = g < rhs ? g : rhs;
    int32_t sx  = unpack_x(_sw_start);
    int32_t sy  = unpack_y(_sw_start);
    int32_t sp  = pack_state(sx, sy);
    int32_t h;
    switch (_sw_heuristic_type) {
        case HEURISTIC_OCTILE:    h = heur_octile(sp, packed);    break;
        case HEURISTIC_CHEBYSHEV: h = heur_chebyshev(sp, packed); break;
        default:                  h = heur_manhattan(sp, packed);  break;
    }
    *k1_out = k2 + h + _sw_km;
    *k2_out = k2;
}


#endif // DSTAR_USE_HW_ISA

#endif // ISA_EXTENSIONS_H_