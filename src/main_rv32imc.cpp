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

#include <stdint.h>

#include "dstar_lite.h"
#include "grid_map.h"
#include "state.h"

extern "C" {
#include "demo_system.h"
// #include "timer.h"
}


static uint32_t rng_state = 42u;

static uint32_t NextRandom(void) {
  rng_state = (1664525u * rng_state) + 1013904223u;
  return rng_state;
}

static int32_t RandomRange(int32_t max_value) {
  return static_cast<int32_t>(NextRandom() % static_cast<uint32_t>(max_value));
}

static inline uint32_t ReadMcycleLow(void) {
  uint32_t value;
  __asm__ volatile("csrr %0, mcycle" : "=r"(value));
  return value;
}

static inline uint32_t ReadMcycleHigh(void) {
  uint32_t value;
  __asm__ volatile("csrr %0, mcycleh" : "=r"(value));
  return value;
}

static inline uint64_t ReadMcycle(void) {
  uint32_t high_before;
  uint32_t low;
  uint32_t high_after;

  do {
    high_before = ReadMcycleHigh();
    low = ReadMcycleLow();
    high_after = ReadMcycleHigh();
  } while (high_before != high_after);

  return (static_cast<uint64_t>(high_before) << 32) | low;
}

static inline uint32_t ReadMinstretLow(void) {
  uint32_t value;
  __asm__ volatile("csrr %0, minstret" : "=r"(value));
  return value;
}

static inline uint32_t ReadMinstretHigh(void) {
  uint32_t value;
  __asm__ volatile("csrr %0, minstreth" : "=r"(value));
  return value;
}

static inline uint64_t ReadMinstret(void) {
  uint32_t high_before;
  uint32_t low;
  uint32_t high_after;

  do {
    high_before = ReadMinstretHigh();
    low = ReadMinstretLow();
    high_after = ReadMinstretHigh();
  } while (high_before != high_after);

  return (static_cast<uint64_t>(high_before) << 32) | low;
}

static void PrintUInt64(uint64_t value) {
  if (value == 0u) {
    putchar('0');
    return;
  }

  char buffer[21];
  int32_t index = 0;

  while (value > 0u) {
    buffer[index] = static_cast<char>('0' + (value % 10u));
    value /= 10u;
    ++index;
  }

  while (index > 0) {
    --index;
    putchar(buffer[index]);
  }
}

static void PrintMetric(const char* label, uint64_t value) {
  puts(label);
  PrintUInt64(value);
  puts("\r\n");
}

int main(void) {
 
  const int32_t kWidth = 32;
  const int32_t kHeight = 32;
  const int32_t kReplans = 200;
  const int32_t kObstaclesPerStep = 2;

  GridMap grid(kWidth, kHeight, Connectivity::kEight);

  State start = {5, 5};
  State goal = {27, 27};

  DStarLite dstar(grid, HeuristicType::kOctile);
  dstar.setStart(start);
  dstar.setGoal(goal);

  puts("Starting D* Lite benchmark...\r\n");

  uint64_t cycle_start;
  uint64_t cycle_end;
  uint64_t instr_start;
  uint64_t instr_end;

  /*
   * Initial plan.
   */
  puts("Starting initial plan...\r\n");

  cycle_start = ReadMcycle();
  instr_start = ReadMinstret();

  dstar.plan();

  instr_end = ReadMinstret();
  cycle_end = ReadMcycle();

  const uint64_t plan_cycles = cycle_end - cycle_start;
  const uint64_t plan_instrs = instr_end - instr_start;

  PrintMetric("Initial plan cycles: ", plan_cycles);
  PrintMetric("Initial plan instructions: ", plan_instrs);

  /*
   * Replanning loop.
   */
  puts("Starting replans...\r\n");

  cycle_start = ReadMcycle();
  instr_start = ReadMinstret();

  for (int32_t i = 0; i < kReplans; ++i) {
    for (int32_t j = 0; j < kObstaclesPerStep; ++j) {
      State s = {
          RandomRange(kWidth),
          RandomRange(kHeight)
      };

      if ((s == start) || (s == goal)) {
        continue;
      }

      const bool blocked = ((NextRandom() & 1u) == 0u);
      dstar.setBlocked(s, blocked);
    }

    dstar.replan();
  }

  instr_end = ReadMinstret();
  cycle_end = ReadMcycle();

  const uint64_t replan_cycles = cycle_end - cycle_start;
  const uint64_t replan_instrs = instr_end - instr_start;

  PrintMetric("Total replan cycles: ", replan_cycles);
  PrintMetric("Total replan instructions: ", replan_instrs);

  if (kReplans > 0) {
    PrintMetric("Cycles per replan: ",
                replan_cycles / static_cast<uint64_t>(kReplans));

    PrintMetric("Instructions per replan: ",
                replan_instrs / static_cast<uint64_t>(kReplans));
  }

  puts("DONE\r\n");

  while (1) {
  }

  return 0;
}