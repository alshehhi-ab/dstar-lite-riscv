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



#ifndef FIXED_POINT_H_
#define FIXED_POINT_H_

#include <cstdint>
#include <climits>

constexpr int32_t kCostScale = 1000;
constexpr int32_t kCostInfinity = INT32_MAX / 4;
constexpr int32_t kCostStraight = 1000;
constexpr int32_t kCostDiagonal = 1414;

#endif