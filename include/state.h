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

#ifndef STATE_H_
#define STATE_H_

#include <cstdint>

/**
 * @brief State representation of D* Lite vertex, contains the x and y indices.
 * 
 */
struct State{
    int32_t x;
    int32_t y;

    bool operator == (const State& otherState) const {
        return ((x == otherState.x)  && (y == otherState.y)); 
    }

    bool operator != (const State& otherState) const {
        return ((x != otherState.x) || (y != otherState.y));
    }

};

#endif