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

#ifndef GRID_MAP_H_
#define GRID_MAP_H_

#include <cstdint>
#include <vector>

#include "state.h"

enum class Connectivity : uint8_t { 
    kFour = 4,
    kEight = 8,
};

class GridMap {

    private:
        int32_t height;
        int32_t width;
        Connectivity connectivity;
        std::vector<bool> blocked; // Blocked list

        int32_t toIndex(const State& state) const;

        bool areNeighbors(const State& state_a, const State& state_b) const;

        bool isDiagonalMove(const State& state_a, const State& state_b) const;

        bool isDiagonalMoveAllowed(const State& state_a, const State& state_b) const;
        
    public:
        GridMap(int32_t width, int32_t height, Connectivity connectivity);

        bool isValid(const State& state) const;

        bool isBlocked(const State& state) const;

        void setBlocked(const State& state, bool blocked);

        int32_t cost(const State& state_a, const State& state_b) const;

        std::vector<State>getNeighbors(const State& state) const;

        std::vector<State> getSucc(const State& state) const;

        std::vector<State> getPred(const State& state) const;
        
        int32_t getWidth() const;

        int32_t getHeight() const;

        Connectivity getConnectivity() const;


};

#endif