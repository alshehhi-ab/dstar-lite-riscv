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

#include "grid_map.h"


GridMap::GridMap(int32_t width, int32_t height, Connectivity connectivity)
{
    //TODO
}

int32_t GridMap::toIndex(const State &state) const
{
    //TODO
    return 0;
}

bool GridMap::isValid(const State &state) const
{
    //TODO
    return false;
}

bool GridMap::isBlocked(const State &state) const
{
    //TODO
    return false;
}

void GridMap::setBlocked(const State &state, bool blocked)
{
    //TODO
}

bool GridMap::areNeighbors(const State &state_a, const State &state_b) const
{
    //TODO
    return false;
}

bool GridMap::isDiagonalMove(const State &state_a, const State &state_b) const
{
    //TODO
    return false;
}

bool GridMap::isDiagonalMoveAllowed(const State &state_a, const State &state_b) const
{
    //TODO
    return false;
}


int32_t GridMap::cost(const State &state_a, const State &state_b) const
{
    //TODO
    return 0;
}

std::vector<State> GridMap::getNeighbors(const State &state) const
{
    //TODO
    return std::vector<State>();
}

std::vector<State> GridMap::getSucc(const State &state) const
{
    //TODO
    return std::vector<State>();
}

std::vector<State> GridMap::getPred(const State &state) const
{
    //TODO
    return std::vector<State>();
}

int32_t GridMap::getWidth() const
{
    return width;
}

int32_t GridMap::getHeight() const
{
    return height;
}

Connectivity GridMap::getConnectivity() const
{   
    //TODO
    return Connectivity();
}
