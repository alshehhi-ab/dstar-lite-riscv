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

#include <cstdlib>
#include "grid_map.h"


GridMap::GridMap(int32_t width, int32_t height, Connectivity connectivity) : width(width), 
                                                                            height(height),
                                                                            connectivity(connectivity),
                                                                            blocked(width * height, 0U) {}

int32_t GridMap::toIndex(const State &state) const
{
   return ((state.y * width) + state.x);
}

bool GridMap::isValid(const State &state) const
{
    return ((state.x >= 0) && (state.x < width) && (state.y >= 0) && (state.y < height));
}

bool GridMap::isBlocked(const State &state) const
{   
    // If the state is not valid, then it is blocked.
    if (!isValid(state)) {
        return true;
    }
    
    // Otherwise check the blocked flag of the state in the blocked list.
    return blocked[toIndex(state)] != 0U;
}

void GridMap::setBlocked(const State& state, bool is_blocked)
{
    if (!isValid(state)){
        return;
    }

    blocked[toIndex(state)] = is_blocked ? 1U : 0U;
}

bool GridMap::areNeighbors(const State& state_a, const State& state_b) const
{
    // Depends on the connectivity.
    // But both share the basic four neighbours. (The eight extends it with four corners)

    // Calculate four straight neighbours
    
    // If either of the states are invalid, then they are not neighbours. (Or at least we don't care about them for succ and pred)
    if (!isValid(state_a) || !isValid(state_b)) {
        return false;
    }

    // Straight neighbours are offset of 1 in one coordinate.
    // Diagonal are offsets in both coordinates by 1.

    // Check offsets in x and y.
    const int32_t dx = std::abs(state_b.x - state_a.x);
    const int32_t dy = std::abs(state_b.y - state_a.y);

    // For a straight neighbour, dx  dy == 1
    const bool straight = ((dx + dy) == 1);

    if (connectivity == Connectivity::kFour) {
        return straight;
    }

    // For a diagonal neigbour both dx and dy must be == 1;
    const bool diagonal = ((dx == 1) && (dy == 1));

    if (connectivity == Connectivity::kEight){
        return (straight || diagonal);
    }

    // Otherwise
    return false;
}

//  Is the motion between state
bool GridMap::isDiagonalMove(const State& state_a, const State& state_b) const
{
    const int32_t dx = std::abs(state_b.x - state_a.x);
    const int32_t dy = std::abs(state_b.y - state_a.y);

    return ((dx == 1) && (dy == 1));
}

// If a state_b is diagonal to state_a, then check if it is blocked. it is blocked if the intersecting neighbours of state_a and state_b are blocked.
// This means that a diagonal traversal from state_a to state_b is not possible, even if b is not blocked.
bool GridMap::isDiagonalMoveAllowed(const State &state_a, const State &state_b) const
{   
    // If it is a straight motion (direct), then there is no issue.
    if (!isDiagonalMove(state_a, state_b)) {
        return true;
    }
    
    return (!isBlocked(State{state_b.x, state_a.y}) && !isBlocked(State{state_a.x, state_b.y}));
    }


int32_t GridMap::cost(const State &state_a, const State &state_b) const
{
    // c(s, s')

    // If either one is invalid, then the cost is infinity.
    if (!isValid(state_a) || !isValid(state_b)) {
        return kInfinity;
    }

    // If either of the states are blocked, then the cost is infinity.
    if (isBlocked(state_a) || isBlocked(state_b)) {
        return kInfinity;
    }

    // state_b must be a successor to state_a, otherwise the cost is inifnity.
    if (!areNeighbors(state_a, state_b)) {
        return kInfinity;
    }

    if (isDiagonalMove(state_a, state_b)) {
        // if cannot move diagonally, then the cost is infinity.
        if (!isDiagonalMoveAllowed(state_a, state_b)) {
            return kInfinity;
        }

        // otherwise it is just the diagonal, with the diagonal cost.
        return kDiagonalCost;
    }

    //Otherwise the cost is just the straight cost.
    return kStraightCost;
}

std::vector<State> GridMap::getNeighbors(const State &state) const
{
    std::vector<State> neighbors;

    // If the state is not a valid state, then it has no neighbours
    if (!isValid(state)){
        return neighbors;
    }

    neighbors.reserve(8);

    // Lambda function: adds if the neighbour is valid.
    auto addIfValid = [&](const State& candidate){
        if (isValid(candidate) && isDiagonalMoveAllowed(state, candidate)) {
            neighbors.push_back(candidate);
        }
    };

    // First do the 4-way connectivity
    addIfValid(State{state.x+1, state.y}); // EAST NEIGHBOUR
    addIfValid(State{state.x-1, state.y}); // WEST NEIGHBOUR
    addIfValid(State{state.x, state.y+1}); // NORTH NEIGHBOUR
    addIfValid(State{state.x, state.y-1}); // SOUTH NEIHGBOUR
    
    // Next add the diagonals if the planner is 8-way.
    if (connectivity == Connectivity::kEight){
        addIfValid(State{state.x+1, state.y+1}); // NE NEIGHBOUR
        addIfValid(State{state.x-1, state.y+1}); // NW NEIGHBOUR
        addIfValid(State{state.x+1, state.y-1}); // SE NEIGHBOUR
        addIfValid(State{state.x-1, state.y-1}); // SW NEIGHBOUR
    }
 
    return neighbors;
}

std::vector<State> GridMap::getSucc(const State &state) const
{
    std::vector<State> succ;
    succ.reserve(8);

    if (!isValid(state) || isBlocked(state)) {
        return succ;
    }

    for (const State& candidate : getNeighbors(state)) {
        // To be a true successor, the state not be blocked.
        if (!isBlocked(candidate)) {
            succ.push_back(candidate);
        }
    }
    return succ;
}

std::vector<State> GridMap::getPred(const State &state) const
{
    std::vector<State> pred;

    if (!isValid(state)) {
        return pred;
    }

    pred.reserve(8);

    for (const State& candidate : getNeighbors(state)) {
        if (!isBlocked(candidate)) {
            pred.push_back(candidate);
        }
    }

    return pred;
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
    return connectivity;
}
