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

#include "dstar_lite.h"

DStarLite::DStarLite(const GridMap &grid, const HeuristicType heuristic_type) :
    grid(grid),
    heuristic_type(heuristic_type){
        state_values.resize(grid.getWidth() * grid.getHeight());
    }

State DStarLite::getStart() const {
    return s_start;
}

State DStarLite::getGoal() const {
    return s_goal;
}

void DStarLite::setStart(const State &start) {
    s_start = start;
}

void DStarLite::setGoal(const State &goal) {
    s_goal = goal;
}

int32_t DStarLite::toIndex(const State &state) const {
    return (state.y * grid.getWidth() + state.x);
}


void DStarLite::setG(const State &state, int32_t g) {
    state_values[toIndex(state)].g = g;
}

void DStarLite::setRhs(const State &state, int32_t rhs) {
    state_values[toIndex(state)].rhs = rhs;
}

int32_t DStarLite::getG(const State &state) const {
    return state_values[toIndex(state)].g;
}

int32_t DStarLite::getRhs(const State &state) const {
    return state_values[toIndex(state)].rhs;
}

void DStarLite::setHeuristicType(HeuristicType heuristic_type) {   
    //TODO: Add checking logic later.
    this->heuristic_type = heuristic_type;
}

HeuristicType DStarLite::getHeuristicType() const {
    return heuristic_type;
}

int32_t DStarLite::heuristic(const State &state_a, const State &state_b) const {
    //TODO: Implement this with different heuristics.
    return 0;
}

void DStarLite::insert(const State &state) {
    //TODO
}

void DStarLite::remove(const State &state) {
    //TODO
}

void DStarLite::initialize(){
    //TODO
}

Key DStarLite::calculateKey(const State &state) const {
    //TODO: Implement this
    return Key();
}

void DStarLite::updateVertex(const State &state) {
    //TODO
}

void DStarLite::computeShortestPath() {
    //TODO
}

void DStarLite::reconstructPath() {
    //TODO
}

bool DStarLite::plan() {
    //TODO
    return false;
}