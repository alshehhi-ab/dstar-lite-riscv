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

#include <iostream>
#include <iomanip>
#include "dstar_lite.h"
#include "grid_map.h"

void printPath(const std::vector<State>& path) {
    std::cout << "Current Path: ";
    if (path.empty()) {
        std::cout << "NO PATH FOUND";
    } else {
        for (const auto& s : path) {
            std::cout << "(" << s.x << "," << s.y << ") -> ";
        }
        std::cout << "DONE";
    }
    std::cout << std::endl;
}

int main() {
    // 1. Create a 10x10 grid with 8-way connectivity
    GridMap map(10, 10, Connectivity::kEight);
    DStarLite planner(map, HeuristicType::kOctile);

    // 2. Define Start and Goal
    State start = {0, 0};
    State goal = {5, 5};
    planner.setStart(start);
    planner.setGoal(goal);

    std::cout << "--- TEST 1: Initial Planning ---" << std::endl;
    if (planner.plan()) {
        printPath(planner.getPath());
    } else {
        std::cout << "Failed to find initial path!" << std::endl;
        return -1;
    }

    std::cout << "\n--- TEST 2: Robot Movement ---" << std::endl;
    // Simulate robot moving to (1,1)
    State next_step = {1, 1};
    planner.updateStart(next_step);
    
    // After moving, we must reconstruct the path from the new start
    // In our loop, we'll just call replan() as it handles math consistency check + path reconstruction
    planner.replan(); 
    std::cout << "Robot moved to (1,1). " << std::endl;
    printPath(planner.getPath());

    std::cout << "\n--- TEST 3: Obstacle Discovery & Replanning ---" << std::endl;
    // Let's block (2,2) which is likely on the diagonal path
    std::cout << "Detected obstacle at (2,2)! Rerouting..." << std::endl;
    planner.setBlocked({2, 2}, true);
    
    if (planner.replan()) {
        printPath(planner.getPath());
    } else {
        std::cout << "Robot is trapped!" << std::endl;
    }

    return 0;
}