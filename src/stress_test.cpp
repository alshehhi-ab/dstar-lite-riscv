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
#include <vector>
#include "dstar_lite.h"
#include "grid_map.h"

void printStatus(const std::string& msg, DStarLite& p) {
    const auto& path = p.getPath();
    std::cout << "[TEST] " << msg << "\n  Start: (" << p.getStart().x << "," << p.getStart().y 
              << ") | Path Size: " << path.size() << " steps" << std::endl;
}

int main() {
    GridMap map(20, 20, Connectivity::kEight);
    DStarLite planner(map, HeuristicType::kOctile);

    planner.setStart({0, 0});
    planner.setGoal({18, 18});

    // 1. Initial Plan
    std::cout << "--- 1. Initial Planning (Empty Map) ---" << std::endl;
    planner.plan();
    printStatus("Baseline path created.", planner);

    // 2. Build a "Great Wall" at X=10 with a single gap at Y=18
    std::cout << "\n--- 2. Building a Wall at X=10 (Gap at Y=18) ---" << std::endl;
    for (int y = 0; y < 18; ++y) {
        planner.setBlocked({10, y}, true);
    }
    
    if (planner.replan()) {
        printStatus("Rerouted through the bottom gap.", planner);
    }

    // 3. Move the robot 5 steps deep into the map
    std::cout << "\n--- 3. Moving Robot (Simulating 5 steps) ---" << std::endl;
    for (int i = 0; i < 5; ++i) {
        if (planner.getPath().size() > 1) {
            planner.updateStart(planner.getPath()[1]);
            planner.replan(); // Refresh path
        }
    }
    printStatus("Robot is mid-mission.", planner);

    // 4. THE STRESS POINT: Close the gap and open a new one at Y=0
    // This forces D* Lite to undo a lot of work and propagate changes far away.
    std::cout << "\n--- 4. DYNAMIC STRESS: Closing Gap at Y=18, Opening Gap at Y=0 ---" << std::endl;
    planner.setBlocked({10, 18}, true); // Close old gap
    planner.setBlocked({10, 19}, true); // Close bottom edge
    planner.setBlocked({10, 0}, false);  // Open new gap at top
    
    std::cout << "Calculating massive detour..." << std::endl;
    if (planner.replan()) {
        printStatus("SUCCESS: Detour found through new top gap!", planner);
    } else {
        std::cout << "FAILED: Robot is trapped." << std::endl;
    }

    return 0;
}