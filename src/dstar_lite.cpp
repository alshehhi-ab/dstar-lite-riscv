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

#include "dstar_lite.h"
#include "isa_extensions.h"

// Constructor

DStarLite::DStarLite(const GridMap &grid, const HeuristicType heuristic_type) :
    grid(grid),
    heuristic_type(heuristic_type){
        state_values.resize(grid.getWidth() * grid.getHeight());
        in_open.assign(grid.getWidth() * grid.getHeight(), 0);  // added for lazy-deletion

#ifdef DSTAR_USE_HW_ISA
        dstar_csr_init(
            state_values.data(),
            nullptr, // blocked_base — added when ECOST is implemented
            grid.getWidth(),
            grid.getHeight(),
            s_start.x,          s_start.y,
            s_goal.x,           s_goal.y,
            km,
            kInfinity,
            kStraightCost,
            kDiagonalCost,
            static_cast<int32_t>(heuristic_type),
            static_cast<int32_t>(grid.getConnectivity())
        );
#endif
}

// Public setters and getters for basic D* Lite utility.
State DStarLite::getStart() const {
    return s_start;
}

State DStarLite::getGoal() const {
    return s_goal;
}

void DStarLite::setStart(const State &start) {
    s_start = start;
    s_last = start;
}

void DStarLite::setGoal(const State &goal) {
    s_goal = goal;
}

// Internal Indexer

int32_t DStarLite::toIndex(const State &state) const {
    return ((state.y * grid.getWidth()) + state.x);
}

// g and rhs functions

void DStarLite::setG(const State &state, int32_t g)
{
    // If invalid state, then do not set g (no g to set)
    
    if (!grid.isValid(state)) {
        return; // Do nothing.
    }

    // Otherwise set g.
#ifdef DSTAR_USE_HW_ISA
    gset(pack_state(state.x, state.y), g);
#else
    state_values[toIndex(state)].g = g;
#endif
}

void DStarLite::setRhs(const State &state, int32_t rhs) {
    // If invalid state, then do not set rhs (no rhs to set)

    if (!grid.isValid(state)) {
        return;
    }
    
    // Otherwise set rhs.
#ifdef DSTAR_USE_HW_ISA
    rset(pack_state(state.x, state.y), rhs);
#else
    state_values[toIndex(state)].rhs = rhs;
#endif
}

int32_t DStarLite::getG(const State &state) const {
    // If the state is invalid, then get infinite g

    if (!grid.isValid(state)) {
        return kInfinity;
    }

    // Otherwise
#ifdef DSTAR_USE_HW_ISA
    return gcost(pack_state(state.x, state.y));
#else
    return state_values[toIndex(state)].g;
#endif
}

int32_t DStarLite::getRhs(const State &state) const {
    // If the state is invalid, then get infinite rhs
    
    if (!grid.isValid(state)) {
        return kInfinity;
    }

    // Otherwise
#ifdef DSTAR_USE_HW_ISA
    return rcost(pack_state(state.x, state.y));
#else
    return state_values[toIndex(state)].rhs;
#endif
}

// Heuristic functions

void DStarLite::setHeuristicType(HeuristicType heuristic_type) {   
    //TODO: Add checking logic later.
    this->heuristic_type = heuristic_type;
}

HeuristicType DStarLite::getHeuristicType() const {
    return heuristic_type;
}

int32_t DStarLite::heuristic(const State &state_a, const State &state_b) const {

#ifdef DSTAR_USE_HW_ISA
    int32_t a = pack_state(state_a.x, state_a.y);
    int32_t b = pack_state(state_b.x, state_b.y);
    switch (heuristic_type) {
        case HeuristicType::kOctile:    return heur_octile(a, b);
        case HeuristicType::kChebyshev: return heur_chebyshev(a, b);
        case HeuristicType::kManhattan: return heur_manhattan(a, b);
        default:                        return heur_manhattan(a, b);
    }

#else    
    // First define dx, dy
    const int32_t dx = std::abs(state_b.x - state_a.x);
    const int32_t dy = std::abs(state_b.y - state_a.y);

    // Based on heuristic type, find the heuristic

    switch (heuristic_type) {

        case HeuristicType::kManhattan: {
            return (kStraightCost * (dx + dy));
        }

        case HeuristicType::kOctile: {
            const int32_t min_dir = (dx < dy) ? dx : dy;
            return ((kStraightCost*(dx + dy)) + ((kDiagonalCost -(2*kStraightCost))*min_dir));
        }

        case HeuristicType::kChebyshev: {
            const int32_t max_dir = (dx > dy) ? dx : dy;
            return (kStraightCost * max_dir);
        }

        default:
            return (kStraightCost * (dx + dy));
    }

#endif
}

// PQ Operations

void DStarLite::insert(const State &state) {
    
    if (!grid.isValid(state)) {
        return;
    }

    // insert a state with a key into the OPEN PQ
    in_open[toIndex(state)] = 1; // Lazy-deletion tracking list
    open.push(PQNode{state, calculateKey(state)});
}

void DStarLite::insert(const State &state, const Key &key) {
    
    if (!grid.isValid(state)){
        return;
    }
    
    in_open[toIndex(state)] = 1; // Lazy-deletion tracking list
    open.push(PQNode{state, key});
}


void DStarLite::remove(const State &state) {
    // Remove the state from the priority queue.

    // Lazy-deletion version:

    if (!grid.isValid(state)) return;
    in_open[toIndex(state)] = 0;

}


// D* Lite core procedures

Key DStarLite::calculateKey(const State &state) const {

#ifdef DSTAR_USE_HW_ISA
    int32_t k1, k2;
    kcalc(pack_state(state.x, state.y), &k1, &k2);
    return Key{k1, k2};
#else
    const int32_t g_val = getG(state);
    const int32_t rhs_val = getRhs(state);
    
    // K2 = min (g(s), rhs(s))
    int32_t k2_val = (g_val < rhs_val) ? g_val : rhs_val;

    // K1 = K2 + h(s_start, s) + km
    int32_t k1_val = k2_val + heuristic(s_start, state) + km;

    return Key{k1_val, k2_val};
#endif
}

void DStarLite::initialize(){
    // Clear the OPEN PQ (U = empty)
    while (!open.empty()){
        open.pop();
    }

    // Set km = 0
    km = 0;

#ifdef DSTAR_USE_HW_ISA
    dstar_csr_set_km(km);
#endif

    path.clear();

    // set all rhs, g, of all states = infinity
    for (StateValue& value : state_values) {
        value.g = kInfinity;
        value.rhs = kInfinity;
    }

    // Reset OPEN membership trackers. (lazy-deletion fix)
    std::fill(in_open.begin(), in_open.end(), 0);

    // Set rhs of s_goal = 0
    setRhs(s_goal, 0);
    insert(s_goal, Key{heuristic(s_start, s_goal), 0});

    s_last = s_start;
}

void DStarLite::updateVertex(const State& state) {
    
    // [1] If (u != s_goal), rhs(u) = min_(s' in succ(u)) (c(u,s') + g(s'))
    
    int32_t min_rhs = kInfinity;
    
    // if u == s_goal, then rhs(u) = 0, otherwise, the rhs(u) is as follows:
    if (state != s_goal) {

        // Get successors of u.
        // Find the successor with the minimal (c(u,s') + g(s'))
        for (const State& successor : grid.getSucc(state)) {
            // Calculate c(u,s') and g(s')
            int32_t edge_cost = grid.cost(state, successor);
            int32_t g_succ = getG(successor);

            // If either of them are infinity, do nothing and ignore.

            if ((edge_cost != kInfinity) && (g_succ != kInfinity)) {
                // Now compute successor rhs
                int32_t succ_rhs = edge_cost + g_succ;

                // If succ_rhs < min_rhs. Then the minimum rhs will be that of the successor.
                min_rhs = (succ_rhs < min_rhs) ? succ_rhs : min_rhs;
            }
        }

        // once the minimum successor has been found, update rhs(u)
        setRhs(state, min_rhs); 
    }

    // [2] If u is in OPEN, then remove it.
    remove(state);

    // [3] If u is locally inconsistent, insert u into open
    
    // Inconsistent when g(u) != rhs(u)
    if (getG(state) != getRhs(state)) {
        insert(state); // Insert will automatically caclulate the new key.
    }
}   

void DStarLite::computeShortestPath() {
    
    // operate on the OPEN PQ.

    // While the top key is less than start's key OR start is locally inconssistent:
    while((!open.empty()) && ((open.top().key < calculateKey(s_start)) || (getRhs(s_start) != getG(s_start)) )) {

        // [1] top and pop U into u.
        const PQNode node_u = open.top();
        open.pop(); // remove u from OPEN since we will update it.

        const State u = node_u.state;

        // Lazy-deletion: skip if u is removed/superseded
        if ((!grid.isValid(u)) || (!in_open[toIndex(u)])){
            continue;
        }

        in_open[toIndex(u)] = 0;

        
        const Key key_old = node_u.key;
        const Key key_new = calculateKey(u);

        // Update if the key has changed.
        if (key_old < key_new) {
            // Update with the new key
            insert(u, key_new);
        }

        // [2] Now deal with inconsistencies

        // overconsistent case 
        else if (getG(u) > getRhs(u)) { //overconsistent, better path found
            // When g(u) > rhs(u), set g(u) = rhs(u)
            setG(u, getRhs(u)); // consistency established.
            // u already removed from OPEN, no need to invoke removal again. 

            // Now update all the vertices of u's predecessors
            for (const State& predecessor : grid.getPred(u)){
                updateVertex(predecessor);
            }
        }
        
        // Underconsistent case g(u) < rhs(u)
        else {
            
            // g(u) = INF
            setG(u, kInfinity);

            // update the vertex of u, and u's predecessors
            updateVertex(u);

            for (const State& predecessor : grid.getPred(u)){
                updateVertex(predecessor);
            }
        }
    }
}


/* void DStarLite::reconstructPath() {
    
    // First clear the path.
    path.clear();

    // If either start or goal are invalid, then no path.
    if ((!grid.isValid(s_start)) || (!grid.isValid(s_goal))){
        return; 
    }

    // If either start or goal are blocked, then no path.
    if ((grid.isBlocked(s_start)) || (grid.isBlocked(s_goal))){
        return;
    }

    // No path exists if the start has an infinite to-go cost.
    if (getG(s_start) == kInfinity){
        return;
    }

    
    int32_t max_steps = grid.getWidth() * grid.getHeight();

    State s_curr = s_start; // Current state, itialise as the starting state.
    path.push_back(s_curr);

    for (int32_t step = 0; step < max_steps; ++step) {

        // When goal is reached, terminate.
        if (s_curr == s_goal){
            return;
        }


    }
} */

void DStarLite::reconstructPath()
{
    path.clear();

    if (getG(s_start) == kInfinity) {
        return;
    }

    State current = s_start;
    path.push_back(current);

    const int32_t max_steps = grid.getWidth() * grid.getHeight();

    for (int32_t step = 0; (step < max_steps) && (current != s_goal); ++step) {
        int32_t best_cost = kInfinity;
        State best_successor{};
        bool found = false;

        for (const State& successor : grid.getSucc(current)) {
            const int32_t edge_cost = grid.cost(current, successor);
            const int32_t g_successor = getG(successor);

            if (edge_cost == kInfinity || g_successor == kInfinity) {
                continue;
            }

            const int32_t candidate_cost = edge_cost + g_successor;

            if (candidate_cost < best_cost) {
                best_cost = candidate_cost;
                best_successor = successor;
                found = true;
            }
        }

        if (!found) {
            path.clear();
            return;
        }

        current = best_successor;
        path.push_back(current);
    }

    if (current != s_goal) {
        path.clear();
    }
}

const std::vector<State> &DStarLite::getPath() const {
    return path;
}


// Main planning functions

bool DStarLite::plan() {
    
    if (!grid.isValid(s_start) || !grid.isValid(s_goal)) {
        path.clear();
        return false;
    }

    if (grid.isBlocked(s_start) || grid.isBlocked(s_goal)) {
        path.clear();
        return false;
    }


    s_last = s_start;
    initialize();
    computeShortestPath();

    reconstructPath();
    return ((!path.empty()) && (path.back() == s_goal));
}   


bool DStarLite::replan() {
    
    computeShortestPath();

    // If g(s_start) = infinity then no path exists.
    if (getG(s_start) == kInfinity) {
        path.clear();
        return false;
    }

    // Now rebuild the path again.
    reconstructPath();

    return ((!path.empty()) && (path.back() == s_goal));
} 



// Dynamic planning helpers
void DStarLite::updateStart(const State &new_start) {

    if ((!grid.isValid(new_start)) || (grid.isBlocked(new_start))){
        return;
    }

    km += heuristic(s_last, new_start);

    s_last = new_start;
    s_start = new_start;

#ifdef DSTAR_USE_HW_ISA
    dstar_csr_set_km(km);
    dstar_csr_set_start(s_start.x, s_start.y);
#endif

    path.clear();
}

void DStarLite::setBlocked(const State &state, bool is_blocked) {

    if ((!grid.isValid(state)) || (grid.isBlocked(state) == is_blocked)){
        return;
    }


    //km += heuristic(s_last, s_start);
    //s_last = s_start;

    // Update the gridmap with the blocked state.
    grid.setBlocked(state, is_blocked);

    // Update vertices that are affected by this block

    // update the state itself
    updateVertex(state);

    // update its neighbours.
    for (const State& neighbor : grid.getNeighbors(state)){
        updateVertex(neighbor);
    }
    
}
