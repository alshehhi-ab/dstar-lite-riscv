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


#ifndef DSTAR_LITE_H_
#define DSTAR_LITE_H_

#include <cstdint>
#include <climits>
#include <iostream>
#include <vector>
#include <queue>

#include "state.h"
#include "grid_map.h"


/* 
constexpr int32_t kInfinity = INT32_MAX / 4;
constexpr int32_t kCostScale = 1000;
constexpr int32_t kStraightCost = 1000;
constexpr int32_t kDiagonalCost = 1414;
*/

enum class HeuristicType : uint8_t {
    kManhattan = 0,
    kOctile = 1,
    kChebyshev = 2
};

/**
 * @brief Lexicographic key used by D*Lite Priority Queue
 * 
 */
struct Key{
    int32_t k1;
    int32_t k2;


    // Comparison operators
    bool operator < (const Key& other) const{
        // check equality of k1. If both k1 values are equal check the k2 values.
        if (k1 != other.k1) { //k1 's are different 
            return (k1 < other.k1);
            }
        else { // k1 's are the same, so check k2
            return (k2 < other.k2);
        }
        }
    
    bool operator > (const Key& other) const{ // opposite of less than operator
        return (other < *this);
    }

    bool operator == (const Key& other) const{ // keys are equal when both elements are the same
        return ((k1 == other.k1) && (k2 == other.k2));
    }

    bool operator != (const Key& other) const{ // Two kets are not equal
        return !(*this == other); // inverse of the equality operator.
    }
    
};

/**
 * @brief Cost values associated with a state.
 * 
 */
struct StateValue{
    int32_t g = kInfinity;
    int32_t rhs = kInfinity;
    //int32_t cost = kInfinity;
};

/**
 * @brief Is a node within the priority queue.
 * 
 * Stores a state and it's associated key when inserted to the OPEN PQ.
 */
struct PQNode{
    State state;
    Key key;
};

/**
 * @brief Comparator definition for the OPEN PQ.
 * 
 */
struct PQNodeCompare {
    bool operator () (const PQNode& first, const PQNode& second) const {
        return (first.key > second.key);
    }
};

class DStarLite {

    private:

    // Variables
        State s_start;
        State s_goal;
        State s_last;

        GridMap grid;

        int32_t km = 0;

        HeuristicType heuristic_type = HeuristicType::kManhattan;

    // PQ and vectors

        /**
         * @brief OPEN PQ
         * 
         * Is D* Lite's "U" list as defined by Koenig and Likhachev 
         */
        std::priority_queue<PQNode, std::vector<PQNode>, PQNodeCompare> open;

        /**
         * @brief Vector of all states' g and rhs values
         * 
         */
        std::vector<StateValue> state_values;

        /**
         * @brief Tracks whether a state is currently in the OPEN PQ.
         * 
         * Used for O(1) lazy ddeletion.
         */
        std::vector<uint8_t>in_open;

        /**
         * @brief Vector of the path taken by the planner
         * 
         */
        std::vector<State> path;
        
    // General Functions

        /**
         * @brief Converts the state's values into an index
         * 
         * @param state 
         * @return int32_t 
         */
        int32_t toIndex(const State& state) const;

        /**
         * @brief Inserts a state into the OPEN PQ
         * 
         * @param state 
         */
        void insert(const State& state);

        void insert(const State& state, const Key& key);

        /**
         * @brief Removes a state from the OPEN PQ.
         * 
         * @param state 
         */
        void remove(const State& state);

        /**
         * @brief Calculates the heuristic cost between two states.
         * 
         * @param state_a 
         * @param state_b 
         * @return int32_t heuristic cost. 
         */
        int32_t heuristic(const State& state_a, const State& state_b) const;

    // DStar Lite specific procedures
        /**
         * @brief Initialises the intial values
         * 
         */
        void initialize();

        /**
         * @brief Calculates the lexicographic key for a given state
         * 
         * @param state state of which the key should be calculated
         * @return Key 
         */
        Key calculateKey(const State& state) const;

        /**
         * @brief Updates the vertex. Is a D* Lite function.
         * 
         * @param state 
         */
        void updateVertex(const State& state);

        /**
         * @brief Computes the shortest path
         * 
         */
        void computeShortestPath();
    
    // Path functions
        /**
         * @brief Reconstructs the path
         * 
         */
        void reconstructPath();


    public:
        
        // CONSTRUCTOR
        /**
         * @brief Construct a new DStarLite object
         * 
         * @param grid 
         * @param heuristic_type (0 for Manhattan, 1 for Octile, 2 for Chebyshev)
         */
        DStarLite(const GridMap& grid, const HeuristicType heuristic_type = HeuristicType::kManhattan);

        // MAIN DSTAR LITE FUNCTIONS
        /**
         * @brief the main planning and replanning loop.
         * 
         * @return bool true if planning successful, false if no plan exists.
         */
        bool plan();

        // HELPER FUNCTIONS

        /**
         * @brief Get the Start object.
         * 
         * @return State 
         */
        State getStart() const;

        /**
         * @brief Get the Goal object.
         * 
         * @return State 
         */
        State getGoal() const;

        /**
         * @brief Set the Start object.
         * 
         * @param start 
         */
        void setStart(const State& start);

        /**
         * @brief Set the Goal object.
         * 
         * @param goal 
         */
        void setGoal(const State& goal);

        void updateStart(const State& new_start);

        void setBlocked(const State& state, bool is_blocked);

        /**
         * @brief Get the Path of states.
         * 
         * @return std::vector<State> 
         */
        const std::vector<State>& getPath() const;

        /**
         * @brief set the g cost of a given state.
         * 
         * @param state 
         * @param g 
         */
        void setG(const State& state, int32_t g);

        /**
         * @brief Set the rhs cost of a given state.
         * 
         * @param state 
         * @param rhs 
         */
        void setRhs(const State& state, int32_t rhs);

        /**
         * @brief gets the g cost of a given state.
         * 
         * @param state 
         * @return int32_t
         */
        int32_t getG(const State& state) const;

        /**
         * @brief gets the rhs cost of a given state.
         * 
         * @param state
         * @return int32_t 
         */
        int32_t getRhs(const State& state) const;

        /**
         * @brief Set the Heuristic Type of the planner
         * 
         * @param heuristic_type (0 for manhattan, 1 for octile, 2 for Chebyshev)
         */
        void setHeuristicType(HeuristicType heuristic_type);
        
        /**
         * @brief Get the Heuristic Type of the planner
         * 
         * @return HeuristicType 
         */
        HeuristicType getHeuristicType() const;

        bool replan();
};


#endif