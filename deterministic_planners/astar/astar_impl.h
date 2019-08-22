//
// planning_library
// Created by yash on 8/4/19.
//

#pragma once


#include <data_types.h>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace pl::algorithms
{

template<typename MapType, typename PathType, typename NodeType>
template <size_t NumberNeighbors>
void astar<MapType, PathType, NodeType>::find_path(const node_type &start, const node_type &goal)
{
    if(graph_->template get_node_property(start, common::cell_type{}) == 1 ||
       graph_->template get_node_property(goal, common::cell_type{}) == 1)
    {
        std::__throw_logic_error("Not able to find path. Start and Goal must be in free space. ");
    }

    // Initialize Open List - Contains all nodes that are currently on the frontier/ that need to be  explore
    // Initialize Closed List - Contains all explored nodes

    // Iterate till the Open List is empty

        // Choose the current node as the node with the least F cost

        // Check if the current node is the goal
            // If goal is reached, backtrack to the start and return the backtracked vector

        // Else iterate through all children. Initialized with cost = current

            // for each child

                // If child in closed list or non traversable, continue

                // Create the f, g, h values of child

                // Else if child in open list
                    // cost of current child less than the cost of same child in open list
                        // replace the parent as current and cost with current cost

                // Add the child to the open list
}

}

