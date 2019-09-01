//
// planning_library
// Created by yash on 8/4/19.
//

#pragma once

#define BOOST_LOG_DYN_LINK 1

#include "visualization_utility.h"
#include "data_types.h"
#include "utility.h"

#include <boost/log/trivial.hpp>
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
        BOOST_LOG_TRIVIAL(warning) << "Not able to find path. Start and Goal must be in free space!";
        return;
    }

    std::vector<node_type> path;

    auto lesser_cost = [&](node_type left_node, node_type right_node) {
        return (graph_->template get_node_property(left_node, common::f_cost_tag{})) >
        (graph_->template get_node_property(right_node, common::f_cost_tag{}));
    };

    // Initialize Open List - Contains all nodes that are currently on the frontier/ that need to be  explore
    std::priority_queue<node_type, std::vector<node_type>, decltype(lesser_cost)> open_list(lesser_cost);

    // Initialize Open List set that contains all nodes on the frontier
    std::unordered_set<node_type> open_list_set;

    // Initialize Closed List - Contains all explored nodes
    std::unordered_set<node_type> closed_set;

    // Node to Parent Map
    std::unordered_map<node_type , node_type> parent_from_node;

    // Add the start to the open list
    open_list.push(start);
    open_list_set.insert(start);

    // Iterate till the Open List is empty
    while(!open_list.empty())
    {
        // Choose the current node as the node with the least F cost
        const auto current_node = open_list.top();

        // Remove the current node from the open list
        open_list.pop();
        open_list_set.erase(current_node);

        // Add the current node to the closed set
        closed_set.insert(current_node);

        // Check if the current node is the goal
        if(current_node == goal)
        {
            // If goal is reached, backtrack to the start and return the backtracked vector
            auto path_node = goal;
            while(path_node != start)
            {
                path.emplace_back(path_node);
                path_node = parent_from_node[path_node];
            }
            path.emplace_back(start);
            path_ = std::move(path);

            if constexpr (debug::debug)
            {
                closed_set_ = std::move(closed_set);
            }

            BOOST_LOG_TRIVIAL(info) << "Path Found!";
            return;
        }

        const auto current_node_g_cost = graph_->template get_node_property(current_node, common::g_cost_tag{});

        // Else iterate through all children. Initialized with cost = current
        graph_->template for_each_adjacent_node<NumberNeighbors>(current_node, [&](const node_type& neighboring_node) {

            // If child in closed list or non traversable, continue
            if(neighboring_node.value_ == 0 && closed_set.find(neighboring_node) == closed_set.end())
            {
                const auto node_transition_cost = pl::common::get_distance(current_node, neighboring_node);
                // If child not in open list
                if(open_list_set.find(neighboring_node) == open_list_set.end())
                {
                    // Add cost as current_node + 1
                    graph_->template update_node_property(neighboring_node, common::g_cost_tag{},
                            current_node_g_cost + node_transition_cost);

                    // compute heuristic value
                    const auto heuristic_value = pl::common::get_distance(neighboring_node, goal);

                    graph_->template update_node_property(neighboring_node, common::f_cost_tag{}, current_node_g_cost +
                            heuristic_value);

                    // Add the child to the open list
                    parent_from_node[neighboring_node] = current_node;
                    open_list_set.insert(neighboring_node);
                    open_list.push(neighboring_node);
                }
                // Else if child in open list
                else
                {
                    // cost of current child less than the cost of same child in open list
                    if(graph_->template get_node_property(neighboring_node, common::g_cost_tag{}) >
                            current_node_g_cost)
                    {
                        graph_->template update_node_property(neighboring_node, common::g_cost_tag{},
                                                              current_node_g_cost + node_transition_cost);

                        // replace the parent as current and cost with current cost
                        parent_from_node[neighboring_node] = current_node;
                    }
                }
            }
        });
    }

    BOOST_LOG_TRIVIAL(warning) << "Not able to find path from start to goal!";
}

} // namespace pl::algorithms

