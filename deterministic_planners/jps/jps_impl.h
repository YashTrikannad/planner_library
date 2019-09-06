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
common::search_direction jps<MapType, PathType, NodeType>::get_direction(const NodeType& current_node, const NodeType& next_node) const
{
    const int dx = next_node.row_index_ - current_node.row_index_;
    const int dy = next_node.column_index_ - current_node.column_index_;
    if (dx == 1)
    {
        switch (dy)
        {
            case 1: return common::search_direction::top_right;
            case 0: return common::search_direction::right;
            case -1: return common::search_direction::bottom_right;
            default: std::__throw_runtime_error("dx and dy value between two neighboring nodes is not correct");
        }
    }
    else if (dx == -1)
    {
        switch (dy)
        {
            case 1: return common::search_direction::top_left;
            case 0: return common::search_direction::left;
            case -1: return common::search_direction::bottom_left;
            default: std::__throw_runtime_error("dx and dy value between two neighboring nodes is not correct");
        }
    }
    else if (dx == 0)
    {
        switch (dy)
        {
            case 1: return common::search_direction::top;
            case -1: return common::search_direction::bottom;
            default: std::__throw_runtime_error("dx and dy value between two neighboring nodes is not correct");
        }
    }
}

template<typename MapType, typename PathType, typename NodeType>
bool jps<MapType, PathType, NodeType>::is_forced(const NodeType& current_node, common::search_direction direction) const
{
    for_each_pruned_neighbor(current_node, direction, [&](const node_type& neighboring_node, common::search_direction direction){
        switch (direction)
        {
            case common::search_direction::top:
                if(const auto node2 = graph_->template get_adjacent_node(current_node, common::right{}); node2 && node2.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::top_right{}); node) return true;
                }
                if(const auto node3 = graph_->template get_adjacent_node(current_node, common::right{}); node3 && node3.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::top_right{}); node) return true;
                }
            case common::search_direction::top_right:
                if(const auto node2 = graph_->template get_adjacent_node(current_node, common::left{}); node2 && node2.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::top_left{}); node) return true;
                }
                if(const auto node3 = graph_->template get_adjacent_node(current_node, common::down{}); node3 && node3.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_right{}); node) return true;
                }
            case common::search_direction::right:
                if(const auto node2 = graph_->template get_adjacent_node(current_node, common::up{}); node2 && node2.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::top_right{}); node) return true;
                }
                if(const auto node3 = graph_->template get_adjacent_node(current_node, common::down{}); node3 && node3.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_right{}); node) return true;
                }
            case common::search_direction::bottom_right:
            {
                if(const auto node2 = graph_->template get_adjacent_node(current_node, common::up{}); node2 && node2.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::top_right{}); node) return true;
                }
                if(const auto node3 = graph_->template get_adjacent_node(current_node, common::left{}); node3 && node3.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_left{}); node) return true;
                }
            }
            case common::search_direction::bottom:
            {
                if(const auto node2 = graph_->template get_adjacent_node(current_node, common::left{}); node2 && node2.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_left{}); node) return true;
                }
                if(const auto node3 = graph_->template get_adjacent_node(current_node, common::right{}); node3 && node3.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_right{}); node) return true;
                }
            }
            case common::search_direction::bottom_left:
            {
                if(const auto node2 = graph_->template get_adjacent_node(current_node, common::up{}); node2 && node2.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::top_left{}); node) return true;
                }
                if(const auto node3 = graph_->template get_adjacent_node(current_node, common::right{}); node3 && node3.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_right{}); node) return true;
                }
            }
            case common::search_direction::left:
            {
                if(const auto node2 = graph_->template get_adjacent_node(current_node, common::up{}); node2 && node2.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::top_left{}); node) return true;
                }
                if(const auto node3 = graph_->template get_adjacent_node(current_node, common::down{}); node3 && node3.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_left{}); node) return true;
                }
            }
            case common::search_direction::top_left:
            {
                if(const auto node2 = graph_->template get_adjacent_node(current_node, common::right{}); node2 && node2.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::top_right{}); node) return true;
                }
                if(const auto node3 = graph_->template get_adjacent_node(current_node, common::down{}); node3 && node3.value_)
                {
                    if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_left{}); node) return true;
                }
            }
        }
    });
    return false;
}

template<typename MapType, typename PathType, typename NodeType>
std::optional<std::vector<NodeType>> jps<MapType, PathType, NodeType>::get_pruned_neighbors(const node_type& current_node, common::search_direction direction) const
{
    std::vector<node_type> pruned_neighbors;
    switch (direction)
    {
        case common::search_direction::top:
        {
            if(const auto node1 = graph_->template get_adjacent_node(current_node, common::up{}); node1 && !node1.value_) pruned_neighbors.emplace_back(node1);
            if(const auto node2 = graph_->template get_adjacent_node(current_node, common::right{}); node2 && node2.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::top_right{}); node) pruned_neighbors.emplace_back(node);
            }
            if(const auto node3 = graph_->template get_adjacent_node(current_node, common::right{}); node3 && node3.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::top_right{}); node) pruned_neighbors.emplace_back(node);
            }
        }
        case common::search_direction::top_right:
        {
            if(const auto node = graph_->template get_adjacent_node(current_node, common::up{}); node && !node.value_) pruned_neighbors.emplace_back(node);
            if(const auto node = graph_->template get_adjacent_node(current_node, common::top_right{}); node && !node.value_) pruned_neighbors.emplace_back(node);
            if(const auto node = graph_->template get_adjacent_node(current_node, common::right{}); node && !node.value_) pruned_neighbors.emplace_back(node);
            if(const auto node2 = graph_->template get_adjacent_node(current_node, common::left{}); node2 && node2.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::top_left{}); node) pruned_neighbors.emplace_back(node);
            }
            if(const auto node3 = graph_->template get_adjacent_node(current_node, common::down{}); node3 && node3.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_right{}); node) pruned_neighbors.emplace_back(node);
            }
        }
        case common::search_direction::right:
        {
            if(const auto node1 = graph_->template get_adjacent_node(current_node, common::right{}); node1 && !node1.value_) pruned_neighbors.emplace_back(node1);
            if(const auto node2 = graph_->template get_adjacent_node(current_node, common::up{}); node2 && node2.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::top_right{}); node) pruned_neighbors.emplace_back(node);
            }
            if(const auto node3 = graph_->template get_adjacent_node(current_node, common::down{}); node3 && node3.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_right{}); node) pruned_neighbors.emplace_back(node);
            }
        }
        case common::search_direction::bottom_right:
        {
            if(const auto node = graph_->template get_adjacent_node(current_node, common::down{}); node && !node.value_) pruned_neighbors.emplace_back(node);
            if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_right{}); node && !node.value_) pruned_neighbors.emplace_back(node);
            if(const auto node = graph_->template get_adjacent_node(current_node, common::right{}); node && !node.value_) pruned_neighbors.emplace_back(node);
            if(const auto node2 = graph_->template get_adjacent_node(current_node, common::up{}); node2 && node2.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::top_right{}); node) pruned_neighbors.emplace_back(node);
            }
            if(const auto node3 = graph_->template get_adjacent_node(current_node, common::left{}); node3 && node3.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_left{}); node) pruned_neighbors.emplace_back(node);
            }
        }
        case common::search_direction::bottom:
        {
            if(const auto node1 = graph_->template get_adjacent_node(current_node, common::down{}); node1 && !node1.value_) pruned_neighbors.emplace_back(node1);
            if(const auto node2 = graph_->template get_adjacent_node(current_node, common::left{}); node2 && node2.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_left{}); node) pruned_neighbors.emplace_back(node);
            }
            if(const auto node3 = graph_->template get_adjacent_node(current_node, common::right{}); node3 && node3.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_right{}); node) pruned_neighbors.emplace_back(node);
            }
        }
        case common::search_direction::bottom_left:
        {
            if(const auto node = graph_->template get_adjacent_node(current_node, common::down{}); node && !node.value_) pruned_neighbors.emplace_back(node);
            if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_left{}); node && !node.value_) pruned_neighbors.emplace_back(node);
            if(const auto node = graph_->template get_adjacent_node(current_node, common::left{}); node && !node.value_) pruned_neighbors.emplace_back(node);
            if(const auto node2 = graph_->template get_adjacent_node(current_node, common::up{}); node2 && node2.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::top_left{}); node) pruned_neighbors.emplace_back(node);
            }
            if(const auto node3 = graph_->template get_adjacent_node(current_node, common::right{}); node3 && node3.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_right{}); node) pruned_neighbors.emplace_back(node);
            }
        }
        case common::search_direction::left:
        {
            if(const auto node1 = graph_->template get_adjacent_node(current_node, common::left{}); node1 && !node1.value_) pruned_neighbors.emplace_back(node1);
            if(const auto node2 = graph_->template get_adjacent_node(current_node, common::up{}); node2 && node2.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::top_left{}); node) pruned_neighbors.emplace_back(node);
            }
            if(const auto node3 = graph_->template get_adjacent_node(current_node, common::down{}); node3 && node3.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_left{}); node) pruned_neighbors.emplace_back(node);
            }
        }
        case common::search_direction::top_left:
        {
            if(const auto node = graph_->template get_adjacent_node(current_node, common::up{}); node && !node.value_) pruned_neighbors.emplace_back(node);
            if(const auto node = graph_->template get_adjacent_node(current_node, common::top_left{}); node && !node.value_) pruned_neighbors.emplace_back(node);
            if(const auto node = graph_->template get_adjacent_node(current_node, common::left{}); node && !node.value_) pruned_neighbors.emplace_back(node);
            if(const auto node2 = graph_->template get_adjacent_node(current_node, common::right{}); node2 && node2.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::top_right{}); node) pruned_neighbors.emplace_back(node);
            }
            if(const auto node3 = graph_->template get_adjacent_node(current_node, common::down{}); node3 && node3.value_)
            {
                if(const auto node = graph_->template get_adjacent_node(current_node, common::bottom_left{}); node) pruned_neighbors.emplace_back(node);
            }
        }
    }
    return pruned_neighbors;
}

template<typename MapType, typename PathType, typename NodeType>
std::optional<NodeType> jps<MapType, PathType, NodeType>::jump(const NodeType& node, common::search_direction direction) const
{
    const auto next_node = graph_->template get_adjacent_node(node, direction);

    if(!next_node || next_node.value_ == 1) return std::nullopt;
    if(next_node == goal_) return next_node;
    if(is_forced(next_node, direction)) return next_node;
    if(direction == common::search_direction::bottom_right)
    {
        jump(next_node, common::search_direction::bottom);
        jump(next_node, common::search_direction::right);
    }
    else if(direction == common::search_direction::bottom_left)
    {
        jump(next_node, common::search_direction::bottom);
        jump(next_node, common::search_direction::left);
    }
    else if(direction == common::search_direction::top_right)
    {
        jump(next_node, common::search_direction::top);
        jump(next_node, common::search_direction::right);
    }
    else if(direction == common::search_direction::top_left)
    {
        jump(next_node, common::search_direction::top);
        jump(next_node, common::search_direction::left);
    }
    return jump(next_node, direction);
}


template<typename MapType, typename PathType, typename NodeType>
void jps<MapType, PathType, NodeType>::identify_successors(const NodeType& node, common::search_direction direction)
{
    std::vector<NodeType> successors;
    for_each_pruned_neighbor(node, direction, [&](const node_type& pruned_node){
        if(const auto successor = jump(node, get_direction(node, pruned_node)); successor)
        {
            // if not present in the closed list
            if(closed_set_.find(pruned_node) == closed_set_.end())
            {
                const auto heuristic = pl::common::get_distance(pruned_node, goal_);

                const auto added_cost = pl::common::get_distance(pruned_node, node);

                // get current node g cost
                const auto current_node_g_cost = graph_->template get_node_property(node, common::g_cost_tag{});

                const auto new_pruned_node_g_cost = current_node_g_cost + added_cost;

                // add condition
                const auto old_pruned_node_g_cost = graph_->template get_node_property(pruned_node, common::g_cost_tag{});

                // get new_g
                if(open_set_.find(pruned_node) == open_set_.end() || new_pruned_node_g_cost < old_pruned_node_g_cost)
                {
                    // update g and h of the jump node
                    graph_->template update_node_property(pruned_node, common::g_cost_tag{}, new_pruned_node_g_cost);
                    graph_->template update_node_property(pruned_node, common::f_cost_tag{}, new_pruned_node_g_cost + heuristic);

                    parent_from_node_[pruned_node] = {node, direction};

                    // if not open_list, add to open_list
                    if(open_set_.find(pruned_node) == open_set_.end())
                    {
                        open_set_.insert(pruned_node);
                    }
                    // else heapify
                }
            }
        }
    });
    return successors;
}


template<typename MapType, typename PathType, typename NodeType>
template <typename Func>
void jps<MapType, PathType, NodeType>::for_each_pruned_neighbor(
        const node_type& current_node, common::search_direction direction, Func&& func) const
{
    const auto pruned_neighbors = get_pruned_neighbors(current_node, direction);
    if(!pruned_neighbors) return;
    graph_->template for_each_adjacent_node<8>(current_node, [&](const node_type& neighboring_node) {
         func(neighboring_node, direction);
    });
}


template<typename MapType, typename PathType, typename NodeType>
template<size_t NumberNeighbors>
void jps<MapType, PathType, NodeType>::find_path(const node_type &start, const node_type &goal)
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

    // Initialize Closed List - Contains all explored nodes
    std::unordered_set<node_type> closed_set;

    // Add the start to the open list
    open_list.push(start);
    open_set_.insert(start);

    // Iterate till the Open List is empty
    while(!open_list.empty())
    {
        // Choose the current node as the node with the least F cost
        const auto current_node = open_list.top();

        // Remove the current node from the open list
        open_list.pop();
        open_set_.erase(current_node);

        // Add the current node to the closed set
        closed_set.insert(current_node);

        // Check if the current node is the goal
        if (current_node == goal)
        {
            // If goal is reached, backtrack to the start and return the backtracked vector
            auto path_node = goal;
            while (path_node != start)
            {
                path.emplace_back(path_node);
                path_node = parent_from_node_[path_node];
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
                // Identify successors and update the open_list
                identify_successors(neighboring_node, get_direction(current_node, neighboring_node));
            }
        });
    }
}


} // namespace pl::algorithms


