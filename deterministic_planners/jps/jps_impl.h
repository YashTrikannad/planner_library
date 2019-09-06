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
std::vector<NodeType> jps<MapType, PathType, NodeType>::identify_successors(const NodeType& node, common::search_direction direction) const
{
    std::vector<NodeType> successors;
    for_each_pruned_neighbor(node, direction, [&](const node_type& pruned_node){
        if(const auto successor = jump(node, get_direction(node, pruned_node)); successor) successors.push_back(successor);
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

} // namespace pl::algorithms


