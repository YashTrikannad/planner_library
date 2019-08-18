//
// Created by yash on 8/6/19.
//

#pragma once

#include "EigenGraph.h"
#include "../pfl_graph.h"
#include "../../common/data_types.h"

#include <vector>

namespace pfl::graph
{

template <typename Graph>
template<size_t N, typename Func>
void eigen_graph<Graph>::for_each_adjacent_node(const node_type &node, Func &&func) const
{
    if constexpr (N == 4)
    {
        const auto neighbors = get_4_neighbor(node);
        for(const auto& neighbor: neighbors)
        {
            func(neighbor);
        }
    }
    else
    {
        static_assert(" Currenly the library only supports 4 neighbor operations. ");
    }
}


template<typename Graph>
template <typename Direction>
typename eigen_graph<Graph>::node_type eigen_graph<Graph>::get_adjacent_node(const node_type& node, Direction direction) const
{
    return {node.row_index_ + Direction::change_rows, node.column_index_ + Direction::change_cols, static_cast<size_t>(
            graph_(node.row_index_ + Direction::change_rows, node.column_index_ + Direction::change_cols))};
}


template<typename Graph>
template <typename Direction>
std::optional<typename eigen_graph<Graph>::node_type> eigen_graph<Graph>::get_adjacent_node_with_check(const node_type& node, Direction direction) const
{
    if constexpr (std::is_same<Direction, common::up>{})
    {
        if(node.row_index_ != 0) return get_adjacent_node(node, direction);
        return std::nullopt;
    }
    else if (std::is_same<Direction, common::down>{})
    {
        if(node.row_index_ != rows_-1) return get_adjacent_node(node, direction);
        return std::nullopt;
    }
    else if (std::is_same<Direction, common::left>{})
    {
        if(node.column_index_ != 0) return get_adjacent_node(node, direction);
        return std::nullopt;
    }
    else if (std::is_same<Direction, common::right>{})
    {
        if(node.column_index_ != cols_-1) return get_adjacent_node(node, direction);
        return std::nullopt;
    }
    else
    {
        static_assert("Direction should be either up, down, left or right");
    }
}


template <typename Graph>
auto eigen_graph<Graph>::get_4_neighbor(const node_type& node) const
{
    if(node.row_index_ != 0 && node.row_index_ != rows_-1 && node.column_index_ != 0 && node.column_index_!= cols_-1)
    {
        return  std::vector<pfl::common::NodeIndex2d>{get_adjacent_node(node, common::up{}),
                                                      get_adjacent_node(node, common::down{}),
                                                      get_adjacent_node(node, common::left{}),
                                                      get_adjacent_node(node, common::right{})};

    }
    else
    {
        std::vector<pfl::common::NodeIndex2d> neighbors{};
        if(const auto node_maybe = get_adjacent_node_with_check(node, common::up{}); node_maybe) neighbors.emplace_back(*node_maybe);
        if(const auto node_maybe = get_adjacent_node_with_check(node, common::down{}); node_maybe) neighbors.emplace_back(*node_maybe);
        if(const auto node_maybe = get_adjacent_node_with_check(node, common::left{}); node_maybe) neighbors.emplace_back(*node_maybe);
        if(const auto node_maybe = get_adjacent_node_with_check(node, common::right{}); node_maybe) neighbors.emplace_back(*node_maybe);
        return neighbors;
    }
}

}
