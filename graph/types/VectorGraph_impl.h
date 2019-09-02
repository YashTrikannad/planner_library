//
// Created by yash on 8/6/19.
//

#pragma once

#include "VectorGraph.h"
#include "../pfl_graph.h"
#include "../../common/data_types.h"

#include <vector>

namespace pl::graph
{

template <typename DataType>
template<size_t N, typename Func>
void graph<std::vector<std::vector<DataType>>, DataType>::for_each_adjacent_node(const node_type &node, Func &&func) const
{
    if constexpr (N == 4)
    {
        const auto neighbors = get_4_neighbor(node);
        for(const auto& neighbor: neighbors)
        {
            func(neighbor);
        }
    }
    else if constexpr (N == 8)
    {
        const auto neighbors = get_8_neighbor(node);
        for(const auto& neighbor: neighbors)
        {
            func(neighbor);
        }
    }
    else
    {
        static_assert(N == 4 || N == 8, " Currenly this graph only supports 4/8 neighbor operations. ");
    }
}


template<typename DataType>
template <typename Direction>
typename graph<std::vector<std::vector<DataType>>, DataType>::node_type graph<std::vector<std::vector<DataType>>, DataType>::get_adjacent_node(const node_type& node, Direction direction) const
{
    return {node.row_index_ + Direction::change_rows, node.column_index_ + Direction::change_cols, static_cast<size_t>(
            container_[node.row_index_ + Direction::change_rows][node.column_index_ + Direction::change_cols])};
}


template<typename DataType>
template <typename Direction>
std::optional<typename graph<std::vector<std::vector<DataType>>, DataType>::node_type> graph<std::vector<std::vector<DataType>>, DataType>::get_adjacent_node_with_check(const node_type& node, Direction direction) const
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
    if constexpr (std::is_same<Direction, common::top_left>{})
    {
        if(node.row_index_ != 0 && node.column_index_ !=0) return get_adjacent_node(node, direction);
        return std::nullopt;
    }
    else if (std::is_same<Direction, common::top_right>{})
    {
        if(node.row_index_ != 0 && node.column_index_ != cols_-1) return get_adjacent_node(node, direction);
        return std::nullopt;
    }
    else if (std::is_same<Direction, common::bottom_left>{})
    {
        if(node.row_index_!= rows_-1 && node.column_index_ != 0) return get_adjacent_node(node, direction);
        return std::nullopt;
    }
    else if (std::is_same<Direction, common::bottom_right>{})
    {
        if(node.row_index_!= rows_-1 && node.column_index_ != cols_-1) return get_adjacent_node(node, direction);
        return std::nullopt;
    }
    else
    {
        static_assert("Direction should be either up, down, left or right");
    }
}


template <typename DataType>
auto graph<std::vector<std::vector<DataType>>, DataType>::get_4_neighbor(const node_type& node) const
{
    if(node.row_index_ != 0 && node.row_index_ != rows_-1 && node.column_index_ != 0 && node.column_index_!= cols_-1)
    {
        return  std::vector<pl::common::NodeIndex2d>{get_adjacent_node(node, common::up{}),
                                                     get_adjacent_node(node, common::down{}),
                                                     get_adjacent_node(node, common::left{}),
                                                     get_adjacent_node(node, common::right{})};

    }
    else
    {
        std::vector<pl::common::NodeIndex2d> neighbors{};
        if(const auto node_maybe = get_adjacent_node_with_check(node, common::up{}); node_maybe) neighbors.emplace_back(*node_maybe);
        if(const auto node_maybe = get_adjacent_node_with_check(node, common::down{}); node_maybe) neighbors.emplace_back(*node_maybe);
        if(const auto node_maybe = get_adjacent_node_with_check(node, common::left{}); node_maybe) neighbors.emplace_back(*node_maybe);
        if(const auto node_maybe = get_adjacent_node_with_check(node, common::right{}); node_maybe) neighbors.emplace_back(*node_maybe);
        return neighbors;
    }
}



template <typename DataType>
auto graph<std::vector<std::vector<DataType>>, DataType>::get_8_neighbor(const node_type& node) const
{
    std::vector<pl::common::NodeIndex2d> neighbors = get_4_neighbor(node);
    if(node.row_index_ != 0 && node.row_index_ != rows_-1 && node.column_index_ != 0 && node.column_index_!= cols_-1)
    {
        neighbors.insert(neighbors.end(), {get_adjacent_node(node, common::top_right{}),
                                           get_adjacent_node(node, common::top_left{}),
                                           get_adjacent_node(node, common::bottom_right{}),
                                           get_adjacent_node(node, common::bottom_left{})});
        return neighbors;
    }
    else
    {
        if(const auto node_maybe = get_adjacent_node_with_check(node, common::top_right{}); node_maybe) neighbors.emplace_back(*node_maybe);
        if(const auto node_maybe = get_adjacent_node_with_check(node, common::top_left{}); node_maybe) neighbors.emplace_back(*node_maybe);
        if(const auto node_maybe = get_adjacent_node_with_check(node, common::bottom_right{}); node_maybe) neighbors.emplace_back(*node_maybe);
        if(const auto node_maybe = get_adjacent_node_with_check(node, common::bottom_left{}); node_maybe) neighbors.emplace_back(*node_maybe);
        return neighbors;
    }
}

}