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
template<typename Func>
void eigen_graph<Graph>::for_each_adjacent_node(const node_type &node, Func &&func) const
{
    {
        std::array<pfl::common::NodeIndex2d, 4> neighbors{};
        if(node.row_index_ != 0 && node.row_index_ != rows_ && node.column_index_ != 0 && node.column_index_!= cols_)
        {
            neighbors[0] =  {node.row_index_ -1, node.column_index_ , graph_(node.row_index_ -1, node.column_index_)};
            neighbors[1] =  {node.row_index_, node.column_index_ + 1, graph_(node.row_index_, node.column_index_ + 1)};
            neighbors[2] =  {node.row_index_ + 1, node.column_index_, graph_(node.row_index_ + 1, node.column_index_)};
            neighbors[3] =  {node.row_index_ , node.column_index_ - 1, graph_(node.row_index_ , node.column_index_ - 1)};

            for (const auto &neighbor: neighbors)
            {
                func(neighbor);
            }

            return;
        }

        std::size_t index = 0;
        if(node.row_index_ != 0)
        {
            neighbors[index++]  = {node.row_index_ - 1, node.column_index_, graph_(node.row_index_ - 1, node.column_index_)};
        }
        if(node.column_index_ != cols_)
        {
            neighbors[index++]  = {node.row_index_, node.column_index_+1, graph_(node.row_index_, node.column_index_+1)};
        }
        if(node.row_index_ != rows_)
        {
            neighbors[index++]  = {node.row_index_ + 1, node.column_index_, graph_(node.row_index_ + 1, node.column_index_)};
        }
        if(node.column_index_ != 0)
        {
            neighbors[index++]  = {node.row_index_, node.column_index_-1, graph_(node.row_index_, node.column_index_-1)};
        }
        for(size_t iterator = 0; iterator<index; iterator++)
        {
            func(neighbors[iterator]);
        }
    }
}


}
