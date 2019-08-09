//
// planning_library
// Created by yash on 8/4/19.
//

#pragma once


#include <data_types.h>
#include <iostream>
#include <queue>


namespace pfl::algorithms
{

template<typename MapType, typename PathType, typename NodeType>
void bfs<MapType, PathType, NodeType>::find_path(const node_type &start, const node_type &goal)
{
    std::queue<pfl::common::NodeIndex2d> frontier;
    frontier.emplace(start);

    while(!frontier.empty())
    {
        const auto current_node = frontier.front();
        if(current_node == goal)
        {
            return;
        }
        graph_->template for_each_adjacent_node(current_node, [&](const node_type neighboring_node) {
            frontier.emplace(neighboring_node);
        });

        frontier.pop();
    }
}

}
