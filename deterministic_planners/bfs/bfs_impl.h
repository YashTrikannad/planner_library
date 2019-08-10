//
// planning_library
// Created by yash on 8/4/19.
//

#pragma once


#include <data_types.h>
#include <iostream>
#include <queue>

#include <unordered_set>

namespace pfl::algorithms
{

template<typename MapType, typename PathType, typename NodeType>
void bfs<MapType, PathType, NodeType>::find_path(const node_type &start, const node_type &goal)
{
    std::unordered_set<node_type> open_list;
    std::unordered_set<node_type> closed_list;

    open_list.insert(start);

    while(!open_list.empty())
    {
        const auto current_node = open_list.front();
        if(current_node == goal)
        {
            return;
        }
        graph_->template for_each_adjacent_node(current_node, [&](const node_type neighboring_node) {
            if(closed_list.find(neighboring_node) == closed_list.end())
            {
                open_list.insert(neighboring_node);
            }
        });

        closed_list.insert(open_list.top());
        open_list.pop();
    }
}

}
