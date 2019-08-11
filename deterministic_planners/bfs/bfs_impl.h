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
    std::queue<node_type> open_list;
    std::unordered_set<node_type> closed_list;
    std::unordered_map<node_type, node_type> parent_from_node;
    std::vector<node_type> path;

    open_list.push(start);

    while(!open_list.empty())
    {
        const auto current_node = open_list.front();
        if(current_node == goal)
        {
            auto path_node = goal;
            while(path_node != start)
            {
                path.emplace_back(path_node);
                path_node = parent_from_node[path_node];
            }
            path.emplace_back(start);
            path_ = std::move(path);
            return;
        }
        graph_->template for_each_adjacent_node(current_node, [&](const node_type& neighboring_node) {
            if(neighboring_node.obstacle_ == 0)
            {
                if (closed_list.find(neighboring_node) == closed_list.end())
                {
                    parent_from_node.insert(std::pair<node_type, node_type>(neighboring_node, current_node));
                    open_list.push(neighboring_node);
                }
            }
        });

        closed_list.insert(open_list.front());
        open_list.pop();
    }
}

}
