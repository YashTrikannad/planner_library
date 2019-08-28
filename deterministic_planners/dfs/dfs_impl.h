//
// planning_library
// Created by yash on 8/4/19.
//

#pragma once


#include <data_types.h>
#include <iostream>
#include <stack>
#include <unordered_set>


namespace pl::algorithms
{

template<typename MapType, typename PathType, typename NodeType>
template <size_t NumberNeighbors>
void dfs<MapType, PathType, NodeType>::find_path(const node_type &start, const node_type &goal)
{
    if(graph_->template get_node_property(start, common::cell_type{}) == 1 ||
       graph_->template get_node_property(goal, common::cell_type{}) == 1)
    {
        std::__throw_logic_error("Not able to find path. Start and Goal must be in free space. ");
    }

    std::vector<node_type> path;
    std::unordered_set<node_type> closed_set;
    std::unordered_map<node_type, node_type> parent_from_node;
    std::stack<node_type> dfs_stack;
    dfs_stack.push(start);

    while (!dfs_stack.empty())
    {
        const auto current_node = dfs_stack.top();
        dfs_stack.pop();

        // If goal is reached construct path back to the start
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

        graph_->template for_each_adjacent_node<NumberNeighbors>(current_node, [&](const node_type& neighboring_node){
            if(neighboring_node.value_ == 0 && closed_set.find(neighboring_node) == closed_set.end())
            {
                parent_from_node.insert(std::pair<node_type, node_type>(neighboring_node, current_node));
                dfs_stack.push(neighboring_node);
            }
        });

        closed_set.insert(current_node);
    }

}

}

