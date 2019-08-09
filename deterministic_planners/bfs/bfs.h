//
// Created by yash on 8/4/19.
//

#pragma once

#include "Eigen/Core"
#include "../algorithms.h"


namespace pfl::algorithms
{

template<typename GraphType, typename PathType, typename NodeType>
class bfs : public solver<GraphType, PathType, NodeType>
{
public:
    using node_type = NodeType;

    explicit bfs(const GraphType *graph) : graph_(graph)
    {
    }

    void update_map(const GraphType &map)
    {
        graph_ = &map;
    }

    void find_path(const node_type &start, const node_type &goal);


    std::optional<PathType> get_path() const
    {
        return path_;
    }

private:
    const GraphType *graph_;
    std::optional<PathType> path_;
};

}

#include "bfs_impl.h"