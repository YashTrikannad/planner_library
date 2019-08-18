//
// Created by yash on 8/4/19.
//

#pragma once

#include "../algorithms.h"
#include <optional>

namespace pl::algorithms
{

template<typename GraphType, typename PathType, typename NodeType>
class dfs : public solver<GraphType, PathType, NodeType>
{
public:
    using node_type = NodeType;

    explicit dfs(const GraphType *graph) : graph_(graph)
    {
    }

    void update_map(const GraphType &map)
    {
        graph_ = &map;
    }

    template <std::size_t NumberNeighbors = 8>
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

#include "dfs_impl.h"