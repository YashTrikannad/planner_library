//
// Created by yash on 8/4/19.
//

#pragma once

#include <optional>
#include "../algorithms.h"


namespace pl::algorithms
{

template<typename GraphType, typename PathType, typename NodeType>
class dijkstra : public solver<GraphType, PathType, NodeType>
{
public:
    using node_type = NodeType;

    explicit dijkstra(const GraphType *graph) : graph_(graph)
    {
    }

    void update_map(const GraphType &map)
    {
        graph_ = &map;
    }

    template <size_t NumberNeighbors = 8>
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

#include "dijkstra_impl.h"
