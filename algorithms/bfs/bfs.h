//
// Created by yash on 8/4/19.
//

#pragma once

#include "Eigen/Core"
#include "../algorithms.h"


namespace pfl::algorithms
{

template<typename MapType, typename PathType, typename NodeType>
class bfs : public solver<MapType, PathType, NodeType>
{
public:
    using node_type = NodeType;

    explicit bfs(const MapType *map) : map_(map)
    {
    }

    void update_map(const MapType &map)
    {
        map_ = &map;
    }

    void find_path(const node_type &start, const node_type &goal)
    {
        path_ = {1, 2, 3};
    }

    std::optional<PathType> get_path() const
    {
        return path_;
    }

private:
    const MapType *map_;
    std::optional<PathType> path_;
};

}