//
// pathfinding_library
// Created by yash on 8/4/19.
//

#pragma once

namespace pfl::algorithms
{

template<typename MapType, typename PathType, typename NodeType>
class solver
{
public:
    using node_type = NodeType;

    /// set the map
    void update_map(const MapType &map) = delete;

    /// find the path from start to goal
    void find_path(const node_type &start, const node_type &goal) = delete;

    /// get path
    PathType get_path() const = delete;
};

} // namespace pfl::algorithms
