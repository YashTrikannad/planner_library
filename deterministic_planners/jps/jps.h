//
// Created by yash on 8/31/19.
//

#pragma once

#define BOOST_LOG_DYN_LINK 1

#include "../algorithms.h"

#include <boost/log/trivial.hpp>
#include <optional>
#include <unordered_set>
#include <data_types.h>

namespace pl::algorithms
{

namespace debug
{
    constexpr bool debug = true;
} // namespace debug

template<typename GraphType, typename PathType, typename NodeType>
class jps : public solver<GraphType, PathType, NodeType>
{
public:
    using node_type = NodeType;

    explicit jps(GraphType *graph) : graph_(graph)
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

    constexpr std::unordered_set<node_type> get_closed_set() const
    {
        if (closed_set_)
        {
            return *closed_set_;
        }
        std::__throw_logic_error("Can only access this if debug option is set as true");
    }

private:
    GraphType *graph_;
    std::optional<PathType> path_;
    std::optional<std::unordered_set<node_type>> closed_set_;

    /// Jump and add nodes to the open set
    /// @param current_node
    /// @param next_node
    template <typename Direction>
    void jump(const node_type& current_node, const node_type& next_node, Direction direction);

    /// Check if the given node has forced neighbors
    /// @param current_node
    /// @return return true if current node has forced neighbors
    template <typename Direction>
    bool check_for_forced_neighbors(const node_type& current_node, Direction direction) const;
};

} // namespace pl::algorithms

#include "jps_impl.h"