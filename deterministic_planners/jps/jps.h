//
// Created by yash on 8/31/19.
//

#pragma once

#define BOOST_LOG_DYN_LINK 1

#include "../algorithms.h"
#include "data_types.h"

#include <boost/log/trivial.hpp>
#include <optional>
#include <unordered_set>

namespace pl::algorithms
{

template <typename NodeType>
struct jps_node
{
    NodeType node_;
    common::search_direction direction_;
    bool is_forced_;
};

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
    node_type goal_;
    GraphType *graph_;
    std::optional<PathType> path_;
    std::optional<std::unordered_set<node_type>> closed_set_;


    /// Get search direction from current node to the next node
    /// @param current_node
    /// @param next_node
    /// @return
    common::search_direction get_direction(const node_type& current_node, const node_type& next_node) const;


    /// check if a given node is forced
    /// @param current_node
    /// @param direction
    /// @return
    bool is_forced(const node_type& current_node, common::search_direction direction) const;

    ///
    /// @param current_node
    /// @param direction
    /// @return
    std::optional<std::vector<node_type>> get_pruned_neighbors(const node_type& current_node, common::search_direction direction) const;

    /// Jump nodes acccording to symmetry in the grid
    /// @param node
    /// @param direction
    std::optional<NodeType> jump(const node_type& node, common::search_direction direction) const;


    /// Return successor nodes
    /// @param node
    /// @return
    std::vector<NodeType> identify_successors(const node_type& node, common::search_direction direction) const;


    /// Applies the given func to all pruned_neighbors
    /// @tparam Func
    /// @param current_node
    /// @param func
    template <typename Func>
    void for_each_pruned_neighbor(const node_type& current_node, common::search_direction direction, Func&& func) const;

};

} // namespace pl::algorithms

#include "jps_impl.h"