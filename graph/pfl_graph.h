//
// pathfinding library
// Created by yash on 8/6/19.
//

#pragma once

#include <cstddef>


namespace pl::graph
{


/// pfl_graph will be the standard graph used in this library.
/// @tparam Graph
template<typename Graph>
class pfl_graph
{
public:
    using graph_type = Graph;
    using node_type = typename Graph::node_type;


    /// Func is applied to each adjacent node of the current node
    /// @tparam Func
    template<typename Func>
    void for_each_adjacent_node(const node_type &node, Func &&func) const = delete;


    /// Func is applied to all the nodes in the graph
    /// @tparam Func
    template<typename Func>
    void for_each_node() const = delete;


    /// @tparam Tag - Property tag required by the user
    /// @param node - queried node
    /// @param tag - property tag for dispatching
    /// @return value of the property inquired for the node
    template<typename Tag>
    typename Tag::type get_node_property(const node_type &node, Tag&& tag) const = delete;
};


}