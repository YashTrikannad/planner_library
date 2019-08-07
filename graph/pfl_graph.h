//
// pathfinding library
// Created by yash on 8/6/19.
//

#pragma once

#include <cstddef>


namespace pfl::graph
{


/// pfl_graph will be the standard graph used in this library.
/// @tparam Graph
template<typename Graph>
class pfl_graph
{
public:
    using graph_type = Graph;
    using node_type = typename Graph::node_type;


    pfl_graph(const graph_type &graph) : graph_(graph)
    {}


    /// Func is applied to each adjacent node of the current node
    /// @tparam Func
    template<typename Func>
    void for_each_adjacent_node(const node_type &node, Func &&func) const = delete;


    /// Func is applied to all the nodes in the graph
    /// @tparam Func
    template<typename Func>
    void for_each_node() const = delete;


    /// Get the node count in the graph
    /// @return
    size_t get_node_count() const
    {
        std::size_t count = 0;
        graph_.template for_each_node([&]() {
            count++;
        });
        return count;
    }


private:
    graph_type graph_;
};


}