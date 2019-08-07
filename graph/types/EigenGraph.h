//
// Created by yash on 8/6/19.
//

#pragma once

#include "../pfl_graph.h"


namespace pfl::graph
{


template<typename Graph>
class eigen_graph
{
public:
    using graph_type = Graph;
    using node_type = typename Graph::Scalar;


    eigen_graph(const graph_type &graph): graph_(graph)
    {
    }


    /// Func is applied to each adjacent node of the current node
    /// @tparam Func
    template<typename Func>
    void for_each_adjacent_node(const node_type &node, Func &&func) const = delete;


    /// Func is applied to all the nodes in the graph
    /// @tparam Func
    template<typename Func>
    void for_each_node() const = delete;


private:
    graph_type graph_;
};


}