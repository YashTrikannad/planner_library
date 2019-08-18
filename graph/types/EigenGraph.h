//
// Created by yash on 8/6/19.
//

#pragma once

#include <vector>
#include "../pfl_graph.h"
#include "../../common/data_types.h"


namespace pfl::graph
{


template<typename Graph>
class eigen_graph
{
public:
    using graph_type = Graph;
    using data_type = typename Graph::Scalar;
    using node_type = pfl::common::NodeIndex2d;

    eigen_graph(const graph_type &graph): graph_(graph), rows_(graph.rows()), cols_(graph.cols())
    {
    }


    /// Func is applied to each adjacent node of the current node
    /// @tparam Func
    template<size_t N, typename Func>
    void for_each_adjacent_node(const node_type &node, Func &&func) const;


    template<typename Tag>
    typename Tag::type get_node_property(const node_type &node, Tag&& tag) const
    {
        if constexpr (std::is_same<Tag, common::cell_type>{}) return graph_(node.row_index_, node.column_index_);
        static_assert(" The given property is not supported by this graph currently ");
    }

    /// Func is applied to all the nodes in the graph
    /// @tparam Func
    template<typename Func>
    void for_each_node(Func &&func) const
    {
        for(size_t col=0; col < cols_; col++)
        {
            for(size_t row=0; row < rows_; row++)
            {
                func(pfl::common::NodeIndex2d{row, col, graph_(cols_, rows_)});
            }
        }
    }


private:
    graph_type graph_;
    size_t rows_;
    size_t cols_;

    ///
    /// @param node - current node
    /// @return get 4 neighbors
    auto get_4_neighbor(const node_type& node) const;

    ///
    /// @param node
    /// @return get 8 neighbors
    auto get_8_neighbor(const node_type& node) const;


    ///
    /// @tparam Graph
    /// @tparam Direction
    /// @param node _ Non Bordering nodes ( Doesn't perform boundary checks)
    /// @param direction
    /// @return adjacent nodes corresponding to the direction
    template <typename Direction>
    node_type get_adjacent_node(const node_type& node, Direction direction) const;


    ///
    /// @tparam Graph
    /// @tparam Direction
    /// @param node - node on the graph boundary
    /// @param direction - direction of adjacent node
    /// @return adjacent node if present else null_opt
    template <typename Direction>
    std::optional<node_type> get_adjacent_node_with_check(const node_type& node, Direction direction) const;
};

}

#include "EigenGraph_impl.h"