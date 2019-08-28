//
// Created by yash on 8/6/19.
//

#pragma once

#include <vector>
#include "../pfl_graph.h"
#include "../../common/data_types.h"


namespace pl::graph
{


/// Base Class with graph type as Eigen Matrix
template<typename Graph>
class eigen_graph
{
public:
    using graph_type = Graph;
    using data_type = typename Graph::Scalar;
    using node_type = pl::common::NodeIndex2d;

    eigen_graph(const graph_type &graph): graph_(graph), rows_(graph.rows()), cols_(graph.cols())
    {
    }

    /// Func is applied to each adjacent node of the current node
    /// @tparam Func
    template<size_t N, typename Func>
    void for_each_adjacent_node(const node_type &node, Func &&func) const;


    template<typename Tag, std::enable_if_t<std::is_same<Tag, pl::common::cell_type>::value, int> = 0>
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
                func(pl::common::NodeIndex2d{row, col, graph_(cols_, rows_)});
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


/// Wrapper Class with added layer of costs required by heuristic algorithms
template <template<class> class Wrapper, typename Graph >
class eigen_cost_graph : public Wrapper<Graph>
{
public:
    using graph_type = Graph;
    using node_type = pl::common::NodeIndex2d;

    eigen_cost_graph(const graph_type& graph) : Wrapper<Graph>(graph),
            cost_graph_(std::vector<std::vector<common::cost_type> >(graph.rows(),
            std::vector<common::cost_type>(graph.cols())))
    {}

    template<typename Tag, std::enable_if_t<std::is_same<Tag, pl::common::f_cost_tag>::value, int> = 0>
    typename Tag::type get_node_property(const node_type &node, Tag&& tag) const
    {
        return cost_graph_[node.row_index_][node.column_index_].f_;
    }

    template<typename Tag, std::enable_if_t<std::is_same<Tag, pl::common::g_cost_tag>::value, int> = 0>
    typename Tag::type get_node_property(const node_type &node, Tag&& tag) const
    {
        return cost_graph_[node.row_index_][node.column_index_].g_;
    }

    ///
    /// @tparam Tag - Type of Tag
    /// @param node
    /// @param tag
    /// @param value
    /// @return update the property value of graph (Cost)
    template<typename Tag, std::enable_if_t<std::is_same<Tag, pl::common::cost_tag>::value, int> = 0>
    void update_node_property(const node_type &node, Tag&& tag, double f_value, double g_value)
    {
        update_node_property(node, common::f_cost_tag{}, f_value);
        update_node_property(node, common::g_cost_tag{}, g_value);
    }

    template<typename Tag, std::enable_if_t<std::is_same<Tag, pl::common::f_cost_tag>::value, int> = 0>
    void update_node_property(const node_type &node, Tag&& tag, double f_value)
    {
        cost_graph_[node.row_index_][node.column_index_].f_ = f_value;
    }

    template<typename Tag, std::enable_if_t<std::is_same<Tag, pl::common::g_cost_tag>::value, int> = 0>
    void update_node_property(const node_type &node, Tag&& tag, double g_value)
    {
        cost_graph_[node.row_index_][node.column_index_].g_ = g_value;
    }

    std::vector<std::vector<common::cost_type>> get_cost_graph()
    {
        return cost_graph_;
    }

    using  Wrapper<Graph>::get_node_property;

private:
    std::vector<std::vector<common::cost_type>> cost_graph_;
};

}

#include "EigenGraph_impl.h"