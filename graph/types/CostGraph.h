//
// Created by yash on 8/31/19.
//

#pragma once

namespace pl::graph
{

/// Wrapper Class with added layer of costs required by heuristic algorithms
template <template<class> class Wrapper, typename Graph>
class cost_graph : public Wrapper<Graph>
{
public:
    using graph_type = Graph;
    using node_type = pl::common::NodeIndex2d;

    cost_graph(const graph_type& graph) : Wrapper<Graph>(graph),
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

    std::vector<std::vector<common::cost_type>> get_cost_graph() const
    {
        return cost_graph_;
    }

    using  Wrapper<Graph>::get_node_property;

private:
    std::vector<std::vector<common::cost_type>> cost_graph_;
};

}
