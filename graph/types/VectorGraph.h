//
// Created by yash on 8/6/19.
//

#pragma once

#include <vector>
#include "../graph.h"
#include "../../common/data_types.h"


namespace pl::graph
{


/// Specialization of Graph for Vector of Vectors
template<typename DataType>
class graph<std::vector<std::vector<DataType>>, DataType>
{
public:
    using container_type = std::vector<std::vector<DataType>>;
    using data_type = DataType;
    using node_type = pl::common::NodeIndex2d;

    graph(const std::vector<std::vector<data_type>> &container) : container_(container),
                                             rows_(container_.size()),
                                             cols_(container_.at(0).size())
    {
    }

    graph(std::vector<std::vector<data_type >> &&container) : container_(std::move(container)),
                                             rows_(container_.size()),
                                             cols_(container_.at(0).size())
    {
    }

    /// Func is applied to each adjacent node of the current node
    /// @tparam Func
    template<size_t N, typename Func>
    void for_each_adjacent_node(const node_type &node, Func &&func) const;


    /// Get the node property of the queried node of the given tag type
    /// @tparam Tag
    /// @param node
    /// @param tag
    /// @return
    template<typename Tag, std::enable_if_t<std::is_same<Tag, pl::common::cell_type>::value, int> = 0>
    typename Tag::type get_node_property(const node_type &node, Tag &&tag) const
    {
        return container_[node.row_index_][node.column_index_];
    }


    /// Func is applied to all the nodes in the graph
    /// @tparam Func
    template<typename Func>
    void for_each_node(Func &&func) const;


private:
    container_type container_;
    size_t rows_;
    size_t cols_;

    ///
    /// @param node - current node
    /// @return get 4 neighbors
    auto get_4_neighbor(const node_type &node) const;

    ///
    /// @param node
    /// @return get 8 neighbors
    auto get_8_neighbor(const node_type &node) const;


    ///
    /// @tparam Graph
    /// @tparam Direction
    /// @param node _ Non Bordering nodes ( Doesn't perform boundary checks)
    /// @param direction
    /// @return adjacent nodes corresponding to the direction
    template<typename Direction>
    node_type get_adjacent_node(const node_type &node, Direction direction) const;


    ///
    /// @tparam Graph
    /// @tparam Direction
    /// @param node - node on the graph boundary
    /// @param direction - direction of adjacent node
    /// @return adjacent node if present else null_opt
    template<typename Direction>
    std::optional<node_type> get_adjacent_node_with_check(const node_type &node, Direction direction) const;
};

}

#include "VectorGraph_impl.h"

