//
// Created by yash on 8/4/19.
//

#pragma once

#include "data_types.h"

namespace pl::common
{

///
/// @param node1
/// @param node2
/// @return Distance (of specified type - Type) between node1 and node2
template <typename Type = l2>
double get_distance(const NodeIndex2d& node1, const NodeIndex2d& node2)
{
    if constexpr (std::is_same<Type, l2>::value)
        return sqrt(static_cast<double>(pow(node1.row_index_-node2.row_index_, 2) + pow(node2.column_index_-node1.column_index_, 2)));
    if constexpr (std::is_same<Type, l1>::value)
        return sqrt(static_cast<double>(pow(node1.row_index_-node2.row_index_, 1) + pow(node2.column_index_-node1.column_index_, 1)));
}

template <typename NodeType>
auto convert_2dvector_to_eigen(const std::vector<std::vector<NodeType>>& map)
{
    Eigen::MatrixXd EigenMap(map.size(), map[0].size());
    for(size_t i =0; i < EigenMap.rows(); ++i)
    {
        for(size_t j =0; j < EigenMap.cols(); ++j)
        {
            EigenMap(i, j) = map[i][j];
        }
    }
    return EigenMap;
}

}
