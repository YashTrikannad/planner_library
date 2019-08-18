//
// Created by yash on 8/5/19.
//

#pragma once

#include <vector>
#include "Eigen/Dense"

namespace pl::convert
{

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
