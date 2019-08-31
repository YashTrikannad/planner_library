//
// Created by yash on 8/4/19.
//

#pragma once

#include "data_types.h"

namespace pl::common
{

double get_distance(const NodeIndex2d& node1, const NodeIndex2d& node2)
{
    const double x_dist = node1.row_index_>node2.row_index_?
          static_cast<double>(node1.row_index_-node2.row_index_): static_cast<double>(node2.row_index_-node1.row_index_);
    const double y_dist = node1.column_index_>node2.column_index_?
          static_cast<double>(node1.column_index_-node2.column_index_): static_cast<double>(node2.column_index_-node1.column_index_);
    return sqrt(pow(x_dist, 2) + pow(y_dist, 2));
}

}
