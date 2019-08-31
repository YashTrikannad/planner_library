//
// Created by yash on 8/4/19.
//

#pragma once

#include "data_types.h"

namespace pl::common
{

double get_distance(const NodeIndex2d& node1, const NodeIndex2d& node2)
{
    return sqrt(static_cast<double>(pow(node1.row_index_-node2.row_index_, 2) + pow(node2.column_index_-node1.column_index_, 2)));
}

}
