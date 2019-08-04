//
// Created by yash on 8/4/19.
//

#pragma once

#include <array>

namespace map
{
    struct NodeIndex2d
    {
        NodeIndex2d(size_t row_index, size_t column_index): row_index_(row_index), column_index_(column_index)
        {
        }
        size_t row_index_;
        size_t column_index_;
    };
}
