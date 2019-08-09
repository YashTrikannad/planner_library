//
// Created by yash on 8/4/19.
//

#pragma once

#include <array>

namespace pfl::common
{

struct NodeIndex2d
{
    NodeIndex2d(size_t row_index, size_t column_index) : row_index_(row_index), column_index_(column_index)
    {
    }

    friend std::ostream& operator<<(std::ostream& os, const NodeIndex2d& node)
    {
        os << "row_index: " << node.row_index_ << "    column_index: " << node.column_index_;
    }

    size_t row_index_;
    size_t column_index_;
};

inline bool operator==(const NodeIndex2d& lhs, const NodeIndex2d& rhs)
{
    return lhs.row_index_ == rhs.row_index_ && lhs.column_index_ == rhs.column_index_;
}

} // namespace pfl::common
