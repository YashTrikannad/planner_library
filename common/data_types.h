//
// Created by yash on 8/4/19.
//

#pragma once

#include <array>
#include <ostream>
#include <boost/functional/hash.hpp>

namespace pl::common
{

struct NodeIndex2d
{
    NodeIndex2d() = default;

    NodeIndex2d(size_t row_index, size_t column_index) : row_index_(row_index), column_index_(column_index)
    {
    }

    NodeIndex2d(size_t row_index, size_t column_index, size_t obstacle) : row_index_(row_index), column_index_(column_index), value_(obstacle)
    {
    }

    friend std::size_t hash_value(NodeIndex2d const& node)
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, node.row_index_);
        boost::hash_combine(seed, node.column_index_);
        return seed;
    }

    friend std::ostream& operator<<(std::ostream& os, const NodeIndex2d& node)
    {
        os << "(" << node.row_index_ << ", " << node.column_index_ <<")";
    }

    size_t value_;
    int row_index_;
    int column_index_;
};

inline bool operator==(const NodeIndex2d& lhs, const NodeIndex2d& rhs)
{
    return lhs.row_index_ == rhs.row_index_ && lhs.column_index_ == rhs.column_index_;
}

inline bool operator!=(const NodeIndex2d& lhs, const NodeIndex2d& rhs)
{
    return lhs.row_index_ != rhs.row_index_ || lhs.column_index_ != rhs.column_index_;
}

// tags used in tag dispatching ( Currently only for 2d )
struct direction
{
};

struct up : direction
{
    static constexpr size_t change_rows = -1;
    static constexpr size_t change_cols = 0;
};

struct down : direction
{
    static constexpr size_t change_rows = +1;
    static constexpr size_t change_cols = 0;
};

struct left : direction
{
    static constexpr size_t change_rows = 0;
    static constexpr size_t change_cols = -1;
};

struct right : direction
{
    static constexpr size_t change_rows = 0;
    static constexpr size_t change_cols = +1;
};

struct top_right : direction
{
    static constexpr size_t change_rows = -1;
    static constexpr size_t change_cols = +1;
};

struct top_left
{
    static constexpr size_t change_rows = -1;
    static constexpr size_t change_cols = -1;
};

struct bottom_right : direction
{
    static constexpr size_t change_rows = +1;
    static constexpr size_t change_cols = +1;
};

struct bottom_left : direction
{
    static constexpr size_t change_rows = +1;
    static constexpr size_t change_cols = -1;
};


struct cell_type
{
    using type = double;
};

struct cost_type
{
    /// f = g + h
    /// f - Total Cost of the node
    /// g - Cost from the start node to the current node
    /// h - Heuristic Cost from current node to the goal
    double f_;
    double g_;
};


struct cost_tag
{
    using type = cost_type;
};

struct f_cost_tag
{
    using type = double;
};

struct g_cost_tag
{
    using type = double ;
};

struct distance_type
{
};

/// tag for l2 squared distance
struct l2 : distance_type
{
    using type = distance_type;
};

/// tag for l1 manhattan distance
struct l1 : distance_type
{
    using type = distance_type;
};

} // namespace pl::common

// Include hash for NodeIndex2d in std namespace
namespace std
{
    template<> struct hash<::pl::common::NodeIndex2d> : boost::hash<::pl::common::NodeIndex2d> {};
}