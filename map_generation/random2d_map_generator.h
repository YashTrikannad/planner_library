//
// planning_library
// Created by yash on 8/4/19.
//


#pragma once

#include "../common/visualization_utility.h"

#include <stdexcept>
#include <string>
#include <vector>
#include <data_types.h>

namespace pl::map
{

template<typename NodeType, NodeType FreeValue, NodeType ObstacleValue>
class ConstraintMapGenerator
{
public:
    using node_type = NodeType;

    ConstraintMapGenerator() = default;

    /// /// Generates a Random Map with n_obstacles
    /// @param rows No of Rows
    /// @param cols No of Columns
    /// @param n_obstacles No of Obstacles
    void generate_map(size_t rows, size_t cols, size_t n_obstacles);

    /// Get a map if already generated otherwise raise a status exception
    auto get_map() const
    {
        if (map_.empty())
            throw std::runtime_error("Generate a Map before using it");
        return map_;
    };

    /// Print a Map if already generated
    void print_map() const
    {
        if (map_.empty())
            throw std::runtime_error("Generate a Map before printing it");
        common::print_2d_vector(map_);
    }


protected:
    std::vector<std::vector<node_type>> map_;
};


// TODO: Find a Good Solution to replace free value and obstacle values with named enums/ something better
template<typename NodeType, size_t PerNodeCost, NodeType FreeValue=0, NodeType ObstacleValue=255>
class UniformCostMapGenerator : public ConstraintMapGenerator<NodeType, FreeValue, ObstacleValue>
{
public:
    UniformCostMapGenerator() = default;

    void generate_cost_map(size_t rows, size_t cols, size_t n_obstacles)
    {
        this->generate_map(rows, cols, n_obstacles);
        for(const auto& row: this->map_)
        {
             for(const auto& element: row)
             {
                 if(element != ObstacleValue)
                 {
                     element = PerNodeCost;
                 }
             }
        }
    }

private:
};


} // namespace pl::map

#include "random2d_map_generator_impl.h"