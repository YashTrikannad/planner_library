//
// Created by yash on 8/4/19.
//

#pragma once

#include "data_types.h"
#include "random2d_map_generator.h"
#include "eigen3/Eigen/Core"

#include <random>
#include <vector>

namespace pfl::map
{

template<typename NodeType, NodeType FreeValue, NodeType ObstacleValue>
void map::MapGenerator<NodeType, FreeValue, ObstacleValue>::generate_map(size_t rows, size_t cols, size_t n_obstacles)
{
    std::vector<std::vector<node_type>> map(rows, std::vector<node_type>(cols, FreeValue));

    std::random_device r;
    std::default_random_engine engine(r());
    std::uniform_int_distribution<int> random_row(1, rows);
    std::uniform_int_distribution<int> random_column(1, cols);
    std::uniform_int_distribution<int> random_height(1, rows / 3);
    std::uniform_int_distribution<int> random_width(1, cols / 3);

    while (n_obstacles > 0)
    {
        const auto upper_left_obstacle = common::NodeIndex2d(random_row(engine), random_column(engine));
        const auto obstacle_height = random_height(engine);
        const auto obstacle_width = random_width(engine);

        for (size_t row_index = upper_left_obstacle.row_index_, height = 0;
             row_index < rows && height < obstacle_height;
             row_index++, height++)
            for (size_t col_index = upper_left_obstacle.column_index_, width = 0; col_index < cols &&
                                                                                  width <
                                                                                  obstacle_width; col_index++, width++)
            {
                map[row_index][col_index] = ObstacleValue;
            }

        n_obstacles--;
    }

    map_ = map;
}

} // namespace pfl::map