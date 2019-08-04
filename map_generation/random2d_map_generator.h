//
// pathfinding_library
// Created by yash on 8/4/19.
//


#pragma once

#include <stdexcept>
#include <string>
#include <vector>


namespace map
{

    template<typename NodeType, size_t FreeValue, size_t ObstacleValue>
    class MapGenerator
    {
    public:
        using node_type = NodeType;

        MapGenerator() = default;

        /// Generates a Random Map with Random Obstacles
        void generate_map(size_t rows, size_t columns, size_t n_obstacles);

        /// Get a map if already generated otherwise raise a status exception
        std::vector<std::vector<node_type>> get_map() const
        {
            if(map_.empty())
                throw std::runtime_error("Generate a Map before using it");
            return map_;
        };

    private:
        std::vector<std::vector<node_type>> map_;
    };

}// namespace map

#include "random2d_map_generator_impl.h"