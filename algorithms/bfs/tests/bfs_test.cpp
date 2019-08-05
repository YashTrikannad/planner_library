//
// Created by yash on 8/4/19.
//

#include "bfs.h"
#include "random2d_map_generator.h"
#include "data_types.h"

int main()
{
    // Define Map Generator Object
    pfl::map::MapGenerator<size_t, 0, 1> map_generator;

    // Generate a Map
    map_generator.generate_map(100, 100, 6);

    const auto map = map_generator.get_map();

    pfl::algorithms::bfs<std::vector<std::vector<size_t>>, std::vector<size_t>, size_t> planner(&map);

    planner.find_path(0, 1);

    const auto path = planner.get_path();

    std::cout << (*path)[0];

    return 0;
}