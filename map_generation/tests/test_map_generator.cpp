//
// Created by yash on 8/4/19.
//

#include "../random2d_map_generator.h"
#include "../../common/visualization_utility.h"

int main()
{
    map::MapGenerator<bool, false, true> map_generator;

    map_generator.generate_map(10, 10, 4);

    const auto map = map_generator.get_map();

    common::print_2d_vector(map);

    return 0;
}