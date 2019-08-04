//
// Created by yash on 8/4/19.
//

#include "../random2d_map_generator.h"
#include "../../common/visualization_utility.h"

int main()
{
    // Define Map Generator Object
    map::MapGenerator<size_t , 0, 2> map_generator;

    // Generate a Map
    map_generator.generate_map(10, 10, 4);

    // Print the Map
    map_generator.print_map();

    return 0;
}