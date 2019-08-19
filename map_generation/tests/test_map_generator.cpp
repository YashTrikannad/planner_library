//
// Created by yash on 8/4/19.
//

#include "../random2d_map_generator.h"
#include "../../common/visualization_utility.h"
#include "convert.h"

int main()
{
    // Define Map Generator Object
    pl::map::ConstraintMapGenerator<size_t, 0, 1> map_generator;

    // Generate a Map
    map_generator.generate_map(100, 100, 7);

    // Convert 2dvector_to_eigen
    const auto EigenMap = pl::convert::convert_2dvector_to_eigen(map_generator.get_map());

    // Print the Map
    std::cout << EigenMap;
//    map_generator.print_map();

    return 0;
}