//
// Created by yash on 8/4/19.
//

#include "bfs.h"
#include "random2d_map_generator.h"
#include "data_types.h"
#include "../../../map_generation/convert.h"
#include "../../../graph/types/EigenGraph.h"

int main()
{
    // Define Map Generator Object
    pfl::map::MapGenerator<size_t, 0, 1> map_generator;

    // Generate a Map
    map_generator.generate_map(100, 100, 6);

    // Test GridMaps
    const auto map = map_generator.get_map();
    const auto EigenMap = pfl::convert::convert_2dvector_to_eigen(map);

    const auto EigenGraph = pfl::graph::eigen_graph<Eigen::MatrixXd>{EigenMap};
    // Set Up Planner
    pfl::algorithms::bfs<std::vector<std::vector<size_t>>, std::vector<size_t>, size_t> vector_planner(&map);
    pfl::algorithms::bfs<pfl::graph::eigen_graph<Eigen::MatrixXd>, std::vector<pfl::common::NodeIndex2d>, pfl::common::NodeIndex2d>
            eigen_planner(&EigenGraph);

//    // Find Path
//    vector_planner.find_path({0, 0}, {1, );
    eigen_planner.find_path({0, 0}, {1, 1});

    // Get Path
//    const auto vector_path = vector_planner.get_path();
    const auto eigen_path = eigen_planner.get_path();

    // Test Check
//    std::cout << (*vector_path)[0];
    std::cout << (*eigen_path)[0];

    return 0;
}