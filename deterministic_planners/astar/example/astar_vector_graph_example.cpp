//
// Created by yash on 8/4/19.
//

#include "../astar.h"
#include "random2d_map_generator.h"
#include "data_types.h"
#include "../../../graph/types/CostGraph.h"
#include "../../../graph/types/VectorGraph.h"


int main()
{
    // Define Map Generator Object
    pl::map::ConstraintMapGenerator<size_t, 0, 1> map_generator;

    // Generate a Map
    map_generator.generate_map(500, 500, 10);

    // Test GridMaps
    auto map = map_generator.get_map();

    auto vector_graph = pl::graph::cost_graph<pl::graph::graph, std::vector<std::vector<size_t>>, size_t>{map};

    // Set Up Planner
    pl::algorithms::astar<pl::graph::cost_graph<pl::graph::graph, std::vector<std::vector<size_t>>, size_t>,
            std::vector<pl::common::NodeIndex2d>, pl::common::NodeIndex2d>
            eigen_planner(&vector_graph);

    // Find Path
    eigen_planner.find_path<8>({0, 0}, {350, 350});

    // Get Path
    const auto eigen_path = eigen_planner.get_path();

    // Test Check
    if(eigen_path)
    {
        // Get Explored Nodes for debug information
        const auto explored_nodes = eigen_planner.get_closed_set();

        pl::common::display_debug(map, *eigen_path, explored_nodes);
    }
    else
    {
        std::cout << " No Path Exists! :( \n" ;
    }

    return 0;
}
