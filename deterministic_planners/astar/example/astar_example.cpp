//
// Created by yash on 8/4/19.
//

#include "../astar.h"
#include "random2d_map_generator.h"
#include "data_types.h"
#include "../../../map_generation/convert.h"
#include "../../../graph/types/EigenGraph.h"


int main()
{
    // Define Map Generator Object
    pl::map::ConstraintMapGenerator<size_t, 0, 1> map_generator;

    // Generate a Map
    map_generator.generate_map(400, 400, 10);

    // Test GridMaps
    const auto map = map_generator.get_map();

    auto EigenMap = pl::convert::convert_2dvector_to_eigen(map);

    const auto EigenGraph = pl::graph::eigen_cost_graph<pl::graph::eigen_graph, Eigen::MatrixXd>{EigenMap};

    // Set Up Planner
    pl::algorithms::astar<pl::graph::eigen_cost_graph<pl::graph::eigen_graph, Eigen::MatrixXd>,
            std::vector<pl::common::NodeIndex2d>, pl::common::NodeIndex2d>
            eigen_planner(&EigenGraph);

    // Find Path
    eigen_planner.find_path<8>({0, 0}, {300, 300});

    // Get Path
    const auto eigen_path = eigen_planner.get_path();

    // Test Check
    if(eigen_path)
    {
        pl::common::display(EigenMap, *eigen_path);
    }
    else
    {
        std::cout << " No Path Exists! :( \n" ;
    }

    return 0;
}
