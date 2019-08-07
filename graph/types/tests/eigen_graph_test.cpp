//
// Created by yash on 8/7/19.
//

#include <iostream>
#include "../../pfl_graph.h"
#include "../EigenGraph.h"
#include "Eigen/Core"

int main()
{
    Eigen::Vector2d m;

    const auto graph = pfl::graph::eigen_graph(m);

    return 0;
}

