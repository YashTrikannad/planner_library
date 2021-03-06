//
// Created by yash on 8/7/19.
//

#include <iostream>
#include "graph.h"
#include "../EigenGraph.h"
#include "../EigenGraph_impl.h"
#include "Eigen/Core"

int main()
{
    Eigen::MatrixXd m(10, 10);

    const auto graph = pl::graph::graph(m);

    graph.for_each_node([&, count = 1](const pl::common::NodeIndex2d& node) mutable {
        std::cout << "Node " << count << ", Row Index: " << node.row_index_ <<
        ", Column Index: " << node.column_index_ << "\n";
        count ++;
    });

    return 0;
}

