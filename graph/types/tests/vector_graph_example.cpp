//
// Created by yash on 8/7/19.
//

#include <iostream>
#include "graph.h"
#include "../VectorGraph.h"
#include "Eigen/Core"

#include <vector>

int main()
{
    const auto input_vector = std::vector<std::vector<int>>(10, std::vector<int>(10, 0));

    const auto graph = pl::graph::graph(input_vector);

    graph.for_each_node([&, count = 1](const pl::common::NodeIndex2d& node) mutable {
        std::cout << "Node " << count << ", Row Index: " << node.row_index_ <<
                  ", Column Index: " << node.column_index_ << "\n";
        count ++;
    });

    return 0;
}

