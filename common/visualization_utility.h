//
// Created by yash on 8/4/19.
//

#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <vector>

namespace pfl::common
{

template<typename ElementType>
void print_2d_vector(const std::vector<std::vector<ElementType>> &container)
{
    for (const auto &vector:container)
    {
        for (const auto &element:vector)
        {
            std::cout << element << " ";
        }
        std::cout << "\n";
    }
}

template <typename Graph>
void display(const Graph& graph)
{
    using namespace cv;
    Mat image;
    eigen2cv(graph, image);
    imshow( "Current Graph", image );
    waitKey(0);
}

} // namespace pfl::common