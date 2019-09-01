//
// Created by yash on 8/4/19.
//

#pragma once

#include "data_types.h"

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <vector>
#include <opencv2/imgproc.hpp>

namespace pl::common
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
    imshow( "Current ContainerType", image );
    waitKey(0);
}

template <typename Graph, typename Path, typename = std::enable_if_t <!std::is_same<Eigen::MatrixXd, Graph>::value>>
void display(Graph& graph, const Path& path)
{
    cv::Mat image(graph.size(), graph.at(0).size(), CV_64FC1);

    for(const auto& node:path)
    {
        graph[node.row_index_][node.column_index_] = 255;
    }

    for(int i=0; i<graph.size(); ++i)
        for(int j=0; j<graph.at(0).size(); ++j)
            image.at<double>(i, j) = graph.at(i).at(j);

    cv::imshow( "Current ContainerType", image );
    cv::waitKey(0);
}

template <typename Path>
void display(Eigen::MatrixXd& graph, const Path& path)
{
    using namespace cv;
    Mat image;
    for(const auto& node:path)
    {
        graph(node.row_index_, node.column_index_) = 0.5;
    }

    eigen2cv(graph, image);

    imshow( "Current ContainerType", image );
    waitKey(0);
}


void display(std::vector<std::vector<common::cost_type>> cost_vector)
{
    using namespace cv;

    cv::Mat image(cost_vector.size(), cost_vector.at(0).size(), CV_64FC1);
    for(int i=0; i<image.rows; ++i)
        for(int j=0; j<image.cols; ++j)
            image.at<double>(i, j) = cost_vector.at(i).at(j).f_;

    imshow( "Current ContainerType", image );
    waitKey(0);
}

template <typename Container, typename Path, typename ExploredNodesType, typename = std::enable_if_t <!std::is_same<Eigen::MatrixXd, Container>::value>>
void display_debug(Container& container, const Path& path, const ExploredNodesType& explored_nodes)
{
    cv::Mat image(container.size(), container.at(0).size(), CV_8UC3, cv::Scalar(0, 0, 0));

    const auto check_if_colored = [&image](const auto& row_index, const auto& column_index){
        return image.at<cv::Vec3b>(row_index, column_index)  == cv::Vec3b(0, 0, 0) ? true : false;
    };

    for(const auto& node:path)
    {
        image.at<cv::Vec3b>(node.row_index_,node.column_index_) = {5, 5, 100};
    }
    cv::dilate(image, image, cv::Mat());

    for(const auto& node:explored_nodes)
    {
        if (check_if_colored(node.row_index_, node.column_index_))
            image.at<cv::Vec3b>(node.row_index_, node.column_index_) = {100, 5, 5};
    }

    for(int i=0; i<container.size(); ++i)
        for(int j=0; j<container.at(0).size(); ++j)
        {
            if (container.at(i).at(j) == 0 && check_if_colored(i, j))
            {
                image.at<cv::Vec3b>(i, j) = {0,0,0};
            }
            else if (container.at(i).at(j) == 1 && check_if_colored(i, j))
            {
                image.at<cv::Vec3b>(i, j) = {255,255,255};
            }
        }

    namedWindow("GridMap", cv::WINDOW_NORMAL);
    cv::imshow("GridMap", image);
    cv::waitKey(0);
}

} // namespace pl::common