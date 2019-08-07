cmake_minimum_required(VERSION 3.14)
project(pathplanning_library)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(common)
add_subdirectory(graph)
add_subdirectory(map_generation)
add_subdirectory(algorithms)


