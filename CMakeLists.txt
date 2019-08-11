cmake_minimum_required(VERSION 3.14)
project(pathplanning_library)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
FIND_PACKAGE( Boost 1.65 COMPONENTS program_options REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_subdirectory(common)
add_subdirectory(graph)
add_subdirectory(map_generation)
add_subdirectory(deterministic_planners)


