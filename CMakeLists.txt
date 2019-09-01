cmake_minimum_required(VERSION 3.14)
project(pathplanning_library)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/benchmark/cmake_modules" ${CMAKE_MODULE_PATH})

FIND_PACKAGE(Boost 1.65 COMPONENTS program_options REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(GoogleBenchmark REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

add_subdirectory(benchmark)
add_subdirectory(common)
add_subdirectory(deterministic_planners)
add_subdirectory(graph)
add_subdirectory(map_generation)


