cmake_minimum_required(VERSION 3.14)
project(pathplanning_library)

set(CMAKE_CXX_STANDARD 17)

add_executable(test_map_generator map_generation/test_map_generator.cpp map_generation/random2d_map_generator.h map_generation/random2d_map_generator_impl.h map_generation/map_data_types.h)
