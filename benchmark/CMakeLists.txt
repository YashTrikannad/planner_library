include_directories(${benchmark_INCLUDE_DIRS})
include_directories(headers)

file(GLOB_RECURSE SOURCE_FILES headers/* src/*)

add_executable(deterministic_planners_benchmark deterministic_planners_benchmark.cpp)

target_link_libraries(deterministic_planners_benchmark map_generation astar common ${benchmark_LIBRARIES}
        ${OpenCV_LIBS} -lpthread -lbenchmark -lboost_log)