add_executable(astar_example astar_example.cpp)
target_link_libraries(astar_example map_generation dfs ${OpenCV_LIBS})