add_executable(dijkstra_example astar_example.cpp)
target_link_libraries(dijkstra_example map_generation dfs ${OpenCV_LIBS})