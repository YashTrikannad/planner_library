add_executable(dijkstra_example dijkstra_example.cpp)
target_link_libraries(dijkstra_example map_generation dfs ${OpenCV_LIBS})