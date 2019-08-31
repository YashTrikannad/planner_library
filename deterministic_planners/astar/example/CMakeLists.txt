add_executable(astar_eigen_graph_example astar_eigen_graph_example.cpp)
target_link_libraries(astar_eigen_graph_example map_generation dfs ${OpenCV_LIBS} -lpthread -lm -lboost_log)

add_executable(astar_vector_graph_example astar_vector_graph_example.cpp)
target_link_libraries(astar_vector_graph_example map_generation dfs ${OpenCV_LIBS} -lpthread -lm -lboost_log)