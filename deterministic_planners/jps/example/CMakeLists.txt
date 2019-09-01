add_executable(jps_vector_graph_example jps_vector_graph_example.cpp)
target_link_libraries(jps_vector_graph_example map_generation dfs ${OpenCV_LIBS} -lpthread -lm -lboost_log)