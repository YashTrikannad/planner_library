add_executable(test_dfs dfs_example.cpp)
target_link_libraries(test_dfs map_generation dfs ${OpenCV_LIBS})