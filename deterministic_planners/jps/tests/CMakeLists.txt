add_executable(jps_test jps_test.cpp)
target_link_libraries(jps_test map_generation dfs ${OpenCV_LIBS} -lpthread -lm -lboost_log)