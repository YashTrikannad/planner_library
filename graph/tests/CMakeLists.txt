add_library(graph_fixture INTERFACE)
target_sources(graph_fixture INTERFACE graph.h)

target_include_directories(graph_fixture INTERFACE "${PROJECT_SOURCE_DIR}/graph")

add_executable(graph_test graph_test.cpp)
target_link_libraries(graph_test map_generation dfs ${OpenCV_LIBS} -lpthread -lm -lboost_log)