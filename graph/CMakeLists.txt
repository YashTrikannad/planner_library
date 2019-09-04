add_library(graph INTERFACE)
target_sources(graph INTERFACE graph.h utility.h)

target_include_directories(graph INTERFACE "${PROJECT_SOURCE_DIR}/graph")

add_subdirectory(types)