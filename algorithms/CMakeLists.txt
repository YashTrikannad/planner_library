add_library(algorithms INTERFACE)
target_sources(algorithms INTERFACE algorithms.h)

target_include_directories(algorithms INTERFACE "${PROJECT_SOURCE_DIR}/algorithms")

add_subdirectory(bfs)