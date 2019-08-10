add_library(algorithms INTERFACE)
target_sources(algorithms INTERFACE algorithms.h)

target_include_directories(algorithms INTERFACE "${PROJECT_SOURCE_DIR}/deterministic_planners")

add_subdirectory(bfs)
add_subdirectory(dfs)