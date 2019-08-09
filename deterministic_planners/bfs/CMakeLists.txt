add_library(bfs INTERFACE)
target_sources(bfs INTERFACE bfs.h bfs_impl.h)

target_include_directories(bfs INTERFACE
        "${PROJECT_SOURCE_DIR}/deterministic_planners/bfs"
        )

target_link_libraries(bfs INTERFACE map_generation common)
add_subdirectory(tests)

