add_library(bfs INTERFACE)
target_sources(bfs INTERFACE bfs.h)

target_include_directories(bfs INTERFACE
        "${PROJECT_SOURCE_DIR}/algorithms/bfs"
        )

target_link_libraries(bfs INTERFACE map_generation common)
add_subdirectory(tests)

