add_library(dfs INTERFACE)
target_sources(dfs INTERFACE dfs.h dfs_impl.h)

target_include_directories(dfs INTERFACE
        "${PROJECT_SOURCE_DIR}/deterministic_planners/dfs"
        )

target_link_libraries(dfs INTERFACE map_generation common)
