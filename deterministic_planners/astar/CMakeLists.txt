add_library(astar INTERFACE)
target_sources(astar INTERFACE astar.h astar_impl.h)

target_include_directories(astar INTERFACE
        "${PROJECT_SOURCE_DIR}/deterministic_planners/astar"
        )

target_link_libraries(astar INTERFACE map_generation common)
add_subdirectory(example)