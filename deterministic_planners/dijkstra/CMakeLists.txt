add_library(dijkstra INTERFACE)
target_sources(dijkstra INTERFACE dijkstra.h dijkstra_impl.h)

target_include_directories(dijkstra INTERFACE
        "${PROJECT_SOURCE_DIR}/deterministic_planners/dijkstra"
        )

target_link_libraries(dijkstra INTERFACE map_generation common)
add_subdirectory(example)