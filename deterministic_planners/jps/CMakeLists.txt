add_library(jps INTERFACE)
target_sources(jps INTERFACE jps.h jps_impl.h)

target_include_directories(jps INTERFACE
        "${PROJECT_SOURCE_DIR}/deterministic_planners/jps"
        )

target_link_libraries(jps INTERFACE map_generation common -lpthread -lm)
add_subdirectory(example)