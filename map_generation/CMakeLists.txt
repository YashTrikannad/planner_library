add_library(map_generation INTERFACE)
target_sources(map_generation INTERFACE random2d_map_generator.h random2d_map_generator_impl.h convert.h cost_generator.h)

target_include_directories(map_generation INTERFACE
        "${PROJECT_SOURCE_DIR}/map_generation"
        )

target_link_libraries(map_generation INTERFACE common)
add_subdirectory(tests)