add_library(types INTERFACE)
target_sources(types INTERFACE EigenGraph.h)

target_include_directories(types INTERFACE "${PROJECT_SOURCE_DIR}/graph/types")

add_subdirectory(tests)