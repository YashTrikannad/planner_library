add_library(types INTERFACE)
target_sources(types INTERFACE EigenGraph.h EigenGraph_impl.h VectorGraph.h VectorGraph_impl.h CostGraph.h)

target_include_directories(types INTERFACE "${PROJECT_SOURCE_DIR}/graph/types")

add_subdirectory(tests)