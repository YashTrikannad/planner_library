add_library(common INTERFACE)
target_sources(common INTERFACE map_data_types.h visualization_utility.h)

target_include_directories(common INTERFACE
        "${PROJECT_SOURCE_DIR}/common"
        )



