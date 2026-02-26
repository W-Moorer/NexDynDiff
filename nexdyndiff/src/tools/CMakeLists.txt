cmake_minimum_required(VERSION 3.9)

# Get parent directory (nexdyndiff)
get_filename_component(NEXDYNDIFF_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

add_executable(NexDynDiff
    NexDynDiff.cpp
)

target_link_libraries(NexDynDiff nexdyndiff)

# Include paths - must match what nexdyndiff library uses
target_include_directories(NexDynDiff PRIVATE
    ${NEXDYNDIFF_ROOT}/include
    ${NEXDYNDIFF_ROOT}/src/core
    ${NEXDYNDIFF_ROOT}/src/models
    ${NEXDYNDIFF_ROOT}/src/utils
    ${NEXDYNDIFF_ROOT}/src/scene
)

set_target_properties(NexDynDiff PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
