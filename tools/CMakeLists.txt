set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../nexdyndiff/extern/Eigen)
if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
set(CMAKE_CXX_FLAGS "-DEIGEN_MPL2_ONLY")

set(SOURCE_FILES
    RunDoublePendulumScene.cpp
)

add_executable(DoublePendulumRunner ${SOURCE_FILES})

target_link_libraries(DoublePendulumRunner PRIVATE nexdyndiff)
target_include_directories(DoublePendulumRunner PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../nexdyndiff/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../nexdyndiff/src/core
    ${CMAKE_CURRENT_SOURCE_DIR}/../nexdyndiff/src/models
    ${CMAKE_CURRENT_SOURCE_DIR}/../nexdyndiff/src/utils
)
target_include_directories(DoublePendulumRunner PUBLIC ${EIGEN3_INCLUDE_DIR})

