cmake_minimum_required(VERSION 3.15...3.27)
project(nexdyndiff)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(nexdyndiff)
add_subdirectory(examples)
add_subdirectory(tests)

option(NEXDYNDIFF_BUILD_PYTHON_BINDINGS "Build Python Bindings" OFF)
if(NEXDYNDIFF_BUILD_PYTHON_BINDINGS)
	if(EXISTS "pystark/cpp")
		add_subdirectory(pystark/cpp)
	endif()
endif()
