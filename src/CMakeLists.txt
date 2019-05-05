# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)

project(mesh_sweep)

if(MSVC)
add_definitions(-D_USE_MATH_DEFINES)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17 /W0  /bigobj  /MP")
endif(MSVC)

include_directories (${CMAKE_CURRENT_SOURCE_DIR})

set(BUILD_TESTING OFF)
add_subdirectory(eigen)
add_subdirectory(Utils)
add_subdirectory(PolygonTriangularization)
add_subdirectory(Geo)
add_subdirectory(Import)
add_subdirectory(Topology)
