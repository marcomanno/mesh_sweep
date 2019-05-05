# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)

project(mesh_sweep)

if(MSVC)
add_definitions(-D_USE_MATH_DEFINES)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++latest /W0  /bigobj")
endif(MSVC)

add_subdirectory(Utils)
add_subdirectory(Geo)
add_subdirectory(Topology)
