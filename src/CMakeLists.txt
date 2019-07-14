# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.13)
cmake_policy(VERSION 3.13)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")

project(mesh_sweep)

if(MSVC)
add_definitions(-D_USE_MATH_DEFINES)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++latest /W3 /WX /bigobj  /MP")
endif(MSVC)

include_directories (${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(ThirdParty)

option(CATCH_BUILD_TESTING "Build SelfTest project" OFF)
option(CATCH_ENABLE_WERROR "Enable all warnings as errors" ON)
set(CATCH_INSTALL_DOCS OFF)
set(CATCH_INSTALL_HELPERS OFF)
add_subdirectory(Catch2)

set(BUILD_TESTING OFF)
set(EIGEN_BUILD_BTL OFF)
set(EIGEN_SPLIT_LARGE_TESTS)
add_subdirectory(eigen)

add_subdirectory(Utils)
add_subdirectory(PolygonTriangularization)
add_subdirectory(Geo)
add_subdirectory(Import)
add_subdirectory(Topology)
add_subdirectory(Boolean)
add_subdirectory(Levelset)

add_subdirectory(TestMe)
