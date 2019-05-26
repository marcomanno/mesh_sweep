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

set (BOOST_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../volumetric_mesh/INSTALL/BOOST")
set (Boost_USE_STATIC_LIBS ON)
find_package(BOOST)

set(BUILD_TESTING OFF)
set(EIGEN_BUILD_BTL OFF)
add_subdirectory(eigen)
add_subdirectory(Utils)
add_subdirectory(PolygonTriangularization)
add_subdirectory(Geo)
add_subdirectory(Import)
add_subdirectory(Topology)

option(CATCH_BUILD_TESTING "Build SelfTest project" OFF)
option(CATCH_ENABLE_WERROR "Enable all warnings as errors" ON)
set(CATCH_INSTALL_DOCS OFF)
set(CATCH_INSTALL_HELPERS OFF)
add_subdirectory(Catch2)

add_subdirectory(TestMe)
