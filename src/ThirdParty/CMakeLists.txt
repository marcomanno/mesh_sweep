
set (TH_PA "${CMAKE_CURRENT_SOURCE_DIR}/../../../3p/volumetric_mesh/INSTALL")

set (BOOST_ROOT "${TH_PA}/BOOST")
find_package(BOOST REQUIRED)
add_library(Boost INTERFACE IMPORTED GLOBAL)
target_link_libraries(Boost INTERFACE ${Boost_LIBRARIES})
set_property(TARGET Boost PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIRS})

set (TBB_ROOT "${TH_PA}/TBB/$<CONFIG>")
add_library(Tbb INTERFACE IMPORTED GLOBAL)
set_property(TARGET Tbb PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${TBB_ROOT}/include")
file (GLOB TBB_LIBS ${TBB_ROOT}/lib/*.lib)
target_link_libraries(Tbb INTERFACE ${TBB_LIBS})

add_library(Openvdb INTERFACE IMPORTED GLOBAL)
SET(OPENVDB_LIB_DIR "${TH_PA}/OpenVDB/$<CONFIG>")
SET(OPENEXR_LIB_DIR "${TH_PA}/OpenEXR/$<CONFIG>")
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    SET(OPENVDB_LIB "libopenvdb.lib")
    SET(HALF_LIB  "Half-2_3_s$<$<CONFIG:Debug>:_d>.lib")
    SET(ZLIB_LIB "zlibstatic$<$<CONFIG:Debug>:d>.lib")
    SET(BLOSC_LIB  "libblosc.lib")
    SET(BOOST_VER  "/boost-1_68")
else()
    SET(OPENVDB_LIB "libopenvdb.a")
    SET(HALF_LIB  "libHalf-2_3_s.a")
    SET(ZLIB_LIB "libz.a")
    SET(BLOSC_LIB  "libblosc.a")
    SET(BOOST_VER  "")
endif()

target_link_libraries(Openvdb INTERFACE 
    "${OPENVDB_LIB_DIR}/lib/${OPENVDB_LIB}"
    "${OPENEXR_LIB_DIR}/lib/${HALF_LIB}"
    "${TH_PA}/ZLIB/$<CONFIG>/lib/${ZLIB_LIB}"
    "${TH_PA}/BLOSC/$<CONFIG>/lib/${BLOSC_LIB}")

set_property(TARGET Openvdb PROPERTY INTERFACE_INCLUDE_DIRECTORIES 
    "${OPENVDB_LIB_DIR}/include"
    "${TH_PA}/OpenEXR/$<CONFIG>/include")
set_property(TARGET Openvdb PROPERTY INTERFACE_COMPILE_DEFINITIONS 
    "NOMINMAX" "OPENVDB_OPENEXR_STATICLIB" "OPENVDB_STATICLIB"
    "_SILENCE_CXX17_UNCAUGHT_EXCEPTION_DEPRECATION_WARNING")
if (WIN32)
    target_compile_options(Openvdb INTERFACE "$<$<CONFIG:DEBUG>:/bigobj>")
endif(WIN32)

