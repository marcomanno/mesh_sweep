
project(Levelset)

file(GLOB HXX_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.hxx")
file(GLOB CXX_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.cxx")

add_library(Levelset STATIC ${CXX_FILES} ${HXX_FILES})

target_link_libraries(Levelset PRIVATE Openvdb Tbb Boost)
