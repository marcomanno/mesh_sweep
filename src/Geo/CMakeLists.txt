
project(Geo)

file(GLOB HXX_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.hh")
file(GLOB CXX_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.cc")

add_library(Geo STATIC ${CXX_FILES} ${HXX_FILES})

target_link_libraries(Geo eigen)
