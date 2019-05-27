
project(Geo)

file(GLOB HXX_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.hh")
file(GLOB CXX_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.cc")

add_library(Geo STATIC ${CXX_FILES} ${HXX_FILES})

target_compile_definitions (Geo PRIVATE _SILENCE_CXX17_NEGATORS_DEPRECATION_WARNING)

target_link_libraries(Geo PRIVATE eigen)
