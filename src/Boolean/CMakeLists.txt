project(Boolean)

include_directories (${Boost_INCLUDE_DIRS})

# source code directories
file(GLOB HXX_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.hh")
file(GLOB CXX_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.cc")

add_library(Boolean STATIC ${CXX_FILES} ${HXX_FILES})

target_link_libraries(Boolean 
  PRIVATE eigen 
  PRIVATE Import)

