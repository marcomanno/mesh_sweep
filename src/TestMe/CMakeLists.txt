set(project_name Test)
project(${project_name})

file(GLOB HXX_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.hxx")
file(GLOB CXX_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.cxx")

# Add source to this project's executable.
add_executable(${project_name} ${CXX_FILES} ${HXX_FILES})
target_link_libraries(${project_name} 
  Import Topology Geo PolygonTriangularization flatten Catch2)
  
add_definitions(-DOUTDIR="${CMAKE_SOURCE_DIR}/../out/TestData")
add_definitions(-DINDIR="${CMAKE_SOURCE_DIR}/Test/Data")
