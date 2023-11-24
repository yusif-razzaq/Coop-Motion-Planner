project(AMP-OMPL-Example)

cmake_minimum_required (VERSION 3.16)
set (CMAKE_CXX_STANDARD 17)

find_package(ompl REQUIRED) # make sure this is the ompl with the K-CBS implementation
find_package(yaml-cpp REQUIRED)

include_directories(
    includes
    ${OMPL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)
link_directories(${OMPL_LIBRARY_DIRS})

file(GLOB DEMO_SOURCES RELATIVE ${CMAKE_SOURCE_DIR}/src/ src/*.cpp )


include(CMakePrintHelpers)
cmake_print_variables(DEMO_SOURCES)
cmake_print_variables(CMAKE_SOURCE_DIR)

foreach( demoFile ${DEMO_SOURCES} )
    string( REPLACE ".cpp" "" demoName ${demoFile} )
    add_executable( ${demoName} src/${demoFile} )
    # Make sure YourLib is linked to each app
    target_link_libraries( ${demoName} ${OMPL_LIBRARIES} ${YAML_CPP_LIBRARIES} ode)
endforeach( demoFile ${DEMO_SOURCES} )

# # ignore BOOST deprecated headers
# add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
# add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")

# cmake_minimum_required(VERSION 3.16)
# project(MyOMPLProject)

# # Set C++ standard
# set(CMAKE_CXX_STANDARD 14)

# # OMPL_LIBRARIES:FILEPATH=/usr/local/lib/libompl.1.6.0.dylib
# # Path to additional header files
# include_directories(includes)

# # Find OMPL package
# find_package(ompl REQUIRED)
# find_package(yaml-cpp REQUIRED)

# # Get all source files in the src directory
# file(GLOB SRC_FILES "src/*.cpp")

# foreach(src_file ${SRC_FILES})
#     # Extracting file name without extension
#     get_filename_component(exec_name ${src_file} NAME_WE)
#     add_executable(${exec_name} ${src_file})
#     # Link OMPL library to each executable
#     target_link_libraries(${exec_name} ${OMPL_LIBRARIES} ${YAML_CPP_LIBRARIES})
# endforeach()

