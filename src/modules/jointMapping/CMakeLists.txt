# YARP needs CMake 3.0 or greater
cmake_minimum_required(VERSION 3.0)

set(PROJECTNAME jointMapping)
project(${PROJECTNAME})

# set up our program
add_executable(${PROJECTNAME} jointMapping.cpp)

# link with YARP libraries
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)
