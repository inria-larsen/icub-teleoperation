# YARP needs CMake 3.0 or greater
cmake_minimum_required(VERSION 3.0)

# find YARP
find_package(YARP REQUIRED)

# set up our program
add_executable(jointMapping jointMapping.cpp)

# link with YARP libraries
target_link_libraries(jointMapping ${YARP_LIBRARIES})
