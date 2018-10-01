# Copyright: (C) 2017 Inria
# Author: Luigi Penco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# YARP needs CMake 3.0 or greater
cmake_minimum_required(VERSION 2.8.12)


set(PROJECTNAME retargeting)
project(${PROJECTNAME})

file(GLOB folder_source ./*.cpp)
source_group("Source Files" FILES ${folder_source})
add_executable(${PROJECT_NAME} ${folder_source})

target_include_directories(${PROJECT_NAME} PUBLIC ${YARP_INCLUDE_DIRS}
					          ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})


install(TARGETS ${PROJECTNAME} DESTINATION bin)
