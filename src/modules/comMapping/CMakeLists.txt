# Copyright: (C) 2017 Inria
# Author: Luigi Penco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# YARP needs CMake 3.0 or greater
cmake_minimum_required(VERSION 3.0)

set(PROJECTNAME comMapping)
project(${PROJECTNAME})

find_package(iDynTree REQUIRED)

find_package(wholeBodyInterface REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)

set(folder_source main.cpp)
source_group("Source Files" FILES ${folder_source})
add_executable(${PROJECT_NAME} ${folder_source})

target_include_directories(${PROJECT_NAME} PUBLIC ${wholeBodyInterface_INCLUDE_DIRS}
					          ${yarpWholeBodyInterface_INCLUDE_DIRS}
					          ${iDynTree_INCLUDE_DIRS}
					          ${skinDynLib_INCLUDE_DIRS}
					          ${ctrlLib_INCLUDE_DIRS}
					          ${YARP_INCLUDE_DIRS}
					          ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES}
                                     ${iDynTree_LIBRARIES}
                                     ${wholeBodyInterface_LIBRARIES}
                                     ${yarpWholeBodyInterface_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)
