# Copyright: (C) 2017 Inria
# Author: Luigi Penco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# YARP needs CMake 3.0 or greater
cmake_minimum_required(VERSION 2.8.12)

add_compile_options(-std=c++11)

set(PROJECTNAME LIP_corrector)
project(${PROJECTNAME})


#.rst:
# FindqpOASES
# -----------
#
# Try to find the qpOASES library.
# Once done this will define the following variables::
#
#  qpOASES_FOUND         - System has qpOASES
#  qpOASES_INCLUDE_DIRS  - qpOASES include directory
#  qpOASES_LIBRARIES     - qpOASES libraries
#
# qpOASES does not have an "install" step, and the includes are in the source
# tree, while the libraries are in the build tree.
# Therefore the environment and cmake variables `qpOASES_SOURCE_DIR` and
# `qpOASES_BINARY_DIR` will be used to locate the includes and libraries.

include(FindPackageHandleStandardArgs)

find_path(qpOASES_INCLUDEDIR
          NAMES qpOASES.hpp
          HINTS "${qpOASES_SOURCE_DIR}"
                ENV qpOASES_SOURCE_DIR
          PATH_SUFFIXES include)
find_library(qpOASES_LIB
             NAMES qpOASES
             HINTS "${qpOASES_BINARY_DIR}"
                   ENV qpOASES_BINARY_DIR
             PATH_SUFFIXES lib
                           libs)

set(qpOASES_INCLUDE_DIRS ${qpOASES_INCLUDEDIR})
set(qpOASES_LIBRARIES ${qpOASES_LIB})

find_package_handle_standard_args(qpOASES DEFAULT_MSG qpOASES_LIBRARIES
                                                      qpOASES_INCLUDE_DIRS)
set(qpOASES_FOUND ${QPOASES_FOUND})
message(${QPOASES_FOUND})

file(GLOB folder_source ./*.cpp)
source_group("Source Files" FILES ${folder_source})
add_executable(${PROJECT_NAME} ${folder_source})

target_include_directories(${PROJECT_NAME} PUBLIC ${YARP_INCLUDE_DIRS}
					          ${EIGEN3_INCLUDE_DIR}
						  ${qpOASES_INCLUDE_DIRS})
message(${qpOASES_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${qpOASES_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)
