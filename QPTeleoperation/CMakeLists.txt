# Copyright: (C) 2017 Inria
# Author: Serena Ivaldi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.12)
set(PROJECTNAME icub-teleoperation)
project(${PROJECTNAME})
set(CMAKE_CXX_STANDARD 11)
set(CXX_STANDARD_REQUIRED YES)
set(CXX_EXTENSIONS NO)

find_package(YARP REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

include(YarpInstallationHelpers)


add_subdirectory(src)
add_subdirectory(app)

