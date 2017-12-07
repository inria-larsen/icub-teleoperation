# YARP needs CMake 3.0 or greater
cmake_minimum_required(VERSION 3.0)

set(PROJECTNAME comMapping)
project(${PROJECTNAME})

find_package(iDynTree REQUIRED)

set(folder_source comMapping.cpp)
source_group("Source Files" FILES ${folder_source})
add_executable(${PROJECT_NAME} ${folder_source})

target_include_directories(${PROJECT_NAME} PUBLIC ${YARP_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES}
                                     ${iDynTree_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)
