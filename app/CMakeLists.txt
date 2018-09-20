# icub-teleoperation
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

file(GLOB scripts  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB scriptsXml  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB conf_ini ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

yarp_install(FILES ${conf_ini} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR})
yarp_install(FILES ${scripts}  DESTINATION ${YARP_APPLICATIONS_INSTALL_DIR})
yarp_install(FILES ${scriptsXml}  DESTINATION ${YARP_APPLICATIONS_INSTALL_DIR})

