# Install script for directory: /home/waldezjr/icub/software/src/icub-teleoperation/app

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/waldezjr/icub/software")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yarp/contexts" TYPE FILE FILES
    "/home/waldezjr/icub/software/src/icub-teleoperation/app/conf/retargetingIcub.ini"
    "/home/waldezjr/icub/software/src/icub-teleoperation/app/conf/MpcSolver.ini"
    "/home/waldezjr/icub/software/src/icub-teleoperation/app/conf/LIP_corrector.ini"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yarp/applications" TYPE FILE FILES "/home/waldezjr/icub/software/src/icub-teleoperation/app/scripts/icub-teleoperation.xml.template")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yarp/applications" TYPE FILE FILES
    "/home/waldezjr/icub/software/src/icub-teleoperation/app/scripts/teleop-dumperNancy.xml"
    "/home/waldezjr/icub/software/src/icub-teleoperation/app/scripts/teleop-dumper.xml"
    "/home/waldezjr/icub/software/src/icub-teleoperation/app/scripts/icub-teleoperation.xml"
    )
endif()

