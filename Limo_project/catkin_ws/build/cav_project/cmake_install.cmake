# Install script for directory: /home/CodesDell/Desktop/Limo_project/catkin_ws/src/cav_project

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/CodesDell/Desktop/Limo_project/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cav_project/msg" TYPE FILE FILES
    "/home/CodesDell/Desktop/Limo_project/catkin_ws/src/cav_project/msg/ControlInfo.msg"
    "/home/CodesDell/Desktop/Limo_project/catkin_ws/src/cav_project/msg/limo_info_array.msg"
    "/home/CodesDell/Desktop/Limo_project/catkin_ws/src/cav_project/msg/limo_info.msg"
    "/home/CodesDell/Desktop/Limo_project/catkin_ws/src/cav_project/msg/QP_solution.msg"
    "/home/CodesDell/Desktop/Limo_project/catkin_ws/src/cav_project/msg/limo_state.msg"
    "/home/CodesDell/Desktop/Limo_project/catkin_ws/src/cav_project/msg/limo_state_matrix.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cav_project/cmake" TYPE FILE FILES "/home/CodesDell/Desktop/Limo_project/catkin_ws/build/cav_project/catkin_generated/installspace/cav_project-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/CodesDell/Desktop/Limo_project/catkin_ws/devel/include/cav_project")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/CodesDell/Desktop/Limo_project/catkin_ws/devel/share/roseus/ros/cav_project")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/CodesDell/Desktop/Limo_project/catkin_ws/devel/share/common-lisp/ros/cav_project")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/CodesDell/Desktop/Limo_project/catkin_ws/devel/share/gennodejs/ros/cav_project")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/CodesDell/Desktop/Limo_project/catkin_ws/devel/lib/python3/dist-packages/cav_project")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/CodesDell/Desktop/Limo_project/catkin_ws/devel/lib/python3/dist-packages/cav_project")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/CodesDell/Desktop/Limo_project/catkin_ws/build/cav_project/catkin_generated/installspace/cav_project.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cav_project/cmake" TYPE FILE FILES "/home/CodesDell/Desktop/Limo_project/catkin_ws/build/cav_project/catkin_generated/installspace/cav_project-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cav_project/cmake" TYPE FILE FILES
    "/home/CodesDell/Desktop/Limo_project/catkin_ws/build/cav_project/catkin_generated/installspace/cav_projectConfig.cmake"
    "/home/CodesDell/Desktop/Limo_project/catkin_ws/build/cav_project/catkin_generated/installspace/cav_projectConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cav_project" TYPE FILE FILES "/home/CodesDell/Desktop/Limo_project/catkin_ws/src/cav_project/package.xml")
endif()

