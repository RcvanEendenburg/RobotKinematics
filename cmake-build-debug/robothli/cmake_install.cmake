# Install script for directory: /home/derk/workspace/src/robothli

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robothli/action" TYPE FILE FILES
    "/home/derk/workspace/src/robothli/action/PickUpObject.action"
    "/home/derk/workspace/src/robothli/action/MoveRobot.action"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robothli/msg" TYPE FILE FILES
    "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectAction.msg"
    "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionGoal.msg"
    "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionResult.msg"
    "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectActionFeedback.msg"
    "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectGoal.msg"
    "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectResult.msg"
    "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/PickUpObjectFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robothli/msg" TYPE FILE FILES
    "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotAction.msg"
    "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionGoal.msg"
    "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionResult.msg"
    "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotActionFeedback.msg"
    "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotGoal.msg"
    "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotResult.msg"
    "/home/derk/workspace/src/cmake-build-debug/devel/share/robothli/msg/MoveRobotFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robothli/cmake" TYPE FILE FILES "/home/derk/workspace/src/cmake-build-debug/robothli/catkin_generated/installspace/robothli-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/derk/workspace/src/cmake-build-debug/devel/include/robothli")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/derk/workspace/src/cmake-build-debug/devel/share/common-lisp/ros/robothli")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/derk/workspace/src/cmake-build-debug/devel/lib/python2.7/dist-packages/robothli")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/derk/workspace/src/cmake-build-debug/devel/lib/python2.7/dist-packages/robothli")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/derk/workspace/src/cmake-build-debug/robothli/catkin_generated/installspace/robothli.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robothli/cmake" TYPE FILE FILES "/home/derk/workspace/src/cmake-build-debug/robothli/catkin_generated/installspace/robothli-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robothli/cmake" TYPE FILE FILES
    "/home/derk/workspace/src/cmake-build-debug/robothli/catkin_generated/installspace/robothliConfig.cmake"
    "/home/derk/workspace/src/cmake-build-debug/robothli/catkin_generated/installspace/robothliConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robothli" TYPE FILE FILES "/home/derk/workspace/src/robothli/package.xml")
endif()

