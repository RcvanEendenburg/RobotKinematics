cmake_minimum_required(VERSION 2.8.3)
project(world)
 add_compile_options(-std=c++17)

find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
        ${MESSAGE_DEPENDENCIES}
        )

find_package( OpenCV 3 REQUIRED)

## Generate messages in the 'msg' folder
 add_message_files(
         FILES
  Point2d.msg
   Shape.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
         ShapeFinderService.srv

 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
   std_msgs # Or other packages containing msgs
 )

catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDENCIES}
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        INCLUDE_DIRS include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/world.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(${PROJECT_NAME}_node src/WorldService.cpp
                                     src/ShapeFinder/ShapeFinder.cpp
                                     src/Color/ColorFilter.cpp
                                     src/WorldService.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
 )
