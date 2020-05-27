cmake_minimum_required(VERSION 2.8.3)
project(tui)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS genmsg roscpp actionlib_msgs actionlib std_msgs geometry_msgs message_generation ${MESSAGE_DEPENDENCIES})
find_package(Boost REQUIRED)
find_package(utilities REQUIRED)
add_action_files(DIRECTORY action FILES PickUpObject.action)


## Generate messages in the 'msg' folder
add_message_files(
        FILES
        Shape.msg
)

## Generate services in the 'srv' folder
add_service_files(
        FILES
        ShapeFinderService.srv

)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS message_runtime actionlib_msgs std_msgs ${MESSAGE_DEPENDENCIES}
)

include_directories(
        include
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${utilities_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp src/Application.cpp src/DevelopMode.cpp src/Mode.cpp src/Communicator.cpp src/FindShapeMode.cpp src/MainMenu.cpp src/SinglePositionMode.cpp src/SequenceMode.cpp src/ChooseShapeMode.cpp)
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} ${utilities_LIBRARIES} pthread)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)