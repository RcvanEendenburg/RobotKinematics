cmake_minimum_required(VERSION 2.8.3)
project(al5d)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS genmsg roscpp actionlib_msgs actionlib std_msgs)
find_package(Boost REQUIRED)
add_action_files(DIRECTORY action FILES MoveRobot.action)


generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS message_runtime actionlib_msgs std_msgs
)

include_directories(
        include
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp src/Communicator.cpp src/Application.cpp src/Joint.cpp src/Controller.cpp
        src/SerialCommunicator.cpp)
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
