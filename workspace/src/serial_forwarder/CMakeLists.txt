cmake_minimum_required(VERSION 2.8.3)
project(serial_forwarder)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS genmsg roscpp actionlib_msgs actionlib std_msgs geometry_msgs utilities)
find_package(Boost REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES utilities
        CATKIN_DEPENDS std_msgs
)

include_directories(
        include
        ${utilities_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp src/Forwarder.cpp src/Publisher.cpp src/SerialPort.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${utilities_LIBRARIES})