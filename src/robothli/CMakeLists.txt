cmake_minimum_required(VERSION 2.8.3)
project(robothli)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS genmsg roscpp actionlib_msgs actionlib std_msgs geometry_msgs)
find_package(Boost REQUIRED)
add_action_files(DIRECTORY action FILES PickUpObject.action MoveRobot.action)


generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS message_runtime actionlib_msgs std_msgs
)

include_directories(
        include
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp src/Communicator.cpp src/Application.cpp src/KinematicChain.cpp src/Al5D.cpp src/Joint.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

catkin_add_gtest(Point-test test/Point_test.cpp)
catkin_add_gtest(Joint-test test/Joint_test.cpp src/Joint.cpp)
catkin_add_gtest(KinematicChain-test test/KinematicChain_test.cpp src/KinematicChain.cpp src/Joint.cpp)
catkin_add_gtest(Pose-test test/Pose_test.cpp src/KinematicChain.cpp src/Joint.cpp)
catkin_add_gtest(Matrix-test test/Matrix_test.cpp)
catkin_add_gtest(GradientDescent-test test/GradientDescent_test.cpp)
