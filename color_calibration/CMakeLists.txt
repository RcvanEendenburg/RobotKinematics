cmake_minimum_required(VERSION 2.8.3)
project(color_calibration)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        utilities
        )

find_package( OpenCV 3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
)

include_directories(
        INCLUDE_DIRS include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${utilities_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/ColorCalibration.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        utilities
        )
