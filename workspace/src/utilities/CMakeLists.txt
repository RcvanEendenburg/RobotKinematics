cmake_minimum_required(VERSION 3.10)
project(utilities)

set(CMAKE_CXX_STANDARD 17)
find_package(Threads)
include_directories(include)

add_library(${PROJECT_NAME} src/Logger.cpp src/LogToCout.cpp src/LogToFile.cpp src/IniParser.cpp)
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES})

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION lib ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION bin ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include ${CMAKE_INSTALL_INCLUDEDIR})