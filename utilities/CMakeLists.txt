cmake_minimum_required(VERSION 3.10)
project(Utilities)

set(CMAKE_CXX_STANDARD 17)
find_package(Threads)
include_directories(include)

add_library(Utilities src/Logger.cpp src/LogToCout.cpp src/LogToFile.cpp src/IniParser.cpp)
target_link_libraries(Utilities ${CMAKE_THREAD_LIBS_INIT})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
