# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/clion-2019.1.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2019.1.2/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/derk/workspace/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/derk/workspace/src/cmake-build-debug

# Include any dependencies generated for this target.
include robothli/CMakeFiles/Joint-test.dir/depend.make

# Include the progress variables for this target.
include robothli/CMakeFiles/Joint-test.dir/progress.make

# Include the compile flags for this target's objects.
include robothli/CMakeFiles/Joint-test.dir/flags.make

robothli/CMakeFiles/Joint-test.dir/test/Joint_test.cpp.o: robothli/CMakeFiles/Joint-test.dir/flags.make
robothli/CMakeFiles/Joint-test.dir/test/Joint_test.cpp.o: ../robothli/test/Joint_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/derk/workspace/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object robothli/CMakeFiles/Joint-test.dir/test/Joint_test.cpp.o"
	cd /home/derk/workspace/src/cmake-build-debug/robothli && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Joint-test.dir/test/Joint_test.cpp.o -c /home/derk/workspace/src/robothli/test/Joint_test.cpp

robothli/CMakeFiles/Joint-test.dir/test/Joint_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Joint-test.dir/test/Joint_test.cpp.i"
	cd /home/derk/workspace/src/cmake-build-debug/robothli && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/derk/workspace/src/robothli/test/Joint_test.cpp > CMakeFiles/Joint-test.dir/test/Joint_test.cpp.i

robothli/CMakeFiles/Joint-test.dir/test/Joint_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Joint-test.dir/test/Joint_test.cpp.s"
	cd /home/derk/workspace/src/cmake-build-debug/robothli && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/derk/workspace/src/robothli/test/Joint_test.cpp -o CMakeFiles/Joint-test.dir/test/Joint_test.cpp.s

robothli/CMakeFiles/Joint-test.dir/src/Joint.cpp.o: robothli/CMakeFiles/Joint-test.dir/flags.make
robothli/CMakeFiles/Joint-test.dir/src/Joint.cpp.o: ../robothli/src/Joint.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/derk/workspace/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object robothli/CMakeFiles/Joint-test.dir/src/Joint.cpp.o"
	cd /home/derk/workspace/src/cmake-build-debug/robothli && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Joint-test.dir/src/Joint.cpp.o -c /home/derk/workspace/src/robothli/src/Joint.cpp

robothli/CMakeFiles/Joint-test.dir/src/Joint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Joint-test.dir/src/Joint.cpp.i"
	cd /home/derk/workspace/src/cmake-build-debug/robothli && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/derk/workspace/src/robothli/src/Joint.cpp > CMakeFiles/Joint-test.dir/src/Joint.cpp.i

robothli/CMakeFiles/Joint-test.dir/src/Joint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Joint-test.dir/src/Joint.cpp.s"
	cd /home/derk/workspace/src/cmake-build-debug/robothli && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/derk/workspace/src/robothli/src/Joint.cpp -o CMakeFiles/Joint-test.dir/src/Joint.cpp.s

# Object files for target Joint-test
Joint__test_OBJECTS = \
"CMakeFiles/Joint-test.dir/test/Joint_test.cpp.o" \
"CMakeFiles/Joint-test.dir/src/Joint.cpp.o"

# External object files for target Joint-test
Joint__test_EXTERNAL_OBJECTS =

devel/lib/robothli/Joint-test: robothli/CMakeFiles/Joint-test.dir/test/Joint_test.cpp.o
devel/lib/robothli/Joint-test: robothli/CMakeFiles/Joint-test.dir/src/Joint.cpp.o
devel/lib/robothli/Joint-test: robothli/CMakeFiles/Joint-test.dir/build.make
devel/lib/robothli/Joint-test: /usr/lib/x86_64-linux-gnu/libgtest.a
devel/lib/robothli/Joint-test: robothli/CMakeFiles/Joint-test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/derk/workspace/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../devel/lib/robothli/Joint-test"
	cd /home/derk/workspace/src/cmake-build-debug/robothli && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Joint-test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
robothli/CMakeFiles/Joint-test.dir/build: devel/lib/robothli/Joint-test

.PHONY : robothli/CMakeFiles/Joint-test.dir/build

robothli/CMakeFiles/Joint-test.dir/clean:
	cd /home/derk/workspace/src/cmake-build-debug/robothli && $(CMAKE_COMMAND) -P CMakeFiles/Joint-test.dir/cmake_clean.cmake
.PHONY : robothli/CMakeFiles/Joint-test.dir/clean

robothli/CMakeFiles/Joint-test.dir/depend:
	cd /home/derk/workspace/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/derk/workspace/src /home/derk/workspace/src/robothli /home/derk/workspace/src/cmake-build-debug /home/derk/workspace/src/cmake-build-debug/robothli /home/derk/workspace/src/cmake-build-debug/robothli/CMakeFiles/Joint-test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : robothli/CMakeFiles/Joint-test.dir/depend

