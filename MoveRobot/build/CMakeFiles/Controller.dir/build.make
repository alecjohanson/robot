# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robo/ros_workspace/MoveRobot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robo/ros_workspace/MoveRobot/build

# Include any dependencies generated for this target.
include CMakeFiles/Controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Controller.dir/flags.make

CMakeFiles/Controller.dir/src/controller.o: CMakeFiles/Controller.dir/flags.make
CMakeFiles/Controller.dir/src/controller.o: ../src/controller.cpp
CMakeFiles/Controller.dir/src/controller.o: ../manifest.xml
CMakeFiles/Controller.dir/src/controller.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/Controller.dir/src/controller.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/Controller.dir/src/controller.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/Controller.dir/src/controller.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/Controller.dir/src/controller.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/Controller.dir/src/controller.o: /home/robo/DD2425_2013/differential_drive/manifest.xml
CMakeFiles/Controller.dir/src/controller.o: /home/robo/DD2425_2013/differential_drive/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robo/ros_workspace/MoveRobot/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Controller.dir/src/controller.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/Controller.dir/src/controller.o -c /home/robo/ros_workspace/MoveRobot/src/controller.cpp

CMakeFiles/Controller.dir/src/controller.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controller.dir/src/controller.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/robo/ros_workspace/MoveRobot/src/controller.cpp > CMakeFiles/Controller.dir/src/controller.i

CMakeFiles/Controller.dir/src/controller.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controller.dir/src/controller.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/robo/ros_workspace/MoveRobot/src/controller.cpp -o CMakeFiles/Controller.dir/src/controller.s

CMakeFiles/Controller.dir/src/controller.o.requires:
.PHONY : CMakeFiles/Controller.dir/src/controller.o.requires

CMakeFiles/Controller.dir/src/controller.o.provides: CMakeFiles/Controller.dir/src/controller.o.requires
	$(MAKE) -f CMakeFiles/Controller.dir/build.make CMakeFiles/Controller.dir/src/controller.o.provides.build
.PHONY : CMakeFiles/Controller.dir/src/controller.o.provides

CMakeFiles/Controller.dir/src/controller.o.provides.build: CMakeFiles/Controller.dir/src/controller.o

# Object files for target Controller
Controller_OBJECTS = \
"CMakeFiles/Controller.dir/src/controller.o"

# External object files for target Controller
Controller_EXTERNAL_OBJECTS =

../bin/Controller: CMakeFiles/Controller.dir/src/controller.o
../bin/Controller: CMakeFiles/Controller.dir/build.make
../bin/Controller: CMakeFiles/Controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/Controller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Controller.dir/build: ../bin/Controller
.PHONY : CMakeFiles/Controller.dir/build

CMakeFiles/Controller.dir/requires: CMakeFiles/Controller.dir/src/controller.o.requires
.PHONY : CMakeFiles/Controller.dir/requires

CMakeFiles/Controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Controller.dir/clean

CMakeFiles/Controller.dir/depend:
	cd /home/robo/ros_workspace/MoveRobot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robo/ros_workspace/MoveRobot /home/robo/ros_workspace/MoveRobot /home/robo/ros_workspace/MoveRobot/build /home/robo/ros_workspace/MoveRobot/build /home/robo/ros_workspace/MoveRobot/build/CMakeFiles/Controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Controller.dir/depend

