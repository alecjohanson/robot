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
CMAKE_SOURCE_DIR = /home/robo/ros_workspace/robot/robot0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robo/ros_workspace/robot/robot0

# Include any dependencies generated for this target.
include CMakeFiles/milestone0.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/milestone0.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/milestone0.dir/flags.make

CMakeFiles/milestone0.dir/milestone0.o: CMakeFiles/milestone0.dir/flags.make
CMakeFiles/milestone0.dir/milestone0.o: milestone0.cpp
CMakeFiles/milestone0.dir/milestone0.o: manifest.xml
CMakeFiles/milestone0.dir/milestone0.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/milestone0.dir/milestone0.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/milestone0.dir/milestone0.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/milestone0.dir/milestone0.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/milestone0.dir/milestone0.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/milestone0.dir/milestone0.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/milestone0.dir/milestone0.o: /home/robo/DD2425_2013/differential_drive/manifest.xml
CMakeFiles/milestone0.dir/milestone0.o: /home/robo/DD2425_2013/differential_drive/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robo/ros_workspace/robot/robot0/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/milestone0.dir/milestone0.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/milestone0.dir/milestone0.o -c /home/robo/ros_workspace/robot/robot0/milestone0.cpp

CMakeFiles/milestone0.dir/milestone0.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/milestone0.dir/milestone0.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/robo/ros_workspace/robot/robot0/milestone0.cpp > CMakeFiles/milestone0.dir/milestone0.i

CMakeFiles/milestone0.dir/milestone0.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/milestone0.dir/milestone0.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/robo/ros_workspace/robot/robot0/milestone0.cpp -o CMakeFiles/milestone0.dir/milestone0.s

CMakeFiles/milestone0.dir/milestone0.o.requires:
.PHONY : CMakeFiles/milestone0.dir/milestone0.o.requires

CMakeFiles/milestone0.dir/milestone0.o.provides: CMakeFiles/milestone0.dir/milestone0.o.requires
	$(MAKE) -f CMakeFiles/milestone0.dir/build.make CMakeFiles/milestone0.dir/milestone0.o.provides.build
.PHONY : CMakeFiles/milestone0.dir/milestone0.o.provides

CMakeFiles/milestone0.dir/milestone0.o.provides.build: CMakeFiles/milestone0.dir/milestone0.o

# Object files for target milestone0
milestone0_OBJECTS = \
"CMakeFiles/milestone0.dir/milestone0.o"

# External object files for target milestone0
milestone0_EXTERNAL_OBJECTS =

bin/milestone0: CMakeFiles/milestone0.dir/milestone0.o
bin/milestone0: CMakeFiles/milestone0.dir/build.make
bin/milestone0: CMakeFiles/milestone0.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/milestone0"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/milestone0.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/milestone0.dir/build: bin/milestone0
.PHONY : CMakeFiles/milestone0.dir/build

CMakeFiles/milestone0.dir/requires: CMakeFiles/milestone0.dir/milestone0.o.requires
.PHONY : CMakeFiles/milestone0.dir/requires

CMakeFiles/milestone0.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/milestone0.dir/cmake_clean.cmake
.PHONY : CMakeFiles/milestone0.dir/clean

CMakeFiles/milestone0.dir/depend:
	cd /home/robo/ros_workspace/robot/robot0 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robo/ros_workspace/robot/robot0 /home/robo/ros_workspace/robot/robot0 /home/robo/ros_workspace/robot/robot0 /home/robo/ros_workspace/robot/robot0 /home/robo/ros_workspace/robot/robot0/CMakeFiles/milestone0.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/milestone0.dir/depend

