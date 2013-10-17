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
CMAKE_SOURCE_DIR = /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot

# Include any dependencies generated for this target.
include CMakeFiles/controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/controller.dir/flags.make

CMakeFiles/controller.dir/src/controller.o: CMakeFiles/controller.dir/flags.make
CMakeFiles/controller.dir/src/controller.o: src/controller.cpp
CMakeFiles/controller.dir/src/controller.o: manifest.xml
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /home/robo/DD2425_2013/differential_drive/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/share/visualization_msgs/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/controller.dir/src/controller.o: /home/robo/DD2425_2013/differential_drive/msg_gen/generated
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/controller.dir/src/controller.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/controller.dir/src/controller.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/controller.dir/src/controller.o -c /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot/src/controller.cpp

CMakeFiles/controller.dir/src/controller.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controller.dir/src/controller.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot/src/controller.cpp > CMakeFiles/controller.dir/src/controller.i

CMakeFiles/controller.dir/src/controller.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controller.dir/src/controller.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot/src/controller.cpp -o CMakeFiles/controller.dir/src/controller.s

CMakeFiles/controller.dir/src/controller.o.requires:
.PHONY : CMakeFiles/controller.dir/src/controller.o.requires

CMakeFiles/controller.dir/src/controller.o.provides: CMakeFiles/controller.dir/src/controller.o.requires
	$(MAKE) -f CMakeFiles/controller.dir/build.make CMakeFiles/controller.dir/src/controller.o.provides.build
.PHONY : CMakeFiles/controller.dir/src/controller.o.provides

CMakeFiles/controller.dir/src/controller.o.provides.build: CMakeFiles/controller.dir/src/controller.o

# Object files for target controller
controller_OBJECTS = \
"CMakeFiles/controller.dir/src/controller.o"

# External object files for target controller
controller_EXTERNAL_OBJECTS =

bin/controller: CMakeFiles/controller.dir/src/controller.o
bin/controller: CMakeFiles/controller.dir/build.make
bin/controller: CMakeFiles/controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/controller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/controller.dir/build: bin/controller
.PHONY : CMakeFiles/controller.dir/build

CMakeFiles/controller.dir/requires: CMakeFiles/controller.dir/src/controller.o.requires
.PHONY : CMakeFiles/controller.dir/requires

CMakeFiles/controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/controller.dir/clean

CMakeFiles/controller.dir/depend:
	cd /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot/CMakeFiles/controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/controller.dir/depend

