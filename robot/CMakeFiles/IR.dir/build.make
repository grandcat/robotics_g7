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
include CMakeFiles/IR.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/IR.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/IR.dir/flags.make

CMakeFiles/IR.dir/src/IR.o: CMakeFiles/IR.dir/flags.make
CMakeFiles/IR.dir/src/IR.o: src/IR.cpp
CMakeFiles/IR.dir/src/IR.o: manifest.xml
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /home/robo/DD2425_2013/differential_drive/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/share/visualization_msgs/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/IR.dir/src/IR.o: /home/robo/DD2425_2013/differential_drive/msg_gen/generated
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/IR.dir/src/IR.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/IR.dir/src/IR.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/IR.dir/src/IR.o -c /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot/src/IR.cpp

CMakeFiles/IR.dir/src/IR.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/IR.dir/src/IR.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot/src/IR.cpp > CMakeFiles/IR.dir/src/IR.i

CMakeFiles/IR.dir/src/IR.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/IR.dir/src/IR.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot/src/IR.cpp -o CMakeFiles/IR.dir/src/IR.s

CMakeFiles/IR.dir/src/IR.o.requires:
.PHONY : CMakeFiles/IR.dir/src/IR.o.requires

CMakeFiles/IR.dir/src/IR.o.provides: CMakeFiles/IR.dir/src/IR.o.requires
	$(MAKE) -f CMakeFiles/IR.dir/build.make CMakeFiles/IR.dir/src/IR.o.provides.build
.PHONY : CMakeFiles/IR.dir/src/IR.o.provides

CMakeFiles/IR.dir/src/IR.o.provides.build: CMakeFiles/IR.dir/src/IR.o

# Object files for target IR
IR_OBJECTS = \
"CMakeFiles/IR.dir/src/IR.o"

# External object files for target IR
IR_EXTERNAL_OBJECTS =

bin/IR: CMakeFiles/IR.dir/src/IR.o
bin/IR: CMakeFiles/IR.dir/build.make
bin/IR: CMakeFiles/IR.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/IR"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/IR.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/IR.dir/build: bin/IR
.PHONY : CMakeFiles/IR.dir/build

CMakeFiles/IR.dir/requires: CMakeFiles/IR.dir/src/IR.o.requires
.PHONY : CMakeFiles/IR.dir/requires

CMakeFiles/IR.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/IR.dir/cmake_clean.cmake
.PHONY : CMakeFiles/IR.dir/clean

CMakeFiles/IR.dir/depend:
	cd /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot /home/robo/DD2425_2013/fuerte_workspace/robotics_g7/robot/CMakeFiles/IR.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/IR.dir/depend
