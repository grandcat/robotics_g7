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
CMAKE_SOURCE_DIR = /home/robo/DD2425_2013/robotics_g7/simulator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robo/DD2425_2013/robotics_g7/simulator

# Include any dependencies generated for this target.
include CMakeFiles/keyboard.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/keyboard.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/keyboard.dir/flags.make

CMakeFiles/keyboard.dir/src/keyboard.o: CMakeFiles/keyboard.dir/flags.make
CMakeFiles/keyboard.dir/src/keyboard.o: src/keyboard.cpp
CMakeFiles/keyboard.dir/src/keyboard.o: manifest.xml
CMakeFiles/keyboard.dir/src/keyboard.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/keyboard.dir/src/keyboard.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/keyboard.dir/src/keyboard.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/keyboard.dir/src/keyboard.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/keyboard.dir/src/keyboard.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/keyboard.dir/src/keyboard.o: /home/robo/DD2425_2013/differential_drive/manifest.xml
CMakeFiles/keyboard.dir/src/keyboard.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/keyboard.dir/src/keyboard.o: /opt/ros/fuerte/share/visualization_msgs/manifest.xml
CMakeFiles/keyboard.dir/src/keyboard.o: /home/robo/DD2425_2013/differential_drive/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robo/DD2425_2013/robotics_g7/simulator/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/keyboard.dir/src/keyboard.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/keyboard.dir/src/keyboard.o -c /home/robo/DD2425_2013/robotics_g7/simulator/src/keyboard.cpp

CMakeFiles/keyboard.dir/src/keyboard.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/keyboard.dir/src/keyboard.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/robo/DD2425_2013/robotics_g7/simulator/src/keyboard.cpp > CMakeFiles/keyboard.dir/src/keyboard.i

CMakeFiles/keyboard.dir/src/keyboard.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/keyboard.dir/src/keyboard.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/robo/DD2425_2013/robotics_g7/simulator/src/keyboard.cpp -o CMakeFiles/keyboard.dir/src/keyboard.s

CMakeFiles/keyboard.dir/src/keyboard.o.requires:
.PHONY : CMakeFiles/keyboard.dir/src/keyboard.o.requires

CMakeFiles/keyboard.dir/src/keyboard.o.provides: CMakeFiles/keyboard.dir/src/keyboard.o.requires
	$(MAKE) -f CMakeFiles/keyboard.dir/build.make CMakeFiles/keyboard.dir/src/keyboard.o.provides.build
.PHONY : CMakeFiles/keyboard.dir/src/keyboard.o.provides

CMakeFiles/keyboard.dir/src/keyboard.o.provides.build: CMakeFiles/keyboard.dir/src/keyboard.o

# Object files for target keyboard
keyboard_OBJECTS = \
"CMakeFiles/keyboard.dir/src/keyboard.o"

# External object files for target keyboard
keyboard_EXTERNAL_OBJECTS =

bin/keyboard: CMakeFiles/keyboard.dir/src/keyboard.o
bin/keyboard: CMakeFiles/keyboard.dir/build.make
bin/keyboard: CMakeFiles/keyboard.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/keyboard"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/keyboard.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/keyboard.dir/build: bin/keyboard
.PHONY : CMakeFiles/keyboard.dir/build

CMakeFiles/keyboard.dir/requires: CMakeFiles/keyboard.dir/src/keyboard.o.requires
.PHONY : CMakeFiles/keyboard.dir/requires

CMakeFiles/keyboard.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/keyboard.dir/cmake_clean.cmake
.PHONY : CMakeFiles/keyboard.dir/clean

CMakeFiles/keyboard.dir/depend:
	cd /home/robo/DD2425_2013/robotics_g7/simulator && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robo/DD2425_2013/robotics_g7/simulator /home/robo/DD2425_2013/robotics_g7/simulator /home/robo/DD2425_2013/robotics_g7/simulator /home/robo/DD2425_2013/robotics_g7/simulator /home/robo/DD2425_2013/robotics_g7/simulator/CMakeFiles/keyboard.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/keyboard.dir/depend

