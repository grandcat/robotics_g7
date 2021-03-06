cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
rosbuild_add_executable(controller src/controller.cpp)
rosbuild_add_executable(odometry src/odometry.cpp)
rosbuild_add_executable(keyboard src/keyboard.cpp)
rosbuild_add_executable(keyboardVW src/keyboardVW.cpp)
rosbuild_add_executable(motors src/motors.cpp)
rosbuild_add_executable(motorsVW src/motorsVW.cpp)
rosbuild_add_executable(wall_follower src/wall_follower.cpp)
rosbuild_add_executable(wall_followerVW src/wall_followerVW.cpp)
rosbuild_add_executable(wall_followerVW2 src/wall_followerVW2.cpp)
rosbuild_add_executable(EKF src/EKF.cpp)
rosbuild_add_executable(EKF2 src/EKF2.cpp)
rosbuild_add_executable(IR src/IR.cpp)
rosbuild_add_executable(phidgets src/phidgets.cpp)
#target_link_libraries(example ${PROJECT_NAME})
