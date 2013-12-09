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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(PCL 1.5 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
# Boost for tools (e.x. point cloud cutter)
rosbuild_add_boost_directories()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(test_depth src/test.cpp)
rosbuild_add_executable(pcl_processing src/pcl_processing.cpp src/pcl_detection.cpp src/headers/conditional_euclidean_clustering.hpp)
target_link_libraries(pcl_processing ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
rosbuild_add_executable(recognition_master src/recognition_master.cpp src/contour_filter.cpp) 
# src/color_filter.cpp)
rosbuild_add_executable(tools_pcl_objects_cutter src/tools_pcl_object_cutter.cpp)
rosbuild_link_boost(tools_pcl_objects_cutter filesystem system)
