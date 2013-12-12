/*
 * pcl_processing.cpp
 *
 *  Created on: Nov 11, 2013
 *      Author: Stefan
 */

#include "headers/pcl_processing.hpp"
#include "headers/pcl_detection.hpp"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
// Point Cloud: base
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Point cloud: downsampling & cleaning noise
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_processing");
  ros::NodeHandle nh;

  ROS_INFO("Starting pcl converter.");

  objRecognition::PclRecognition processPipeline(nh);
  processPipeline.start();

  ros::spin();
  return 0;
}
