/*
 * pcl_processing.cpp
 *
 *  Created on: Nov 11, 2013
 *      Author: robo
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

//  // Create a ROS subscriber for thle input point cloud
//  sub_pcl_primesense = nh.subscribe("/camera/depth_registered/points", 1, process_pcl_data);
//  pub_pcl_filtered = nh.advertise<sensor_msgs::PointCloud2>("/camera/filtered_points", 1);
//  ROS_INFO("Subscribed to depth pointcloud.");


  ros::spin();
  return 0;
}


