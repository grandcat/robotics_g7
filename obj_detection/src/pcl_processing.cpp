/*
 * pcl_processing.cpp
 *
 *  Created on: Nov 11, 2013
 *      Author: robo
 */

#include "headers/pcl_processing.hpp"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
// Point Clouds
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>

using namespace std;

ros::Subscriber sub_pcl_primesense;
ros::Publisher pub_pcl_filtered;

/**
 * @brief process_pcl_data
 * @param plc_raw
 */
void process_pcl_data(const sensor_msgs::PointCloud2ConstPtr& plc_raw)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*plc_raw, *pclCloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // Reduce visible points to close view
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(pclCloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(*cloud_filtered);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_filtered, output);
  pub_pcl_filtered.publish(output);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_processing");
  ros::NodeHandle nh;

  ROS_INFO("Starting pcl converter.");

  // Create a ROS subscriber for thle input point cloud
  sub_pcl_primesense = nh.subscribe("/camera/depth_registered/points", 3, process_pcl_data);
  pub_pcl_filtered = nh.advertise<sensor_msgs::PointCloud2>("/camera/filtered_points", 3);
  ROS_INFO("Subscribed to depth pointcloud.");

  ros::spin();
  return 0;
}


