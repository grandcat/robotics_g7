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
// Point Cloud: base
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Point cloud: downsampling & cleaning noise
//#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>

#define VOXEL_LEAF_SIZE 0.005

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
//  pcl::PassThrough<pcl::PointXYZ> pass;
//  pass.setInputCloud(pclCloud);
//  pass.setFilterFieldName("z");
//  pass.setFilterLimits(0.0, 1.0);
//  pass.filter(*cloud_filtered);

  // Reduce view to the front (Z axes) to 100cm and downsample amount of points
//  sensor_msgs::PointCloud2 cloudVoxel;
  pcl::VoxelGrid<pcl::PointXYZ> pclVoxelFilter;
  pclVoxelFilter.setInputCloud(pclCloud);
  pclVoxelFilter.setFilterFieldName("z");
  pclVoxelFilter.setFilterLimits(0.0, 1.0);  //< only take XYZ points maximum 1m away from camera on z axis
  pclVoxelFilter.setLeafSize(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
//  pclVoxelFilter.filter(*cloud_filtered);

//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoiseFree(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> pclOutlierFilter;
//  pclOutlierFilter.setInputCloud(cloud_filtered);
//  pclOutlierFilter.setMeanK(50);
//  pclOutlierFilter.setStddevMulThresh(1.0);
//  pclOutlierFilter.filter(*cloudNoiseFree);

  // Determine plane surfaces (like walls)
  // TODO: increase performance by Voxel representation with z axes limiation (integrate part above)
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inlierIndices (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> pclSegmentation;
  pclSegmentation.setInputCloud(pclCloud);
  pclSegmentation.setModelType(pcl::SACMODEL_PLANE);
  pclSegmentation.setMethodType(pcl::SAC_RANSAC);
  pclSegmentation.setDistanceThreshold(0.01); //< how close point must be to object to be inlier
  pclSegmentation.setMaxIterations(1000);
  pclSegmentation.segment (*inlierIndices, *coefficients);
  // Remove points of possible walls
  pcl::ExtractIndices<pcl::PointXYZ> pclObstacle;
  pclObstacle.setInputCloud(pclCloud);
  pclObstacle.setIndices(inlierIndices);
  pclObstacle.setNegative(true);    // inverse: remove walls
  pclObstacle.filter(*cloud_filtered);

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
  sub_pcl_primesense = nh.subscribe("/camera/depth_registered/points", 1, process_pcl_data);
  pub_pcl_filtered = nh.advertise<sensor_msgs::PointCloud2>("/camera/filtered_points", 1);
  ROS_INFO("Subscribed to depth pointcloud.");

  ros::spin();
  return 0;
}


