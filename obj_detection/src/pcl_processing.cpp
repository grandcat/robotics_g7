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

/*
 *  New overall TODO for recognition (PCL):
 *  - (done) Create new class of point processing
 *  - Take several frames (like 3), remove walls, extract all objects using Euclidean approach
 *  - Remove noise by comparing middle point of objects and search for close objects to these points
 *  --> if not exist: probably noise
 *  - Trying matching: http://www.pointclouds.org/documentation/tutorials/template_alignment.php
 *
 *  - also cut sample pcd files (remove all background) !
 */

#define VOXEL_LEAF_SIZE 0.005

using namespace std;

ros::Subscriber sub_pcl_primesense;
ros::Publisher pub_pcl_filtered;

/**
 * @brief process_pcl_data
 * @param pc_raw
 */
//void process_pcl_data(const sensor_msgs::PointCloud2ConstPtr& pc_raw)
//{
//  // Reduce visible points to close view
////  pcl::PassThrough<pcl::PointXYZ> pass;
////  pass.setInputCloud(pclCloud);
////  pass.setFilterFieldName("z");
////  pass.setFilterLimits(0.0, 1.0);
////  pass.filter(*cloud_filtered);

//  // Reduce view 100cm to the front (Z axes) and downsample amount of points
//  sensor_msgs::PointCloud2 cloudVoxel;
//  pcl::VoxelGrid<sensor_msgs::PointCloud2> pclVoxelFilter;
//  pclVoxelFilter.setInputCloud(pc_raw);
//  pclVoxelFilter.setFilterFieldName("z");
//  pclVoxelFilter.setFilterLimits(0.0, 1.0);  //< only take XYZ points maximum 1m away from camera on z axis
//  pclVoxelFilter.setLeafSize(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
//  pclVoxelFilter.filter(cloudVoxel);

//  // Convert to PCL data representation for more advanced treatment of Pointclouds
//  pcl::PointCloud<pcl::PointXYZ>::Ptr pclFiltered(new pcl::PointCloud<pcl::PointXYZ>),
//      pclProcessed(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::fromROSMsg(cloudVoxel, *pclFiltered);

//  ROS_INFO("Points filtered: amount->%i", pclFiltered->width * pclFiltered->height);

////  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoiseFree(new pcl::PointCloud<pcl::PointXYZ>);
////  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> pclOutlierFilter;
////  pclOutlierFilter.setInputCloud(cloud_filtered);
////  pclOutlierFilter.setMeanK(50);
////  pclOutlierFilter.setStddevMulThresh(1.0);
////  pclOutlierFilter.filter(*cloudNoiseFree);

//  // Determine plane surfaces (only vertical walls)
//  Eigen::Vector3f axisBackPlane = Eigen::Vector3f(0.0, 0.0, 1.0);

//  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//  pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
//  pcl::SACSegmentation<pcl::PointXYZ> pclSegmentation;
//  pclSegmentation.setInputCloud(pclFiltered);
//  pclSegmentation.setOptimizeCoefficients(true);
//  pclSegmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
//  pclSegmentation.setMethodType(pcl::SAC_RANSAC);
//  pclSegmentation.setAxis(axisBackPlane);     // only check for
//  pclSegmentation.setEpsAngle(15.0f * (M_PI/180.0f));
//  pclSegmentation.setDistanceThreshold(0.02); // how close point must be to object to be inlier
//  pclSegmentation.setMaxIterations(1000);
//  pclSegmentation.segment(*inlierIndices, *coefficients); //< TODO: nothing detected, take another frame
//  // Remove points of possible walls
//  pcl::ExtractIndices<pcl::PointXYZ> pclObstacle;
//  pclObstacle.setInputCloud(pclFiltered);
//  pclObstacle.setIndices(inlierIndices);
//  pclObstacle.setNegative(true);    // inverse: remove walls
//  pclObstacle.filter(*pclProcessed);

//  // Generate debugging output for rViz
//  sensor_msgs::PointCloud2 output;
//  pcl::toROSMsg(*pclProcessed, output);
//  pub_pcl_filtered.publish(output);
//}


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


