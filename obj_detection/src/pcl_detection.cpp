/*
 *  pcl_detection.cpp
 *
 *  Get frame sequence from Primesense, remove unnecessary objects (walls) and try removing noise and compare
 *  object pcd examples with current depth image
 */

#include "headers/pcl_detection.hpp"

// Point cloud: downsampling & cleaning noise
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#define VOXEL_LEAF_SIZE 0.005

namespace objRecognition
{

void ImageFetchSmooth::rcvPointCloud(const sensor_msgs::PointCloud2ConstPtr &pc_raw)
{
  if (!processingActive)
    {
      ROS_INFO("Ignore pointcloud frame (not started!)");

      return;
    }

  // Reduce view 100cm to the front (Z axes) and downsample amount of points
  sensor_msgs::PointCloud2 cloudVoxel;
  pcl::VoxelGrid<sensor_msgs::PointCloud2> pclVoxelFilter;
  pclVoxelFilter.setInputCloud(pc_raw);
  pclVoxelFilter.setFilterFieldName("z");
  pclVoxelFilter.setFilterLimits(0.0, 1.0);  //< only take XYZ points maximum 1m away from camera on z axis
  pclVoxelFilter.setLeafSize(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
  pclVoxelFilter.filter(cloudVoxel);

  // Convert to PCL data representation for more advanced treatment of Pointclouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclFiltered(new pcl::PointCloud<pcl::PointXYZ>),
      pclProcessed(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloudVoxel, *pclFiltered);

  ROS_INFO("Points filtered: amount->%i", pclFiltered->width * pclFiltered->height);

  //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoiseFree(new pcl::PointCloud<pcl::PointXYZ>);
  //  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> pclOutlierFilter;
  //  pclOutlierFilter.setInputCloud(cloud_filtered);
  //  pclOutlierFilter.setMeanK(50);
  //  pclOutlierFilter.setStddevMulThresh(1.0);
  //  pclOutlierFilter.filter(*cloudNoiseFree);

  // Determine plane surfaces (only vertical walls)
  Eigen::Vector3f axisBackPlane = Eigen::Vector3f(0.0, 0.0, 1.0);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> pclSegmentation;
  pclSegmentation.setInputCloud(pclFiltered);
  pclSegmentation.setOptimizeCoefficients(true);
  pclSegmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  pclSegmentation.setMethodType(pcl::SAC_RANSAC);
  pclSegmentation.setAxis(axisBackPlane);     // only check for
  pclSegmentation.setEpsAngle(12.0f * (M_PI/180.0f));
  pclSegmentation.setDistanceThreshold(0.02); // how close point must be to object to be inlier
  pclSegmentation.setMaxIterations(1000);
  pclSegmentation.segment(*inlierIndices, *coefficients); //< TODO: nothing detected, take another frame
  // Remove points of possible walls
  pcl::ExtractIndices<pcl::PointXYZ> pclObstacle;
  pclObstacle.setInputCloud(pclFiltered);
  pclObstacle.setIndices(inlierIndices);
  pclObstacle.setNegative(true);    // inverse: remove walls
  pclObstacle.filter(*pclProcessed);

  // Generate debugging output for rViz
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*pclProcessed, output);
  pub_pcl_filtered.publish(output);
}

}
