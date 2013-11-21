/*
 * pcl_processing.cpp
 *
 *  Created on: Nov 11, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <pcl_ros/point_cloud.h>
// Point Cloud: base
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sstream>

const int maxPcdID = 20;
const std::string dstDir = "pcl_objects_cut/";

void extractCloudOfPCD(const std::string pSrcPath)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcRaw(new pcl::PointCloud<pcl::PointXYZRGB>);
  // Read point cloud into local memory
  if (pcl::io::loadPCDFile(pSrcPath, *pcRaw) == -1)   //< PCD file could not be loaded
    {
      ROS_ERROR("PCD File %s could not be loaded, aborting!", pSrcPath.c_str());
      return;
    }
  ROS_INFO("Loaded PCL file '%s' with %i * %i points", pSrcPath.c_str(), pcRaw->width, pcRaw->height);

  // Cut pointcloud: remove background
  pcl::PassThrough<pcl::PointXYZRGB> pclPass;
  pclPass.setInputCloud(pcRaw);
  pclPass.setFilterFieldName("z");
  pclPass.setFilterLimits(0.0, 1.0);
  pclPass.filter(*pcRaw);
  // Cut pointcloud: remove fixing of object (bottom)
  pclPass.setInputCloud(pcRaw);
  pclPass.setFilterFieldName("y");
  pclPass.setFilterLimits(-1.0, -0.001);
  pclPass.filter(*pcRaw);
  // Cut pointcloud: remove outliners left and right to the object
  pclPass.setInputCloud(pcRaw);
  pclPass.setFilterFieldName("x");
  pclPass.setFilterLimits(-0.5, 0.18);
  pclPass.filter(*pcRaw);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> pclOutlierFilter;
  pclOutlierFilter.setInputCloud(pcRaw);
  pclOutlierFilter.setMeanK(50);
  pclOutlierFilter.setStddevMulThresh(1.0);
  pclOutlierFilter.filter(*pcRaw);

  // Save processed cloud
  ROS_INFO("PCD cut write to '%s'.", (dstDir + pSrcPath).c_str());
  pcl::io::savePCDFile(dstDir + pSrcPath, *pcRaw, true);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tools_pcl_object_cutter");

  ROS_INFO("Starting pcl object cutter.");

  if (argc == 1)
  {
    ROS_ERROR("Please provide directory / directories for reading .pcl files!");
    return -1;
  }

   // For every directory given as param: create subdirectory with processed pointclouds
  for (int dirID = 1; dirID < argc; ++dirID)
  {
    char path[64];
    for (int i = 0; i <= maxPcdID; ++i)
    {
      sprintf(path, "%s%s", dstDir.c_str(), argv[dirID]);
      // Create directory if not exists
      boost::filesystem::path dir(path);
      if (boost::filesystem::create_directories(path))
        ROS_INFO("Created path: %s", path);

      // Start processing
      sprintf(path, "%s/%i.pcd", argv[dirID], i);
      extractCloudOfPCD(path);
    }
  }


//  ros::spin();
  return 0;
}


