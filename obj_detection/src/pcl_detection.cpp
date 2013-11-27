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
#include <Eigen/Core>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
// PCL: normals & feature
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#define VOXEL_LEAF_SIZE 0.005

namespace objRecognition
{

/*
 *
 * Class ModelCloud
 *
 */
void FeatureCloud::computeNormals()
{
  surfaceNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEstimation;
  normEstimation.setInputCloud(objModel);
  normEstimation.setSearchMethod(kdsearchTree);
  normEstimation.setRadiusSearch(0.02f);
  normEstimation.compute(*surfaceNormals);
}

void FeatureCloud::computeFeatureDescriptor()
{
  objFeatures = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> featureDesc;
  featureDesc.setInputCloud(objModel);
  featureDesc.setInputNormals(surfaceNormals);
  featureDesc.setSearchMethod(kdsearchTree);
  featureDesc.setRadiusSearch(0.02f);
  featureDesc.compute(*objFeatures);
}


/*
 *
 * Class ImageFetchSmooth
 *
 */
void ImageFetchSmooth::rcvPointCloud(const sensor_msgs::PointCloud2ConstPtr &pc_raw)
{
  if (!processingActive)  // TODO: replace by shutdown subscribtion when not needed
    {
      ROS_INFO("Ignore pointcloud frame (not started!)");

      return;
    }
  // Only process every 30th frame (TODO: improve)
  ++cWaitFrames;
  cWaitFrames %= 30;
  if (cWaitFrames != 0)
    {
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
  pclFiltered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclProcessed(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloudVoxel, *pclFiltered);
  ROS_INFO("Points filtered: amount->%i", pclFiltered->width * pclFiltered->height);

  // TESTING
  static FeatureCloud objModel;
  compareModelWithScene(objModel);

//  //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoiseFree(new pcl::PointCloud<pcl::PointXYZ>);
//  //  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> pclOutlierFilter;
//  //  pclOutlierFilter.setInputCloud(cloud_filtered);
//  //  pclOutlierFilter.setMeanK(50);
//  //  pclOutlierFilter.setStddevMulThresh(1.0);
//  //  pclOutlierFilter.filter(*cloudNoiseFree);

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
//  pclSegmentation.setEpsAngle(12.0f * (M_PI/180.0f));
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
//  pcl::toROSMsg(*pclFiltered, output);
//  pub_pcl_filtered.publish(output);
}

void ImageFetchSmooth::compareModelWithScene(FeatureCloud& model)
{
  // Params of Sample Consensus Intial Alignment algorithm
  sacIA.setMinSampleDistance(0.05f);
  sacIA.setMaximumIterations(500);
  sacIA.setMaxCorrespondenceDistance(0.001f);

  // Create target scene
  FeatureCloud featureTarget;
  featureTarget.setInputCloud(pclFiltered);
  sacIA.setInputTarget(featureTarget.getObjModel());
  sacIA.setTargetFeatures(featureTarget.getObjFeatures());

  // TESTING (TODO: remove static, several obstacle pcls have to be compared to target pcl)
  static FeatureCloud featureObstacle;
  featureObstacle.loadPcdFile("pcl_objects_cut/pcd_objects/corn/0.pcd");
  sacIA.setInputCloud(featureObstacle.getObjModel());
  sacIA.setSourceFeatures(featureObstacle.getObjFeatures());

  pcl::PointCloud<pcl::PointXYZ> outputAlignResult;
  sacIA.align(outputAlignResult);
  float alignScore = sacIA.getFitnessScore(0.0001f);
  Eigen::Matrix4f resTransform = sacIA.getFinalTransformation();

  // DEBUG Output
  Eigen::Matrix3f rotation = resTransform.block<3,3>(0, 0);
  Eigen::Vector3f translation = resTransform.block<3,1>(0, 3);

  printf ("Align score: %f\n", alignScore);
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

  pcl::PointCloud<pcl::PointXYZ> pclTransformed;
//  Eigen::Matrix4f baseTrans;
  pcl::transformPointCloud(*(featureObstacle.getObjModel()), pclTransformed, resTransform);

  // Generate debugging output for rViz
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pclTransformed, output);
  output.header.frame_id = pclFiltered->header.frame_id;
  pub_pcl_filtered.publish(output);
}

} // END Namespace objRecognition
