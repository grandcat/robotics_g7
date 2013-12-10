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
#include <Eigen/Geometry>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
// PCL: normals & feature
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
// PCL Detetion and extraction
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include "headers/conditional_euclidean_clustering.hpp"   // Copied version of PCL 1-7

#define VOXEL_LEAF_SIZE 0.005

namespace objRecognition
{

const float CLUSTER_DROP_HEIGHT = 0.075;  // Drop elements higher than this cluster center height (e.x. traps)
const float EDGE_MAX_Z_DEPTH = 0.01;

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

/**
 * @brief customEuclideanSegmCond
 *  This function is mainly used as a work around for a bug in the original cluster segmentation of PCL 1.5
 * @param point_a
 * @param point_b
 * @param squared_distance
 * @return
 */
bool
customEuclideanSegmCond(const pcl::PointXYZ& point_a, const pcl::PointXYZ& point_b, float squared_distance)
{
  return true;
}

void PclRecognition::rcvPointCloud(const sensor_msgs::PointCloud2ConstPtr &pc_raw)
{
  if (!processingActive)  // TODO: replace by shutdown subscribtion when not needed
    {
      ROS_INFO_ONCE("Ignore pointcloud frame (not started or stopped!)");
      sub_pcl_primesense.shutdown();
      return;
    }
  // Process and collect obj information during 5 frames to reduce noise
  ++cProcessedFrames;
  cProcessedFrames %= 4;
  if (cProcessedFrames == 0)
    {
      processingActive = false;
      // Process collected information here
      if (lastObjCmPos.empty())
      {
        // no object detected
        explorer::Object pcl_msg;
        pcl_msg.x = -1;
        pub_pcl_position.publish(pcl_msg);
        return;
      }
      // Detected at least 1 object
      Eigen::Vector4f clustCentroid;
      Eigen::Vector4f bestPointCentroid;
      float closestZ = 1.5;
      for (std::vector<Eigen::Vector4f>::const_iterator it = lastObjCmPos.begin(); it != lastObjCmPos.end(); ++it)
      {
        clustCentroid = *it;
        if (clustCentroid[2] < closestZ)
        {
          closestZ = clustCentroid[2];
          bestPointCentroid = clustCentroid;
        }
      }

      explorer::Object pcl_msg;
      pcl_msg.y = bestPointCentroid[0];
      pcl_msg.x = bestPointCentroid[2];
      pub_pcl_position.publish(pcl_msg);

      // Clear old objects
      lastObjCmPos.clear();
      return;
    }

  // Convert to PCL data representation for more advanced treatment of Pointclouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclRaw(new pcl::PointCloud<pcl::PointXYZ>);
  pclFiltered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pc_raw, *pclRaw);

  // Rotate whole point cloud to correct sensor orientation (pose)
  pcl::transformPointCloud(*pclRaw, *pclRaw, cameraPoseTransform);

  // Reduce view 100cm to the front (Z axes) and downsample amount of points
  // Note: bug in VoxelGrid implementation: use different input/output clouds!
  pcl::VoxelGrid<pcl::PointXYZ> pclVoxelFilter;
  pclVoxelFilter.setInputCloud(pclRaw);
  pclVoxelFilter.setFilterFieldName("z");
  pclVoxelFilter.setFilterLimits(0.0, 1.0);  //< only take XYZ points maximum 1m away from camera on z axis
  pclVoxelFilter.setLeafSize(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
  pclVoxelFilter.filter(*pclFiltered);
  ROS_INFO("Points filtered: amount->%i", pclFiltered->width * pclFiltered->height);

  /*
   * Determine plane surfaces and remove
   */
  Eigen::Vector3f axisBottomPlane = Eigen::Vector3f(0.0, 1.0, 0.0); // bottom plane
  Eigen::Vector3f axisBackPlane = Eigen::Vector3f(0.0, 0.0, 1.0);   // back plane

  double segmentThreshold;
  ros::param::param<double>("sac_tre", segmentThreshold, 0.02);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> pclSegmentation;
  pclSegmentation.setInputCloud(pclFiltered);
  pclSegmentation.setOptimizeCoefficients(true);
  pclSegmentation.setModelType(pcl::SACMODEL_PLANE); // pcl::SACMODEL_PERPENDICULAR_PLANE
  pclSegmentation.setMethodType(pcl::SAC_RANSAC);
  pclSegmentation.setAxis(axisBottomPlane);
  pclSegmentation.setEpsAngle(25.0f * (M_PI/180.0f));
  pclSegmentation.setDistanceThreshold(segmentThreshold); // how close point must be to object to be inlier
  pclSegmentation.setMaxIterations(1000);
  pclSegmentation.segment(*inlierIndices, *coefficients); //< TODO: nothing detected, take another frame

  if (!coefficients)
  {
    ROS_WARN("[PCL processing] Couldn't calculate bottom plane coefficient"
             "(due to missing point), aborting PCL recognition for this frame");
    return;
  }
  std::cout << "Bottom plane: Model coefficients: " << coefficients->values[0] << " "
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " "
                                        << coefficients->values[3] << std::endl;
  std::cout << "Model inliers: " << inlierIndices->indices.size () << std::endl;

  // Approximate left and right edge of plane surface
  std::vector<Eigen::Vector3f> edgeLeft(100, Eigen::Vector3f(0, 0, 0)), edgeRight(100, Eigen::Vector3f(0, 0, 0));
  for (size_t i = 0; i < inlierIndices->indices.size(); ++i)
  {
    // Map points to grid: point 0.432 --> 43, point 0.439 --> 43
    int gridPos = pclFiltered->points[inlierIndices->indices[i]].z * 100;

    if (pclFiltered->points[inlierIndices->indices[i]].x < edgeLeft[gridPos][0])
    {
      edgeLeft[gridPos][0] = pclFiltered->points[inlierIndices->indices[i]].x;
      edgeLeft[gridPos][1] = pclFiltered->points[inlierIndices->indices[i]].y;
      edgeLeft[gridPos][2] = pclFiltered->points[inlierIndices->indices[i]].z;
    }
    else if (pclFiltered->points[inlierIndices->indices[i]].x > edgeRight[gridPos][0])
    {
      edgeRight[gridPos][0] = pclFiltered->points[inlierIndices->indices[i]].x;
      edgeRight[gridPos][1] = pclFiltered->points[inlierIndices->indices[i]].y;
      edgeRight[gridPos][2] = pclFiltered->points[inlierIndices->indices[i]].z;
    }
  }
  float planeEdge_mostLeft = filteredMeanfromPlaneEdge(edgeLeft);
  float planeEdge_mostRight = filteredMeanfromPlaneEdge(edgeRight);
  std::cerr << "Most left: " << planeEdge_mostLeft
            << ", Most right: " << planeEdge_mostRight << std::endl;

  // Remove points of side walls
  bool rmBottomPlane;
  ros::param::param<bool>("pcl_remove_bottom", rmBottomPlane, true);
  pcl::ExtractIndices<pcl::PointXYZ> pclExtractIdx;
  pclExtractIdx.setNegative(true);    // inverse: remove walls
  if (rmBottomPlane)
  {
    pclExtractIdx.setInputCloud(pclFiltered);
    pclExtractIdx.setIndices(inlierIndices);
    pclExtractIdx.filter(*pclFiltered);
    std::cerr << "Points left after bottom removal:" << pclFiltered->points.size() << std::endl;
  }

  // Create box which removes everything left and right to bottom plane description
  pcl::CropBox<pcl::PointXYZ> pclCropUnusedArea;
  pclCropUnusedArea.setInputCloud(pclFiltered);
  pclCropUnusedArea.setMin(Eigen::Vector4f(std::min(planeEdge_mostLeft + 0.01f, -0.1f), -0.2, 0.0, 1));    // y: max height of object
  pclCropUnusedArea.setMax(Eigen::Vector4f(std::max(planeEdge_mostRight - 0.02f, 0.1f), 0.03, 1.0, 1));
  pclCropUnusedArea.filter(*pclFiltered);
  std::cerr << "Points left after cropping:" << pclFiltered->points.size() << std::endl;


  // Determine and remove back plane
  pclSegmentation.setInputCloud(pclFiltered);
  pclSegmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  pclSegmentation.setAxis(axisBackPlane);  // back plane
  pclSegmentation.setDistanceThreshold(0.02);
  pclSegmentation.segment(*inlierIndices, *coefficients);
  std::cerr << "Back plane: Model coefficients: " << coefficients->values[0] << " "
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " "
                                        << coefficients->values[3] << std::endl;
//  std::cerr << "Model inliers: " << inlierIndices->indices.size () << std::endl;
  // Only remove if wer're sure it's not an object
  if (inlierIndices->indices.size() > 400 &&
      (pclFiltered->points.size() - inlierIndices->indices.size()) > 30)
  {
    // Remove points of walls: back plane
    pclExtractIdx.setInputCloud(pclFiltered);
    pclExtractIdx.setIndices(inlierIndices);
    pclExtractIdx.filter(*pclFiltered);
    ROS_INFO("Detected backwall: Remove these points (inliers: %lu)", inlierIndices->indices.size());
  }

  std::cerr << "Points left after cropping all walls:" << pclFiltered->points.size() << std::endl;
  // If no points left: stop analysis to prevent seg faults
  if (pclFiltered->points.size() == 0)
  {
    ROS_INFO("[PCL processing] Reject this frame, no points left after removing all walls.");
    return;
  }

  // Eucledian cluster extraction
  pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
  pcl::ConditionalEuclideanClustering<pcl::PointXYZ> pclCluster;
  pclCluster.setClusterTolerance (0.04);
  pclCluster.setConditionFunction(&customEuclideanSegmCond);
  pclCluster.setMinClusterSize(32);
  pclCluster.setMaxClusterSize(10000);
//  pclCluster.setSearchMethod(sTree);
  pclCluster.setInputCloud(pclFiltered);
  pclCluster.segment(*clusters);

  std::cerr << "Number of extracted clusters: " << clusters->size() << std::endl;
  int clustId = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = clusters->begin (); it != clusters->end (); ++it)
  {
    Eigen::Vector4f clustCentroid;
    pcl::compute3DCentroid(*pclFiltered, *it, clustCentroid);
    // Drop cluster centers which are too high and probably only noise (centroid y is mostly negative)
    if ((clustCentroid[1] + CLUSTER_DROP_HEIGHT) < 0)
      continue;

//    std::vector<int>::const_iterator clustIter = it->indices.begin();
//    std::cerr << "Cluster point" << clustId << " ("
//              << pclFiltered->points[*clustIter].x << " "
//              << pclFiltered->points[*clustIter].y << " "
//              << pclFiltered->points[*clustIter].z << ") "
//              << " Count: " << it->indices.size() << std::endl;
    std::cout << "Cluster Centroid " << clustId << " ("
              << clustCentroid[0] << " "
              << clustCentroid[1] << " "
              << clustCentroid[2] << ") "
              << " Count: " << it->indices.size() << std::endl;

    lastObjCmPos.push_back(clustCentroid);
  ++clustId;
  }

  // Generate debugging output for rViz
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*pclFiltered, output);
  pub_pcl_filtered.publish(output);

  // TESTING PCL Object recognition
  //  pclFiltered = pclProcessed;
  //  static FeatureCloud objModel;
  //  compareModelWithScene(objModel);

}

/**
 * @brief PclRecognition::filteredMeanfromPlaneEdge
 *  Only take first bunch of edge data, filter sequence of data which is not connected (noise)
 * @param pEdge
 * @return
 */
float PclRecognition::filteredMeanfromPlaneEdge(const std::vector<Eigen::Vector3f>& pEdge)
{
  // TODO: use biggest sequence instead for mean value
  bool dataSeen = false;
  float avgVals = 0.;
  int countVals = 0;
  for (std::vector<Eigen::Vector3f>::const_iterator it = pEdge.begin(); it != pEdge.end(); ++it)
  {
    if ((*it)[0] == 0. && !dataSeen)
      continue;
    else if ((*it)[0] == 0. && dataSeen)
      break;

    avgVals += (*it)[0];
    ++countVals;
    dataSeen = true;

//    std::cerr << "Point on left edge: ("
//              << (*it)[0] << " "
//              << (*it)[1] << " "
//              << (*it)[2] << ") " << std::endl;
  }

  return (avgVals / countVals);
}



void PclRecognition::compareModelWithScene(FeatureCloud& model)
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

  //g TESTING (TODO: remove static, several obstacle pcls have to be compared to target pcl)
  static FeatureCloud featureObstacle;
  featureObstacle.loadPcdFile("pcl_objects_cut/pcd_objects/tiger/0.pcd");
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
  pub_pcl_obj_alignment.publish(output);
}

} // END Namespace objRecognition
