#ifndef PCL_DETECTION_HPP
#define PCL_DETECTION_HPP

#include <ros/ros.h>
// PCL<->ROS connection
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
// PCL base
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// PCL recognition
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ia_ransac.h>

namespace objRecognition
{

/**
 * @brief ModelCloud class
 */
class FeatureCloud
{
public:
  FeatureCloud()
  {
    kdsearchTree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
  }

  void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudImport)
  {
    objModel = pCloudImport;
    computeNormals();
    computeFeatureDescriptor();
  }

  void loadPcdFile(const std::string pFilepath)
  {
    objModel = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(pFilepath, *objModel) == -1)   //< PCD file could not be loaded
      {
        ROS_ERROR("PCD File %s could not be loaded, aborting!", pFilepath.c_str());
        return;
      }

    // Computer normals and local feature descriptors
    computeNormals();
    computeFeatureDescriptor();
  }

  inline pcl::PointCloud<pcl::PointXYZ>::Ptr getObjModel() const
  {
    return objModel;
  }

  inline pcl::PointCloud<pcl::Normal>::Ptr getSurfaceNormals() const
  {
    return surfaceNormals;
  }

  inline pcl::PointCloud<pcl::FPFHSignature33>::Ptr getObjFeatures() const
  {
    return objFeatures;
  }

private:
  void computeNormals();
  void computeFeatureDescriptor();

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr objModel;
  pcl::PointCloud<pcl::Normal>::Ptr surfaceNormals;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr objFeatures;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdsearchTree;
};


/**
 * @brief ImageFetchSmooth class
 */
class ImageFetchSmooth
{
public:
  ImageFetchSmooth(ros::NodeHandle& nh) : nh_(nh), processingActive(false), cWaitFrames(0)
  {
    // Subscribe to Primesense camera and publish debugging data
    sub_pcl_primesense = nh_.subscribe("/camera/depth_registered/points", 1,
                                       &objRecognition::ImageFetchSmooth::rcvPointCloud, this);
    pub_pcl_filtered = nh_.advertise<sensor_msgs::PointCloud2>("/camera/filtered_points", 1);
    ROS_INFO("pcl_detection: Subscribed to pointcloud data.");

  }

  /**
   * @brief rcvPointCloud Process point clouds as soon as start() was triggered
   * @param pc_raw  Point cloud data from camera
   */
  void rcvPointCloud(const sensor_msgs::PointCloud2ConstPtr& pc_raw);

  void compareModelWithScene(FeatureCloud& model);

  /**
   * @brief start Start point cloud processing for next frames
   *  Robot should stay at one place for smoothing & recognition process
   */
  void start()
  {
    processingActive = true;
  }

  bool getProcessStatus() const
  {
    return processingActive;
  }

private:
  // ROS connection
  ros::NodeHandle& nh_;
  ros::Subscriber sub_pcl_primesense;
  ros::Publisher pub_pcl_filtered;

  // Process behavior
  bool processingActive;
  int cWaitFrames;

  // Point cloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclFiltered;
  // Sample Consensus Initial Alignment
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sacIA;
};

}

#endif // PCL_DETECTION_HPP
