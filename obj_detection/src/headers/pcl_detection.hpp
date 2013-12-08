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
#include <Eigen/Core>
#include <Eigen/Geometry>
// PCL recognition
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ia_ransac.h>

namespace objRecognition
{
const int AMOUNT_COMPARE_FRAMES = 5;


 /*
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
class PclRecognition
{
  /*
   * Configuration
   */
  // Manual camera pose calibration
  static const float camera_translation_z = -0.27; // second: translation -0.47
  static const float camera_pose_rotation = -33.0 / 180.0 * M_PI;

  /*
   * Class PclRecognition
   */
public:
  PclRecognition(ros::NodeHandle& nh) : nh_(nh), processingActive(false), cProcessedFrames(0)
  {
    // Initialize camera pose transformation
//    cameraPoseTransform = Eigen::AngleAxisf(camera_pose_rotation, Eigen::Vector3f::UnitX()) *
//        Eigen::Translation3f(Eigen::Vector3f(0, 0, camera_translation_z));
    cameraPoseTransform = Eigen::Translation3f(Eigen::Vector3f(0, camera_translation_z, 0)) *
        Eigen::AngleAxisf(camera_pose_rotation, Eigen::Vector3f::UnitX());

    // Subscribe to Primesense camera and publish debugging data
    sub_pcl_primesense = nh_.subscribe("/camera/depth_registered/points", 1,
                                       &objRecognition::PclRecognition::rcvPointCloud, this);
    pub_pcl_filtered = nh_.advertise<sensor_msgs::PointCloud2>("/camera/filtered_points", 1);
    pub_pcl_obj_alignment = nh_.advertise<sensor_msgs::PointCloud2>("/camera/obj_aligned_points", 1);
    ROS_INFO("pcl_detection: Subscribed to pointcloud data.");

    // Prepare obj detection
    lastObjCmPos.reserve(AMOUNT_COMPARE_FRAMES);
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
    cProcessedFrames = 0;
    processingActive = true;
  }

  bool getProcessStatus() const
  {
    return processingActive;
  }

private:
  static float filteredMeanfromPlaneEdge(const std::vector<Eigen::Vector3f>& pEdge);

private:
  // ROS connection
  ros::NodeHandle& nh_;
  ros::Subscriber sub_pcl_primesense;
  ros::Publisher pub_pcl_filtered, pub_pcl_obj_alignment;

  // Process behavior
  bool processingActive;
  int cProcessedFrames;

  // Point cloud data
  Eigen::Affine3f cameraPoseTransform;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclFiltered;
  std::vector<std::vector<Eigen::Vector4f> > lastObjCmPos;
  // Sample Consensus Initial Alignment
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sacIA;
};

}

#endif // PCL_DETECTION_HPP
