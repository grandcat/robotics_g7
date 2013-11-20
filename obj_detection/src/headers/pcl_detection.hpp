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

namespace objRecognition
{

  class ImageFetchSmooth
  {
  public:
    ImageFetchSmooth(ros::NodeHandle& nh) : nh_(nh), processingActive(false)
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

    /**
     * @brief start Start point cloud processing for next frames
     *  Robot should stay at one place for smoothing & recognition process
     */
    void start()
    {
      processingActive = true;
    }

  private:
    // ROS connection
    ros::NodeHandle& nh_;
    ros::Subscriber sub_pcl_primesense;
    ros::Publisher pub_pcl_filtered;

    bool processingActive;
    // Point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclFiltered;
  };

}

#endif // PCL_DETECTION_HPP
