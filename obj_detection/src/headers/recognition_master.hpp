#ifndef RECOGNITION_MASTER_HPP
#define RECOGNITION_MASTER_HPP

#include <ros/ros.h>
// Message headers (for communication)
#include "explorer/Object.h"
#include "color_filter/Objects.h"
// Primesense images (distance of object)
#include <image_transport/image_transport.h>
#include <opencv/cv.h>

namespace objRecognition
{

class RecognitionMaster
{
public:
  RecognitionMaster(ros::NodeHandle &nh)
    : nh_(nh), it_(nh), cRejectedFrames(0)
  {
    // Subscribe to Recognition slaves & publish results
    sub_obj_detection = nh_.subscribe("/recognition/detect", 5,
                                          &objRecognition::RecognitionMaster::rcvSlaveRecognition, this);
    subscribeDepthImg();  // TODO: remove with optimization
    pub_recognition_result = nh_.advertise<explorer::Object>("/recognition/object_pos_relative", 5);
    ROS_INFO("Object recognition master: Subscribed to recognition slaves, advertising to explorer.");

  }

  void subscribeDepthImg()
  {
    // Subscribe when not already connected
    if (sub_depth_img == 0)
    {
      cRejectedFrames = 0;  // just subscribed (again), no images processed yet
      sub_depth_img = it_.subscribe("/camera/depth/image_rect", 1,
                                    &objRecognition::RecognitionMaster::rcvDepthImg, this);
    }
  }

  void rcvSlaveRecognition(const color_filter::Objects::ConstPtr &msg);

  void rcvDepthImg(const sensor_msgs::ImageConstPtr& msg);

  explorer::Object translateCvToMap(int y, int x);




private:
  // ROS connection
  ros::NodeHandle& nh_;
  ros::Subscriber sub_obj_detection;
  ros::Publisher pub_recognition_result;
  // Depth image (for distance)
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_depth_img;
  cv::Mat curDepthImg;
  int cRejectedFrames;                        //< counts not used depth frames since last position estimation
};



}

#endif // RECOGNITION_MASTER_HPP
