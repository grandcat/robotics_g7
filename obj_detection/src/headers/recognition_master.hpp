#ifndef RECOGNITION_MASTER_HPP
#define RECOGNITION_MASTER_HPP

#include <ros/ros.h>
#include "recognition_constants.hpp"
// Object recognition: color filter
#include "contour_filter.hpp"
// Message headers (for communication)
#include "explorer/Object.h"
#include "color_filter/Objects.h"
#include <object_recognition/Recognized_objects.h>
// Primesense images (distance of object)
#include <image_transport/image_transport.h>
#include <opencv/cv.h>

namespace objRecognition
{

class RecognitionMaster
{
public:
  RecognitionMaster(ros::NodeHandle &nh)
    : nh_(nh), it_(nh), cRcvdDepthFrames(0),
      lastRecognizedId(OBJTYPE_NO_OBJECT), lastRememberedObjId(OBJTYPE_NO_OBJECT),
      lastSendObjId(OBJTYPE_NO_OBJECT), count_LastObjType(0), debugWindows(false)
  {
    // Subscribe to RGB & depth image: Processing by slaves and estimation of objects position
    sub_rgb_img = it_.subscribe("/camera/depth/image_rect", 1,
                                &objRecognition::RecognitionMaster::runRecognitionPipeline, this);
    // TODO: Initialize recognition slave: color_filter

    sub_obj_recogn_type = nh_.subscribe("/recognition/recognized", 1,
                                        &objRecognition::RecognitionMaster::rcvObjType, this);
//    subscribeDepthImg();  // TODO: remove with optimization
    pub_recognition_result = nh_.advertise<explorer::Object>("/recognition/object_pos_relative", 5);
    ROS_INFO("[Recognition master] Subscribed to recognition slaves, advertising to explorer.");

  }

  void subscribeDepthImg()
  {
    // Subscribe when not already connected
    if (sub_depth_img == 0)
    {
      cRcvdDepthFrames = 0;  // just subscribed (again), no images processed yet
      sub_depth_img = it_.subscribe("/camera/depth/image_rect", 1,
                                    &objRecognition::RecognitionMaster::rcvDepthImg, this);
    }
  }

  void runRecognitionPipeline(const sensor_msgs::ImageConstPtr& msg);

  void rcvObjType(const object_recognition::Recognized_objects::ConstPtr msg);

  void rcvDepthImg(const sensor_msgs::ImageConstPtr& msg);

  inline void setDebugView(bool pContourFilter = true, bool pDepthImg = true)
  {
    objRecog_Contourfilter.showDebugOutput(pContourFilter);
    debugWindows = pDepthImg;
  }

private:
  explorer::Object translateCvToMap(int y, int x);

  // Deprecated functions
//  void rcvSlaveRecognition(const color_filter::Objects::ConstPtr &msg);
  // END Deprecated functions


private:
  // ROS connection
  ros::NodeHandle nh_;
  ros::Subscriber sub_obj_detection, sub_obj_recogn_type;
  image_transport::Subscriber sub_rgb_img;
  ros::Publisher pub_recognition_result;

  // Recognition slaves: color_filter, feature_recognition
  Contour_Filter objRecog_Contourfilter;
  std::vector<DetectedObject> lastObjPositions;

  // Depth image (for distance)
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_depth_img;
  cv::Mat curDepthImg;
  int cRcvdDepthFrames;                        //< counts not used depth frames since last position estimation
  // Object type
  enum EObjectTypes lastRecognizedId, lastRememberedObjId, lastSendObjId;
  int count_LastObjType;

  // Debug
  bool debugWindows;
};

}

#endif // RECOGNITION_MASTER_HPP
