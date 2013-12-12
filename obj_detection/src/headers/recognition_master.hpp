/*
 * recognition_master.hpp
 *
 *      Author: Stefan
 */


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
#include <explorer/Stop_EKF.h>
// Primesense images (distance of object)
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
// Competition
#include <contest_msgs/evidence.h>

namespace objRecognition
{

class RecognitionMaster
{
public:
  RecognitionMaster(ros::NodeHandle &nh)
    : nh_(nh), it_(nh), cRcvdDepthFrames(0),
      lastRecognizedId(OBJTYPE_NO_OBJECT), lastRememberedObjId(OBJTYPE_NO_OBJECT),
      lastSendObjId(OBJTYPE_NO_OBJECT), count_LastObjType(0), activeDetection(true),
      debugWindows(false)
  {
    // Subscribe to RGB & depth image: Processing by slaves and estimation of objects position
    sub_depth_img = it_.subscribe("/camera/depth/image_rect", 1,
                                &objRecognition::RecognitionMaster::runRecognitionPipeline, this);
    sub_rgb_img = it_.subscribe("/camera/rgb/image_color", 1, &RecognitionMaster::rcvRGBImg, this);
    // Receive classification of objects
    sub_obj_recogn_type = nh_.subscribe("/recognition/recognized", 1,
                                        &objRecognition::RecognitionMaster::rcvObjType, this);
    // Robots EKF (turning of robot is used)
    sub_ekf_turn = nh_.subscribe("/motion/Stop_EKF", 1,
                                 &objRecognition::RecognitionMaster::rcvEKFStop, this);
    // Objects' position estimated by point cloud processing
    sub_pcl_obj_pos = nh_.subscribe("/recognition/pcl_object_pos_relative", 1,
                                    &objRecognition::RecognitionMaster::rcvPclObjPos, this);
    lastPclObjPos[0] = -1;

    // Publish results
    pub_recognition_result = nh_.advertise<explorer::Object>("/recognition/object_pos_relative", 1);
    pub_evidence = nh_.advertise<contest_msgs::evidence>("/contest_evidence", 1);
    ROS_INFO("[Recognition master] Subscribed to recognition slaves, advertising to explorer.");
  }

  void runRecognitionPipeline(const sensor_msgs::ImageConstPtr& msg);

  void rcvObjType(const object_recognition::Recognized_objects::ConstPtr msg);

  void rcvDepthImg(const sensor_msgs::ImageConstPtr& msg);

  void rcvRGBImg(const sensor_msgs::ImageConstPtr& msg);

  void rcvEKFStop(const explorer::Stop_EKF::ConstPtr& msg);

  void rcvPclObjPos(const explorer::Object::ConstPtr &msg);

  inline void setDebugView(bool pContourFilter = true, bool pDepthImg = true)
  {
    objRecog_Contourfilter.showDebugOutput(pContourFilter);
    debugWindows = pDepthImg;
  }

private:
  /*
   * Helper functions
   */
  explorer::Object translateCvToMap(int y, int x);

private:
  // ROS connection
  ros::NodeHandle nh_;
  ros::Subscriber sub_obj_recogn_type, sub_ekf_turn, sub_pcl_obj_pos;
  ros::Publisher pub_recognition_result;

  // Recognition slaves: color_filter, feature_recognition
  Contour_Filter objRecog_Contourfilter;
  std::vector<DetectedObject> lastObjPositions;
  float lastPclObjPos[2];
  ros::Time lastPclTime;

  // Depth image (for distance)
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_depth_img, sub_rgb_img;
  cv::Mat curDepthImg;
  int cRcvdDepthFrames;                        //< counts not used depth frames since last position estimation

  // Object type
  enum EObjectTypes lastRecognizedId, lastRememberedObjId, lastSendObjId;
  int count_LastObjType;

  // Behaviour
  bool activeDetection;

  // Contest evidence
  sensor_msgs::Image curRGBImg;
  ros::Publisher pub_evidence;

  // Debug
  bool debugWindows;
};

}

#endif // RECOGNITION_MASTER_HPP
