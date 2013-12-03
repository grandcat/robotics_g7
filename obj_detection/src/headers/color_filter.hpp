/*
 * color_filter.h
 *
 *  Created on: Nov 25, 2013
 *      Author: Andreas
 */

#ifndef COLOR_FILTER_H_
#define COLOR_FILTER_H_

// ROS base and communication
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp> //SimpleBlobDetector (is out-commented for the moment)
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp> //groupRectangles

//#include "recognition_constants.hpp"

#include <color_filter/Rect2D_.h> //msg for ROI of an object
#include <color_filter/Objects.h>//msg containing ROI:s of objects

namespace objRecognition
{
// initial min and max HSV filter values.
// these can be changed in runtime through the keyboard node (TODO, add trackbar instead)
const int CHANGE = 10;
const int MAX = 256;

struct Cf_Params
{
  int H_MIN, H_MAX;
  int S_MIN, S_MAX;
  int V_MIN, V_MAX;

  Cf_Params() :
    H_MIN(0), H_MAX(MAX), S_MIN(0), S_MAX(MAX), V_MIN(0), V_MAX(MAX) {}
};


//flag for which filter we are adjusting
enum Filter_mode
{
  WALLS = 0,
  FLOOR = 1
};

//object contour parameters
const double minArea = 1000.0;
const double maxArea = 100000.0;

//object rectangle parameter
//double minRatio = 4.3;
const double minRatio = 5;


struct ObjectRectangle
{
  unsigned int ROI_id;
  cv::Rect boundRect;
};

struct DetectedObject:ObjectRectangle
{
  std::vector<cv::Point> contours_poly;
  cv::RotatedRect rotatedRect;
  cv::Point mc;

//    friend std::ostream & operator<<(std::ostream & stream, const DetectedObject &a)
//    {
//    	stream 	<< "id: "<<a.ROI_id<<"\ncontours_poly: "<<a.contours_poly.size()<<"\nboundRect: ["<<a.boundRect.tl()<<", "<<a.boundRect.br()<<"]"
//    			<<"\nrotatedRect: "<<a.rotatedRect.angle<<"\nmc: "<<a.mc;
//    	return stream;
//    }

};


class Color_Filter
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber keyboard_sub;
  ros::Publisher obj_pub_;
  image_transport::Publisher img_pub_;

  // Debug settings
  bool FLAG_SHOW_IMAGE;
  bool FLAG_CHANGE_THRESHOLD;

  // previous ROI
  std::vector<ObjectRectangle> prev_rects;
  int ROI_id_counter;
  // stored information about the objects detected in the image
  std::vector<DetectedObject> objects;

public:
  Color_Filter() : it_(nh_), ROI_id_counter(0),
    FLAG_SHOW_IMAGE(false), FLAG_CHANGE_THRESHOLD(false)
  {
//    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &Color_Filter::color_filter, this);
    keyboard_sub = nh_.subscribe("/keyboard/input", 1, &Color_Filter::ChangeThreshold, this);
    img_pub_ = it_.advertise("/color_filter/filtered_image", 1);
    obj_pub_ = nh_.advertise<color_filter::Objects>("/recognition/detect", 1);

    ROS_INFO("[Color filter] Up and running!");
    ROS_INFO("Message is being sent to /recognition/detect");
  }

  ~Color_Filter()
  {
    // TODO: only destroy windows created by color_filter
    cv::destroyAllWindows();
  }

  /**
   * @brief ChangeThreshold
   * @param msg
   */
  void ChangeThreshold(const std_msgs::String::ConstPtr& msg);

  /**
   * @brief showConfigurationPanel
   *  Show OpenCV slide bars to configurate color/lighting dependent parameters
   */
  void showConfigurationPanel();

  /**
   * @brief showDebugOutput
   *  Show OpenCV debug windows of original and processed images
   * @param pShow
   *  Activate (= default) or deactivate debug windows
   */
  inline void showDebugOutput(bool pShow = true)
  {
    FLAG_SHOW_IMAGE = pShow;
  }

  /**
   * @brief color_filter
   * @param msg
   */
  void color_filter(const sensor_msgs::ImageConstPtr& msg);

  inline std::vector<DetectedObject> getDetectedObjects()
  {
    return objects;
  }

};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
} // END namespace objRecognition

#endif /* COLOR_FILTER_H_ */
