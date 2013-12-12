/*
 * contour_filter.h
 *
 *  Created on: Nov 25, 2013
 *      Author: Andreas Pettersson
 */

#ifndef CONTOUR_FILTER_H_
#define CONTOUR_FILTER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp> //SimpleBlobDetector (is out-commented for the moment)
#include <opencv2/core/core.hpp>
#include <string.h>
#include <opencv2/objdetect/objdetect.hpp> //groupRectangles

#include <algorithm> //for find function
#include "recognition_constants.hpp"
#include <typeinfo>

//object contour parameters
const double minArea = 1000.0;
const double maxArea = 17000.0;

//object rectangle parameter
const double minRatio = 3.5;

struct ObjectRectangle
{
  unsigned int ROI_id;
  cv::Rect boundRect;
};

struct DetectedObject : ObjectRectangle
{
  std::vector<cv::Point> contours_poly;
  cv::RotatedRect rotatedRect;
  cv::Point mc;
};

//parameters for the shadow filter
const int SHADOW_FILTER_MASK_SIZE = 7;
const float SHADOW_FILTER_THRESHOLD = 0.02;
const int SHADOW_FILTER_ERODE_DILATE_SIZE = 15;

//contour translation between depth to color image
const int DEPTH_TO_COLOR_DX = -30;
const int DEPTH_TO_COLOR_DY = 0;

class Contour_Filter
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher obj_pub_;
  image_transport::Publisher img_pub_;

  std::vector<DetectedObject> objects;
  image_transport::Subscriber image_sub_color;

  bool FLAG_SHOW_IMAGE;

  static const int SIZE_OF_IMAGE_ARRAY = 5;
  cv::Mat im_array[SIZE_OF_IMAGE_ARRAY];  //circular array to store 5 images
  int array_ptr;                  //circular array pointer
  int nr_of_stored_images; //to check if the image array is "full"
  cv::Mat lastRgbImg;
  std_msgs::Header lastRgbHeader;

public:
  Contour_Filter(): it_(nh_), FLAG_SHOW_IMAGE(false), array_ptr(0), nr_of_stored_images(0)
  {
    image_sub_color = it_.subscribe("/camera/rgb/image_color", 1, &Contour_Filter::rcvRgbImage, this);
    img_pub_ = it_.advertise("/color_filter/filtered_image", 1);
  }

  ~Contour_Filter()
  {
    cv::destroyAllWindows();
  }

  void rcvRgbImage(const sensor_msgs::ImageConstPtr& msg);
  void show_detected_objects();
  void moveContour(std::vector<cv::Point>& , int , int );
  cv::Mat remove_noise(const cv::Mat& , int , int );
  cv::Mat shadow_filter(const cv::Mat& );
  std::vector<std::vector<cv::Point> > detect_good_contours(cv::Mat&);
  std::vector<std::vector<cv::Point> > merge_contours(std::vector<std::vector<cv::Point> >&);
  void depth_contour_filter(const cv::Mat& src);

  inline void showDebugOutput(bool pShow = true)
  {
    FLAG_SHOW_IMAGE = pShow;
  }

  inline std::vector<DetectedObject> getDetectedObjects()
  {
    return objects;
  }

};



#endif /* CONTOUR_FILTER_H_ */
