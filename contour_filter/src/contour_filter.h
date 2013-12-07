/*
 * contour_filter.h
 *
 *  Created on: Nov 25, 2013
 *      Author: Andreas
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

#include "recognition_constants.hpp"
#include <typeinfo>


//#include <contour_filter/Rect2D_.h> //msg for ROI of an object
//#include <contour_filter/Objects.h>//msg containing ROI:s of objects

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////  Global variables  ////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////


bool FLAG_SHOW_IMAGE = false;
bool FLAG_CHANGE_THRESHOLD = false;



//flag for which filter we are adjusting
enum Filter_mode
{
	WALLS = 0,
	FLOOR = 1
};
Filter_mode mode;

//object contour parameters
double minArea = 1000.0;
//double minArea = 10.0;
//double maxArea = 100000.0;
double maxArea = 17000.0;

//object rectangle parameter
//double minRatio = 4.3;
const double minRatio = 5;

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

//    friend std::ostream & operator<<(std::ostream & stream, const DetectedObject &a)
//    {
//    	stream 	<< "id: "<<a.ROI_id<<"\ncontours_poly: "<<a.contours_poly.size()<<"\nboundRect: ["<<a.boundRect.tl()<<", "<<a.boundRect.br()<<"]"
//    			<<"\nrotatedRect: "<<a.rotatedRect.angle<<"\nmc: "<<a.mc;
//    	return stream;
//    }

};

//previous ROI
cv::Rect prev_rect;
int ROI_id_counter = 0;

//parameters for the shadow filter
int SHADOW_FILTER_MASK_SIZE = 7;
float SHADOW_FILTER_THRESHOLD = 0.02;

//contour translation between depth to color image
int DEPTH_TO_COLOR_DX = -30;
int DEPTH_TO_COLOR_DY = 0;
#endif /* CONTOUR_FILTER_H_ */
