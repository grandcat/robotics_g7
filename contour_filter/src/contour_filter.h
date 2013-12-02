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
//double minArea = 1000.0;
double minArea = 10.0;
//double maxArea = 100000.0;
double maxArea = 1000.0;

//previous ROI
cv::Rect prev_rect;
int ROI_id_counter = 0;

#endif /* CONTOUR_FILTER_H_ */
