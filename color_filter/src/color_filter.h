/*
 * color_filter.h
 *
 *  Created on: Nov 25, 2013
 *      Author: Andreas
 */

#ifndef COLOR_FILTER_H_
#define COLOR_FILTER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp> //SimpleBlobDetector (is out-commented for the moment)
#include <opencv2/core/core.hpp>
#include <string.h>

//for the keyboard input (will remove later and only use the Trackbar)
#include "std_msgs/String.h"
#include <sstream>      //std::stringstream
#include <stdio.h>      //getchar()
#include <termios.h>    //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

#include <color_filter/Rect2D_.h> //msg for ROI of an object
#include <color_filter/Objects.h>//msg containing ROI:s of objects

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////  Global variables  ////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////


bool FLAG_SHOW_IMAGE = false;
bool FLAG_CHANGE_THRESHOLD = false;

//initial min and max HSV filter values.
//these can be changed in runtime through the keyboard node (TODO, add trackbar instead)

int CHANGE = 10;
int MAX = 256;

int H_MIN = 0;
int H_MAX = MAX;
int S_MIN = 0;
int S_MAX = 106;
int V_MIN = 0;
int V_MAX = MAX;

//walls
int wall_H_MIN = 0;
int wall_H_MAX = MAX;
int wall_S_MIN = 0;
int wall_S_MAX = 106;
int wall_V_MIN = 0;
int wall_V_MAX = MAX;

//floor
int floor_H_MIN = 0;
int floor_H_MAX = MAX;
int floor_S_MIN = 0;
int floor_S_MAX = 106;
int floor_V_MIN = 0;
int floor_V_MAX = MAX;


//flag for which filter we are adjusting
enum Filter_mode
{
	WALLS = 0,
	FLOOR = 1
};
Filter_mode mode;

//object contour parameters
double minArea = 1000.0;
double maxArea = 100000.0;

#endif /* COLOR_FILTER_H_ */
