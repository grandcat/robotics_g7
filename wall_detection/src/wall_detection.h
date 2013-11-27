/*
 * wall_detection.h
 *
 *  Created on: Nov 20, 2013
 *      Author: robo
 */

#ifndef WALL_DETECTION_H_
#define WALL_DETECTION_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/core/core.hpp>
#include <vector>
#include <string.h>
#include <math.h>		// M_PI
#include <algorithm>    // std::min
#include <wall_detection/Line2D.h> //msg for a Line
#include <wall_detection/Lines2D.h>//msg containing Line(s)



#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include <iostream>
#include <stdio.h>
#include <sstream>


static const char WINDOW[] = "Image window";

bool FLAG_SHOW_OUTPUT = false;
bool FLAG_SHOW_IMAGE = false;

struct Line2D
{
	int x0;
	int y0;
	int x1;
	int y1;
	double angle;
	std::string description;
};


class Wall_Detection
{

	ros::NodeHandle nh_;
	ros::Publisher lines_pub_;
	ros::Subscriber lines_sub_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;


public:
	bool isPathFree;
	std::vector<Line2D> lines_;
	std::vector<int> indices_left;
	std::vector<int> indices_right;
	std::vector<int> indices_horizontal;
	std::vector<int> indices_vertical;
	std::vector<int> indices_obstacle;
	std::vector<int> indices_uninteresting;

	Wall_Detection();
	~Wall_Detection();
	void HoughLinesDetector(const sensor_msgs::ImageConstPtr& msg);
	void wall_checker(const wall_detection::Lines2D::ConstPtr &msg);

};

#endif /* WALL_DETECTION_H_ */
