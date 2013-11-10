/*
 * pcl_processing.cpp
 *
 *  Created on: Nov 11, 2013
 *      Author: robo
 */

#include "headers/pcl_processing.hpp"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>		//< will be detected when reimported in eclipse
#include <sensor_msgs/PointCloud2.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "math.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

using namespace std;

class ImageConverter
{
	ros::NodeHandle nh_;
	ros::Subscriber pcl_recv;
	image_transport::Subscriber image_sub_depth_;

public:
	IplImage* img;

	ImageConverter()
	{
		ROS_INFO("Starting pcl converter.");
		pcl_recv = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1,
				&ImageConverter::process_pcl_data, this);
		ROS_INFO("Subscribed to depth pointcloud.");
	}

	~ImageConverter()
	{
//		cvReleaseImage(&img);
//		cvReleaseImage(&img_depth);
	}

	void process_pcl_data(const sensor_msgs::PointCloud2ConstPtr& cloud) {

	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "PCL Processing");
	ros::NodeHandle nh;

	ImageConverter ic;
	ros::spin();

	cvDestroyAllWindows();

	return 0;
}



