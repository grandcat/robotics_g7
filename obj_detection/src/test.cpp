/*
 * test.cpp
 *
 *  Created on: Nov 4, 2013
 *      Author: robo
 */

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "math.h"
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <string.h>
#include <sstream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

using namespace std;

int averageDepthAtPos(cv::Mat &img_depth, int y, int x) {
	int depth;
	float sumDistance = 0;
	int nAvg = 0;
	for (int i = 0; i <= 10; i++) {
		for (int j = 0; j <= 10; j++) {
			if ((((y + j - 5) < img_depth.rows) & ((x + i - 5) < img_depth.cols))
					&& (((y + j - 5) >= 0) & ((x + i - 5) >= 0))) {
				float curDepth = img_depth.at<float>(y + j - 5, x + i - 5);
//				ROS_INFO("Raw value: %f", curDepth);
				if (!isnan(curDepth)) {
					sumDistance += curDepth;
					++nAvg;
				}
			}
		}
	}
	if (nAvg != 0) {
		depth = sumDistance / nAvg * 1000;
	} else {
		depth = 0;
	}

	return depth;
}

void computeDepthLine(cv::Mat &img_depth, int height) {
	// Move horizontal on given height and
	int sumDepth = 0, count = 0;
	for (int curPos = 5; curPos < img_depth.cols; curPos += 5) {
		int depthPos = averageDepthAtPos(img_depth, height, curPos);
		sumDepth += depthPos;
		if (depthPos != 0)
			++count;
		ROS_INFO("Pos %i --> Val: %i (c: %i)", curPos, depthPos, count);
	}
	// Create line with relative deviation
	if (count == 0)
		return;
	int avgDepth = sumDepth / count;

	cv::Mat img = cv::Mat::zeros(480, 640, CV_8UC3);
	for (int curPos = 5; curPos < img_depth.cols; curPos += 5) {
		int relVal = averageDepthAtPos(img_depth, height, curPos) - avgDepth + img_depth.rows / 2;
		cv::circle(img, cv::Point(curPos, relVal), 1, cv::Scalar(0, 255, 0), 2, 8, 0);
	}

	cv::imshow("Depth line", img);
}

void drawDepthLine() {
	cv::Mat img = cv::Mat::zeros(480, 640, CV_8UC3);
//	cv::line(img, cv::Point(20, 40), cv::Point(80, 20), cv::Scalar(0, 255, 0), 2, 8);

	cv::imshow("Depth line", img);
}

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Subscriber image_sub_depth_;

public:
	IplImage* img;
//	IplImage* img_depth;
	IplImage* hsv_image;
	IplImage* hsv_mask;

	ImageConverter()
	: it_(nh_)
	{
		ROS_INFO("Starting imageConverter.");
//		image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
		image_sub_depth_ = it_.subscribe("/camera/depth/image_rect", 1, &ImageConverter::imageCb_depth, this);
		ROS_INFO("Subscribed to depth image.");
	}

	~ImageConverter()
	{
//		cvReleaseImage(&img);
//		cvReleaseImage(&img_depth);
	}

	void imageCb_depth(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

//		img_depth = new IplImage(cv_ptr->image);
		// Get closest pixels
		// DEBUG
		ROS_INFO("Get depth: %i", cv_ptr->image.depth());
		int depthPos = averageDepthAtPos(cv_ptr->image, cv_ptr->image.rows / 2, cv_ptr->image.cols / 2);
		ROS_INFO("Depth in middle: %i", depthPos);
//		ROS_INFO("Resolution: %i", img_depth->depth);
		computeDepthLine(cv_ptr->image, cv_ptr->image.rows - 100);

		cv::imshow("depth image", cv_ptr->image);
		cvWaitKey(10);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		img = new IplImage(cv_ptr->image);

		CvSize sz = cvGetSize(img);
		hsv_image = cvCreateImage( sz, 8, 3);
		hsv_mask = cvCreateImage( sz, 8, 1);
		CvScalar  hsv_min = cvScalar(0, 30, 80, 0);
		CvScalar  hsv_max = cvScalar(20, 150, 255, 0);
		cvCvtColor(img, hsv_image, CV_BGR2HSV);
		cvInRangeS (hsv_image, hsv_min, hsv_max, hsv_mask);

		// Filter

		cvNamedWindow("hsv-msk",1); cvShowImage("hsv-msk", hsv_mask);
		cvWaitKey(10);
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "Tracking");
	ros::NodeHandle nh;

	ImageConverter ic;
	ros::spin();

	cvDestroyAllWindows();

	return 0;
}



