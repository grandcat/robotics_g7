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
#include <algorithm>    // std::min

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
		//ROS_INFO("Pos %i --> Val: %i (c: %i)", curPos, depthPos, count);
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
	//cv::line(img, cv::Point(20, 40), cv::Point(80, 20), cv::Scalar(0, 255, 0), 2, 8);

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

	static const int SIZE_OF_IMAGE_ARRAY = 5;
	cv::Mat im_array[SIZE_OF_IMAGE_ARRAY];	//circular array to store 5 images
	int array_ptr;			//circular array pointer
	int nr_of_stored_images; //to check if the image array is "full"

	ImageConverter()
	: it_(nh_)
	{
		ROS_INFO("Starting imageConverter.");
		array_ptr = 0;
		nr_of_stored_images = 0;

//		image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
		image_sub_depth_ = it_.subscribe("/camera/depth/image_rect", 1, &ImageConverter::imageCb_depth, this);
		ROS_INFO("Subscribed to depth image.");
	}

	~ImageConverter()
	{
//		cvReleaseImage(&img);
//		cvReleaseImage(&img_depth);
	}

	cv::Mat remove_noise(const cv::Mat& src, int iter = 0, int kernel_size = 9)
	{
		//only start to remove the noise if we have enough previous images saved
		if (nr_of_stored_images < SIZE_OF_IMAGE_ARRAY)
		{
			im_array[array_ptr] = src;
			array_ptr++;
			nr_of_stored_images++;
			if (array_ptr > SIZE_OF_IMAGE_ARRAY-1)
				array_ptr = 0;
			//std::cout<<"ptr: "<<array_ptr<<"stored_images: "<<nr_of_stored_images<<std::endl;
			return src; //simply return the current image and don't remove the noise
		}

		//we have enough images stored to remove the background noise by comparing previous images background
		cv::Mat noise_free_im = src.clone();
		for (int im_i = 0; im_i < SIZE_OF_IMAGE_ARRAY; im_i++ )
		{
			for (int i = 0; i< src.rows; i++)
			{
				for (int j=0; j < src.cols; j++)
				{
					/*
					std::cout<<"min( "<<noise_free_im.at<float>(i,j)<<", "<<im_array[im_i].at<float>(i,j)<<"): ";
					if ( isnan(noise_free_im.at<float>(i,j)) || isnan(im_array[im_i].at<float>(i,j)) )
						std::cout<< 0.0/0.0 <<std::endl;
					else
						std::cout<< min( noise_free_im.at<float>(i,j), im_array[im_i].at<float>(i,j)  ) <<std::endl;
					*/
					if ( isnan(noise_free_im.at<float>(i,j)) || isnan(im_array[im_i].at<float>(i,j)) )
						noise_free_im.at<float>(i,j) = 0;
					else
						noise_free_im.at<float>(i,j) = min( noise_free_im.at<float>(i,j), im_array[im_i].at<float>(i,j)  );
				}
			}
		}

		//save current image for future noise elimination
		im_array[array_ptr] = src;
		array_ptr++;
		if (array_ptr > SIZE_OF_IMAGE_ARRAY-1)
			array_ptr = 0;

		//remove extra noise by using the closing operation ( dilation(erosion(image) )
		//and then do some extra dilation to increase the size of everything
		if (iter != 0)
		{
			cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size), cv::Point(-1,-1) ); //create the convolution matrix
			cv::morphologyEx(noise_free_im, noise_free_im, CV_MOP_OPEN, element); //OPEN(src) := dilate( erode(src) )
			cv::dilate(noise_free_im, noise_free_im, element, cv::Point(-1, -1), iter, 1, 1);
		}

		return noise_free_im;

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
		//ROS_INFO("Get depth: %i", cv_ptr->image.depth());
		//int depthPos = averageDepthAtPos(cv_ptr->image, cv_ptr->image.rows / 2, cv_ptr->image.cols / 2);
		//ROS_INFO("Depth in middle: %i", depthPos);
//		ROS_INFO("Resolution: %i", img_depth->depth);

		cv::Mat noise_free_image = remove_noise(cv_ptr->image);
		cv::Mat better_noise_free_image = remove_noise(cv_ptr->image,1);

		//computeDepthLine(noise_free_image, cv_ptr->image.rows - 100);

		cv::imshow("depth image", cv_ptr->image);
		cv::imshow("depth image removed noise", noise_free_image);
		cv::imshow("depth image removed noise EVEN BETTER?!", better_noise_free_image);

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



