/*
 * test.cpp
 *
 *  Created on: Nov 4, 2013
 *      Author: robo
 */
/*
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
*/
#include "wall_detection.h"
#include <cvblob.h>
#include <typeinfo>

class Wall_Detection;
struct Line2D; //forward declare the struct Line2D

namespace enc = sensor_msgs::image_encodings;

class Wall_Detection; //forward declare Wall_Detection so that class Wall_Detection_And_Depth can use it

class Wall_Detection_And_Depth
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_depth_;
	ros::Subscriber lines_sub_;


public:
	//IplImage* img;
//	IplImage* img_depth;
	//IplImage* hsv_image;
	//IplImage* hsv_mask;

	//Wall_Detection* wd;
	//Wall_Detection wd;
	static const int SIZE_OF_IMAGE_ARRAY = 5;
	cv::Mat im_array[SIZE_OF_IMAGE_ARRAY];	//circular array to store 5 images
	int array_ptr;			//circular array pointer
	int nr_of_stored_images; //to check if the image array is "full"

	//get min and max depth pixel value
	float min_depth_;
	float max_depth_;

	//variables to save the message from wall_detection.cpp
	bool isPathFree;
	std::vector<Line2D> lines_;
	std::vector<int> indices_left;
	std::vector<int> indices_right;
	std::vector<int> indices_horizontal;
	std::vector<int> indices_vertical;
	std::vector<int> indices_obstacle;
	std::vector<int> indices_uninteresting;

	Wall_Detection_And_Depth()
	: it_(nh_)
	{
		ROS_INFO("Starting imageConverter.");
		array_ptr = 0;
		nr_of_stored_images = 0;
		min_depth_ = 255;
		max_depth_ = 0;
		//Wall_Detection wd;

//		image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &Wall_Detection_And_Depth::imageCb, this);
		image_sub_depth_ = it_.subscribe("/camera/depth/image_rect", 1, &Wall_Detection_And_Depth::imageCb_depth, this);
		lines_sub_ = nh_.subscribe("/image/2Dlines", 1, &Wall_Detection_And_Depth::get_walls_message, this);
		ROS_INFO("Subscribed to depth image.");
	}

	~Wall_Detection_And_Depth()
	{
//		cvReleaseImage(&img);
//		cvReleaseImage(&img_depth);
	}

	void get_walls_message(const wall_detection::Lines2D::ConstPtr &msg)
	{
		lines_.clear();
		lines_.reserve(msg->lines.size());
		//for some reason, I cannot copy the vector with the = operator.
		for (unsigned int i = 0; i<msg->lines.size(); i++)
		{
			Line2D line_;
			line_.x0 = msg->lines[i].x0;
			line_.x1 = msg->lines[i].x1;
			line_.y0 = msg->lines[i].y0;
			line_.y1 = msg->lines[i].y1;
			line_.angle = msg->lines[i].angle;
			line_.description =  msg->lines[i].description;
			lines_.push_back(line_);
		}
		isPathFree = msg->isPathFree;
		indices_left = msg->indices_left;
		indices_right = msg->indices_right;
		indices_horizontal = msg->indices_horizontal;
		indices_vertical = msg->indices_vertical;
		indices_obstacle = msg->indices_obstacle;
		indices_uninteresting = msg->indices_uninteresting;
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
		float cur_max = 0;
		float cur_min = 0;
		max_depth_ = 0;
		min_depth_ = 255;
		float max_dist = 1.0;
		for (int i = 0; i< src.rows; i++)
		{
			for (int j=0; j < src.cols; j++)
			{
				for (int im_i = 0; im_i < SIZE_OF_IMAGE_ARRAY; im_i++ )
				{
					if ( isnan(noise_free_im.at<float>(i,j)) || isnan(im_array[im_i].at<float>(i,j))  || std::max( noise_free_im.at<float>(i,j), im_array[im_i].at<float>(i,j) ) > max_dist )
						noise_free_im.at<float>(i,j) = 0;
					else
						cur_min = std::min( noise_free_im.at<float>(i,j), im_array[im_i].at<float>(i,j)  );
						cur_max = std::max( noise_free_im.at<float>(i,j), im_array[im_i].at<float>(i,j)  );
						noise_free_im.at<float>(i,j) = std::min( noise_free_im.at<float>(i,j), im_array[im_i].at<float>(i,j)  );
						//std::cout<< typeid(noise_free_im.at<float>(i,j)).name() << std::endl;
						if (cur_max > max_depth_)
							max_depth_ = cur_max;
						if (cur_min < min_depth_ && cur_min > 0)
							min_depth_ = cur_min;
				}
			}
		}

		//std::cout<<"MAX: "<<max<<std::endl;

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
		//Wall_Detection* wd = new Wall_Detection();
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


		cv::Mat noise_free_image = remove_noise(cv_ptr->image);
		cv::Mat better_noise_free_image = remove_noise(cv_ptr->image,1);

		//computeDepthLine(noise_free_image, cv_ptr->image.rows - 100);

		cv::imshow("depth image", cv_ptr->image);
		cv::imshow("depth image removed noise", noise_free_image);
		cv::imshow("depth image removed noise EVEN BETTER?!", better_noise_free_image);
		draw_lines(better_noise_free_image);

		//turn to a binary image
		IplImage *binary_im = new IplImage(noise_free_image);
		IplImage* img = new IplImage(noise_free_image);
		//IplImage img = noise_free_image;
		double threshold = 0.01;
		double maxValue = 1;
		cvThreshold(img, binary_im, threshold, maxValue, CV_THRESH_BINARY);

		cvShowImage( "binary", binary_im );

		//std::cout<<"max_depth: "<<max_depth_<<" min_depth: "<<min_depth_<<std::endl;

		//get the blobs
		IplImage *labelImg=cvCreateImage(cvGetSize(binary_im), IPL_DEPTH_LABEL, 1);

		/*cvb::CvBlobs blobs;
		unsigned int result=cvLabel(binary_im, labelImg, blobs);
		cvRenderBlobs(labelImg, blobs, img, img);
		cvShowImage( "blobs", img );
		*/

		cvWaitKey(10);
	}
	void draw_lines(cv::Mat& src_grey_image)
	{
		// create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
		cv::Mat img_rgb(src_grey_image.size(), CV_8UC3);

		// convert greyscale to color image
		cv::cvtColor(src_grey_image, img_rgb, CV_GRAY2RGB);

		//Left side off road (/) orange
		for (unsigned int i = 0; i < indices_left.size(); i++)
		{
			Line2D line_ = lines_[indices_left[i]];
			cv::line( img_rgb, cv::Point(line_.x0, line_.y0), cv::Point(line_.x1, line_.y1), cv::Scalar(0,165,255), 3);
		}

		//Right side of road (\) yellow
		for (unsigned int i = 0; i < indices_right.size(); i++)
		{
			Line2D line_ = lines_[indices_right[i]];
			cv::line( img_rgb, cv::Point(line_.x0, line_.y0), cv::Point(line_.x1, line_.y1), cv::Scalar(0,255,255), 3);
		}

		//vertical line -- , Green
		for (unsigned int i = 0; i < indices_vertical.size(); i++)
		{
			Line2D line_ = lines_[indices_vertical[i]];
			cv::line( img_rgb, cv::Point(line_.x0, line_.y0), cv::Point(line_.x1, line_.y1), cv::Scalar(0,255,0), 3);
		}

		//horizontal line |, Blue
		for (unsigned int i = 0; i < indices_horizontal.size(); i++)
		{
			Line2D line_ = lines_[indices_horizontal[i]];
			cv::line( img_rgb, cv::Point(line_.x0, line_.y0), cv::Point(line_.x1, line_.y1), cv::Scalar(255,0,0), 3);

		}
		//obstacles | Red
		for (unsigned int i = 0; i< indices_obstacle.size(); i++)
		{
			Line2D line_ = lines_[indices_obstacle[i]];
			cv::line( img_rgb, cv::Point(line_.x0, line_.y0), cv::Point(line_.x1, line_.y1), cv::Scalar(0,0,255), 3);
		}


		cv::imshow("depth image with lines", img_rgb);
	}


};


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "Tracking");
	ros::NodeHandle nh;

	Wall_Detection_And_Depth ic;
	//ic.wd = new Wall_Detection();
	ros::spin();

	cvDestroyAllWindows();

	return 0;
}



