/*
 * recognition.cpp
 *
 *  Created on: Nov 14, 2013
 *      Author: robo
 */

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "math.h"
#include <iostream>
#include <stdio.h>
#include "headers/recognition.h"
#include <ros/ros.h>
#include <camera/Position.h>
#include <string.h>
#include <sstream>
#include <robot/Object.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

using namespace std;
using namespace camera;
using namespace robot;


double uniformRandom()
{
	return (double)(rand())/(double)(RAND_MAX);
}


double normalRandom()
{
	// Box-Muller transform
	double u1 = uniformRandom();
	double u2 = uniformRandom();
	return cos(2*M_PI*u2)*sqrt(-2.*log(u1));
}


void object_coordinate()
{
	x = 0;
	y = 0;
	for(int i = 0; i < N; i++)
	{
		x += Px[i];
		y += Py[i];
	}
	x = x/N+sx/2;
	y = y/N+sy/2;
}


void showCoord(IplImage* img)
{
	for(int i = 0; i <= 10; i++)
	{
		for(int j = 0; j <= 10; j++)
		{
			if((((x+j-5) < height) & ((y+i-5) < width)) & (((x+j-5) >= 0) & ((y+i-5) >= 0)))
			{
				img->imageData[((x+j-5)*img->widthStep)+(y+i-5)] = 130;
			}
		}
	}
}


void showParticle(IplImage* img)
{
	for(int i = 0; i < N; i++)
	{
		if((((Px[i]+sx/2) < height) & ((Py[i]+sy/2) < width)) & (((Px[i]+sx/2) >= 0) & ((Py[i]+sy/2) >= 0)))
		{
			img->imageData[((Px[i]+sx/2)*img->widthStep)+(Py[i]+sy/2)] = 130;
		}
	}
}


double evaluate(int x, int y, IplImage* img)
{
	double sum = 0;
	for(int i = 0; i < sx; i++)
	{
		for(int j = 0; j < sy; j++)
		{
			if((((x+j) < height) & ((y+i) < width)) & (((x+j) >= 0) & ((y+i) >= 0)))
			{
				sum -= img->imageData[((x+j)*img->widthStep)+(y+i)];
			}
		}
	}

	if(sum > 0.2*sx*sy)
	{
		Object object;
		object.n = 1;
		object_detection_pub.publish(object);

		//printf("OBJECT\n");
	}

	return sum/sx/sy;
}


void init_part()
{
	for(int i = 0; i < N; i++)
	{
		Px[i] = rand() % height;
		Py[i] = rand() % width;
		W[i] = 1;
	}
}


void particle_filter(IplImage* img)
{
	// Diffusion
	double diffusion_x [N];
	double diffusion_y [N];

	for(int i = 0; i < N; i++)
	{
		diffusion_x[i] = sigma_diffusion*normalRandom();
		Px[i] += diffusion_x[i];
		diffusion_y[i] = sigma_diffusion*normalRandom();
		Py[i] += diffusion_y[i];
	}

	// Weighting
	double norm = 0;
	for(int i = 0; i < N; i++)
	{
		W[i] = evaluate(Px[i],Py[i],img);
		norm += W[i];
	}
	for(int i = 0; i < N; i++)
	{
		W[i] = W[i]/norm;
	}

	// Resampling
	double cdf[N];
	cdf[0] = W[0];
	for(int i = 1; i < N; i++)
	{
		cdf[i] = cdf[i-1] + W[i];
	}

	double r = uniformRandom()/N;
	for(int i = 0; i < N; i++)
	{
		for(int j = 0; j < N; j++)
		{
			if(cdf[j] >= r)
			{
				Px[i] = Px[j];
				Py[i] = Py[j];
				break;
			}
		}
		W[i] = (double)1/N;
		r += (double)1/N;
	}

	// Update object coordinate
	object_coordinate();
}


class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;

public:
	IplImage* img;
	IplImage* filtered_img;

	ImageConverter()
	: it_(nh_)
	{
		image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
	}

	~ImageConverter()
	{
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
		filtered_img = cvCreateImage( sz, 8, 1);
		CvScalar  bgr_min = cvScalar(0, 0, 50, 0);
		CvScalar  bgr_max = cvScalar(50, 50, 255, 0);
		cvInRangeS (img, bgr_min, bgr_max, filtered_img);

		if(init > 50)
		{
		// Filter
		particle_filter(filtered_img);

		// Publish y and depth
		Position pos;
		pos.x = width/2-y;
		pos.depth = 0;
		pos_pub.publish(pos);

		// Print
		showParticle(filtered_img);
		showCoord(filtered_img);
		}
		else {init++;}

		cvNamedWindow("filtered_img",1); cvShowImage("filtered_img", filtered_img);
		cvWaitKey(10);
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "Tracking");
	ros::NodeHandle nh;
	pos_pub = nh.advertise<Position>("/pos",100);

	object_detection_pub = nh.advertise<Object>("/object_detection",100);

	init_part();
	object_coordinate();

	ImageConverter ic;
	ros::spin();

	cvDestroyAllWindows();

	return 0;
}
