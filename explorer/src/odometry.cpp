/*
 * odometry.cpp
 *
 *  Created on: Nov 17, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <differential_drive/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include "headers/odometry.h"
#include "headers/parameters.h"

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace differential_drive;
using namespace cv;


void create_node(double x, double y)
{
	static int index;
	index++;

	Node n;
	n.x = x;
	n.y = y;

	discrete_map.push_back(n);

	// Marker
	uint32_t shape = visualization_msgs::Marker::CUBE;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = index;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;

	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.05;

	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);
}

void receive_odometry(const Odometry::ConstPtr &msg)
{
	x = msg->x;
	y = msg->y;
	theta = msg->theta;


	// Node
	static double nx,ny;
	double dist = sqrt((x-nx)*(x-nx)+(y-ny)*(y-ny));
	if(dist > 0.1)
	{
		create_node(x,y);

		nx = x;
		ny = y;
	}


	// Robot marker
	uint32_t shape = visualization_msgs::Marker::ARROW;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;

	tf::Quaternion q = tf::createQuaternionFromYaw(theta);
	marker.pose.orientation.x = q.x();
	marker.pose.orientation.y = q.y();
	marker.pose.orientation.z = q.z();
	marker.pose.orientation.w = q.w();

	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 0.3;

	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);


	// Robot
	int rx = (x-map.info.origin.position.x)/map.info.resolution;
	int ry = (y-map.info.origin.position.y)/map.info.resolution;
	for(int i = -2; i <= 2; i++)
	{
		for(int j = -2; j <= 2; j++)
		{
			if(((ry+j)*map.info.width+(rx+i) >= 0) | ((ry+j)*map.info.width+(rx+i) < map.info.width*map.info.height))
			{
				map.data[(ry+j)*map.info.width+(rx+i)] = 0;
			}
		}
	}
	map_pub.publish(map);
}


void receive_sensors(const AnalogC::ConstPtr &msg)
{
	double s1 = a_short*pow(msg->ch1,b_short);
	double s2 = a_short*pow(msg->ch2,b_short);

	// Wall
	if(s1 < 0.3)
	{
		int wx = ((x+x_s1*cos(theta)-y_s1*sin(theta)-s1*sin(theta))-map.info.origin.position.x)/map.info.resolution;
		int wy = ((y+x_s1*sin(theta)+y_s1*cos(theta)+s1*cos(theta))-map.info.origin.position.y)/map.info.resolution;
		if((wy*map.info.width+wx >= 0) | (wy*map.info.width+wx < map.info.width*map.info.height))
		{
			map.data[wy*map.info.width+wx] = 100;
		}
	}

	if(s2 < 0.3)
	{
		int wx = ((x+x_s2*cos(theta)-y_s2*sin(theta)+s2*sin(theta))-map.info.origin.position.x)/map.info.resolution;
		int wy = ((y+x_s2*sin(theta)+y_s2*cos(theta)-s2*cos(theta))-map.info.origin.position.y)/map.info.resolution;
		if((wy*map.info.width+wx >= 0) | (wy*map.info.width+wx < map.info.width*map.info.height))
		{
			map.data[wy*map.info.width+wx] = 100;
		}
	}

	map_pub.publish(map);

	Hough(map);
}


void Hough(nav_msgs::OccupancyGrid map)
{
	// Map to Image
	Mat mat = Mat::zeros(height,width,CV_8UC1);

	for(int i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++)
		{
			if(map.data[i*map.info.width+j] == 100)
			{
				mat.at<uchar>(height-i-1,j) = 255;
			}
		}
	}


	Mat mat2 = Mat::zeros(height,width,CV_8UC1);

	vector<Vec2f> lines;
	HoughLines(mat, lines, 1, 90*CV_PI/180, 10, 0, 0 );

	for( size_t i = 0; i < lines.size(); i++ )
	{
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 500*(-b));
		pt1.y = cvRound(y0 + 500*(a));
		pt2.x = cvRound(x0 - 500*(-b));
		pt2.y = cvRound(y0 - 500*(a));
		line( mat2, pt1, pt2, Scalar(255), 1);
	}


	GaussianBlur( mat, mat, Size(3,3), 0, 0 );
	for(int i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++)
		{
			if(mat.at<uchar>(i,j) == 0)
			{
				mat2.at<uchar>(i,j) = 0;
			}
		}
	}


	for(int i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++)
		{
			if(map.data[i*map.info.width+j] == 0)
			{
				mat2.at<uchar>(height-i-1,j) = 100;
			}
		}
	}


	int sz = 5;
	for(int i = 0; i < height-sz; i++)
	{
		for(int j = 0; j < width-sz; j++)
		{
			if(mat2.at<uchar>(height-i-1,j) == 100 &
					mat2.at<uchar>(height-i-sz-1,j) == 100 &
					mat2.at<uchar>(height-i-1,j+sz) == 100 &
					mat2.at<uchar>(height-i-sz-1,j+sz) == 100)
			{
				bool flag = false;
				for(int n = 0; n < sz; n++)
				{
					for(int m = 0; m < sz; m++)
					{
						if(mat2.at<uchar>(height-i-n-1,j+m) == 255)
						{
							flag = true;
						}
					}
				}
				if(!flag)
				{
					for(int n = 0; n < sz; n++)
					{
						for(int m = 0; m < sz; m++)
						{
							mat2.at<uchar>(height-i-n-1,j+m) = 100;
						}
					}
				}
			}
		}
	}


	// Interesting points
	int sz2 = 7;
	for(int i = 0; i < height-sz2; i++)
	{
		// Check go left
		for(int j = 0; j < width-sz2; j++)
		{
			bool flag1 = false;
			for(int n = 0; n < sz; n++)
			{
				for(int m = 0; m < sz2-1; m++)
				{
					if(mat2.at<uchar>(height-i-n-1,j+m) != 0)
					{
						flag1 = true;
					}
				}
				if(mat2.at<uchar>(height-i-n-1,j+sz2) != 100)
				{
					flag1 = true;
				}
			}
			if(!flag1)
			{
				mat2.at<uchar>(height-i-2-1,j+sz2-2) = 200;
			}

		}
	}


	namedWindow("Map", CV_WINDOW_NORMAL);
	imshow("Map", mat2);

	cvWaitKey(1);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry");
	ros::NodeHandle nh;

	odom_sub = nh.subscribe("/motion/Odometry",1000,receive_odometry);
	marker_pub = nh.advertise<visualization_msgs::Marker>("/marker", 1);
	sensors_sub = nh.subscribe("/sensors/ADC",1000,receive_sensors);
	map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map",1);

	create_node(x,y);

	// Map init
	map.header.frame_id = "/my_frame";
	map.header.stamp = ros::Time::now();
	// Map properties
	map.info.resolution = resolution;
	map.info.origin.position.x = position_x;
	map.info.origin.position.y = position_y;
	map.info.height = height;
	map.info.width = width;
	map.data.resize(map.info.width*map.info.height);

	for(int i = 0; i < map.info.width*map.info.height; i++)
	{
		map.data[i] = -1;
	}


	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}



