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

using namespace differential_drive;


void receive_odometry(const Odometry::ConstPtr &msg)
{
	x = msg->x;
	y = msg->y;
	theta = msg->theta;

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
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry");
	ros::NodeHandle nh;

	odom_sub = nh.subscribe("/motion/Odometry",1000,receive_odometry);
	marker_pub = nh.advertise<visualization_msgs::Marker>("/marker", 1);
	sensors_sub = nh.subscribe("/sensors/ADC",1000,receive_sensors);
	map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map",1);


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



