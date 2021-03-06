/*
 * odometry.cpp
 *
 *  Created on: Sep 22, 2013
 *      Author: robo
 */

/*
 * controller.cpp
 *
 *  Created on: Sep 18, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Encoders.h>
#include <differential_drive/Odometry.h>
#include "headers/odometry.h"
#include <visualization_msgs/Marker.h>

using namespace differential_drive;


void receive_encoder(const Encoders::ConstPtr &msg)
{
	static double x,y;
	static double theta;

	int delta_right = msg->delta_encoder2;
	int delta_left = msg->delta_encoder1;

	x += cos(theta)*r/2*(delta_right+delta_left)/ticks_rev*2*M_PI;
	y += sin(theta)*r/2*(delta_right+delta_left)/ticks_rev*2*M_PI;
	theta += -r/2/l*(delta_right-delta_left)/ticks_rev*2*M_PI;

	printf("x = %f, y = %f, theta = %f\n",x,y,theta/M_PI*180);

	while((theta > M_PI) | (theta <= -M_PI))
	{
		if(theta > 0)
		{
			theta -= 2*M_PI;
		}
		else
		{
			theta += 2*M_PI;
		}
	}

	Odometry odometry;
	odometry.x = x;
	odometry.y = y;
	odometry.theta = theta;
	odom_pub.publish(odometry);


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

	marker.pose.orientation.x = cos(theta/2);
	marker.pose.orientation.y = sin(theta/2);
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 0;

	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 0.3;

	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry");
	ros::NodeHandle nh;
	odom_pub = nh.advertise<Odometry>("/odometry", 100);
	enc_sub = nh.subscribe("/motors/encoders/",1000,receive_encoder);
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}



