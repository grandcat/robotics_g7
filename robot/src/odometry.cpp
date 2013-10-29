/*
 * odometry.cpp
 *
 *  Created on: Oct 15, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <differential_drive/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include "headers/odometry.h"
#include "headers/parameters.h"

using namespace differential_drive;


/*
void receive_encoder(const Encoders::ConstPtr &msg)
{
	static double x,y;
	static double theta;

	int delta_right = msg->delta_encoder2;
	int delta_left = msg->delta_encoder1;

	x += cos(theta)*r/2*(delta_right+delta_left)/ticks_rev*2*M_PI;
	y += sin(theta)*r/2*(delta_right+delta_left)/ticks_rev*2*M_PI;
	theta += -r/l*(delta_right-delta_left)/ticks_rev*2*M_PI; // verif

	// printf("x = %f, y = %f, theta = %f\n",x,y,theta/M_PI*180);

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
}
*/


void receive_odometry(const Odometry::ConstPtr &msg)
{
	double x,y,theta;
	x = msg->x;
	y = msg->y;
	theta = msg->theta;

	// Visualization
	uint32_t shape = visualization_msgs::Marker::ARROW;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";

	static int i;
	marker.id = i;
	i++;

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
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry");
	ros::NodeHandle nh;
	//odom_pub = nh.advertise<Odometry>("/motion/Odometry", 100);
	//enc_sub = nh.subscribe("/motion/Encoders",1000,receive_encoder);
	odom_sub = nh.subscribe("/motion/Odometry",1000,receive_odometry);
	marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}



