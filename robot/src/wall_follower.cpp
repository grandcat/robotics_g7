/*
 * wall_follower.cpp
 *
 *  Created on: Oct 16, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <differential_drive/Speed.h>
#include <differential_drive/Odometry.h>
#include <differential_drive/AnalogC.h>
#include "headers/wall_follower.h"

using namespace differential_drive;


void receive_odom(const Odometry::ConstPtr &msg)
{

}


double angle(double theta)
{
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

	return theta;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "wall_follower");
	ros::NodeHandle nh;
	speed_pub = nh.advertise<Speed>("/motion/Speed", 100);
	odom_sub = nh.subscribe("/motion/Odometry",1000,receive_odom);
	sensors_sub = nh.subscribe("/sensors/ADC",1000,receive_sensors);

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
