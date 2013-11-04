/*
 * IR.cpp
 *
 *  Created on: Oct 25, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <differential_drive/AnalogC.h>
#include "headers/parameters.h"

using namespace differential_drive;


void receive_sensors(const AnalogC::ConstPtr &msg)
{
	int s1 = msg->ch2;
	printf("Range 1 = %f\n",a_short*pow(s1,b_short));
	int s3 = msg->ch3;
	//printf("Range 3 = %f\n",a_long*pow(s3,b_long));
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "EKF");
	ros::NodeHandle nh;
	ros::Subscriber sensors_sub = nh.subscribe("/sensors/ADC",1000,receive_sensors);

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
