/*
 * phidgets.cpp
 *
 *  Created on: Nov 6, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include "headers/parameters.h"

using namespace sensor_msgs;


void receive_imu(const Imu::ConstPtr &msg)
{
	tf::Quaternion q;
	q.setX(msg->orientation.x);
	q.setY(msg->orientation.y);
	q.setZ(msg->orientation.z);
	q.setW(msg->orientation.w);

	tf::Matrix3x3 m(q);
	double roll,pitch, yaw;
	m.getRPY(roll,pitch,yaw);

	printf("roll = %f, pitch = %f, yaw = %f\n",roll,pitch,yaw);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "phidgets");
	ros::NodeHandle nh;
	ros::Subscriber imu_sub = nh.subscribe("/imu/data",1000,receive_imu);

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
