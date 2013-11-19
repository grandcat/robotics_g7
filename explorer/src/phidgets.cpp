/*
 * phidgets.cpp
 *
 *  Created on: Nov 17, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include "headers/parameters.h"
#include <differential_drive/Encoders.h>

using namespace sensor_msgs;
using namespace differential_drive;


double x,y,z;
double dx,dy,dz;

int avg;
const int n = 3;
double T,T_old;

double speedV, speedW;
double speed1, speed2;

const double threshold = 0.1;


void receive_enc(const Encoders::ConstPtr &msg)
{
	// Compute current speed
	speed1 += msg->delta_encoder2*2*M_PI/ticks_rev/msg->timestamp/1E-3;
	speed2 += msg->delta_encoder1*2*M_PI/ticks_rev/msg->timestamp/1E-3;
	T += msg->timestamp*1E-3;
	avg++;
	if(avg == n)
	{
		speedV = r*(speed1/n/2 + speed2/n/2);
		speedW = r/l*(speed1/n - speed2/n);
		speed1 = 0;
		speed2 = 0;
		avg = 0;

		// Compare speeds
		if(speedV - dx > threshold)
		{
			printf("PROBLEM\n");
		}
	}
}


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


	x = msg->linear_acceleration.x;
	y = msg->linear_acceleration.y;
	z = msg->linear_acceleration.z;

	T = ros::Time::now().toSec() - T_old;
	T_old = ros::Time::now().toSec();

	// Speed
	dx += x*T;
	dy += y*T;
	dz += z*T;

	printf("x = %f, y = %f, z = %f\n",x,y,z);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "phidgets");
	ros::NodeHandle nh;
	ros::Subscriber imu_sub = nh.subscribe("/imu/data",1000,receive_imu);
	ros::Subscriber enc_sub = nh.subscribe("/motion/Encoders",1000,receive_enc);

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
