/*
 * wall_follower.cpp
 *
 *  Created on: Oct 16, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <differential_drive/Speed.h>
#include <differential_drive/AnalogC.h>
#include "robot/EKF.h"
#include "headers/wall_follower.h"

using namespace differential_drive;
using namespace robot;


void receive_EKF(const EKF::ConstPtr &msg)
{
	double x,y,y_wall;
	double theta;

	double diff_ang = 0;
	double diff_ang_cmd = 0;
	double dtheta = 0;
	double dist = 0;

	Speed speed;
	speed.W1 = 0;
	speed.W2 = 0;

	x = msg->x;
	y = msg->y;
	theta = msg->theta;
	y_wall = msg->y_wall;

	double x_cmd = x + x_cmd_traj;
	double y_cmd = y_wall + (y-y_wall)/abs(y-y_wall)*y_cmd_traj;

	diff_ang = atan((y_cmd-y)/(x_cmd-x))-theta;
	if((x_cmd-x) < 0)
	{
		if((y_cmd-y) > 0)
		{
			diff_ang += M_PI;
		}
		else
		{
			diff_ang -= M_PI;
		}

	}
	diff_ang = angle(diff_ang);

	dist = sqrt((x_cmd-x)*(x_cmd-x)+(y_cmd-y)*(y_cmd-y));

	if(flag)
	{
		if((dist > 4) & ((-diff_ang > M_PI/20) | (-diff_ang < -M_PI/20)))
		{
			dist = 0;
			diff_ang_cmd = 0;
		}

		speed.W1 = rho*dist-alpha*diff_ang;
		speed.W2 = rho*dist+alpha*diff_ang;
	}

	speed_pub.publish(speed);
}


void receive_sensors(const Encoders::ConstPtr &msg)
{
	// Obstacle
	// Rotate
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
	EKF_sub = nh.subscribe("/motion/EKF",1000,receive_EKF);
	sensors_sub = nh.subscribe("/sensors/ADC",1000,receive_sensors);

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
