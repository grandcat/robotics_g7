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
#include "robot/Rotate.h"
#include "headers/parameters.h"
#include "headers/wall_follower.h"

using namespace differential_drive;
using namespace robot;


void receive_EKF(const EKF::ConstPtr &msg)
{
	double x,y,y_wall;
	double theta;

	double diff_ang = 0;
	double dist = 0;

	Speed speed;
	speed.W1 = 0;
	speed.W2 = 0;

	x = msg->x;
	y = msg->y;
	theta = msg->theta;
	y_wall = msg->y_wall;

	double x_cmd = x + x_cmd_traj;
	double y_cmd;
	if(msg->right_sensor) {y_cmd = y_wall + y_cmd_traj;}
	else {y_cmd = y_wall - y_cmd_traj;}
	if(msg->wall) {y_cmd = y;}

	diff_ang = atan((y_cmd-y)/(x_cmd-x))-theta;
	diff_ang = angle(diff_ang);

	dist = x_cmd_traj;

	if(!obstacle)
	{
		speed.W1 = rho*dist-alpha*diff_ang;
		speed.W2 = rho*dist+alpha*diff_ang;
	}
	else
	{
		double dtheta = theta - theta_cmd;
		dtheta = angle(dtheta);
		//printf("dtheta = %f\n",dtheta);

		// Rotation done
		if(dtheta*dtheta < M_PI*M_PI/180/180*theta_error*theta_error)
		{
			obstacle = false;
			Rotate r;
			r.right = right;
			rotate_pub.publish(r);
		}

		speed.W1 = alpha*dtheta/4; //3
		speed.W2 = -(alpha*dtheta/4); //3
	}

	speed_pub.publish(speed);
}


void receive_sensors(const AnalogC::ConstPtr &msg)
{
	double s1 = a_short*pow(msg->ch1,b_short);
	double s2 = a_short*pow(msg->ch2,b_short);
	double s3 = a_long*pow(msg->ch3,b_long);

	// Obstacle
	if((s3 < dist_obstacle) & !obstacle)
	{
		obstacle = true;

		if(s1 < s2)
		{
			theta_cmd = -M_PI/2;
			right = true;
		}
		else
		{
			theta_cmd = M_PI/2;
			right = false;
		}


		// Send a message to EKF
		Rotate r;
		r.right = right;
		rotate_pub.publish(r);
	}
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
	speed_pub = nh.advertise<Speed>("/motion/Speed",100);
	rotate_pub =nh.advertise<Rotate>("/motion/Rotate",100);
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
