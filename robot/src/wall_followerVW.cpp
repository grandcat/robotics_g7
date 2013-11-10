/*
 * wall_followerVW.cpp
 *
 *  Created on: Oct 16, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include "robot/SpeedVW.h"
#include <differential_drive/AnalogC.h>
#include "robot/EKF.h"
#include "robot/Rotate.h"
#include "headers/parameters.h"
#include "headers/wall_followerVW.h"

using namespace differential_drive;
using namespace robot;


/**
 * Receive the robot and wall position
 * Follow the wall or rotate
 * @param msg
 */
void receive_EKF(const EKF::ConstPtr &msg)
{
	double y,y_wall;
	double theta;

	double diff_ang = 0;
	double dist = 0;

	SpeedVW speed;
	speed.V = 0;
	speed.W = 0;

	x = msg->x;
	y = msg->y;
	theta = msg->theta;
	y_wall = msg->y_wall;

	double x_cmd = x + x_cmd_traj;
	double y_cmd;
	if(msg->right_sensor) {y_cmd = y_wall + y_cmd_traj;}
	else {y_cmd = y_wall - y_cmd_traj;}
	if(!msg->wall | hurt_wall) {y_cmd = y;}

	dist = x_cmd_traj;


	if(hurt_wall)
	{
		// Go backward
		dist = x_collision-x-x_backward_dist;
		x_cmd = x - x_cmd_traj;

		// Done
		if((dist*dist < x_error*x_error) & !backward)
		{
			backward = true;
		}

		// Rotate 90
		if(backward & !first_rotation)
		{
			obstacle = true;
			first_rotation = true;
		}

		// Rotate 90
		if(backward & !obstacle)
		{
			obstacle = true;

			// Send a message to EKF
			right = true;
			Rotate r;
			r.right = right;
			rotate_pub.publish(r);

			// Re-init
			backward = false;
			hurt_wall = false;
			first_rotation = false;
		}
	}


	diff_ang = atan((y_cmd-y)/(x_cmd-x))-theta;
	diff_ang = angle(diff_ang);


	if(!obstacle & !hurt_wall)
	{
		speed.V = rho*dist*r;
		speed.W = -2*r/l*alpha*diff_ang;
	}

	if(!obstacle & hurt_wall)
	{
		speed.V = 5*rho*dist*r;
		speed.W = -2*r/l*alpha*diff_ang;
		//printf("Backward\n");
	}

	if(obstacle)
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

		speed.V = 0;
		speed.W = 2*r/l*alpha*dtheta/4;
	}


	speed_pub.publish(speed);


	//printf("obstacle = %s, hurt_wall = %s\n",(obstacle)?"true":"false",(hurt_wall)?"true":"false");
}


/**
 * Check if there is an obstacle
 * @param msg
 */
void receive_sensors(const AnalogC::ConstPtr &msg)
{
	// IR sensor
	double s1 = a_short*pow(msg->ch1,b_short);
	double s2 = a_short*pow(msg->ch2,b_short);
	double s3 = a_long*pow(msg->ch3,b_long);

	// Bumpers
	bool s6 = (msg->ch6 > bumper_threshold);
	bool s7 = (msg->ch7 > bumper_threshold);
	bool s8 = (msg->ch8 > bumper_threshold);

	// Hurt a wall
	if((s6 | s7 | s8) & !obstacle & !hurt_wall)
	{
		hurt_wall = true;
		x_collision = x;

		theta_cmd = -M_PI/2;

		// Send a message to EKF
		right = true;
		Rotate r;
		r.right = right;
		rotate_pub.publish(r);
	}

	// Obstacle
	if((s3 < dist_obstacle) & !obstacle & !hurt_wall)
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


/**
 * Angle between ]-pi,pi]
 * @param th
 * @return
 */
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
	ros::init(argc, argv, "wall_followerVW");
	ros::NodeHandle nh;
	speed_pub = nh.advertise<SpeedVW>("/motion/SpeedVW",100);
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
