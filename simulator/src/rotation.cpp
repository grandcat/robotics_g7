/*
 * rotation.cpp
 *
 *  Created on: Oct 13, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Odometry.h>
#include "headers/rotation.h"

using namespace differential_drive;


void receive_odom(const Odometry::ConstPtr &msg)
{
	double x,y;
	double theta;

	double dtheta_cmd = 0;
	double dist_cmd = 0;

	double dtheta = 0;
	double dist = 0;

	static double dtheta_cmd1;
	static double dtheta_cmd2;
	static double dist_cmd1;
	static double dist_cmd2;

	static double dtheta1;
	static double dtheta2;
	static double dist1;
	static double dist2;

	PWM pwm;

	x = msg->x;
	y = msg->y;
	theta = msg->theta;

	dist = -(y + L*sin(theta));
	dist_cmd = a11*dist_cmd1 + a21*dist_cmd + a02*dist + a12*dist1 + a22*dist2;

	dtheta = theta_cmd-theta;
	dtheta_cmd = b11*dtheta_cmd1 + b21*dtheta_cmd2 + b02*dtheta + b12*dtheta1 + b22*dtheta2;

	//printf("dtheta_cmd = %f, dist_cmd = %f\n",dtheta_cmd,dist_cmd);
	//printf("d1 = %f, d2 = %f\n",dist_cmd1,dist_cmd2);

	if(!((1000*dtheta*dtheta < error) & (sqrt(dist*dist) < error)))
	{
		pwm.PWM2 = rho*dist_cmd+alpha*dtheta_cmd;
		pwm.PWM1 = rho*dist_cmd-alpha*dtheta_cmd;
		pwm.header.stamp = ros::Time::now();
		cmd_pub.publish(pwm);
	}
	else
	{
		if(flag)
		{
			printf("DONE\n");
			flag = false;

			pwm.PWM1 = 0;
			pwm.PWM2 = 0;
			pwm.header.stamp = ros::Time::now();
			cmd_pub.publish(pwm);
		}
	}

	dist2 = dist1;
	dist1 = dist;
	dist_cmd2 = dist_cmd1;
	dist_cmd1 = dist_cmd;

	dtheta2 = dtheta1;
	dtheta1 = dtheta;
	dtheta_cmd2 = dtheta_cmd1;
	dtheta_cmd1 = dtheta_cmd;
}


void enterCmd()
{
	char l[50];
	std::cout << "theta =\n";
	std::cin.getline(l,50);
	theta_cmd = atof(l);
	theta_cmd = theta_cmd/180*M_PI;

	flag = true;

	return;
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
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	cmd_pub = nh.advertise<PWM>("/motors/pwm", 100);
	odom_sub = nh.subscribe("/odometry",1000,receive_odom);

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		if(!flag)
		{
			enterCmd();
		}

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}


