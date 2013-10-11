/*
 * controller.cpp
 *
 *  Created on: Sep 18, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Odometry.h>
#include "headers/controller.h"

using namespace differential_drive;


void receive_odom(const Odometry::ConstPtr &msg)
{
	double x,y;
	double theta;

	double diff_ang = 0;
	double diff_ang_cmd = 0;
	double dtheta = 0;
	double dist = 0;

	PWM pwm;

	x = msg->x;
	y = msg->y;
	theta = msg->theta;

	diff_ang = atan((y_cmd-y)/(x_cmd-x))-theta;
	diff_ang_cmd = atan((y_cmd-y)/(x_cmd-x))-theta_cmd;
	if((x_cmd-x) < 0)
	{
		if((y_cmd-y) > 0)
		{
			diff_ang += M_PI;
			diff_ang_cmd += M_PI;
		}
		else
		{
			diff_ang -= M_PI;
			diff_ang_cmd -= M_PI;
		}

	}
	diff_ang = angle(diff_ang);
	diff_ang_cmd = angle(diff_ang_cmd);

	dist = sqrt((y_cmd-y)*(y_cmd-y)+(x_cmd-x)*(x_cmd-x));

	//printf("diff_ang = %f\n",-diff_ang/M_PI*180);
	//printf("diff_ang_cmd = %f\n",-diff_ang_cmd/M_PI*180);

	dtheta = theta-theta_cmd;
	dtheta = angle(dtheta);
	//printf("theta = %f theta_cmd = %f dtheta = %f\n",theta,theta_cmd,dtheta);
	if((100*dtheta*dtheta > error) | (dist > error))
	{
		if((dist > 4) & ((-diff_ang > M_PI/20) | (-diff_ang < -M_PI/20)))
		{
			dist = 0;
			diff_ang_cmd = 0;
		}

		pwm.PWM2 = rho*(dist+0.25)-alpha*diff_ang-beta*diff_ang_cmd;
		pwm.PWM1 = rho*(dist+0.25)+alpha*diff_ang+beta*diff_ang_cmd;
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
}


void enterCmd()
{
	char l[50];
	std::cout << "x =\n";
	std::cin.getline(l,50);
	x_cmd = atof(l);

	std::cout << "y =\n";
	std::cin.getline(l,50);
	y_cmd = atof(l);

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
