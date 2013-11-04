/*
 * keyboard.cpp
 *
 *  Created on: Oct 15, 2013
 *      Author: robo
 */

#include <iostream>
#include <ros/ros.h>
#include "robot/SpeedVW.h"
#include <differential_drive/PWM.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

using namespace differential_drive;
using namespace robot;


int kfd = 0;
struct termios cooked, raw;


void arrowsCmd(ros::Publisher cmd_pub,ros::Publisher pwm_pub)
{
	char c;
	bool dirty=false;
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use arrow keys to move the robot.");

	for(;;)
	{
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		SpeedVW cmd;
		cmd.V = cmd.V = 0;

		PWM pwm;
		pwm.PWM1 = pwm.PWM2 = 0;

		ROS_DEBUG("value: 0x%02X\n", c);

		switch(c)
		{
		case KEYCODE_L:
			ROS_DEBUG("LEFT");
			cmd.V = 0;
			cmd.W = 0.3;
			dirty = true;
			break;
		case KEYCODE_R:
			ROS_DEBUG("RIGHT");
			cmd.V = 0;
			cmd.W = -0.3;
			dirty = true;
			break;
		case KEYCODE_U:
			ROS_DEBUG("UP");
			cmd.V = 0.05;
			cmd.W = 0;
			dirty = true;
			break;
		case KEYCODE_D:
			ROS_DEBUG("DOWN");
			cmd.V = 0;
			cmd.W = 0;
			dirty = true;
			break;
		}

		if(dirty ==true)
		{
			cmd_pub.publish(cmd);
			ros::Duration(0.2).sleep();

			//ros::Duration(0.1).sleep();
			//pwm.header.stamp = ros::Time::now();
			//pwm_pub.publish(pwm);

			dirty=false;
		}
	}
	return;
}


void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "keyboardVW");
	ros::NodeHandle nh;
	ros::Publisher cmd_pub = nh.advertise<SpeedVW>("/motion/SpeedVW",100);
	ros::Publisher pwm_pub = nh.advertise<PWM>("/motion/PWM",100);

	signal(SIGINT,quit);
	arrowsCmd(cmd_pub,pwm_pub);

	return 0;
}
