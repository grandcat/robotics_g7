/*
 * keyboard.cpp
 *
 *  Created on: Sep 17, 2013
 *      Author: robo
 */

#include <iostream>
#include <ros/ros.h>
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


int kfd = 0;
struct termios cooked, raw;


void arrowsCmd(ros::Publisher cmd_pub_)
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

		PWM cmd;
		cmd.PWM1 = cmd.PWM2 = 0;

		ROS_DEBUG("value: 0x%02X\n", c);

		switch(c)
		{
		case KEYCODE_L:
			ROS_DEBUG("LEFT");
			cmd.PWM1 = -100;
			cmd.PWM2 = 100;
			cmd.header.stamp = ros::Time::now();
			dirty = true;
			break;
		case KEYCODE_R:
			ROS_DEBUG("RIGHT");
			cmd.PWM1 = 100;
			cmd.PWM2 = -100;
			cmd.header.stamp = ros::Time::now();
			dirty = true;
			break;
		case KEYCODE_U:
			ROS_DEBUG("UP");
			cmd.PWM1 = 100;
			cmd.PWM2 = 100;
			cmd.header.stamp = ros::Time::now();
			dirty = true;
			break;
		case KEYCODE_D:
			ROS_DEBUG("DOWN");
			cmd.PWM1 = -100;
			cmd.PWM2 = -100;
			cmd.header.stamp = ros::Time::now();
			dirty = true;
			break;
		}

		if(dirty ==true)
		{
			cmd_pub_.publish(cmd);
			ros::Duration(0.02).sleep();
			cmd.PWM1 = 0;
			cmd.PWM2 = 0;
			cmd.header.stamp = ros::Time::now();
			cmd_pub_.publish(cmd);
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
	ros::init(argc, argv, "keyboard");
	ros::NodeHandle nh;
	ros::Publisher cmd_pub_ =  nh.advertise<PWM>("/motors/pwm", 100);

	signal(SIGINT,quit);
	arrowsCmd(cmd_pub_);

	return 0;
}
