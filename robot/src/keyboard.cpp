/*
 * keyboard.cpp
 *
 *  Created on: Oct 15, 2013
 *      Author: robo
 */

#include <iostream>
#include <ros/ros.h>
#include <differential_drive/Speed.h>
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

		Speed cmd;
		cmd.W1 = cmd.W2 = 0;

		PWM pwm;
		pwm.PWM1 = pwm.PWM2 = 0;

		ROS_DEBUG("value: 0x%02X\n", c);

		switch(c)
		{
		case KEYCODE_L:
			ROS_DEBUG("LEFT");
			pwm.PWM1 = 100;
			pwm.PWM2 = 100;
			dirty = true;
			break;
		case KEYCODE_R:
			ROS_DEBUG("RIGHT");
			pwm.PWM1 = 0;
			pwm.PWM2 = 0;
			dirty = true;
			break;
		case KEYCODE_U:
			ROS_DEBUG("UP");
			cmd.W1 = 6;
			cmd.W2 = 6;
			dirty = true;
			break;
		case KEYCODE_D:
			ROS_DEBUG("DOWN");
			cmd.W1 = 0;
			cmd.W2 = 0;
			dirty = true;
			break;
		}

		if(dirty ==true)
		{
			ros::Duration(0.1).sleep();
			cmd.header.stamp = ros::Time::now();
			cmd_pub.publish(cmd);

			ros::Duration(0.1).sleep();
			pwm.header.stamp = ros::Time::now();
			pwm_pub.publish(pwm);

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
	ros::Publisher cmd_pub = nh.advertise<Speed>("/motion/Speed",100);
	ros::Publisher pwm_pub = nh.advertise<PWM>("/motion/PWM",100);

	signal(SIGINT,quit);
	arrowsCmd(cmd_pub,pwm_pub);

	return 0;
}
