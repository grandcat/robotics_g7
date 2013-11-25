/*
 * keyboard.cpp
 *
 *  Created on: Nov 14, 2013
 *      Author: Andreas
 */

#include <iostream>
#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "std_msgs/String.h"


//using namespace differential_drive;

int a = 0;
int kfd = 0;
struct termios cooked, raw;

enum Filter_mode
{
	WALLS = 0,
	FLOOR = 1
};

Filter_mode mode;

void arrowsCmd(ros::Publisher msg_pub_)
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
	puts("Use H_min:(a,q) H_max(s,w) S_min(d,e) S_max(f,r) V_min(g,t) V_max(h,y) keys to change the HSV color filter.\n"
			"To change modes, press 1 or 2\n"
			"To stop the color filter algorithm, press z.");

	for(;;)
	{
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		//Change mode
		if (c == '1')
		{
			mode = WALLS;
			//puts("CHANGED TO WALLS");
		}

		else if(c == '2')
		{
			mode = FLOOR;
			//puts("CHANGED TO FLOOR");
		}

		std_msgs::String msg;
		switch(mode)
		{
			case WALLS:
			{
				msg.data = std::string("1")+c;
				msg_pub_.publish(msg);
				break;
			}
			case FLOOR:
			{
				msg.data = std::string("2")+c;
				msg_pub_.publish(msg);
				break;
			}
			default:
				break;
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
	mode = WALLS; //as default

	ros::NodeHandle nh;
	ros::Publisher msg_pub_ = nh.advertise<std_msgs::String>("/keyboard/input", 1);
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param("a", a, int(40));

	signal(SIGINT,quit);
	arrowsCmd(msg_pub_);

	return 0;
}
