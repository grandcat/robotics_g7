/*
 * controller.cpp
 *
 *  Created on: Sep 18, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <differential_drive/Speed.h>
#include <camera/Position.h>

using namespace differential_drive;
using namespace camera;

ros::Publisher cmd_pub;


const int depth0 = 550;
const float alpha = 0.04;
const float beta = 0.04;
void receive_cam(const Position::ConstPtr &msg)
{
	int pos = msg->x;
	int depth = 0;

	if(msg->depth != 0 & msg->depth < 850)
	{
		depth = depth0 - msg->depth;
	}

	Speed speed_msg;
	speed_msg.W1 = -alpha*pos - beta*depth;
	speed_msg.W2 = alpha*pos - beta*depth;
	speed_msg.header.stamp = ros::Time::now();
	cmd_pub.publish(speed_msg);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "Controller");
	ros::NodeHandle nh;
	cmd_pub = nh.advertise<Speed>("/motion/Speed",100);
	ros::Subscriber pos_sub = nh.subscribe("/pos",100,receive_cam);

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
