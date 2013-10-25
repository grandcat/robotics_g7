/*
 * EKF.cpp
 *
 *  Created on: Oct 16, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <differential_drive/AnalogC.h>

using namespace differential_drive;


const double a = 17.43215;
const double b = -0.9035;
void receive_sensors(const AnalogC::ConstPtr &msg)
{
	int s1 = msg->ch1;
	printf("Range = %f\n",a*pow(s1,b));
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "EKF");
	ros::NodeHandle nh;
	ros::Subscriber sensors_sub = nh.subscribe("/sensors/ADC",1000,receive_sensors);

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}


