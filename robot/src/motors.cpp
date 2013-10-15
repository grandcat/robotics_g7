/*
 * motors.cpp
 *
 *  Created on: Oct 15, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Speed.h>
#include <differential_drive/Encoders.h>
#include "headers/odometry.h"

using namespace differential_drive;


ros::Publisher pwm_pub;
ros::Subscriber speed_sub;
ros::Subscriber enc_sub_m;


double speed1, speed2;
double speed_instruction1, speed_instruction2;

int avg;
const int n = 10;

const double k = 3;
const double kI = 1;
const double int_max = 30;
double integral1, integral2;

void receive_enc(const Encoders::ConstPtr &msg)
{
	// Compute current speed
	speed1 += msg->delta_encoder1*2*M_PI/ticks_rev/msg->timestamp*1E-3;
	speed2 += msg->delta_encoder2*2*M_PI/ticks_rev/msg->timestamp*1E-3;
	avg++;
	if(avg == n)
	{
		speed1 = speed1/n;
		speed2 = speed2/n;
		speed1 = 0;
		speed2 = 0;
		avg = 0;
	}

	// Control the speed
	PWM pwm;
	double T = msg->timestamp;
	// Motor 1
	double error1 = speed_instruction1 - speed1;
	integral1 += error1*T;

	if(integral1 > int_max) {integral1 = int_max;}
	else if(integral1 < -int_max)  {integral1 = -int_max;}

	double u1 = k*error1 + kI*integral1;

	//Threshold
	if(u1 > 5) {u1 += 35;}
	else if(u1 < -5) {u1-=35;}

	if(u1 > 255) {u1 = 255;}
	else if(u1 < -255)  {u1 = -255;}

	pwm.PWM1 = u1;

	// Motor 2
	double error2 = speed_instruction2 - speed2;
	integral2 += error2*T;

	if(integral2 > int_max) {integral2 = int_max;}
	else if(integral2 < -int_max)  {integral2 = -int_max;}

	double u2 = k*error2 + kI*integral2;

	//Threshold
	if(u2 > 5) {u2 += 35;}
	else if(u2 < -5) {u2 -= 35;}

	if(u2 > 255) {u2 = 255;}
	else if(u2 < -255)  {u2 = -255;}

	pwm.PWM2 = u2;

	// Publish
	pwm.header.stamp = ros::Time::now();
	pwm_pub.publish(pwm);
}



void receive_speed(const Speed::ConstPtr &msg)
{
	speed_instruction1 = msg->W1;
	speed_instruction2 = msg->W2;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "motors");
	ros::NodeHandle nh;
	pwm_pub = nh.advertise<PWM>("/motion/PWM", 100);
	speed_sub = nh.subscribe("/motion/Speed",1000,receive_speed);
	enc_sub_m = nh.subscribe("/motion/Encoders",1000,receive_enc);

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
