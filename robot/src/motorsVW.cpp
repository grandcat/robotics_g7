/*
 * motorsVW.cpp
 *
 *  Created on: Nov 2, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Encoders.h>
#include "headers/parameters.h"
#include "robot/SpeedVW.h"

using namespace differential_drive;
using namespace robot;


ros::Publisher pwm_pub;
ros::Subscriber speed_sub;
ros::Subscriber enc_sub_m;


double speedV, speedW;
double speed_instructionV, speed_instructionW;
double speed1, speed2;

int avg;
const int n = 5;
double T;


const double k_V = 10;
const double kI_V = 20;
const double kD_V = 0.00015;

const double k_W = 10;
const double kI_W = 20;
const double kD_W = 0.00015;


const double int_max = 130;

double integralV, integralW;
double p_errorV, p_errorW;

void receive_enc(const Encoders::ConstPtr &msg)
{
	// Compute current speed
	speed1 += msg->delta_encoder2*2*M_PI/ticks_rev/msg->timestamp/1E-3;
	speed2 += msg->delta_encoder1*2*M_PI/ticks_rev/msg->timestamp/1E-3;
	T += msg->timestamp*1E-3;
	avg++;
	if(avg == n)
	{
		speedV = r*(speed1/n/2 + speed2/n/2);
		speedW = r/l*(speed1/n - speed2/n);
		speed1 = 0;
		speed2 = 0;
		avg = 0;

		// Control the speed
		PWM pwm;

		// V
		double errorV = speed_instructionV - speedV;
		integralV += kI_V*errorV*T;
		double derivativeV = (errorV-p_errorV)/T;
		p_errorV = errorV;

		printf("errorV = %f, deriv = %f, current = %f, int = %f\n",errorV,derivativeV,speedV,integralV);

		if(integralV > int_max) {integralV = int_max;}
		else if(integralV < -int_max)  {integralV = -int_max;}

		double V = k_V*errorV + integralV + kD_V*derivativeV;


		// W
		double errorW = speed_instructionW - speedW;
		integralW += kI_W*errorW*T;
		double derivativeW = (errorW-p_errorW)/T;
		p_errorW = errorW;

		printf("errorW = %f, deriv = %f, current = %f, int = %f\n",errorW,derivativeW,speedW,integralW);

		if(integralW > int_max) {integralW = int_max;}
		else if(integralW < -int_max)  {integralW = -int_max;}

		double W = k_W*errorW + integralW + kD_W*derivativeW;


		double u1 = V/r + W*l/r/2;
		double u2 = V/r - W*l/r/2;


		// Threshold
		if(u1 > 5) {u1 += 35;}
		if(u1 < -5)  {u1 -= -35;}

		if(u1 > 255) {u1 = 255;}
		else if(u1 < -255)  {u1 = -255;}

		pwm.PWM1 = u1;


		// Threshold
		if(u2 > 5) {u2 += 35;}
		if(u2 < -5)  {u2 -= -35;}

		if(u2 > 255) {u2 = 255;}
		else if(u2 < -255)  {u2 = -255;}

		pwm.PWM2 = -u2;


		// Publish
		pwm.header.stamp = ros::Time::now();
		pwm_pub.publish(pwm);

		T = 0;
	}
}



void receive_speed(const SpeedVW::ConstPtr &msg)
{
	speed_instructionV = msg->V;
	speed_instructionW = msg->W;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "motorsVW");
	ros::NodeHandle nh;
	pwm_pub = nh.advertise<PWM>("/motion/PWM", 1);
	speed_sub = nh.subscribe("/motion/SpeedVW",1000,receive_speed);
	enc_sub_m = nh.subscribe("/motion/Encoders",1000,receive_enc);

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
