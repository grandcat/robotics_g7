/*
 * EKF.cpp
 *
 *  Created on: Oct 16, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <differential_drive/Encoders.h>
#include <differential_drive/AnalogC.h>
#include <differential_drive/Speed.h>
#include <differential_drive/Odometry.h>
#include "Eigen/Dense"
#include "robot/EKF.h"
#include "robot/Rotate.h"
#include "headers/EKF.h"
#include "headers/parameters.h"

using namespace differential_drive;
using namespace robot;
using namespace Eigen;


void receive_enc(const Encoders::ConstPtr &msg)
{
	int delta_right = msg->delta_encoder2;
	int delta_left = msg->delta_encoder1;

	x_bar += cos(theta)*r/2*(delta_right+delta_left)/ticks_rev*2*M_PI;
	y_bar += sin(theta)*r/2*(delta_right+delta_left)/ticks_rev*2*M_PI;
	theta_bar += -r/l*(delta_right-delta_left)/ticks_rev*2*M_PI;
	y_wall_bar += 0;

	G(1,3) += -sin(theta)*r/2*(delta_right+delta_left)/ticks_rev*2*M_PI;
	G(2,3) += cos(theta)*r/2*(delta_right+delta_left)/ticks_rev*2*M_PI;
}


void receive_sensors(const AnalogC::ConstPtr &msg)
{
	double s1 = a_short*pow(msg->ch1,b_short); // left
	double s2 = a_short*pow(msg->ch1,b_short); // right

	// Init
	if(y_wall_bar == 0)
	{
		y_wall_bar = s1+y_s1;
	}

	if(!flag)
	{
		// Prediction
		sigma_bar = G*sigma*G.transpose() + R;

		// Correction
		double s1_hat = (y_wall_bar-y_bar-x_s1*sin(theta_bar)-y_s1*cos(theta_bar))/cos(theta_bar);

		H(0,1) = -1/cos(theta_bar);
		H(0,2) = ((y_wall_bar-y_bar)*sin(theta_bar)-x_s1)/cos(theta_bar)/cos(theta_bar);
		H(0,3) = 1/cos(theta_bar);

		MatrixXd S = H*sigma_bar*H.transpose();
		double s = S(0,0) + Q;
		K = sigma_bar*H.transpose()/s;

		MatrixXd mu_bar(4,1);
		mu_bar = K*(s1-s1_hat);
		x_bar += mu_bar(0,0);
		y_bar += mu_bar(1,0);
		theta_bar += mu_bar(2,0);
		y_wall_bar += mu_bar(3,0);

		sigma_bar = (MatrixXd::Identity(4,4)-K*H)*sigma_bar;

		// Debug
		printf("s1 = %f, s1_hat = %f\n",s1,s1_hat);
		//printf("K(0,0) = %f, K(1,0) = %f, K(2,0) = %f, K(3,0) = %f\n",K(0,0),K(1,0),K(2,0),K(3,0));
	}

	x = x_bar;
	y = y_bar;
	theta = theta_bar;
	y_wall = y_wall_bar;

	sigma = sigma_bar;

	G(1,3) = 0;
	G(2,3) = 0;

	// Publish
	EKF ekf;
	ekf.x = x;
	ekf.y = y;
	ekf.theta = theta;
	ekf.y_wall = y_wall;
	EKF_pub.publish(ekf);
}


void receive_rotate(const Rotate::ConstPtr &msg)
{
	right = msg->right;

	// Rotation done
	if(flag)
	{
		rotate(right);
	}

	flag = !flag;
}


void rotate(bool right)
{
	// Save true values
	if(theta_true == 0)
	{
		x_true += x;
		y_true += y;
	}
	if(theta_true == M_PI/2)
	{
		x_true += -y;
		y_true += x;
	}
	if(theta_true == -M_PI/2)
	{
		x_true += y;
		y_true += -x;
	}
	if(theta_true == M_PI)
	{
		x_true += -x;
		y_true += -y;
	}

	if(right) {theta_true -= M_PI/2;}
	else {theta_true += M_PI/2;}
	theta_true = angle(theta_true);

	//Publish
	Odometry odometry;
	odometry.x = x_true;
	odometry.y = y_true;
	odometry.theta = theta_true;
	Odometry_pub.publish(odometry);

	// Reset
	x_bar = y_bar = theta_bar = y_wall_bar = 0;
}


double angle(double th)
{
	while((th > M_PI) | (th <= -M_PI))
	{
		if(th > 0)
		{
			th -= 2*M_PI;
		}
		else
		{
			th += 2*M_PI;
		}
	}

	return th;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "EKF");
	ros::NodeHandle nh;
	enc_sub = nh.subscribe("/motion/Encoders",1000,receive_enc);
	sensors_sub = nh.subscribe("/sensors/ADC",1000,receive_sensors);
	rotate_sub = nh.subscribe("/motion/Rotate",1000,receive_rotate);
	EKF_pub = nh.advertise<EKF>("/motion/EKF",100);
	Odometry_pub = nh.advertise<EKF>("/motion/Odometry",100);


	// Init
	sigma = 1E-8 * MatrixXd::Identity(4,4);
	R = 1E-8 * MatrixXd::Identity(4,4);
	R(3,3) = 1E-2;
	Q = 1E-4;
	G = MatrixXd::Identity(4,4);


	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}


