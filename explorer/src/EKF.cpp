/*
 * EKF.cpp
 *
 *  Created on: Nov 17, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <differential_drive/Encoders.h>
#include <differential_drive/AnalogC.h>
#include <differential_drive/Speed.h>
#include <differential_drive/Odometry.h>
#include "Eigen/Dense"
#include "explorer/EKF.h"
#include "explorer/Stop_EKF.h"
#include "headers/EKF.h"
#include "headers/parameters.h"

using namespace differential_drive;
using namespace explorer;
using namespace Eigen;


/**
 * Compute the odometry and the G matrix for the EKF
 * @param msg
 */
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


/**
 * Compute the EKF algorithm if there is a wall
 * Publish the odometry and wall position
 * @param msg
 */
void receive_sensors(const AnalogC::ConstPtr &msg)
{
	double s1 = a_short*pow(msg->ch1,b_short); // left
	double s2 = a_short*pow(msg->ch2,b_short); // right

	static double s0,x_s0,y_s0;
	if((s1 < 0.2) & !wall & (s1 < s2))
	{
		init();
		x_s0 = x_s1;
		y_s0 = y_s1;
		right_sensor = false;
		wall = true;
	}
	if((s2 < 0.2) & !wall & (s2 < s1))
	{
		init();
		x_s0 = x_s2;
		y_s0 = y_s2;
		right_sensor = true;
		wall = true;
	}

	// Sensor measurement
	if(wall & right_sensor)
	{
		s0 = -s2;
	}
	if(wall & !right_sensor)
	{
		s0 = s1;
	}

	// Init y_wall_bar
	if((y_wall_bar == 0) & wall)
	{
		y_wall_bar = s0 + y_s0;
	}

	// Prediction
	double s0_hat = (y_wall_bar-y_bar-x_s0*sin(theta_bar)-y_s0*cos(theta_bar))/cos(theta_bar);
	double diff = s0 - s0_hat;

	if((diff*diff > 0.03*0.03) | (y_wall*y_wall > 0.3*0.3))
	{
		wall = false;
	}

	if(!stop & wall)
	{
		// Prediction
		sigma_bar = G*sigma*G.transpose() + R;

		// Correction
		H(0,1) = -1/cos(theta_bar);
		H(0,2) = ((y_wall_bar-y_bar)*sin(theta_bar)-x_s0)/cos(theta_bar)/cos(theta_bar);
		H(0,3) = 1/cos(theta_bar);

		MatrixXd S = H*sigma_bar*H.transpose();
		double s = S(0,0) + Q;
		K = sigma_bar*H.transpose()/s;

		MatrixXd mu_bar(4,1);
		mu_bar = K*(s0-s0_hat);
		x_bar += mu_bar(0,0);
		y_bar += mu_bar(1,0);
		theta_bar += mu_bar(2,0);
		y_wall_bar += mu_bar(3,0);

		sigma_bar = (MatrixXd::Identity(4,4)-K*H)*sigma_bar;
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
	ekf.right_sensor = right_sensor;
	ekf.wall = wall;
	EKF_pub.publish(ekf);

	// Publish odometry to visualize it
	Odometry odometry;
	odometry.x = x_true + x*cos(theta_true) - y*sin(theta_true);
	odometry.y = y_true + x*sin(theta_true) + y*cos(theta_true);
	odometry.theta = angle(theta_true + theta);
	Odometry_pub.publish(odometry);
}


/**
 * Receive a message to stop the EKF
 * First message means that there is an obstacle
 * Second message means that the rotation is done, and we can restart the EKF
 *
 * @param msg Rotation
 */
void receive_stop(const Stop_EKF::ConstPtr &msg)
{
	stop = msg->stop;
	rotation_angle = msg->rotation_angle;

	// Rotation done
	if(!stop)
	{
		rotate(rotation_angle);
	}
}


/**
 * Save the odometry before reinitialising
 * @param rotation_angle Rotation
 */
void rotate(double rotation_angle)
{
	// Save true values
	x_true += x*cos(theta_true) - y*sin(theta_true);
	y_true += x*sin(theta_true) + y*cos(theta_true);

	// Update theta_true
	if(rotation_angle == 0) {theta_true += theta_bar;}
	else {theta_true += rotation_angle;}
	theta_true = angle(theta_true);

	// Reset
	init();
	x_bar = y_bar = theta_bar = 0;
}


/**
 * Angle between ]-pi,pi]
 * @param th
 * @return
 */
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


void init()
{
	sigma = 1E-10 * MatrixXd::Identity(4,4);
	sigma(2,2) = 1E-6; //-4
	sigma(3,3) = 1E-6;
	R = 1E-10 * MatrixXd::Identity(4,4);
	R(3,3) = 1E-8;
	Q = 1E-8;
	G = MatrixXd::Identity(4,4);
	y_wall_bar = 0;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "EKF");
	ros::NodeHandle nh;
	enc_sub = nh.subscribe("/motion/Encoders",1000,receive_enc);
	sensors_sub = nh.subscribe("/sensors/ADC",1000,receive_sensors);
	stop_EKF_sub = nh.subscribe("/motion/Stop_EKF",1000,receive_stop);
	EKF_pub = nh.advertise<EKF>("/motion/EKF",100);
	Odometry_pub = nh.advertise<Odometry>("/motion/Odometry",100);


	// Init
	init();


	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}


