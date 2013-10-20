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
#include "headers/EKF.h"
#include "headers/parameters.h"

using namespace differential_drive;


double uniformRandom()
{
	return (double)(rand())/(double)(RAND_MAX);
}


double normalRandom()
{
	// Box-Muller transform
	double u1 = uniformRandom();
	double u2 = uniformRandom();
	return cos(2*M_PI*u2)*sqrt(-2.*log(u1));
}


Mat operator*(const Mat &A, const Mat &B)
{
	Mat res;
	int sizeA = A.size();
	int sizeB = B.at(0).size();
	int size = B.size();
	for(int i = 0; i < sizeA; i++)
	{
		for(int j = 0; j < sizeB; j++)
		{
			for(int n = 0; n < size; n++)
			{
				res[i][j] += A[i][n]*B[n][j];
			}
		}
	}
	return res;
}


Mat transpose(const Mat &A)
{
	Mat res;
	int sizeR = A.size();
	int sizeC = A.at(0).size();
	for(int i = 0; i < sizeC; i++)
	{
		for(int j = 0; j < sizeR; j++)
		{
			res[i][j] = A[j][i];
		}
	}
	return res;
}


Mat operator+(const Mat &A, const Mat &B)
{
	Mat res;
	int size = A.size();
	for(int i = 0; i < size; i++)
	{
		for(int j = 0; j < size; j++)
		{
			res[i][j] = A[i][j] + B[i][j];
		}
	}
	return res;
}


Mat inv(const Mat &A)
{
	Mat S;
	return S;
}


Mat eye(int size)
{
	Mat I;
	for(int i = 0; i < size; i++)
	{
		I[i][i] = 1;
	}
	return I;
}


void receive_enc(const Encoders::ConstPtr &msg)
{
	int delta_right = msg->delta_encoder2;
	int delta_left = msg->delta_encoder1;

	x_bar += cos(theta)*r/2*(delta_right+delta_left)/ticks_rev*2*M_PI;
	y_bar += sin(theta)*r/2*(delta_right+delta_left)/ticks_rev*2*M_PI;
	theta_bar += -r/2/l*(delta_right-delta_left)/ticks_rev*2*M_PI;
	y_wall_bar += 0;

	G[1][3] += -sin(theta)*r/2*(delta_right+delta_left)/ticks_rev*2*M_PI;
	G[2][3] += cos(theta)*r/2*(delta_right+delta_left)/ticks_rev*2*M_PI;
}


void receive_sensors(const AnalogC::ConstPtr &msg)
{
	int s1 = msg->ch1; // left
	int s2 = msg->ch2;
	int s3 = msg->ch3; // right
	int s4 = msg->ch4;

	// Prediction
	sigma_bar = G*sigma*transpose(G) + R;

	// Correction
	double s1_hat = (y_wall_bar-y_bar)/cos(theta_bar);

	H[0][1] = -1/cos(theta_bar);
	H[0][2] = (y_wall_bar-y_bar)*sin(theta_bar)/cos(theta_bar)/cos(theta_bar);
	H[0][3] = 1/cos(theta_bar);

	Mat S = H*sigma_bar*transpose(H) + Q;
	K = sigma_bar*transpose(H)*inv(S);

	Mat mu_bar = K*(s1-s1_hat);
	x_bar += mu_bar[0][0];
	y_bar += mu_bar[1][0];
	theta_bar += mu_bar[2][0];
	y_wall += mu_bar[3][0];

	sigma_bar = (eye(4)-K*H)*sigma_bar;

	x = x_bar;
	y = y_bar;
	theta = theta_bar;
	y_wall = y_wall_bar;

	sigma = sigma_bar;

	G[1][3] = 0;
	G[2][3] = 0;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "EKF");
	ros::NodeHandle nh;
	enc_sub = nh.subscribe("/motion/Encoders",1000,receive_enc);
	sensors_sub = nh.subscribe("/sensors/ADC",1000,receive_sensors);


	// Init
	for(int i = 0; i < 4; i++)
	{
		sigma[i][i] = 1E-6;
		R[i][i] = 1E-6;
		Q[i][i] = 1E-4;
		G[i][i] = 1;
	}


	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}


