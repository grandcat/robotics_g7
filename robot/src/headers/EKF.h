/*
 * EKF.h
 *
 *  Created on: Oct 16, 2013
 *      Author: robo
 */

#ifndef EKF_H_
#define EKF_H_

#include <ros/ros.h>
#include <vector>

using namespace differential_drive;
using namespace std;

typedef vector<double> Vec;
typedef vector<Vec> Mat;


ros::Subscriber enc_sub;
ros::Subscriber sensors_sub;


double x,y,theta,y_wall;
double x_bar,y_bar,theta_bar,y_wall_bar;
Mat sigma,sigma_bar,K;
Mat R,Q;
Mat G,H;


double uniformRandom();

double normalRandom();

Mat operator*(const Mat &A, const Mat &B);

Mat operator+(const Mat &A, const Mat &B);

Mat transpose(const Mat &A);

Mat eye(int size);

void receive_enc(const Encoders::ConstPtr &msg);

void receive_sensors(const AnalogC::ConstPtr &msg);


#endif /* EKF_H_ */
