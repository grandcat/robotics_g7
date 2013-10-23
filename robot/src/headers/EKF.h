/*
 * EKF.h
 *
 *  Created on: Oct 16, 2013
 *      Author: robo
 */

#ifndef EKF_H_
#define EKF_H_

#include <ros/ros.h>
#include "../Eigen/Dense"

using namespace differential_drive;
using namespace Eigen;


ros::Subscriber enc_sub;
ros::Subscriber sensors_sub;
ros::Publisher EKF_pub;


double x_true,y_true,theta_true;
double x,y,theta,y_wall;
double x_bar,y_bar,theta_bar,y_wall_bar;
Matrix4d sigma,sigma_bar,K;
Matrix4d R,Q;
Matrix4d G,H;


double uniformRandom();

double normalRandom();

void receive_enc(const Encoders::ConstPtr &msg);

void receive_sensors(const AnalogC::ConstPtr &msg);

void rotate(bool right);

double angle(double theta);


#endif /* EKF_H_ */
