/*
 * EKF.h
 *
 *  Created on: Nov 17, 2013
 *      Author: robo
 */

#ifndef EKF_H_
#define EKF_H_

#include <ros/ros.h>
#include "../Eigen/Dense"

using namespace differential_drive;
using namespace Eigen;
using namespace explorer;


// Modes
int mode = 0;

enum MODE {
	EXPLORE = 0,
	GOTO_TARGETS,
};


ros::Subscriber enc_sub;
ros::Subscriber sensors_sub;
ros::Subscriber stop_EKF_sub;
ros::Publisher EKF_pub;
ros::Publisher Odometry_pub;


// Absolute values
double x_true,y_true,theta_true;

// Local current values
double x,y,theta,y_wall;
double x_bar,y_bar,theta_bar,y_wall_bar;

Matrix4d sigma,sigma_bar,K;
Matrix4d R;
double Q;
Matrix4d G,H;

// Stop EKF
bool stop = false;
double rotation_angle; // rotation

// Wall following
bool right_sensor; // sensor used to follow a wall
bool wall = false; // wall detected


void receive_enc(const Encoders::ConstPtr &msg);

void receive_sensors(const AnalogC::ConstPtr &msg);

void receive_stop(const Stop_EKF::ConstPtr &msg);

void rotate(double rotation_angle);

double angle(double theta);

void init();


#endif /* EKF_H_ */
