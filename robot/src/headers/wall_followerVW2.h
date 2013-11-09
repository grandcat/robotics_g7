/*
 * wall_followerVW2.h
 *
 *  Created on: Oct 16, 2013
 *      Author: robo
 */

#ifndef WALL_FOLLOWERVW2_H_
#define WALL_FOLLOWERVW2_H_

#include <ros/ros.h>
#include <differential_drive/Encoders.h>
#include "robot/EKF.h"

using namespace differential_drive;
using namespace robot;


struct Action
{
	int n;
	double parameter;
};

ros::Publisher speed_pub;
ros::Publisher stop_EKF_pub;
ros::Subscriber EKF_sub;
ros::Subscriber sensors_sub;


// Control filter parameters
const double rho = 9;
const double alpha = 10;

// Distances
const double x_cmd_traj = 0.2;
const double y_cmd_traj = 0.20;
double y_cmd_change = 0.0;
const double x_backward_dist = 0.05;
const double dist_front_wall = 0.22;

// Temporary variable
double x;
double x_collision;
double theta_cmd;

// Errors
const double x_error = 0.01;
const double theta_error = 5;

// Actions sequence
bool busy = false;
std::list<Action> actions;
Action current_action;

// Actions
const int forward = 0;
const int backward = 1;
const int rotation = 2;
const int change_y_cmd_traj = 4;


void receive_EKF(const EKF::ConstPtr &msg);

void receive_sensors(const AnalogC::ConstPtr &msg);

double angle(double theta);


#endif /* WALL_FOLLOWERVW2_H_ */
