/*
 * wall_followerVW.h
 *
 *  Created on: Oct 16, 2013
 *      Author: robo
 */

#ifndef WALL_FOLLOWERVW_H_
#define WALL_FOLLOWERVW_H_

#include <ros/ros.h>
#include <differential_drive/Encoders.h>
#include "robot/EKF.h"

using namespace differential_drive;
using namespace robot;


ros::Publisher speed_pub;
ros::Publisher rotate_pub;
ros::Subscriber EKF_sub;
ros::Subscriber sensors_sub;


const double rho = 9;
const double alpha = 10;

const double x_cmd_traj = 0.2;
const double y_cmd_traj = 0.20;
const double dist_obstacle = 0.22;
double x;
double x_collision;
double theta_cmd;

const double x_error = 0.01;
const double theta_error = 5;

bool obstacle = false;
bool right;
bool hurt_wall = false;
bool backward = false;


void receive_EKF(const EKF::ConstPtr &msg);

void receive_sensors(const AnalogC::ConstPtr &msg);

double angle(double theta);


#endif /* WALL_FOLLOWERVW_H_ */
