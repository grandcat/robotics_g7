/*
 * wall_follower.h
 *
 *  Created on: Oct 16, 2013
 *      Author: robo
 */

#ifndef WALL_FOLLOWER_H_
#define WALL_FOLLOWER_H_

#include <ros/ros.h>
#include <differential_drive/Encoders.h>
#include "robot/EKF.h"

using namespace differential_drive;
using namespace robot;


ros::Publisher speed_pub;
ros::Publisher rotate_pub;
ros::Subscriber EKF_sub;
ros::Subscriber sensors_sub;


const double rho = 12;
const double alpha = 20;

const double x_cmd_traj = 0.16;
const double y_cmd_traj = 0.2;

bool obstacle = false;
bool right;


void receive_EKF(const EKF::ConstPtr &msg);

void receive_sensors(const AnalogC::ConstPtr &msg);

double angle(double theta);


#endif /* WALL_FOLLOWER_H_ */
