/*
 * wall_follower.h
 *
 *  Created on: Oct 16, 2013
 *      Author: robo
 */

#ifndef WALL_FOLLOWER_H_
#define WALL_FOLLOWER_H_

#include <ros/ros.h>
#include <differential_drive/Odometry.h>

using namespace differential_drive;


ros::Publisher speed_pub;
ros::Subscriber odom_sub;
ros::Subscriber sensors_sub;


const double error = 0.005;

const double rho = 60;
const double alpha = 60;
const double beta = 30;

double x_cmd = 0;
double y_cmd = 0;
double theta_cmd = 0;


void receive_odom(const Odometry::ConstPtr &msg);

double angle(double theta);


#endif /* WALL_FOLLOWER_H_ */
