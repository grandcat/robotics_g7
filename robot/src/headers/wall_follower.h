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



double x_cmd = 0.2;


void receive_odom(const Odometry::ConstPtr &msg);

double angle(double theta);


#endif /* WALL_FOLLOWER_H_ */
