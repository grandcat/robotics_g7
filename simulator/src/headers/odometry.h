/*
 * odometry.h
 *
 *  Created on: Sep 22, 2013
 *      Author: robo
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_


#include <ros/ros.h>
#include <differential_drive/Encoders.h>

using namespace differential_drive;


ros::Publisher odom_pub;
ros::Subscriber enc_sub;
ros::Publisher marker_pub;

const double r = 0.1;
const double l = 0.2;
const float ticks_rev = 500;


void receive_encoder(const Encoders::ConstPtr &msg);


#endif /* ODOMETRY_H_ */
