/*
 * odometry.h
 *
 *  Created on: Oct 15, 2013
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


void receive_encoder(const Encoders::ConstPtr &msg);


#endif /* ODOMETRY_H_ */
