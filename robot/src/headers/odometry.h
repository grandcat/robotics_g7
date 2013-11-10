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
#include <differential_drive/Odometry.h>
#include <differential_drive/AnalogC.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace differential_drive;


ros::Subscriber odom_sub;
ros::Publisher marker_pub;
ros::Subscriber sensors_sub;
ros::Publisher map_pub;


double x,y,theta;

// Map parameters
nav_msgs::OccupancyGrid map;
const double resolution = 0.04;
const double position_x = -4;
const double position_y = -4;
const double height = 200;
const double width = 200;


//void receive_encoder(const Encoders::ConstPtr &msg);

void receive_odometry(const Odometry::ConstPtr &msg);

void receive_sensors(const AnalogC::ConstPtr &msg);


#endif /* ODOMETRY_H_ */
