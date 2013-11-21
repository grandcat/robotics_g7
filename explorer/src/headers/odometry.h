/*
 * odometry.h
 *
 *  Created on: Nov 17, 2013
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

// Map points
struct Node
{
	double x,y;
};

std::list<Node> discrete_map;


void receive_odometry(const Odometry::ConstPtr &msg);

void receive_sensors(const AnalogC::ConstPtr &msg);

void Hough(nav_msgs::OccupancyGrid map);

void create_node(double x, double y);


#endif /* ODOMETRY_H_ */
