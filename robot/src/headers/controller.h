/*
 * controller.h
 *
 *  Created on: Oct 15, 2013
 *      Author: robo
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <ros/ros.h>
#include <differential_drive/Odometry.h>

using namespace differential_drive;


ros::Publisher speed_pub;
ros::Subscriber odom_sub;


const double error = 0.01;

const double rho = 10;
const double alpha = 30;
const double beta = 10;

double x_cmd = 0;
double y_cmd = 0;
double theta_cmd = 0;

bool flag = false;


void enterCmd();

void receive_odom(const Odometry::ConstPtr &msg);

double angle(double theta);


#endif /* CONTROLLER_H_ */
