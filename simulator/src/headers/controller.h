/*
 * controller.h
 *
 *  Created on: Sep 19, 2013
 *      Author: robo
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <ros/ros.h>
#include <differential_drive/PWM.h>

using namespace differential_drive;


ros::Publisher cmd_pub;
ros::Subscriber odom_sub;

const double r = 0.1;
const double l = 0.2;
const float ticks_rev = 500;

const double error = 0.07;

const double rho = 60;
const double alpha = 60;
const double beta = 30;

double x_cmd = 0;
double y_cmd = 0;
double theta_cmd = 0;

bool flag = false;


void enterCmd();

void receive_odom(const Odometry::ConstPtr &msg);

double angle(double theta);


#endif /* CONTROLLER_H_ */
