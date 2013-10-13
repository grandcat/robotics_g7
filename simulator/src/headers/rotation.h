/*
 * rotation.h
 *
 *  Created on: Oct 13, 2013
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

const double error = 0.007;

const double rho = 50;
const double alpha = 80;

double L = 0.1;

double a11 = 0.95;
double a21 = -0.6;
double a02 = 2;
double a12 = -1;
double a22 = 1;

double b11 = 1.2;
double b21 = -0.5;
double b02 = 2;
double b12 = -1;
double b22 = 1;

double theta_cmd = 0;

bool flag = false;


void enterCmd();

void receive_odom(const Odometry::ConstPtr &msg);

double angle(double theta);


#endif /* ROTATION_H_ */
