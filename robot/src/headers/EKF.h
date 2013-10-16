/*
 * EKF.h
 *
 *  Created on: Oct 16, 2013
 *      Author: robo
 */

#ifndef EKF_H_
#define EKF_H_

#include <ros/ros.h>

using namespace differential_drive;


ros::Subscriber enc_sub;
ros::Subscriber sensors_sub;


void receive_enc(const Encoders::ConstPtr &msg);

void receive_sensors(const AnalogC::ConstPtr &msg);


#endif /* EKF_H_ */
