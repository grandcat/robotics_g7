/*
 * parameters.h
 *
 *  Created on: Nov 17, 2013
 *      Author: robo
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_


// Robot size
const double r = 0.05;
const double l = 0.249;

// Encoders
const float ticks_rev = 360;

// IR sensors
// Position
const double x_s1 = 0.058, y_s1 = 0.105; // left
const double x_s2 = 0.058, y_s2 = -0.105; // right
// Calibration short range
const double a_short = 17.43215;
const double b_short = -0.9035;
// Calibration long range
const double a_long = 65.48559;
const double b_long = -1.0321;

// Bumper threshold
const int bumper_threshold = 500;


#endif /* PARAMETERS_H_ */
