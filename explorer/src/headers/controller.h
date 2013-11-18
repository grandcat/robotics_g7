/*
 * controller.h
 *
 *  Created on: Nov 17, 2013
 *      Author: robo
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <ros/ros.h>
#include <differential_drive/Encoders.h>
#include "explorer/EKF.h"

using namespace differential_drive;
using namespace explorer;

// Actions
enum EACTIONS {
	ACTION_BACKWARD = 1,
	ACTION_FORWARD,
	ACTION_ROTATION,
	ACTION_CHANGE_Y_CMD_TRAJ,
	ACTION_STOP,
	ACTION_GOTO,
};

struct Action
{
	enum EACTIONS n;
	double parameter;
};


// Map points
struct Node
{
	double x,y;
	double rotation;
};


// ros topics
ros::Publisher speed_pub;
ros::Publisher stop_EKF_pub;
ros::Subscriber EKF_sub;
ros::Subscriber sensors_sub;
ros::Subscriber odometry_sub;
ros::Publisher servo_pub;


// Control filter parameters
const double rho = 13; // 9
const double alpha = 10;

// Distances
const double x_cmd_traj = 0.2;
const double y_cmd_traj = 0.20;
double y_cmd_change = 0.0;
const double x_backward_dist = 0.05;
const double x_forward_dist = 0.2;
const double dist_front_wall = 0.22;
const double x_catch_wall = 0.15;

// Temporary variable
double x;
double x_pb;
double theta_cmd;

// Odometry
double x_true,y_true;

// Errors
const double x_error = 0.01;
const double theta_error = 5;

// Actions sequence
bool busy = false;
std::list<Action> actions;
Action current_action;

// IR sensor mean
const int obstacle = 5;
int cmpt;

// IR sensor value
double s1;


// MAP
std::list<Node> discrete_map;
bool back = false;


void receive_EKF(const EKF::ConstPtr &msg);

void receive_sensors(const AnalogC::ConstPtr &msg);

void receive_odometry(const Odometry::ConstPtr &msg);

void create_node(double rotation);

double angle(double theta);


#endif /* CONTROLLER_H_ */
