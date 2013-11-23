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

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace differential_drive;
using namespace cv;
using namespace explorer;

// Actions
enum EACTIONS {
	ACTION_BACKWARD = 1,
	ACTION_ROTATION,
	ACTION_STOP,
	ACTION_GOTO,
	ACTION_GOTO_FORWARD,
	ACTION_GOTO_ROTATION,
};

struct Action
{
	enum EACTIONS n;
	double parameter1;
	double parameter2;
};


// Map points
struct Node
{
	double x,y;
};

struct Pixel
{
	int i,j;
};


// ros topics
ros::Publisher speed_pub;
ros::Publisher stop_EKF_pub;
ros::Publisher servo_pub;
ros::Subscriber EKF_sub;
ros::Subscriber sensors_sub;
ros::Subscriber odometry_sub;
ros::Subscriber object_sub;



// Control filter parameters
const double rho = 13; // 9
const double alpha = 5; // 10

// Distances
const double x_cmd_traj = 0.2;
const double y_cmd_traj = 0.20;
double y_cmd_change = 0.0;
const double x_backward_dist = 0.05;
const double x_forward_dist = 0.18;
const double dist_front_wall = 0.24;
const double x_catch_wall = 0.17;

// Temporary variable
double x;
double x_pb;
double theta_cmd;
double x_collision;

// Odometry
double x_true,y_true,theta_true;

// Errors
const double x_error = 0.01;
const double theta_error = 2;
const double dist_error = 0.03;

// Actions sequence
bool busy = false;
std::list<Action> actions;
Action current_action;

// IR sensor mean
const int obstacle = 3;
int cmpt;

// IR sensor value
double s1,s2;

// Map
std::list<Node> discrete_map;
std::list<Node> toDiscover;

Mat proc_map, robot_map, map;
const int origin_x = -4;
const int origin_y = -4;
const int height = 200;
const int width = 200;
const double resolution = 0.04;

const int sz1 = 5;
const int sz2 = 3;

bool visited_flag = false;


// Receive functions
void receive_EKF(const EKF::ConstPtr &msg);

void receive_sensors(const AnalogC::ConstPtr &msg);

void receive_odometry(const Odometry::ConstPtr &msg);

void receive_object(const Object::ConstPtr &msg);


// Map explorer
void update_map(double s1, double s2);

void Hough();

void merge_areas();

void interesting_nodes();

void create_node(double x, double y);

void create_interesting_node(int i,int j);

void interesting_node();

Node find_closest_node(std::list<Node> list);

void path_finding(Node node);

bool visited_area();

Pixel nodeToPixel(Node node);

void goto_node(Node node);


// Angle between ]-pi,pi]
double angle(double theta);


#endif /* CONTROLLER_H_ */
