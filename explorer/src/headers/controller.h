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

#include <boost/unordered_map.hpp>

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
	ACTION_NO,
};

enum BUSY {
	NOT_BUSY = 1,
	BUSY_PRIORITY,
	BUSY_ACTIONS,
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

	bool operator==(const Node& n) const
	{
	    return ((n.x == x) & (n.y == y));
	}
};

struct Pixel
{
	int i,j;
};


// Modes
int mode = 0;

enum MODE {
	EXPLORE = 0,
	GOTO_TARGETS,
	TEST,
};


// ros topics
ros::Publisher speed_pub;
ros::Publisher stop_EKF_pub;
ros::Publisher servo_pub;
ros::Publisher chatter_pub;
ros::Subscriber EKF_sub;
ros::Subscriber sensors_sub;
ros::Subscriber odometry_sub;
ros::Subscriber object_sub;


// Control filter parameters
const double rho = 13; // 9
const double alpha = 5; // 10


// Distances
const double x_cmd_traj = 0.2;
const double y_cmd_traj = 0.20; // 0.18
double y_cmd_change = 0.0;
const double x_backward_dist = 0.12; // 0.08
const double x_forward_dist = 0.18;
const double dist_front_wall = 0.20; // 0.18
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
const double theta_error = 3;
const double dist_error = 0.05;


// Actions sequence
enum BUSY busy = NOT_BUSY;
std::list<Action> actions;
std::list<Action> priority;
Action current_action;


// Go somewhere
bool goto_target = false;
Node target;

Node current_node;


// IR sensor mean
const int obstacle = 2; // 2
int cmpt;


// IR sensor value
double s1,s2;


// Map
std::vector<Node> discrete_map;
std::vector<Node> objects;
std::vector<Node> near_objects;

typedef std::pair<Node,Node> Nodes;
std::vector<Nodes> important_nodes;
std::vector<Nodes> important_nodes_targets;

std::vector<Node> toDiscover;

Mat proc_map, robot_map, wall_map, map;
const int origin_x = -6;
const int origin_y = -6;
const int height = 600;
const int width = 600;
const double resolution = 0.02;

// Map constants
const int sz1 = 11;
const int sz2 = 11;

bool visited_flag = false;


// Path
/*
int hash_value(Node const &n) {
    boost::hash<int> hasher;
    return hasher(n.x) + hasher(n.y);
}
typedef std::vector<Node> Path;
typedef boost::unordered_map<Node,Path> Hash;
typedef std::pair<Node,Path> Pair;
*/


// Phase 2
int const avoid_repetition = 10;


// Receive functions
void receive_EKF(const EKF::ConstPtr &msg);

void receive_sensors(const AnalogC::ConstPtr &msg);

void receive_odometry(const Odometry::ConstPtr &msg);

void receive_object(const Object::ConstPtr &msg);

void merge_objects();


// Map explorer
void update_map(double s1, double s2);

void Hough();

void merge_areas();

void interesting_nodes();

void create_node(double x, double y);

void create_interesting_node(int i,int j);

void interesting_node();

Node find_closest_node(std::vector<Node> vector);

void path_finding(Node node);

//Path path(Node n1, Node n2);

bool visited_area();

bool isPath(Node n1, Node n2);

Pixel nodeToPixel(Node node);

Node pixelToNode(Pixel pixel);

void goto_node(Node node);

//void pathToActions(Path path);

void create_important_node(double x1, double y1, double x2, double y2);

void create_important_node_targets(double x1, double y1, double x2, double y2);


// Angle between ]-pi,pi]
double angle(double theta);

int nPi2(double theta);


#endif /* CONTROLLER_H_ */
