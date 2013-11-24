/*
 * controller.cpp
 *
 *  Created on: Nov 17, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include "explorer/Speed.h"
#include <differential_drive/AnalogC.h>
#include <differential_drive/Odometry.h>
#include "explorer/EKF.h"
#include "explorer/Object.h"
#include "explorer/Stop_EKF.h"
#include "headers/parameters.h"
#include "headers/controller.h"
#include <differential_drive/Servomotors.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace differential_drive;
using namespace cv;
using namespace explorer;


/**
 * Receive the robot and wall position
 * Follow the wall or rotate
 * @param msg
 */
void receive_EKF(const EKF::ConstPtr &msg)
{
	// Temp variables
	double y,y_wall,theta;
	double diff_ang = 0;
	double dist = 0;

	// Speed
	explorer::Speed speed;
	speed.V = 0;
	speed.W = 0;

	// Receive EKF odometry
	x = msg->x;
	y = msg->y;
	theta = msg->theta;
	y_wall = msg->y_wall;


	// Wall follower
	if(actions.empty())
	{
		double x_cmd = x + x_cmd_traj;
		double y_cmd;
		if(msg->right_sensor) {y_cmd = y_wall + y_cmd_traj - y_cmd_change;}
		else {y_cmd = y_wall - y_cmd_traj + y_cmd_change;}
		if(!msg->wall) {y_cmd = y;}

		dist = x_cmd_traj;
		diff_ang = atan((y_cmd-y)/(x_cmd-x))-theta;
		diff_ang = angle(diff_ang);

		speed.V = rho*dist*r;
		speed.W = -2*r/l*alpha*diff_ang;
	}


	// Do a special movement
	else
	{
		if(!busy)
		{
			current_action = actions.front();
			busy = true;

			if(current_action.n != ACTION_GOTO_FORWARD)
			{
				// Stop EKF
				Stop_EKF s;
				s.stop = true;
				stop_EKF_pub.publish(s);
			}
		}


		else
		{
			// Go backward
			if(current_action.n == ACTION_BACKWARD)
			{
				double x_cmd = x - x_cmd_traj;
				double y_cmd = y;

				dist = x_collision-x-x_backward_dist;
				diff_ang = atan((y_cmd-y)/(x_cmd-x))-theta;
				diff_ang = angle(diff_ang);

				speed.V = 5*rho*dist*r;
				speed.W = -2*r/l*alpha*diff_ang;

				// Done
				if(dist*dist < x_error*x_error)
				{
					actions.pop_front();
					busy = false;

					// Relaunch EKF
					Stop_EKF s;
					s.stop = false;
					s.rotation_angle = 0;
					stop_EKF_pub.publish(s);
				}
			}


			// Rotation
			if(current_action.n == ACTION_ROTATION)
			{
				theta_cmd = current_action.parameter1;
				double dtheta = theta - theta_cmd;
				dtheta = angle(dtheta);

				// Rotation saturation
				if(dtheta > M_PI/2) {dtheta = M_PI/2;}
				if(dtheta < -M_PI/2) {dtheta = -M_PI/2;}

				speed.V = 0;
				speed.W = 2*r/l*alpha*dtheta/4*2;

				// Rotation done
				if(dtheta*dtheta < M_PI*M_PI/180/180*theta_error*theta_error)
				{
					create_node(x_true,y_true);

					actions.pop_front();
					busy = false;

					// Relaunch EKF
					Stop_EKF s;
					s.stop = false;
					s.rotation_angle = current_action.parameter1;
					stop_EKF_pub.publish(s);
				}
			}


			// Stop
			if(current_action.n == ACTION_STOP)
			{

			}


			// GOTO with true odometry
			if(current_action.n == ACTION_GOTO)
			{
				double x_cmd = current_action.parameter1;
				double y_cmd = current_action.parameter2;

				dist = sqrt((x_cmd-x_true)*(x_cmd-x_true)+(y_cmd-y_true)*(y_cmd-y_true));
				diff_ang = atan((y_cmd-y_true)/(x_cmd-x_true))-theta_true;
				if((x_cmd-x_true) < 0)
				{
					if((y_cmd-y_true) > 0)
					{
						diff_ang += M_PI;
					}
					else
					{
						diff_ang -= M_PI;
					}
				}
				diff_ang = angle(diff_ang);


				// Rotate first if needed
				if(fabs(diff_ang) > M_PI/180*15)
				{
					// Rotation saturation
					if(diff_ang > M_PI/2) {diff_ang = M_PI/2;}
					if(diff_ang < -M_PI/2) {diff_ang = -M_PI/2;}

					speed.V = 0;
					speed.W = -2*r/l*alpha*diff_ang/4*2;
				}
				else
				{
					// Saturation
					if(dist > x_cmd_traj)
					{
						dist = x_cmd_traj;
					}

					speed.V = rho*dist*r;
					speed.W = -2*r/l*alpha*diff_ang;

					// Done
					if(dist*dist < x_error*x_error)
					{
						actions.pop_front();
						busy = false;

						// Relaunch EKF
						Stop_EKF s;
						s.stop = false;
						s.rotation_angle = 0;
						stop_EKF_pub.publish(s);
					}
				}
			}


			// Rotate without correction
			if(current_action.n == ACTION_GOTO_ROTATION)
			{
				double x_cmd = current_action.parameter1;
				double y_cmd = current_action.parameter2;

				diff_ang = atan((y_cmd-y_true)/(x_cmd-x_true))-theta_true;
				if((x_cmd-x_true) < 0)
				{
					if((y_cmd-y_true) > 0)
					{
						diff_ang += M_PI;
					}
					else
					{
						diff_ang -= M_PI;
					}
				}
				diff_ang = angle(diff_ang);


				// Init
				static double rotation;
				static bool flag;
				if(!flag)
				{
					 rotation = nPi2(diff_ang)*(M_PI/2);
					 flag = true;
				}


				// Rotation saturation
				if(diff_ang > M_PI/2) {diff_ang = M_PI/2;}
				if(diff_ang < -M_PI/2) {diff_ang = -M_PI/2;}

				speed.V = 0;
				speed.W = -2*r/l*alpha*diff_ang/4*2;


				// Done
				if(fabs(diff_ang) > M_PI/180*theta_error)
				{
					actions.pop_front();
					busy = false;

					// Relaunch EKF
					Stop_EKF s;
					s.stop = false;
					s.rotation_angle = rotation;
					stop_EKF_pub.publish(s);

					flag = false;
				}
			}


			// Forward with correction
			if(current_action.n == ACTION_GOTO_FORWARD)
			{
				double x_cmd = current_action.parameter1;
				double y_cmd = current_action.parameter2;

				dist = sqrt((x_cmd-x_true)*(x_cmd-x_true)+(y_cmd-y_true)*(y_cmd-y_true));


				// Saturations
				if(dist > x_cmd_traj)
				{
					dist = x_cmd_traj;
				}
				if(diff_ang > M_PI/2) {diff_ang = M_PI/2;}
				if(diff_ang < -M_PI/2) {diff_ang = -M_PI/2;}


				speed.V = rho*dist*r;
				speed.W = -2*r/l*alpha*diff_ang;


				if(dist < dist_error)
				{
					actions.pop_front();
					busy = false;
				}
			}
		}
	}

	speed_pub.publish(speed);
}


/**
 * Check if there is an obstacle
 * or if the robot hurts a wall
 * @param msg
 */
void receive_sensors(const AnalogC::ConstPtr &msg)
{
	// IR sensor
	s1 = a_short*pow(msg->ch1,b_short); // left
	s2 = a_short*pow(msg->ch2,b_short); // right
	double s3 = a_short*pow(msg->ch3,b_short); // center


	update_map(s1,s2);


	// Bumpers
	bool s6 = (msg->ch6 > bumper_threshold); // right
	bool s7 = (msg->ch7 > bumper_threshold); // center
	bool s8 = (msg->ch8 > bumper_threshold); // left

	s6 = s7 = s8 = false;


	if(actions.empty())
	{
		// Wall in front of the robot
		if(s3 < dist_front_wall)
		{
			if(cmpt >= obstacle)
			{
				cmpt = 0;

				Action action;
				action.n = ACTION_ROTATION;
				if(s1 < s2){action.parameter1 = -M_PI/2;}
				else {action.parameter1 = M_PI/2;}
				actions.push_back(action);

				//printf("Front wall\n");

				return;
			}
			cmpt++;
			return;
		}
		else
		{
			cmpt = 0;
		}


		// Bumpers
		if(s6)
		{
			Action action;
			x_collision = x;

			action.n = ACTION_BACKWARD;
			action.parameter1 = x_backward_dist;
			actions.push_back(action);

			action.n = ACTION_GOTO;
			action.parameter1 = x_true + 0.05*cos(theta_true) + 0.02*sin(theta_true);
			action.parameter2 = y_true + 0.05*sin(theta_true) - 0.02*cos(theta_true);
			actions.push_back(action);

			return;
		}

		if(s8)
		{
			Action action;
			x_collision = x;

			action.n = ACTION_BACKWARD;
			action.parameter1 = x_backward_dist;
			actions.push_back(action);

			action.n = ACTION_GOTO;
			action.parameter1 = x_true + 0.05*cos(theta_true) - 0.02*sin(theta_true);
			action.parameter2 = y_true + 0.05*sin(theta_true) + 0.02*cos(theta_true);
			actions.push_back(action);

			return;
		}

		if(s7)
		{
			Action action;

			action.n = ACTION_BACKWARD;
			action.parameter1 = x_backward_dist;
			actions.push_back(action);

			action.n = ACTION_ROTATION;
			if(s1 < s2){action.parameter1 = -M_PI/2;}
			else {action.parameter1 = M_PI/2;}
			actions.push_back(action);

			return;
		}
	}
}


void update_map(double s1, double s2)
{
	// Robot
	int rx = (x_true-origin_x)/resolution;
	int ry = (y_true-origin_y)/resolution;
	for(int i = -2; i <= 2; i++)
	{
		for(int j = -2; j <= 2; j++)
		{
			if(((ry+j)*width+(rx+i) >= 0) | ((ry+j)*width+(rx+i) < width*height))
			{
				robot_map.at<uchar>(width-(ry+j)-1,rx+i) = 100;
			}
		}
	}

	// Sensors - Wall
	if(s1 < 0.3)
	{
		int wx = ((x_true+x_s1*cos(theta_true)-y_s1*sin(theta_true)-s1*sin(theta_true))-origin_x)/resolution;
		int wy = ((y_true+x_s1*sin(theta_true)+y_s1*cos(theta_true)+s1*cos(theta_true))-origin_y)/resolution;
		if((wy*width+wx >= 0) | (wy*width+wx < width*height))
		{
			wall_map.at<uchar>(width-wy-1,wx) = 255;
		}
	}

	if(s2 < 0.3)
	{
		int wx = ((x_true+x_s2*cos(theta_true)-y_s2*sin(theta_true)+s2*sin(theta_true))-origin_x)/resolution;
		int wy = ((y_true+x_s2*sin(theta_true)+y_s2*cos(theta_true)-s2*cos(theta_true))-origin_y)/resolution;
		if((wy*width+wx >= 0) | (wy*width+wx < width*height))
		{
			wall_map.at<uchar>(width-wy-1,wx) = 255;
		}
	}


	map = Mat::zeros(height,width,CV_8UC1);


	// Wall processing
	Hough();


	// Robot path processing
	merge_areas();


	// Explore
	interesting_nodes();


	proc_map = map.clone();


	// Check visited area
	visited_flag = visited_area();
	if(visited_flag & actions.empty())
	{
		printf("Already visited !\n");
		path_finding(find_closest_node(toDiscover));
	}


	namedWindow("Map",CV_WINDOW_NORMAL);
	imshow("Map",proc_map);

	cvWaitKey(10);
}


void Hough()
{
	vector<Vec2f> lines;
	HoughLines(wall_map, lines, 1, 90*CV_PI/180, 10, 0, 0 );

	for( size_t i = 0; i < lines.size(); i++ )
	{
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 500*(-b));
		pt1.y = cvRound(y0 + 500*(a));
		pt2.x = cvRound(x0 - 500*(-b));
		pt2.y = cvRound(y0 - 500*(a));
		line(map, pt1, pt2, Scalar(255), 1);
	}

	Mat temp_map = wall_map.clone();
	GaussianBlur(temp_map,temp_map, Size(3,3), 0, 0 );
	for(int i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++)
		{
			if(temp_map.at<uchar>(i,j) == 0)
			{
				map.at<uchar>(i,j) = 0;
			}
		}
	}
}


void merge_areas()
{
	for(int i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++)
		{
			if((robot_map.at<uchar>(i,j) == 100) & (map.at<uchar>(i,j) == 0))
			{
				map.at<uchar>(i,j) = 100;
			}
		}
	}


	int sz = 5;
	for(int i = 0; i < (height-sz); i++)
	{
		for(int j = 0; j < (width-sz); j++)
		{
			if((map.at<uchar>(i,j) == 100) &
					(map.at<uchar>(i+sz-1,j) == 100) &
					(map.at<uchar>(i,j+sz-1) == 100) &
					(map.at<uchar>(i+sz-1,j+sz-1) == 100))
			{
				bool flag = false;
				for(int n = 0; n < sz; n++)
				{
					for(int m = 0; m < sz; m++)
					{
						if(map.at<uchar>(i+n,j+m) == 255)
						{
							flag = true;
						}
					}
				}
				if(!flag)
				{
					for(int n = 0; n < sz; n++)
					{
						for(int m = 0; m < sz; m++)
						{
							map.at<uchar>(i+n,j+m) = 100;
						}
					}
				}
			}
		}
	}
}


void interesting_nodes()
{
	toDiscover.clear();


	// Check left
	for(int i = 0; i < height-sz2; i++)
	{
		for(int j = 0; j < width-sz1; j++)
		{
			bool flag = false;
			for(int n = 0; n < sz2; n++)
			{
				for(int m = 0; m < sz1-1; m++)
				{
					if(map.at<uchar>(i+n,j+m) != 0)
					{
						flag = true;
					}
				}
				if(map.at<uchar>(i+n,j+sz1-1) != 100)
				{
					flag = true;
				}
			}
			if(!flag)
			{
				map.at<uchar>(i+2,j+sz1-2) = 200;
				create_interesting_node(i+2,j+sz1-2);
			}
		}
	}


	// Check right
	for(int i = 0; i < height-sz2; i++)
	{
		for(int j = 0; j < width-sz1; j++)
		{
			bool flag = false;
			for(int n = 0; n < sz2; n++)
			{
				for(int m = 1; m < sz1; m++)
				{
					if(map.at<uchar>(i+n,j+m) != 0)
					{
						flag = true;
					}
				}
				if(map.at<uchar>(i+n,j) != 100)
				{
					flag = true;
				}
			}
			if(!flag)
			{
				map.at<uchar>(i+2,j+1) = 200;
				create_interesting_node(i+2,j+1);
			}
		}
	}


	// Check bottom
	for(int i = 0; i < height-sz2; i++)
	{
		for(int j = 0; j < width-sz1; j++)
		{
			bool flag = false;
			for(int n = 0; n < sz2; n++)
			{
				for(int m = 1; m < sz1; m++)
				{
					if(map.at<uchar>(j+m,i+n) != 0)
					{
						flag = true;
					}
				}
				if(map.at<uchar>(j,i+n) != 100)
				{
					flag = true;
				}
			}
			if(!flag)
			{
				map.at<uchar>(j+1,i+2) = 200;
				create_interesting_node(j+1,i+2);
			}
		}
	}


	// Check top
	for(int i = 0; i < height-sz2; i++)
	{
		for(int j = 0; j < width-sz1; j++)
		{
			bool flag = false;
			for(int n = 0; n < sz2; n++)
			{
				for(int m = 0; m < sz1-1; m++)
				{
					if(map.at<uchar>(j+m,i+n) != 0)
					{
						flag = true;
					}
				}
				if(map.at<uchar>(j+sz1-1,i+n) != 100)
				{
					flag = true;
				}
			}
			if(!flag)
			{
				map.at<uchar>(j+sz1-2,i+2) = 200;
				create_interesting_node(j+sz1-2,i+2);
			}
		}
	}
}


bool visited_area()
{
	int rx = (x_true-origin_x+0.2*cos(theta_true))/resolution;
	int ry = (y_true-origin_y+0.2*sin(theta_true))/resolution;

	if(proc_map.at<uchar>(height-ry-1,rx) == 100)
	{
		return true;
	}

	return false;
}


void path_finding(Node n)
{
	Node n_true;
	n_true.x = x_true;
	n_true.y = y_true;

	Pixel p_true = nodeToPixel(n_true);
	Pixel p = nodeToPixel(n);


	// Try 2 ways

	// Up Down
	bool flag1 = false;
	for(int i = 1; i < abs(p.i-p_true.i); i++)
	{
		if(p.i <= p_true.i)
		{
			if(map.at<uchar>(p.i+i,p.j) != 100)
			{
				flag1 = true;
			}
		}
		else
		{
			if(map.at<uchar>(p.i-i,p.j) != 100)
			{
				flag1 = true;
			}
		}

	}
	for(int j = 1; j < abs(p.j-p_true.j); j++)
	{
		if(p.j <= p_true.j)
		{
			if(map.at<uchar>(p_true.i,p.j+j) != 100)
			{
				flag1 = true;
			}
		}
		else
		{
			if(map.at<uchar>(p_true.i,p.j-j) != 100)
			{
				flag1 = true;
			}
		}
	}
	if(!flag1)
	{
		Node interm;
		interm.x = n.x;
		interm.y = y_true;

		goto_node(interm);

		goto_node(n);

		return;
	}


	// Left Right
	bool flag2 = false;
	for(int i = 1; i < abs(p.i-p_true.i); i++)
	{
		if(p.i <= p_true.i)
		{
			if(map.at<uchar>(p.i+i,p_true.j) != 100)
			{
				flag2 = true;
			}
		}
		else
		{
			if(map.at<uchar>(p.i-i,p_true.j) != 100)
			{
				flag2 = true;
			}
		}

	}
	for(int j = 1; j < abs(p.j-p_true.j); j++)
	{
		if(p.j <= p_true.j)
		{
			if(map.at<uchar>(p.i,p.j+j) != 100)
			{
				flag2 = true;
			}
		}
		else
		{
			if(map.at<uchar>(p.i,p.j-j) != 100)
			{
				flag2 = true;
			}
		}
	}
	if(!flag2)
	{
		Node interm;
		interm.x = x_true;
		interm.y = n.y;

		goto_node(interm);

		goto_node(n);

		return;
	}


	/*
	// No path
	if(flag1 & flag2)
	{
		Action action;
		action.n = ACTION_STOP;
		actions.push_back(action);
	}
	 */
}


std::list<Action> path(Node n1, Node n2)
		{
	std::list<Action> list;
	return list;
		}


Pixel nodeToPixel(Node node)
{
	int px = (node.x-origin_x)/resolution;
	int py = (node.y-origin_y)/resolution;

	Pixel pixel;
	pixel.i = height-py-1;
	pixel.j = px;

	return pixel;
}


void goto_node(Node node)
{
	Action action;

	action.n = ACTION_GOTO_ROTATION;
	action.parameter1 = node.x;
	action.parameter2 = node.y;
	actions.push_back(action);

	action.n = ACTION_GOTO_FORWARD;
	action.parameter1 = node.x;
	action.parameter2 = node.y;
	actions.push_back(action);

	printf("Goto node: x = %f, y = %f\n",node.x,node.y);
}


void receive_odometry(const Odometry::ConstPtr &msg)
{
	x_true = msg->x;
	y_true = msg->y;
	theta_true = msg->theta;
}


void create_node(double x, double y)
{
	Node n;

	// Position
	n.x = x;
	n.y = y;

	discrete_map.push_back(n);

	// Debug
	printf("New node:  x = %f, y = %f\n",x,y);
}


void create_interesting_node(int i,int j)
{
	Node node;

	node.x = j*resolution+origin_x;
	node.y = (height-i-1)*resolution+origin_y;
	toDiscover.push_back(node);

	//Pixel pixel = nodeToPixel(node);
	//printf("pi = %d, pj = %d, i = %d, j = %d\n",pixel.i,pixel.j,i,j);

	// Debug
	//printf("New interesting node:  x = %f, y = %f\n",node.x,node.y);
}


Node find_closest_node(std::list<Node> list)
{
	Node node;
	node.x = 0;
	node.y = 0;

	double dist = INFINITY;

	if(!list.empty())
	{
		for(int i = 0; i < list.size(); i++)
		{
			Node n = list.back();
			list.pop_back();
			double d = sqrt((n.x-x_true)*(n.x-x_true)+(n.y-y_true)*(n.y-y_true));
			if(d < dist)
			{
				node.x = n.x;
				node.y = n.y;
				dist = d;
			}
		}
	}

	//printf("Node found: x = %f, y = %f\n",node.x,node.y);

	return node;
}


void receive_object(const Object::ConstPtr &msg)
{
	double x_object = x_true + cos(theta_true);
	double y_object = y_true + sin(theta_true);
	printf("x_object = %f, y_object = %f\n",x_object,y_object);
}


/**
 * Angle between ]-pi,pi]
 * @param th
 * @return
 */
double angle(double theta)
{
	while((theta > M_PI) | (theta <= -M_PI))
	{
		if(theta > 0)
		{
			theta -= 2*M_PI;
		}
		else
		{
			theta += 2*M_PI;
		}
	}

	return theta;
}


int nPi2(double theta)
{
	int res = 0;
	double th = theta;

	if(theta > 0)
	{
		while(th > 0)
		{
			th = th - M_PI/4;
			res++;
		}
	}

	if(theta < 0)
	{
		while(th < 0)
		{
			th = th + M_PI/4;
			res--;
		}
	}

	return res/2;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	speed_pub = nh.advertise<explorer::Speed>("/motion/Speed",100);
	stop_EKF_pub =nh.advertise<Stop_EKF>("/motion/Stop_EKF",100);
	EKF_sub = nh.subscribe("/motion/EKF",1000,receive_EKF);
	sensors_sub = nh.subscribe("/sensors/ADC",1000,receive_sensors);
	servo_pub = nh.advertise<Servomotors>("/actuator/Servo",100);
	odometry_sub = nh.subscribe("/motion/Odometry",1000,receive_odometry);
	object_sub = nh.subscribe("/motion/Object",1000,receive_object);


	proc_map = Mat::zeros(height,width,CV_8UC1);
	robot_map = Mat::zeros(height,width,CV_8UC1);
	wall_map = Mat::zeros(height,width,CV_8UC1);


	create_node(0,0);


	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
