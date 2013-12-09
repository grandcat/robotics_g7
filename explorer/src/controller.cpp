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

#include <boost/unordered_map.hpp>

#include "std_msgs/String.h"
#include <sstream>

#include <iostream>
#include <fstream>
#include <algorithm>

#include "../../object_recognition/src/recognition_constants.hpp"


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


	//printf("x = %f, y = %f\n",x_true,y_true);


	// Wall follower
	if(actions.empty() & priority.empty())
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
		static bool flag;


		if((busy == BUSY_ACTIONS) & !priority.empty())
		{
			busy = NOT_BUSY;
			current_action.n = ACTION_NO;

			// Relaunch EKF
			Stop_EKF s;
			s.stop = false;
			s.rotation_angle = 0;
			stop_EKF_pub.publish(s);
		}


		if(busy == NOT_BUSY)
		{
			if(!priority.empty())
			{
				current_action = priority.front();
				busy = BUSY_PRIORITY;
			}
			else
			{
				current_action = actions.front();
				busy = BUSY_ACTIONS;
			}


			if(current_action.n != ACTION_GOTO_FORWARD)
			{
				// Stop EKF
				Stop_EKF s;
				s.stop = true;
				s.rotation_angle = current_action.parameter1;
				stop_EKF_pub.publish(s);
			}


			flag = false;
		}


		else
		{
			// Go backward
			if(current_action.n == ACTION_BACKWARD)
			{
				double x_cmd = x - x_cmd_traj;

				static double y_cmd;

				if(!flag)
				{
					y_cmd = y;
					x_collision = x;
					flag = true;
				}

				dist = x_collision-x-x_backward_dist;
				diff_ang = atan((y_cmd-y)/(x_cmd-x))-theta;
				diff_ang = angle(diff_ang);

				speed.V = rho*dist*r;
				speed.W = -2*r/l*alpha*diff_ang;

				//printf("Backward\n");

				// Done
				if(dist*dist < x_error*x_error)
				{
					if(busy == BUSY_ACTIONS) {actions.pop_front();}
					if(busy == BUSY_PRIORITY) {priority.pop_front(); actions.clear();}

					busy = NOT_BUSY;
					current_action.n = ACTION_NO;

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
					if(!goto_target) {create_node(x_true,y_true);}

					if(busy == BUSY_ACTIONS) {actions.pop_front();}
					if(busy == BUSY_PRIORITY) {priority.pop_front(); actions.clear();}

					busy = NOT_BUSY;
					current_action.n = ACTION_NO;

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
				if(fabs(diff_ang) > M_PI/180*10)
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
						if(busy == BUSY_ACTIONS) {actions.pop_front();}
						if(busy == BUSY_PRIORITY) {priority.pop_front(); actions.clear();}

						busy = NOT_BUSY;
						current_action.n = ACTION_NO;

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

				// Init
				static double rotation;

				if(!flag)
				{
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

					rotation = nPi2(diff_ang)*(M_PI/2);
					flag = true;

					printf("diff_ang = %f ,rotation = %f\n",diff_ang,rotation);
				}


				diff_ang = rotation-theta;
				diff_ang = angle(diff_ang);


				// Rotation saturation
				if(diff_ang > M_PI/2) {diff_ang = M_PI/2;}
				if(diff_ang < -M_PI/2) {diff_ang = -M_PI/2;}

				speed.V = 0;
				speed.W = -2*r/l*alpha*diff_ang/4*2;


				// Done
				if(fabs(diff_ang) < M_PI/180*theta_error)
				{
					if(!goto_target) {create_node(x_true,y_true);}


					if(busy == BUSY_ACTIONS) {actions.pop_front();}
					if(busy == BUSY_PRIORITY) {priority.pop_front(); actions.clear();}

					busy = NOT_BUSY;
					current_action.n = ACTION_NO;

					// Relaunch EKF
					Stop_EKF s;
					s.stop = false;
					s.rotation_angle = rotation;
					stop_EKF_pub.publish(s);

					//printf("rotation = %f\n",rotation);
				}
			}


			// Forward with correction

			if(current_action.n == ACTION_GOTO_FORWARD)
			{
				static double x_cmd;
				static double y_cmd;


				// Init
				if(!flag)
				{
					x_cmd = current_action.parameter1;
					y_cmd = current_action.parameter2;

					diff_ang = atan((y_cmd-y_true)/(x_cmd-x_true));
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
					double rotation = nPi2(diff_ang)*(M_PI/2);

					double distance = sqrt((x_cmd-x_true)*(x_cmd-x_true)+(y_cmd-y_true)*(y_cmd-y_true));
					y_cmd = distance*sin(diff_ang-rotation);
					x_cmd = distance*cos(diff_ang-rotation);

					flag = true;

					printf("Goto forward: x_cmd = %f, y_cmd = %f\n",x_cmd,y_cmd);
				}


				diff_ang = atan((y_cmd-y)/(x_cmd-x))-theta;
				diff_ang = angle(diff_ang);

				dist = x_cmd-x;

				// Saturations
				if(dist > x_cmd_traj)
				{
					dist = x_cmd_traj;
				}

				if(diff_ang > M_PI/2) {diff_ang = M_PI/2;}
				if(diff_ang < -M_PI/2) {diff_ang = -M_PI/2;}


				speed.V = rho*dist*r;
				speed.W = -2*r/l*alpha*diff_ang;


				if(dist < 0.05) // dist_error // 0.08
				{
					if(busy == BUSY_ACTIONS) {actions.pop_front();}
					if(busy == BUSY_PRIORITY) {priority.pop_front(); actions.clear();}

					busy = NOT_BUSY;
					current_action.n = ACTION_NO;

					printf("Done !\n");

					Node n;
					n.x = current_action.parameter1;
					n.y = current_action.parameter2;
					current_node = n;


					if(mode == EXPLORE)
					{
						for(int i = 0; i < important_nodes.size(); i++)
						{
							if(important_nodes.at(i).first == n)
							{
								important_nodes.resize(i);
								//important_nodes.erase(important_nodes.begin()+i);
							}
						}
					}


					if(mode == GOTO_TARGETS)
					{
						for(int i = 0; i < important_nodes.size(); i++)
						{
							if(important_nodes.at(i).second == n)
							{
								for(int j = 0; j <= i; j++)
								{
									important_nodes_targets.erase(important_nodes_targets.begin()+j);
								}
							}
						}
					}


					/*
					// Relaunch EKF
					Stop_EKF s;
					s.stop = false;
					s.rotation_angle = 0;
					stop_EKF_pub.publish(s);
					 */
				}
			}

			/*
			if(current_action.n == ACTION_GOTO_FORWARD)
			{
				static double x_cmd;

				// Init
				if(!flag)
				{
					x_cmd = x + current_action.parameter1;
					flag = true;
				}


				diff_ang = atan((y_cmd-y)/(x_cmd-x))-theta;
				diff_ang = angle(diff_ang);

				dist = x_cmd-x;

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
					if(busy == BUSY_ACTIONS) {actions.pop_front();}
					if(busy == BUSY_PRIORITY) {priority.pop_front(); actions.clear();}

					busy = NOT_BUSY;
					current_action.n = ACTION_NO;

					printf("Done !\n");
				}
			}
			 */
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

	//s6 = s7 = s8 = false;
	s7 = false;


	/*
	if(actions.empty() & priority.empty()) // PUT IN THE OTHER LOOP
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
				priority.push_back(action);

				printf("Front wall\n");

				if((s1 > 0.15) & (s2 > 0.15) & !goto_target)
				{
					create_important_node(discrete_map.back().x,discrete_map.back().y,x_true,y_true);
				}

				return;
			}
			cmpt++;
			return;
		}
		else
		{
			cmpt = 0;
		}
	}
	 */


	if(priority.empty() & (current_action.n != ACTION_GOTO_ROTATION) & (current_action.n != ACTION_ROTATION))
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
				priority.push_back(action);

				printf("Front wall\n");


				if((s1 > 0.15) & (s2 > 0.15) & !goto_target)
				{
					create_important_node(discrete_map.back().x,discrete_map.back().y,x_true,y_true);

					double theta = nPi2(theta_true)*M_PI/2;
					double x2,y2;
					if(s1 < s2)
					{
						x2 = x_true + 0.2*sin(theta);
						y2 = y_true - 0.2*cos(theta);
					}
					else
					{
						x2 = x_true - 0.2*sin(theta);
						y2 = y_true + 0.2*cos(theta);
					}
					create_important_node_targets(x_true,y_true,x2,y2);
				}


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
		if(s8)
		{
			Action action;

			action.n = ACTION_BACKWARD;
			action.parameter1 = x_backward_dist;
			priority.push_back(action);

			action.n = ACTION_GOTO_FORWARD;
			action.parameter1 = x_true + 0.01*cos(theta_true) + 0.03*sin(theta_true);
			action.parameter2 = y_true + 0.01*sin(theta_true) - 0.03*cos(theta_true);
			priority.push_back(action);

			printf("Hurt wall\n");

			return;
		}

		if(s6)
		{
			Action action;

			action.n = ACTION_BACKWARD;
			action.parameter1 = x_backward_dist;
			priority.push_back(action);

			action.n = ACTION_GOTO_FORWARD;
			action.parameter1 = x_true + 0.01*cos(theta_true) - 0.03*sin(theta_true);
			action.parameter2 = y_true + 0.01*sin(theta_true) + 0.03*cos(theta_true);
			priority.push_back(action);

			printf("Hurt wall\n");

			return;
		}

		/*
		if(s7)
		{
			Action action;

			action.n = ACTION_BACKWARD;
			action.parameter1 = x_backward_dist;
			priority.push_back(action);

			action.n = ACTION_ROTATION;
			if(s1 < s2){action.parameter1 = -M_PI/2;}
			else {action.parameter1 = M_PI/2;}
			priority.push_back(action);

			return;
		}
		 */
	}
}


/**
 * Update the map
 * Logic to find paths
 */
void update_map(double s1, double s2)
{
	// Robot
	int rx = (x_true-origin_x)/resolution;
	int ry = (y_true-origin_y)/resolution;
	for(int i = -5; i <= 5; i++)
	{
		for(int j = -5; j <= 5; j++)
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


	// Objects
	for(int i = 0; i < objects.size(); i++)
	{
		Pixel object = nodeToPixel(objects.at(i));

		if((object.i < height-2) & (object.j < width-2) & (object.i > 1) & (object.j > 1))
		{
			map.at<uchar>(object.i,object.j) = 250;

			map.at<uchar>(object.i+1,object.j) = 250;
			map.at<uchar>(object.i+2,object.j) = 250;

			map.at<uchar>(object.i-1,object.j) = 250;
			map.at<uchar>(object.i-2,object.j) = 250;

			map.at<uchar>(object.i,object.j+1) = 250;
			map.at<uchar>(object.i,object.j+2) = 250;

			map.at<uchar>(object.i,object.j-1) = 250;
			map.at<uchar>(object.i,object.j-2) = 250;
		}
	}


	proc_map = map.clone();


	// Test
	if(mode == 2)
	{
		if((discrete_map.size() > 4) & !goto_target)
		{
			goto_target = true;
			target.x = 0;
			target.y = 0;

			Action action;
			action.n = ACTION_ROTATION;
			action.parameter1 = M_PI;
			priority.push_back(action);
		}
	}

	// Goto target
	if(goto_target & actions.empty() & priority.empty())
	{
		Node n;
		n.x = x_true;
		n.y = y_true;


		if((mode == EXPLORE) |(mode == 2))
		{
			for(int i = 0; i < important_nodes.size(); i++)
			{
				if(isPath(n,important_nodes.at(i).second))
				{
					printf("PATH FOUND\n");
					path_finding(important_nodes.at(i).second);
					goto_node(important_nodes.at(i).first);
					break;
				}
				else
				{
					printf("No path to x = %f, y = %f\n",important_nodes.at(i).second.x,important_nodes.at(i).second.y);
				}
			}
		}

		/*
		else
		{
			path_finding(target);
		}
		 */


		if(mode == GOTO_TARGETS)
		{
			for(int i = 0; i < important_nodes.size(); i++)
			{
				int j = important_nodes.size()-i-1;
				if(isPath(n,important_nodes.at(j).first))
				{
					printf("PATH FOUND\n");
					path_finding(important_nodes.at(j).first);
					goto_node(important_nodes.at(j).second);
					break;
				}
				else
				{
					printf("No path to x = %f, y = %f\n",important_nodes.at(j).first.x,important_nodes.at(j).first.y);
				}
			}
		}


		if(sqrt((x_true-target.x)*(x_true-target.x)+(y_true-target.y)*(y_true-target.y)) < 0.05)
		{
			Action action;
			action.n = ACTION_STOP;
			priority.push_back(action);
		}
	}


	// Check visited area
	visited_flag = visited_area();
	if(visited_flag & actions.empty() & priority.empty() & !goto_target)
	{
		printf("Already visited !\n");

		if(mode != 2)
		{
			//Path p = path(find_closest_node(toDiscover),target);
			//pathToActions(p);

			path_finding(find_closest_node(toDiscover));
		}
	}


	Node n;
	n.x = x_true;
	n.y = y_true;
	Pixel p = nodeToPixel(n);
	proc_map.at<uchar>(p.i,p.j) = 250;


	namedWindow("Map",CV_WINDOW_NORMAL);
	imshow("Map",proc_map);


	cvWaitKey(10);
}


/**
 * Draw the walls using Hough transform
 */
void Hough()
{
	vector<Vec4i> lines;
	HoughLinesP(wall_map,lines, 1,CV_PI/2,3,3,8);
	for( size_t i = 0; i < lines.size(); i++ )
	{
		line(map, Point(lines[i][0], lines[i][1]),
				Point(lines[i][2], lines[i][3]), Scalar(255), 1 );
	}

	/*
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
	 */
}


/**
 * Merge close areas (of the robot path) if there is no wall between then
 */
void merge_areas()
{
	for(int i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++)
		{
			if(robot_map.at<uchar>(i,j) == 100)
			{
				map.at<uchar>(i,j) = 100;
			}
		}
	}


	for(int i = 0; i < (height-sz1); i++)
	{
		for(int j = 0; j < (width-sz2); j++)
		{
			if((map.at<uchar>(i,j) == 100) &
					(map.at<uchar>(i+sz1-1,j) == 100) &
					(map.at<uchar>(i,j+sz2-1) == 100) &
					(map.at<uchar>(i+sz1-1,j+sz2-1) == 100))
			{
				bool flag = false;
				for(int n = 0; n < sz1; n++)
				{
					for(int m = 0; m < sz2; m++)
					{
						if(map.at<uchar>(i+n,j+m) == 255)
						{
							flag = true;
						}
					}
				}
				if(!flag)
				{
					for(int n = 0; n < sz1; n++)
					{
						for(int m = 0; m < sz2; m++)
						{
							map.at<uchar>(i+n,j+m) = 100;
						}
					}
				}
			}
		}
	}
}


/**
 * Find interesting areas that the robot has to discover
 * and update the toDiscover map
 */
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
						break;
					}
				}
				if(map.at<uchar>(i+n,j+sz1-1) != 100)
				{
					flag = true;
					break;
				}
			}
			if(!flag)
			{
				map.at<uchar>(i+5,j+sz1-2) = 200;
				create_interesting_node(i+5,j+sz1-2);
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
						break;
					}
				}
				if(map.at<uchar>(i+n,j) != 100)
				{
					flag = true;
					break;
				}
			}
			if(!flag)
			{
				map.at<uchar>(i+5,j+1) = 200;
				create_interesting_node(i+5,j+1);
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
						break;
					}
				}
				if(map.at<uchar>(j,i+n) != 100)
				{
					flag = true;
					break;
				}
			}
			if(!flag)
			{
				map.at<uchar>(j+1,i+5) = 200;
				create_interesting_node(j+1,i+5);
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
						break;
					}
				}
				if(map.at<uchar>(j+sz1-1,i+n) != 100)
				{
					flag = true;
					break;
				}
			}
			if(!flag)
			{
				map.at<uchar>(j+sz1-2,i+5) = 200;
				create_interesting_node(j+sz1-2,i+5);
			}
		}
	}
}


/**
 * Return true if we already visited the area where the robot is moving.
 */
bool visited_area()
{
	int rx = (x_true-origin_x+0.15*cos(theta_true))/resolution; // 0.2
	int ry = (y_true-origin_y+0.15*sin(theta_true))/resolution; // 0.2

	if(proc_map.at<uchar>(height-ry-1,rx) == 100)
	{
		return true;
	}

	return false;
}


/**
 * Find a simple path to reach n
 * from the current position
 */
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
				break;
			}
		}
		else
		{
			if(map.at<uchar>(p.i-i,p.j) != 100)
			{
				flag1 = true;
				break;
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
				break;
			}
		}
		else
		{
			if(map.at<uchar>(p_true.i,p.j-j) != 100)
			{
				flag1 = true;
				break;
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
				break;
			}
		}
		else
		{
			if(map.at<uchar>(p.i-i,p_true.j) != 100)
			{
				flag2 = true;
				break;
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
				break;
			}
		}
		else
		{
			if(map.at<uchar>(p.i,p.j-j) != 100)
			{
				flag2 = true;
				break;
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
}


/**
 * Return a path with nodes that the robot has to follow
 * to go from n1 to n2
 */
/*
Path path(Node n1, Node n2)
{
	Path path;
	std::vector<Pair> hash,end;

	for(int i = 0; i < discrete_map.size(); i++)
	{
		if(!(discrete_map.at(i) == n1))
		{
			if(isPath(n1,discrete_map.at(i)))
			{
				Path p;
				p.push_back(discrete_map.at(i));

				Pair t;
				t.first = discrete_map.at(i);
				t.second = p;
				hash.push_back(t);

				//hash[discrete_map.at(i)] = p;
			}
		}
	}

	Path p;
	p.push_back(n2);
	Pair t;
	t.first = n2;
	t.second = p;
	end.push_back(t);
	//end[n2] = p;

	for(int i = 0; i < hash.size(); i++)
	{
		if(hash.at(i).first == n2)
		{
			path.push_back(n2);
			return path;
		}
	}

	// BFS
	//Hash::iterator it;
	//for(it = hash.begin(); it != hash.end(); it++) // PROBLEM !
	for(int i = 0; i < hash.size(); i++)
	{
		//Node n = it->first;
		//Path p = hash[n];

		Node n = hash.at(i).first;
		Path p = hash.at(i).second;

		printf("Treat node: x = %f, y = %f\n",n.x,n.y);

		// Expand nodes
		for(int i = 0; i < discrete_map.size(); i++)
		{
			bool alreadyInHash = false;
			for(int j = 0; j < hash.size(); j++)
			{
				if(discrete_map.at(i) == hash.at(j).first)
				{
					alreadyInHash = true;
				}
			}
			if((!(discrete_map.at(i) == n1)) & !alreadyInHash)
			{
				if(isPath(n,discrete_map.at(i)))
				{
					Path pp = Path(p);
					pp.push_back(discrete_map.at(i));
					//hash[discrete_map.at(i)] = pp;
					Pair t;
					t.first = discrete_map.at(i);
					t.second = pp;
					hash.push_back(t);
					printf("Add node to hash: x = %f, y = %f\n",discrete_map.at(i).x,discrete_map.at(i).y);

					if(discrete_map.at(i) == n2)
					{
						path = pp;
						return path;
					}
				}
			}
		}
	}


	return path;
}
 */


/**
 * Transform coordinates into position on the map
 */
Pixel nodeToPixel(Node node)
{
	int px = (node.x-origin_x)/resolution;
	int py = (node.y-origin_y)/resolution;

	Pixel pixel;
	pixel.i = height-py-1;
	pixel.j = px;

	return pixel;
}


/**
 * Transform a position on the map into real coordinates
 */
Node pixelToNode(Pixel pixel)
{
	Node node;
	node.x = pixel.j*resolution+origin_x;
	node.y = (height-pixel.i-1)*resolution+origin_y;
	return node;
}


/**
 * @brief isPath  Check whether already explored path exists between Node n1 and n2
 * @param n1
 * @param n2
 * @return
 */
bool isPath(Node n1, Node n2)
{
	Pixel p1 = nodeToPixel(n1);
	Pixel p2 = nodeToPixel(n2);


	// Try 2 ways

	// Up Down
	bool flag1 = false;
	for(int i = 1; i < abs(p1.i-p2.i); i++)
	{
		if(p1.i <= p2.i)
		{
			if(map.at<uchar>(p1.i+i,p1.j) != 100)
			{
				flag1 = true;
				break;
			}
		}
		else
		{
			if(map.at<uchar>(p1.i-i,p1.j) != 100)
			{
				flag1 = true;
				break;
			}
		}

	}
	for(int j = 1; j < abs(p1.j-p2.j); j++)
	{
		if(p1.j <= p2.j)
		{
			if(map.at<uchar>(p2.i,p1.j+j) != 100)
			{
				flag1 = true;
				break;
			}
		}
		else
		{
			if(map.at<uchar>(p2.i,p1.j-j) != 100)
			{
				flag1 = true;
				break;
			}
		}
	}
	if(!flag1)
	{
		return true;
	}


	// Left Right
	bool flag2 = false;
	for(int i = 1; i < abs(p1.i-p2.i); i++)
	{
		if(p1.i <= p2.i)
		{
			if(map.at<uchar>(p1.i+i,p2.j) != 100)
			{
				flag2 = true;
				break;
			}
		}
		else
		{
			if(map.at<uchar>(p1.i-i,p2.j) != 100)
			{
				flag2 = true;
				break;
			}
		}

	}
	for(int j = 1; j < abs(p1.j-p2.j); j++)
	{
		if(p1.j <= p2.j)
		{
			if(map.at<uchar>(p1.i,p1.j+j) != 100)
			{
				flag2 = true;
				break;
			}
		}
		else
		{
			if(map.at<uchar>(p1.i,p1.j-j) != 100)
			{
				flag2 = true;
				break;
			}
		}
	}
	if(!flag2)
	{
		return true;
	}

	return false;
}


/**
 * Logic to reach a node
 * Turn and go forward
 */
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


/**
 * Save the current position
 */
void receive_odometry(const Odometry::ConstPtr &msg)
{
	x_true = msg->x;
	y_true = msg->y;
	theta_true = msg->theta;
}


/**
 * Create a node and add it to the discrete map
 */
void create_node(double x, double y)
{
	Node n;

	// Position
	n.x = x;
	n.y = y;

	discrete_map.push_back(n);

	current_node = n;

	// Debug
	printf("New node:  x = %f, y = %f\n",x,y);
}


/**
 * Create interesting node we have to visit
 * and save it to the toDiscover map
 */
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


/**
 * Find the node in a vector which is the closest to the actual position
 */
Node find_closest_node(std::vector<Node> vector)
{
	Node node;
	node.x = 0;
	node.y = 0;

	double dist = INFINITY;

	for(int i = 0; i < vector.size(); i++)
	{
		Node n = vector.at(i);
		double d = sqrt((n.x-x_true)*(n.x-x_true)+(n.y-y_true)*(n.y-y_true));
		if(d < dist)
		{
			node.x = n.x;
			node.y = n.y;
			dist = d;
		}
	}

	printf("Node found: x = %f, y = %f\n",node.x,node.y);
	printf("Position: x = %f, y = %f\n",x_true,y_true);

	return node;
}


/**
 * Logic when an object is detected
 */
void receive_object(const Object::ConstPtr &msg)
{
	double x_object = x_true + (msg->x+x_prime)*cos(theta_true) - (msg->y+y_prime)*sin(theta_true);
	double y_object = y_true + (msg->x+x_prime)*sin(theta_true) + (msg->y+y_prime)*cos(theta_true);

	//double x_object = x_true + (0.3+x_prime)*cos(theta_true) - (0+y_prime)*sin(theta_true);
	//double y_object = y_true + (0.3+x_prime)*sin(theta_true) + (0+y_prime)*cos(theta_true);

	Node node;

	// Update vectors
	node.x = x_object;
	node.y = y_object;
	objects.push_back(node);

	node.x = x_true;
	node.y = y_true;
	near_objects.push_back(node);

	merge_objects();


	// Talk
	std_msgs::String talk;
	std::stringstream ss;
	ss << "I see something. ";

	switch(msg->id)
	{
	case objRecognition::OBJTYPE_AVOCADO : {ss << "It is an avocado"; break;}
	case objRecognition::OBJTYPE_BANANA : {ss << "It is a banana"; break;}
	case objRecognition::OBJTYPE_BROCCOLI : {ss << "It is a broccoli"; break;}
	case objRecognition::OBJTYPE_CARROT : {ss << "It is a carrot"; break;}
	case objRecognition::OBJTYPE_CHILI : {ss << "It is a chili"; break;}
	case objRecognition::OBJTYPE_CORN : {ss << "It is a corn"; break;}
	case objRecognition::OBJTYPE_ELEPHANT : {ss << "It is an elephant"; break;}
	case objRecognition::OBJTYPE_GIRAFFE : {ss << "It is a giraffe"; break;}
	case objRecognition::OBJTYPE_GREEN_PUMPKIN : {ss << "It is a green pumpkin"; break;}
	case objRecognition::OBJTYPE_HIPPO : {ss << "It is a hippo"; break;}
	case objRecognition::OBJTYPE_LEMON : {ss << "It is a lemon"; break;}
	case objRecognition::OBJTYPE_LION : {ss << "It is a lion"; break;}
	case objRecognition::OBJTYPE_ONION : {ss << "It is an onion"; break;}
	case objRecognition::OBJTYPE_PEACH : {ss << "It is a peach"; break;}
	case objRecognition::OBJTYPE_PEAR : {ss << "It is a pear"; break;}
	case objRecognition::OBJTYPE_POTATO : {ss << "It is a potato"; break;}
	case objRecognition::OBJTYPE_TIGER : {ss << "It is a tiger"; break;}
	case objRecognition::OBJTYPE_TOMATO : {ss << "It is a tomato"; break;}
	case objRecognition::OBJTYPE_WATERMELON : {ss << "It is a watermelon"; break;}
	case objRecognition::OBJTYPE_ZEBRA : {ss << "It is a zebra"; break;}
	case objRecognition::OBJTYPE_RED_PLATE : {ss << "It is a red plate"; break;}
	default : break;
	}

	talk.data = ss.str();
	chatter_pub.publish(talk);

	std::cout << ss.str() << std::endl;

	if((mode == 0) & (objects.size() > 2)) // Change condition (number of objects)
	{
		// Goto start
		target.x = 0;
		target.y = 0;
		goto_target = true;

		// Turn
		Action action;
		action.n = ACTION_ROTATION;
		action.parameter1 = M_PI;
		priority.push_back(action);
	}


	if(mode == EXPLORE)
	{
		double theta = nPi2(theta_true)*M_PI/2;
		double x2 = x_true + 0.2*cos(theta);
		double y2 = y_true + 0.2*sin(theta);

		create_important_node_targets(x_true,y_true,x2,y2);
	}
}


/**
 * Remove new position near an object and object position
 * if it already exists another one close to this position
 */
void merge_objects()
{
	// Near objects
	Node o1 = near_objects.back();
	for(int i = 0; i < near_objects.size()-1; i++)
	{
		Node o2 = near_objects.at(i);
		double distance = sqrt((o1.x-o2.x)*(o1.x-o2.x)+(o1.y-o2.y)*(o1.y-o2.y));
		if(distance < 0.1)
		{
			near_objects.pop_back();
			break;
		}
	}

	// Objects
	o1 = objects.back();
	for(int i = 0; i < objects.size()-1; i++)
	{
		Node o2 = objects.at(i);
		double distance = sqrt((o1.x-o2.x)*(o1.x-o2.x)+(o1.y-o2.y)*(o1.y-o2.y));
		if(distance < 0.1)
		{
			objects.pop_back();
			return;
		}
	}
}


/**
 * Put the path in the actions list
 */
/*
void pathToActions(Path path)
{
	path_finding(path.at(0));

	// Debug
	printf("------------------------------------\n");
	printf("Path found:\n");
	for(int i = 0; i < path.size(); i++)
	{
		printf("x = %f, y = %f\n",path.at(i).x,path.at(i).y);
	}
	printf("------------------------------------\n");
}
 */


/**
 * Create nodes if the robot made a choice when it has to turn
 */
void create_important_node(double x1, double y1, double x2, double y2)
{
	Nodes nodes;
	nodes.first.x = x1;
	nodes.first.y = y1;
	nodes.second.x = x2;
	nodes.second.y = y2;
	important_nodes.push_back(nodes);

	printf("Important nodes: x1 = %f, y1 = %f, x2 = %f, y2 = %f\n",x1,y1,x2,y2);
}


/**
 * Create nodes if the robot made a choice when it has to turn
 * Made to find path to targets
 */
void create_important_node_targets(double x1, double y1, double x2, double y2)
{
	Nodes nodes;
	nodes.first.x = x1;
	nodes.first.y = y1;
	nodes.second.x = x2;
	nodes.second.y = y2;
	important_nodes_targets.push_back(nodes);

	printf("Important nodes targets: x1 = %f, y1 = %f, x2 = %f, y2 = %f\n",x1,y1,x2,y2);
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


/**
 * Return the number of times we have to multiply pi/2 to
 * have the closest angle to theta
 */
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
	ros::NodeHandle nh("~");

	nh.getParam("mode",mode);

	speed_pub = nh.advertise<explorer::Speed>("/motion/Speed",100);
	stop_EKF_pub =nh.advertise<Stop_EKF>("/motion/Stop_EKF",100);
	EKF_sub = nh.subscribe("/motion/EKF",1000,receive_EKF);
	sensors_sub = nh.subscribe("/sensors/ADC",100,receive_sensors);
	servo_pub = nh.advertise<Servomotors>("/actuator/Servo",100);
	odometry_sub = nh.subscribe("/motion/Odometry",1000,receive_odometry);
	object_sub = nh.subscribe("/recognition/object_pos_relative",1,receive_object);
	chatter_pub = nh.advertise<std_msgs::String>("/robot/talk", 10);


	// Map init
	proc_map = Mat::zeros(height,width,CV_8UC1);
	robot_map = Mat::zeros(height,width,CV_8UC1);
	wall_map = Mat::zeros(height,width,CV_8UC1);


	// Robot_talk
	/*
	string say_out = string("espeak \"") + "Go" + string("\"");
	system(say_out.c_str());
	 */


	// Mode
	if(mode == EXPLORE | mode == 2)
	{
		create_node(0,0);
		create_important_node(0,0,0.2,0);
		create_important_node_targets(0,0,0.2,0);
	}


	if(mode == GOTO_TARGETS)
	{
		// Open important_nodes_targets map
		important_nodes_targets.resize(500);
		std::ifstream is("/home/robo/explorer/important_nodes_targets.dat",std::ios::binary);
		is.read(reinterpret_cast<char*>(&(important_nodes_targets[0])),important_nodes_targets.size()*sizeof(Nodes));
		is.close();

		// Resize
		for(int i = 1; i < important_nodes_targets.size(); i++)
		{
			if((important_nodes_targets.at(i).second.x == 0) & (important_nodes_targets.at(i).second.y == 0))
			{
				//printf("test\n");
				important_nodes_targets.resize(i);
				break;
			}
		}


		// Open objects
		objects.resize(500);
		std::ifstream is2("/home/robo/explorer/objects.dat",std::ios::binary);
		is2.read(reinterpret_cast<char*>(&(objects[0])),objects.size()*sizeof(Node));
		is2.close();

		// Resize
		for(int i = 0; i < objects.size(); i++)
		{
			if((objects.at(i).x == 0) & (objects.at(i).y == 0))
			{
				objects.resize(i);
				break;
			}
		}


		// Open near_objects
		near_objects.resize(500);
		std::ifstream is3("/home/robo/explorer/near_objects.dat",std::ios::binary);
		is3.read(reinterpret_cast<char*>(&(near_objects[0])),near_objects.size()*sizeof(Node));
		is3.close();

		// Resize
		for(int i = 0; i < near_objects.size(); i++)
		{
			if((near_objects.at(i).x == 0) & (near_objects.at(i).y == 0))
			{
				near_objects.resize(i);
				break;
			}
		}


		// Open maps
		robot_map = imread("/home/robo/explorer/robot_map.png",0);
		wall_map = imread("/home/robo/explorer/wall_map.png",0);


		// Goto targets
		target.x = important_nodes_targets.back().second.x;
		target.y = important_nodes_targets.back().second.y;
		goto_target = true;


		// Debug
		//Node test = discrete_map.at(0);
		//printf("Target: x = %f, y = %f\n",test.x,test.y);
	}



	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}


	if(mode == EXPLORE)
	{
		// Save important_nodes_targets map
		important_nodes_targets.resize(500);
		std::ofstream os("/home/robo/explorer/important_nodes_targets.dat",std::ios::binary);
		os.write(reinterpret_cast<const char*>(&(important_nodes_targets[0])),important_nodes_targets.size()*sizeof(Nodes));
		os.close();

		// Save objects
		objects.resize(500);
		std::ofstream os2("/home/robo/explorer/objects.dat",std::ios::binary);
		os2.write(reinterpret_cast<const char*>(&(objects[0])),objects.size()*sizeof(Node));
		os2.close();

		// Save near_objects
		near_objects.resize(500);
		std::ofstream os3("/home/robo/explorer/near_objects.dat",std::ios::binary);
		os3.write(reinterpret_cast<const char*>(&(near_objects[0])),near_objects.size()*sizeof(Node));
		os3.close();

		// Save maps
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(0);

		imwrite("/home/robo/explorer/robot_map.png",robot_map,compression_params);
		imwrite("/home/robo/explorer/wall_map.png",wall_map,compression_params);
		imwrite("/home/robo/explorer/proc_map.png",proc_map,compression_params);
	}


	return 0;
}
