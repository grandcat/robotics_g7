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

using namespace differential_drive;
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
		if(!msg->wall) {y_cmd = y;}
		else {y_cmd = y_wall - y_cmd_traj + y_cmd_change;}

		dist = x_cmd_traj;
		diff_ang = atan((y_cmd-y)/(x_cmd-x))-theta;
		diff_ang = angle(diff_ang);

		speed.V = rho*dist*r;
		speed.W = -2*r/l*alpha*diff_ang;

		//printf("Wall following\n");
	}


	// Do a special movement
	else
	{
		if(!busy)
		{
			current_action = actions.front();
			busy = true;

			// Stop EKF
			Stop_EKF s;
			s.stop = true;
			stop_EKF_pub.publish(s);
		}


		else
		{
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
				speed.W = 2*r/l*alpha*dtheta/4;

				// Rotation done
				if(dtheta*dtheta < M_PI*M_PI/180/180*theta_error*theta_error)
				{
					create_node(x_true,y_true);

					if(s1 > 0.2)
					{
						x_pb = x;

						Action action;
						action.n = ACTION_FORWARD;
						action.parameter1 = x_forward_dist;
						actions.push_back(action);
					}

					actions.pop_front();
					busy = false;

					// Relaunch EKF
					Stop_EKF s;
					s.stop = false;
					s.rotation_angle = current_action.parameter1;
					stop_EKF_pub.publish(s);
				}

				// Re-init y_cmd_change
				y_cmd_change = 0;

				//printf("Rotation\n");
			}


			// Go backward
			if(current_action.n == ACTION_BACKWARD)
			{
				double x_cmd = x - x_cmd_traj;
				double y_cmd = y;

				dist = x_pb-x-x_backward_dist;
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

				//printf("Backward %f\n",dist);
			}


			// Go forward
			if(current_action.n == ACTION_FORWARD)
			{
				double x_cmd = x + x_cmd_traj;
				double y_cmd = y;

				dist = x_pb-x+current_action.parameter1;
				diff_ang = atan((y_cmd-y)/(x_cmd-x))-theta;
				diff_ang = angle(diff_ang);

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
					if(s1 > 0.2)
					{
						Action action;
						action.n = ACTION_ROTATION;
						action.parameter1 = M_PI/2;
						actions.push_back(action);
					}

					actions.pop_front();
					busy = false;

					// Relaunch EKF
					Stop_EKF s;
					s.stop = false;
					s.rotation_angle = 0;
					stop_EKF_pub.publish(s);
				}
			}


			// Change y_cmd_traj
			if(current_action.n == ACTION_CHANGE_Y_CMD_TRAJ)
			{
				y_cmd_change = current_action.parameter1;

				actions.pop_front();
				busy = false;

				// Relaunch EKF
				Stop_EKF s;
				s.stop = false;
				s.rotation_angle = 0;
				stop_EKF_pub.publish(s);
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

				//printf("dist = %f\n",dist);


				static bool init;
				if(!init)
				{
					init = true;

					printf("goto node: x = %f, y = %f\n",current_action.parameter1,current_action.parameter2);
				}


				// Rotate first if needed
				if(fabs(diff_ang) > M_PI/180*15)
				{
					// Rotation saturation
					if(diff_ang > M_PI/2) {diff_ang = M_PI/2;}
					if(diff_ang < -M_PI/2) {diff_ang = -M_PI/2;}

					speed.V = 0;
					speed.W = -2*r/l*alpha*diff_ang/4;
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

						init = false;
					}
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

	// Bumpers
	//bool s6 = (msg->ch6 > bumper_threshold); // center
	//bool s7 = (msg->ch7 > bumper_threshold); // right
	//bool s8 = (msg->ch8 > bumper_threshold); // left
	bool s6 = false;
	bool s7 = false;
	bool s8 = false;

	if(actions.empty())
	{
		if(discrete_map.size() > 1000)
		{
			path_finding();
			return;
		}


		// Hurt a wall
		if(s7 | s8)
		{
			x_pb = x;

			Action action;
			action.n = ACTION_BACKWARD; action.parameter1 = x_backward_dist;
			actions.push_back(action);
			action.n = ACTION_CHANGE_Y_CMD_TRAJ; action.parameter1 = 0.04;
			actions.push_back(action);

			return;
		}

		if(s6)
		{
			x_pb = x;

			Action action;
			action.n = ACTION_BACKWARD; action.parameter1 = x_backward_dist;
			actions.push_back(action);
			action.n = ACTION_ROTATION; action.parameter1 = -M_PI;
			actions.push_back(action);

			return;
		}


		// Wall in front of the robot
		if(s3 < dist_front_wall)
		{
			if(cmpt >= obstacle)
			{
				cmpt = 0;

				Action action;
				action.n = ACTION_ROTATION;
				action.parameter1 = -M_PI/2;
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


		// No left wall
		if(s1 > 0.2)
		{
			x_pb = x;

			Action action;
			action.n = ACTION_FORWARD;
			action.parameter1 = x_catch_wall;
			actions.push_back(action);

			//printf("No left wall\n");
		}
	}
}


void path_finding()
{
	// Go back to the base;
	std::list<Node> back_map = discrete_map;
	back_map.pop_back();


	while(!back_map.empty())
	{
		goto_node(back_map.back());
		back_map.pop_back();
	}

	Action action;
	action.n = ACTION_STOP;
	actions.push_back(action);
}


void goto_node(Node node)
{
	Action action;

	action.n = ACTION_GOTO;
	action.parameter1 = node.x;
	action.parameter2 = node.y;
	actions.push_back(action);
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

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
