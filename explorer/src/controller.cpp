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
		else {y_cmd = y_wall + y_cmd_traj - y_cmd_change;}

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
				theta_cmd = current_action.parameter;
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
					if(!back)
					{
						create_node(current_action.parameter);
					}

					if((s1 > 0.2) & !back)
					{
						x_pb = x;

						Action action;
						action.n = ACTION_FORWARD;
						action.parameter = x_forward_dist;
						actions.push_back(action);
					}

					actions.pop_front();
					busy = false;

					// Relaunch EKF
					Stop_EKF s;
					s.stop = false;
					s.rotation_angle = current_action.parameter;
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

				dist = x_pb-x+current_action.parameter;
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
					if((s1 > 0.2) & !back)
					{
						Action action;
						action.n = ACTION_ROTATION;
						action.parameter = M_PI/2;
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
				y_cmd_change = current_action.parameter;

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
	double s2 = a_short*pow(msg->ch2,b_short); // right
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
		// Go back to the base;
		if(discrete_map.size() > 3)
		{
			back = true;
			std::list<Node> back_map = discrete_map;
			Action action;

			// Turn to go back
			action.n = ACTION_ROTATION;
			action.parameter = back_map.back().rotation;
			actions.push_back(action);

			action.n = ACTION_FORWARD;
			action.parameter = back_map.back().x;
			actions.push_back(action);

			back_map.pop_back();


			while(!back_map.empty())
			{
				action.n = ACTION_ROTATION;
				action.parameter = -back_map.back().rotation;

				action.n = ACTION_FORWARD;
				action.parameter = back_map.back().x;
				actions.push_back(action);

				back_map.pop_back();
			}


			action.n = ACTION_STOP;
			actions.push_back(action);

			return;
		}


		// Hurt a wall
		if(s7 | s8)
		{
			x_pb = x;

			Action action;
			action.n = ACTION_BACKWARD; action.parameter = x_backward_dist;
			actions.push_back(action);
			action.n = ACTION_CHANGE_Y_CMD_TRAJ; action.parameter = 0.04;
			actions.push_back(action);

			return;
		}

		if(s6)
		{
			x_pb = x;

			Action action;
			action.n = ACTION_BACKWARD; action.parameter = x_backward_dist;
			actions.push_back(action);
			action.n = ACTION_ROTATION; action.parameter = -M_PI;
			actions.push_back(action);

			return;
		}


		// Wall in front of the robot
		if(s3 < dist_front_wall)
		{
			if(cmpt >= obstacle)
			{
				Action action;
				action.n = ACTION_ROTATION;
				action.parameter = -M_PI/2;
				actions.push_back(action);

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
			action.parameter = x_catch_wall;
			actions.push_back(action);
		}
	}
}


void receive_odometry(const Odometry::ConstPtr &msg)
{
	x_true = msg->x;
	y_true = msg->y;
}


void create_node(double rotation)
{
	Node n;

	// Position
	n.x = x;

	// Rotation
	n.rotation = rotation;

	discrete_map.push_back(n);

	// Debug
	printf("New node:  x = %f, rotation = %f\n",x,rotation);
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

	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
