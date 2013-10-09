/*
 * tracking.h
 *
 *  Created on: Oct 2, 2013
 *      Author: robo
 */

#include <ros/ros.h>


// Publisher
ros::Publisher pos_pub;

// Size
int height = 480;
int width = 640;

// Detector frame size
int sx = 100;
int sy = 100;

// Object coordinate
int x;
int y;
int depth;

// Diffusion variance
int sigma_diffusion = 50;

// Number of particles
const int N = 100;

// Generate N random particles
int Px [N];
int Py [N];
double W [N];


double uniformRandom();
double normalRandom();
void object_coordinate();
void object_depth(IplImage* img_depth);
void showCoord(IplImage* img);
void showParticle(IplImage* img);
double evaluate(int x, int y, IplImage* img);
void init_part();
void particle_filter(IplImage* img);
