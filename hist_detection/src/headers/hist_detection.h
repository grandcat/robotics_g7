/*
 * hist_detection.h
 *
 *  Created on: Nov 14, 2013
 *      Author: robo
 */

#include <ros/ros.h>

using namespace cv;


// Publisher
ros::Publisher object_detection_pub;


// Size
int height = 480;
int width = 640;

// Detector frame size
int sx = 120;
int sy = 120;

// Object coordinate
int x;
int y;
int depth;
int object;

// Diffusion variance
int sigma_diffusion = 30;

// Number of particles
const int N = 300;

// Generate N random particles
int Px [N];
int Py [N];
double W [N];
int n [N];


// Images
const int nb_images = 8;
const int nb_frames_init = 50;
const int detector = 10;
int cmpt = 0;
int current_object;
const double threshold_proba = 1.5;

// Hist
Mat hist[nb_images][3];
const int norm_hist = 500;
double errors[nb_images];
const int comp = CV_COMP_BHATTACHARYYA;


void init();

double uniformRandom();
double normalRandom();
void object_coordinate();
void showCoord(IplImage* img);
void showParticle(IplImage* img);
double evaluate(int x, int y, IplImage* img,int n_particule);
void init_part();
void particle_filter(IplImage* img);
