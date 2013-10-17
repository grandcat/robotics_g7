 /*
 * Bezier.h
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * Bezier Class
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define N 100

const float e=3E-1;

class Bezier{
//private:
public:

float x_control_point_[4];
float y_control_point_[4];

float x_coef_[4];
float y_coef_[4];

float V_r[N];
float V_l[N];
float t[N];

float get_x_value(float u);
float get_y_value(float u);

float get_dx_value(float u);
float get_dy_value(float u);

float get_ddx_value(float u);
float get_ddy_value(float u);

float get_ds_value(float u);
float get_r_value(float u);
	

//public:
	Bezier();

	~Bezier(void);

	int set_control_points(float x0, float y0, float theta0,float x1, float y1, float theta1);

	int compute_curve(float V_max,float A_max);



	


};

