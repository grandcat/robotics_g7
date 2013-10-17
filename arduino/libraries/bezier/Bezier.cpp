 /*
 * Bezier.cpp
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * Bezier Class
 */

#include "Bezier.h"
#include <math.h>

/*** Bezier Class ***/

Bezier::Bezier() {}

Bezier::~Bezier(void) {}

/**
 * Generic methods
 */

/**
 * Setting the Control points. 
 * @ Initial position : x0, y0, theta0
 * @ Destination position : x1, y1, theta1
 */
int Bezier::set_control_points(float x0, float y0, float theta0,float x1, float y1, float theta1)	{
	if(x0 == x1 && y0 == y1) {
	printf("Not smarter enough to solve the problem... Same initial and final positions \n");
	return 1;
	}	
	
	float d=0.5*sqrt(pow((x0-x1),2)+pow((y0-y1),2));	
	this->x_control_point_[0] = x0 ;
	this->x_control_point_[1] = x0+d*cos(theta0) ;
	this->x_control_point_[2] = x1-d*cos(theta1) ;
	this->x_control_point_[3] = x1 ;
	this->y_control_point_[0] = y0 ;
	this->y_control_point_[1] = y0+d*sin(theta0) ;
	this->y_control_point_[2] = y1-d*sin(theta1) ;
	this->y_control_point_[3] = y1 ;
        return 0;
}

/**
 * Computing the curve from the Control points
 * @ Initial position : x0, y0, theta0
 * @ Destination position : x1, y1, theta1
 * @ float V_max : Maximal speed
 * @ float A_max : Maximal acceleration
 *
 *
 *	The curve is in a parametric representation
 *		X(u) = x_coef_(0)*u^3 + x_coef_(1)*u^2 + x_coef_(2)*u + x_coef_(3)
 *	        Y(u) = y_coef_(0)*u^3 + y_coef_(1)*u^2 + y_coef_(2)*u + y_coef_(3)
 *
 *   For further details : http://en.wikipedia.org/wiki/B%C3%A9zier_curve
 */

int Bezier::compute_curve(float V_max,float A_max)	{
	int i;
	this->x_coef_[0] = this->x_control_point_[3] - 3*this->x_control_point_[2] + 3*this->x_control_point_[1] - this->x_control_point_[0] ;
	this->x_coef_[1] = 3*this->x_control_point_[2] - 6*this->x_control_point_[1] + 3*this->x_control_point_[0] ;
	this->x_coef_[2] = 3*this->x_control_point_[1] - 3*this->x_control_point_[0];
	this->x_coef_[3] = this->x_control_point_[0];

	this->y_coef_[0] = this->y_control_point_[3] - 3*this->y_control_point_[2] + 3*this->y_control_point_[1] - this->y_control_point_[0] ;
	this->y_coef_[1] = 3*this->y_control_point_[2] - 6*this->y_control_point_[1] + 3*this->y_control_point_[0] ;
	this->y_coef_[2] = 3*this->y_control_point_[1] -3*this->y_control_point_[0];
	this->y_coef_[3] = this->y_control_point_[0];

	float curve_length = 0 ;
	for(i=0;i<N;i++)	{
		curve_length += get_ds_value(i*1./N)*1./N ;
	}
	printf("Curve Length : %f \n",curve_length);
	float eta = V_max/A_max+curve_length/V_max ;
	printf("ETA : %f \n",eta);
	float time_step = eta*1./N ;
	float ratio ;
	float r;
	float v;
	float u_c = 0;
	float V_lin_cons;
	
	for(i=0;i<N;i++)	{
		if(i==0){this->t[i] = 0 ; }
		else{this->t[i] = this->t[i-1] + time_step;}	
	
		r = get_r_value(u_c);
		if(isnan(r))	{ratio = 1.;}
		else{ratio = (r+e/2.)/(r-e/2.);}

		//printf("bend radius : %f speed ratio : %f \n",r,ratio);		

		V_lin_cons = fmin(V_max,fmin(A_max*this->t[i], A_max*(eta-this->t[i])));


		if(r<1)	{
			v = 2*V_lin_cons/(1.+1./ratio);
			this->V_r[i]= v ;
            		this->V_l[i]= v/ratio; 
		}
		else	{
			v = V_lin_cons*2./(1.+ratio);
			this->V_l[i]= v;
            		this->V_r[i]= v*ratio;
		}
		//printf("V_r : %f V_l : %f \n",V_r[i],V_l[i]);
		u_c += 0.5*(V_l[i]+V_r[i])*time_step/get_ds_value(i*1./N);
	}
	return 0;
		
}

/**
 * Get the (X,Y) value in the parameter u. 
 * @ float u : curve parameter
 */
float Bezier::get_x_value(float u)	{
	//if(u<0 || u>1) {return -1;}
	
	return this->x_coef_[0]*pow(u,3) + this->x_coef_[1]*pow(u,2) + this->x_coef_[2]*u + this->x_coef_[3];
}

float Bezier::get_y_value(float u)	{
	//if(u<0 || u>1) {return -1;}

	return this->y_coef_[0]*pow(u,3) + this->y_coef_[1]*pow(u,2) + this->y_coef_[2]*u + this->y_coef_[3];

}

/**
 * Get the (dX,dY) value in the parameter u. 
 * @ float u : curve parameter
 */
float Bezier::get_dx_value(float u)	{
	//if(u<0 || u>1) {return -1;}
	return 3*this->x_coef_[0]*pow(u,2) + 2*this->x_coef_[1]*u + this->x_coef_[2] ;
}

float Bezier::get_dy_value(float u)	{
	//if(u<0 || u>1) {return -1;}
	return 3*this->y_coef_[0]*pow(u,2) + 2*this->y_coef_[1]*u + this->y_coef_[2] ;
}

/**
 * Get the (d2X,d2Y) value in the parameter u. 
 * @ float u : curve parameter
 */
float Bezier::get_ddx_value(float u)	{
	//if(u<0 || u>1) {return -1;}
	return 6*this->x_coef_[0]*u + 2*this->x_coef_[1] ;
}

float Bezier::get_ddy_value(float u)	{
	//if(u<0 || u>1) {return -1;}

	return 6*this->y_coef_[0]*u + 2*this->y_coef_[1] ;
}

/**
 * Get the derivative value of the curvi-linear abscissa in the parameter u. 
 * @ float u : curve parameter
 */
float Bezier::get_ds_value(float u)	{
	//if(u<0 || u>1) {return -1;}
	float dx,dy;
	dx = get_dx_value(u);
	dy = get_dy_value(u);
	float ds = sqrt(pow(dx,2) + pow(dy,2));
	return ds;
}

/**
 * Get the bend radius value in the parameter u.
 * @ float u : curve parameter
 */
float Bezier::get_r_value(float u)	{
	//if(u<0 || u>1) {return -1;}	
	float ds = get_ds_value(u);
	float dx = get_dx_value(u);
	float d2x = get_ddx_value(u);
	float dy = get_dy_value(u);
	float d2y = get_ddy_value(u);

	float r = pow(ds,3)/(dx*d2y-dy*d2x);
	return r;
}

