 /*
 * Motors.h
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * Motors Class
 */

#include "Arduino.h"

class Motors{

public:
int _k ;
int _ki ;
int _int_max ;
char _dir_pin ;
char _pwm_pin ;
char _brk_pin ;
char _cfb_pin ;	
int _ticks_per_rev ;

int _encoder_value_old ;
int _encoder_value ;



float _speed_instruction ;
float _speed ;

float _error ;
float _int;

Motors(char dir_pin,char pwm_pin,char brk_pin,char cfb_pin);
~Motors(void);

/* Generic methods */

/*
 * Motors::Set_speed(int u)
 * Set the duty cycle of the PWM
 * parameter >
 * 		@ int u : duty cycle value, between 0 (no motion) and 255 (maximal speed)
 * 		
 * 	Assuming that Vcc is you power supply voltage, voltage V applied to the motor is 
 * 	V = Vcc*u/255
 * 	
 */
int Set_speed(int u);

/*
 * Motors::Speed_regulation
 * Control the speed of the motors
 * parameters >
 * 		@ float W : 		 desired speed, rad/s
 * 		@ float Te : 		 sampling period, in seconds
 * 		@ int encoder : 	 encoder value
 * 		@ int encoder_old :	 encoder old value
 */
int Speed_regulation(float W, float Te, int encoder, int encoder_old);

/*
 * Motors::Read_Current
 * Returns the measured Current, in Amps
 */
float Read_current();

/*
 * Motors::Read_Speed
 * Returns the measured Speed, in rad/s
 * parameters >
 * 		@ float Te : 		 sampling period, in seconds
 * 		@ int encoder : 	 encoder value
 * 		@ int encoder_old :	 encoder old value
 */
float Read_speed(int encoder, int encoder_old, float Te);

/*
 * Motors::Set_control_parameters
 * Set the values of the control parameters
 * parameters >
 * 		@ float K :				proportionnal gain
 * 		@ float KI : 	 		integral gain
 * 		@ int INT_MAX :	 		integral saturation value
 * 		@ int ticks_per_rev :	Encoders ticks per revolution
 */
int Set_control_parameters(int K, int KI, int INT_MAX,int ticks_per_rev);

int Reset();

};

