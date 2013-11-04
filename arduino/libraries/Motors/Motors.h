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
float _k ;
float _ki ;
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

float _nlin_slope, _nlin_intercept;

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
void Set_speed(int u);

/*
 * Motors::Speed_regulation
 * Control the speed of the motors
 * parameters >
 * 		@ float W : 		 desired speed, rad/s
 * 		@ float Te : 		 sampling period, in seconds
 * 		@ int16_t deltaStep : 	 encoder difference
 */
void Speed_regulation(float W, float Te, int16_t deltaStep);

/*
 * Motors::Read_Current
 * Returns the measured Current, in Amps
 */
float Read_current();

/*
 * Motors::Set_control_parameters
 * Set the values of the control parameters
 * parameters >
 * 		@ float K :				proportionnal gain
 * 		@ float KI : 	 		integral gain
 * 		@ int INT_MAX :	 		integral saturation value
 * 		@ int ticks_per_rev :	Encoders ticks per revolution
 */
void Set_control_parameters(float K, float KI, int i_max,int ticks_per_rev);

void Set_nlin_parameters(float nlin_slope,float nlin_intercept);

void Reset();

};

