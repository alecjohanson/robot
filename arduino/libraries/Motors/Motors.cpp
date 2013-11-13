/*
 * Motors.h
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * Motors Class
 *
 * Without indications, value are in International System of Units
 *
 */

#include <Motors.h>

/* Constructor */
Motors::Motors(char dir_pin,char pwm_pin,char brk_pin,char cfb_pin):
	_int_max(30.),
	_int(0.),
	_k(12.),
	_ki(5.),
	_nlin_intercept(59.),
	_nlin_slope(0.64),
	_speed(0.),
	_ticks_per_rev(360)
{
	_dir_pin = dir_pin ;
	_pwm_pin = pwm_pin ;
	_brk_pin = brk_pin ;
	_cfb_pin = cfb_pin ;

	pinMode(_dir_pin,OUTPUT);
	pinMode(_pwm_pin,OUTPUT);
	pinMode(_brk_pin,OUTPUT);

}

/* Destructor */
Motors::~Motors(void) {

}

/**
 * Generic methods
 */


/*
 * Motors::Set_speed(int u)
 * Set the duty cycle of the PWM
 * parameter >
 * 		@ int u : duty cycle value, between -255 and 255.
 *                        -255: (reverse maximal speed)
 *                        0: (no motion)
 *                        255: (maximal speed)
 *
 * 	Assuming that Vcc is you power supply voltage, voltage V applied to the motor is
 * 	V = Vcc*u/255
 *
 */
void Motors::Set_speed(int u)	{
	digitalWrite(_cfb_pin, LOW);  // No brake
	if (u<0)  {
		//if(u>-40)	{u=0;}
		digitalWrite(_dir_pin, LOW);
		analogWrite(_pwm_pin,-u);
	}
	else {
		//if(u<40)	{u=0;}
		digitalWrite(_dir_pin, HIGH);
		analogWrite(_pwm_pin,u);
	}
	return;
}

/*
 * Motors::Speed_regulation
 * Control the speed of the motors
 * parameters >
 * 		@ float W : 		 desired speed, rad/s
 * 		@ float Te : 		 sampling period, in seconds
 */
void Motors::Speed_regulation(float W, float Te) {
	float _error = W - _speed;
	_int+= _error*Te;

	if(_int>_int_max) {_int = _int_max;}
	else if(_int<-_int_max)  {_int = -_int_max;}

	float ctrlOut = _k*_error+_ki*_int;

	int pwm;
	if(abs(W)<0.01) pwm=0;
	else if(signbit(ctrlOut)==signbit(W)) pwm=_nlin_intercept+ctrlOut*ctrlOut*_nlin_slope;
	else pwm=_nlin_intercept-abs(ctrlOut);
	if(pwm > 255) pwm = 255;
	if(pwm<0) pwm=0;

	if(W<0.0) digitalWrite(_dir_pin, LOW);
	else digitalWrite(_dir_pin, HIGH);
	analogWrite(_pwm_pin,pwm);
}

/*
 * parameters >
 * 		@ int16_t deltaStep : 	 encoder difference
 */
void Motors::calculateSpeed(float Te, int16_t deltaStep) {
	_speed = deltaStep*2.*M_PI/(_ticks_per_rev*Te);
}

/*
 * Motors::Read_Current
 * Returns the measured Current, in Amps
 */
float Motors::Read_current() {
	return analogRead(_cfb_pin)*5./1024.*1.65;
}


/*
 * Motors::Set_control_parameters
 * Set the values of the control parameters
 * parameters >
 * 		@ float K :				proportionnal gain
 * 		@ float KI : 	 		integral gain
 * 		@ int INT_MAX :	 		integral saturation value
 * 		@ int ticks_per_rev :	Encoders ticks per revolution
 */
void Motors::Set_control_parameters(float K, float KI, int i_max, int ticks_per_rev) {
	_k = K;
	_ki = KI;
	_int_max = i_max;
	_ticks_per_rev = ticks_per_rev;
	return;
}

void Motors::Set_nlin_parameters(float nlin_slope,float nlin_intercept) {
	_nlin_slope=nlin_slope;
	_nlin_intercept=nlin_intercept;
	return;
}

void Motors::Reset()	{
	_int = 0;
}


