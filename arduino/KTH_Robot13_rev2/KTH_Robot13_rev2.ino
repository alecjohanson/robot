/*
 *
 * KTH_Robot13_rev2.ino
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * Device target : Arduino Mega 2560 Rev3
 * Shields : | Official ArduinoMotorShield
 *           | Home made KTHDD2425 board
 *
 * Main embedded project for KTH DD2425
 *
 * Speed regulation is done in a timer interrupt.
 * Don't forget to initialize the controller values, and your robot's size
 * by using the "/human/Parameters" topic
 *
 * Subscribes on topics :
 *      | "/motion/Speed"        to receive speed instructions (rad/s)
 *      | "/human/Parameters"    to receive parameters
 *      | "/actuator/Servo"      to receive servo desired position
 *
 * Publishes on topics :
 *      | "/motion/Odometry"      send odometry
 *      | "/sensors/ADC"          send ADC values, including battery voltage
 */


#ifndef Arduino_h
//just to make the eclipse indexer happy
#include <Arduino.h>
#endif

#include <ros.h>
#include <differential_drive/Speed.h>
#include <differential_drive/Odometry.h>
#include <differential_drive/Encoders.h>
#include <differential_drive/AnalogC.h>
#include <differential_drive/Params.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Servomotors.h>
#include <std_msgs/Header.h>
#include <Motors.h>
#include <Servo.h>
#include <stdint.h>
#include <math.h>
//#include <music.h>

/* PinOut definition */
#define ChA_Dir 12
#define ChA_Pwm 3
#define ChA_Brk 9
#define ChA_CFb A0

#define ChB_Dir 13
#define ChB_Pwm 11
#define ChB_Brk 8
#define ChB_CFb A1

//some hacks to generate register and ISR names
#define CONCAT(a,b) a ## b
#define CONCAT3(a,b,c) a ## b ## c
#define EXTINT(n) CONCAT3(INT,n,_vect)
#define PIN(c) CONCAT(PIN,c)

//Wheel encoders
#define ENC1_PORT D
#define ENC1_PHASE1 0 ///<Arduino digital pin 20
#define ENC1_PHASE2 1 ///<Arduino digital pin 21
#define ENC2_PORT D
#define ENC2_PHASE1 2 ///<Arduino digital pin 18
#define ENC2_PHASE2 3 ///<Arduino digital pin 19

//Timing constants
#define PERIOD_TIMER_MS 50
#define PERIOD_WATCHDOG_MS 2000
#define PERIOD_ADC_MS 80 ///< IR sensors have a measurement period of 40.
#define PERIOD_BATT_MS 2000

#define N 2 // Averaging of measured speed with the N last values

/* Battery monitoring const */
#define seuil_cell 3.6
#define seuil_batt 10.8

volatile static uint16_t adcvals[16];

int led_pin[6] = {38,40,42,44,46,48};

/* Motor objects creation */
/* Please refer to "Motors.h" for further information */
Motors MotorA(ChA_Dir,ChA_Pwm,ChA_Brk,ChA_CFb);  // connector A = yellow tape
Motors MotorB(ChB_Dir,ChB_Pwm,ChB_Brk,ChB_CFb);

/* Servo motors definition */
Servo servo[8];
char servo_pin[] = {22,24,26,28,30,32,34,36};

/// Set speed value for motor 1
volatile static float W1_set;
/// Set speed value for motor 2
volatile static float W2_set;
/// timer period of encoder measurements and PI control
static const float Te=PERIOD_TIMER_MS*1e-3;
/// Wheel base of the robot
static float B = 205e-3;
/// Radius of the right wheel
static float r_r = 0.5*99.8e-3;
/// Radius of the left wheel
static float r_l = 0.5*99.8e-3;

static float x;
static float y;
static float theta;

volatile static int16_t encoder1, encoder2;

volatile static unsigned long wdtime;

volatile static uint8_t flags;
#define FLAG_ENCUPDATE (1<<0)
#define FLAG_ADCRESULT (1<<1)
#define FLAG_PWMDIRECT (1<<7)

static void messageSpeed( const differential_drive::Speed& cmd_msg);
static void messageParameters(const differential_drive::Params& params);
static void messageServo(const differential_drive::Servomotors& params);
static void messagePWM(const differential_drive::PWM& pwm);
static void updateOdometry(void);

/* ROS stuff */
ros::NodeHandle  nh;
differential_drive::Encoders encmsg;
differential_drive::Odometry odomsg;
differential_drive::AnalogC amsg;
ros::Publisher pubEnc("/motion/Encoders", &encmsg);
ros::Publisher pubOdom("/motion/Odometry", &odomsg);
ros::Publisher sensor("/sensors/ADC", &amsg);
ros::Subscriber<differential_drive::Speed> subSpeed("/motion/Speed", &messageSpeed);
ros::Subscriber<differential_drive::Servomotors> subServo("/actuator/Servo", &messageServo);
ros::Subscriber<differential_drive::Params> subParam("/motion/Parameters", &messageParameters);
ros::Subscriber<differential_drive::PWM> subPWM("/motion/PWM", &messagePWM);

void setup()  {
	/* Set the motors in stby */
	MotorA.Set_speed(0);
	MotorB.Set_speed(0);
	flags|=FLAG_PWMDIRECT;

	//todo: These values need some more adjustment.
	MotorA.Set_control_parameters(6.0, 2, 30, 360);
	MotorA.Set_nlin_parameters(0.64,57.0);
	MotorB.Set_control_parameters(6.0, 2, 30, 360);
	MotorB.Set_nlin_parameters(0.64,63.0);

	//configure external interrupts
	EICRA=0x55;  //set interrupts 0..3 to trigger on both edges
	EIMSK|=0x0f; //activate interrupts 0..3

	/* define the outputs pins */
	for(int i=0;i<6;i++) {
		pinMode(led_pin[i],OUTPUT);
	}
	pinMode(7,OUTPUT);

	TIMSK4=_BV(ICIE4); //enable timer 4 interrupt
	ICR4=(F_CPU/1000L/64L*PERIOD_TIMER_MS)-1; //set timer 4 period
	//configure timer 4 for CTC mode, prescaler=64
	TCCR4A=0;
	TCCR4B=_BV(WGM43)|_BV(WGM42)|3;

	// Configure servo motor pins
	for(int i=0;i<8;i++) {
		servo[i].attach(servo_pin[i]);

	}

	// Initialize ROS stuff
	nh.initNode();  // initialize node

	nh.advertise(pubEnc);  // advertise on pubEnc
	nh.advertise(pubOdom);  // advertise on pubOdom
	nh.advertise(sensor);  // advertise on sensor

	nh.subscribe(subSpeed);  // Subscribe
	nh.subscribe(subServo);  // Subscribe
	nh.subscribe(subParam);  // Subscribe
	nh.subscribe(subPWM);  // Subscribe

	sei(); //switch interrupts on
}

/*****************************
 * Main Loop
 *
 * Watchdog timer : if no message received during 2s, set the motors in stby,
 * computer may have crashed.
 ******************************/
void loop()  {
	static unsigned long t_bat;
	static unsigned long t_ADC;
	static boolean low_batt = false ;
	nh.spinOnce();
	unsigned long t=millis();

	// Watchdog timer
	if(t-wdtime >= PERIOD_WATCHDOG_MS)  {
		cli();
		MotorA.Set_speed(0);
		MotorB.Set_speed(0);
		flags|=FLAG_PWMDIRECT;
		sei();
		wdtime += PERIOD_WATCHDOG_MS;
	}

	//trigger measurement of ADC values
	if(t-t_ADC>PERIOD_ADC_MS && !(ADCSRA&_BV(ADSC))) {
		//the battery values are included less often
		if(t-t_bat > PERIOD_BATT_MS) {
			ADMUX=_BV(REFS0)|5;
			ADCSRB=0;
			t_bat += PERIOD_BATT_MS;
		} else {
			ADMUX=_BV(REFS0);
			ADCSRB=_BV(MUX5);
		}
		ADCSRA=_BV(ADEN)|_BV(ADSC)|_BV(ADIE)|3;
		t_ADC += PERIOD_ADC_MS;
	}

	if(flags&FLAG_ENCUPDATE) {
		flags&=~FLAG_ENCUPDATE;
		pubEnc.publish(&encmsg);
		updateOdometry();
	}

	if(flags&FLAG_ADCRESULT) {
		flags&=~FLAG_ADCRESULT;
		amsg.ch1 = adcvals[8];
		amsg.ch2 = adcvals[9];
		amsg.ch3 = adcvals[10];
		amsg.ch4 = adcvals[11];
		amsg.ch5 = adcvals[12];
		amsg.ch6 = adcvals[13];
		amsg.ch7 = adcvals[14];
		amsg.ch8 = adcvals[15];
		float v1 = adcvals[7]*0.0049*1.5106;
		float v2 = adcvals[6]*0.0049*2.9583;
		float v3 = adcvals[5]*0.0049*2.9583;
		amsg.cell1 = v1;
		amsg.cell2 = v2 - v1;
		amsg.cell3 = v3 - v2;
		amsg.on_batt = digitalRead(10);
		for(int m=0;m<floor((v3-seuil_batt)*4);m++)
			digitalWrite(led_pin[m],HIGH);
		for(int m=floor((v3-seuil_batt)*4);m<6;m++)
			digitalWrite(led_pin[m],LOW);
		if((amsg.cell1<seuil_cell || amsg.cell2<seuil_cell || amsg.cell3<seuil_cell) && v1>2)
			low_batt = true ;
		if(low_batt && ((amsg.cell1>seuil_cell+0.2 || amsg.cell2>seuil_cell+0.2 || amsg.cell3>seuil_cell+0.2) || v1<2))
			low_batt = false;
		if(low_batt) tone(7,440,500);
		sensor.publish(&amsg);
	}
}

/* Subscriber Callback */
static void messageSpeed( const differential_drive::Speed& cmd_msg) {
	cli();
	// get the speed from message
	W1_set = cmd_msg.W1;
	W2_set = cmd_msg.W2;
	wdtime = millis() ;  // store the time for Watchdog
	flags&=~FLAG_PWMDIRECT;
	sei();
}

static void messageParameters(const differential_drive::Params& params) {
	cli();
	MotorA.Set_control_parameters(params.K,params.KI,params.INT_MAX,params.ticks);
	MotorB.Set_control_parameters(params.K,params.KI,params.INT_MAX,params.ticks);
	r_r = params.r_r;
	r_l = params.r_l;
	B = params.B;
	sei();
}

static void messageServo(const differential_drive::Servomotors& params) {
	cli();
	for(int i=0;i<8;i++)
		servo[i].write(params.servoangle[i]);
	sei();
}

static void messagePWM(const differential_drive::PWM& pwm) {
	wdtime=millis();
	MotorA.Set_speed(pwm.PWM1);
	MotorB.Set_speed(pwm.PWM2);
	flags|=FLAG_PWMDIRECT;
}

static void updateOdometry(void) {
	static int i = 0;
	static float speeds_lin[N],speeds_rot[N];
	//put new rotational speed values
	speeds_lin[i] = 0.5*(r_r*MotorA._speed+r_l*MotorB._speed);
	speeds_rot[i] = 1.0/B*(r_r*MotorA._speed-r_l*MotorB._speed);
	if(i<N-1)  {i++;}
	else  {i=0;}

	// Speed averaging
	float V_lin=0.0, V_rot=0.0;
	for(int k=0;k<N;k++)  {
		V_lin+=speeds_lin[k];
		V_rot+=speeds_rot[k];
	}
	V_lin/=N;
	V_rot/=N;

	//calculate odometry
	theta+= V_rot*Te;
	x+= V_lin*Te*cos(theta);
	y+= V_lin*Te*sin(theta);

	/*static uint8_t cpt = 0;
	if(cpt<10)  {cpt++;}
	else  {
		cpt = 0;*/
		// Publish Odometry
		odomsg.x = x ;
		odomsg.y = y;
		odomsg.theta = theta ;
		pubOdom.publish(&odomsg);
	//}
}

ISR(EXTINT(ENC1_PHASE1)) {
	register const uint8_t pin=PIN(ENC1_PORT);
	if(pin&(uint8_t)_BV(ENC1_PHASE1)) {
		if(pin&(uint8_t)_BV(ENC1_PHASE2)) --encoder1;
		else ++encoder1;
	} else {
		if(pin&(uint8_t)_BV(ENC1_PHASE2)) ++encoder1;
		else --encoder1;
	}
}

ISR(EXTINT(ENC1_PHASE2)) {
	register const uint8_t pin=PIN(ENC1_PORT);
	if(pin&(uint8_t)_BV(ENC1_PHASE1)) {
		if(pin&(uint8_t)_BV(ENC1_PHASE2)) ++encoder1;
		else --encoder1;
	} else {
		if(pin&(uint8_t)_BV(ENC1_PHASE2)) --encoder1;
		else ++encoder1;
	}
}

ISR(EXTINT(ENC2_PHASE1)) {
	register const uint8_t pin=PIN(ENC2_PORT);
	if(pin&(uint8_t)_BV(ENC2_PHASE1)) {
		if(pin&(uint8_t)_BV(ENC2_PHASE2)) --encoder2;
		else ++encoder2;
	} else {
		if(pin&(uint8_t)_BV(ENC2_PHASE2)) ++encoder2;
		else --encoder2;
	}
}

ISR(EXTINT(ENC2_PHASE2)) {
	register const uint8_t pin=PIN(ENC2_PORT);
	if(pin&(uint8_t)_BV(ENC2_PHASE1)) {
		if(pin&(uint8_t)_BV(ENC2_PHASE2)) ++encoder2;
		else --encoder2;
	} else {
		if(pin&(uint8_t)_BV(ENC2_PHASE2)) --encoder2;
		else ++encoder2;
	}
}

ISR(ADC_vect) {
	uint8_t i=ADMUX&0x1f;
	if(ADCSRB&_BV(MUX5)) i+=8;
	adcvals[i]=ADC;
	if(i<15) {
		++i;
		ADMUX=_BV(REFS0)|(i&0x07);
		ADCSRB=(i>7)?_BV(MUX5):0;
		ADCSRA=_BV(ADEN)|_BV(ADSC)|_BV(ADIE)|3;
	} else {
		flags|=FLAG_ADCRESULT;
	}
}

ISR(TIMER4_CAPT_vect) {
	//read encoder value
	int16_t encoder1_loc = encoder1;
	int16_t encoder2_loc = encoder2;
	encmsg.delta_encoder1 = (int16_t)(encoder1_loc-(int16_t)encmsg.encoder1);
	encmsg.delta_encoder2 = (int16_t)(encoder2_loc-(int16_t)encmsg.encoder2);
	encmsg.encoder1 = encoder1_loc;
	encmsg.encoder2 = encoder2_loc;
	uint8_t f=flags;
	MotorA.calculateSpeed(Te,encmsg.delta_encoder1);
	MotorB.calculateSpeed(Te,encmsg.delta_encoder2);
	if(!(f&FLAG_PWMDIRECT)) {
		MotorA.Speed_regulation(W1_set,Te);
		MotorB.Speed_regulation(W2_set,Te);
	}
	encmsg.timestamp = PERIOD_TIMER_MS*1e-3;
	f|=FLAG_ENCUPDATE;
	flags=f;
}
