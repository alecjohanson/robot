#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Encoders.h>
#include <turtlesim/Velocity.h>
#include <string.h>

using namespace differential_drive;
ros::Subscriber	enc_sub;
ros::Subscriber	key_sub;
ros::Publisher	pwm_pub;

typedef struct {
	double dcgain_inv;
	double k_p;
	double k_i;
	double k_d;
} pidproperties_t;

#define FILTER_TAP_NUM 11

typedef struct {
	double set;
	double errint;
	double lasterrs[FILTER_TAP_NUM-1];
	double *currenterr;
	double lasterr;
} piddata_t;

#define TKRIT1 0.16
#define TKRIT2 0.16
static const pidproperties_t left_plant={1020.0/19.0, 0.5, 0.5*TKRIT1, 0.12*TKRIT1};
static const pidproperties_t right_plant={1020.0/19.0, 0.5, 0.5*TKRIT2, 0.12*TKRIT2};

static piddata_t left_data;
static piddata_t right_data;

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 100 Hz

* 0 Hz - 5 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 1.852591762227721 dB

* 20 Hz - 50 Hz
  gain = 0
  desired attenuation = -50 dB
  actual attenuation = -52.42335831617553 dB

*/
static double lpfilter(double newerr,piddata_t *piddata) {
	static const double filter_taps[FILTER_TAP_NUM] = {
	  0.00997906182309545,
	  0.03449137262437207,
	  0.07621036856553794,
	  0.12660206213679626,
	  0.16862853338729203,
	  0.18505666744689508,
	  0.16862853338729203,
	  0.12660206213679626,
	  0.07621036856553794,
	  0.03449137262437207,
	  0.00997906182309545
	};
	double result=newerr*filter_taps[0];
	double *p=piddata->currenterr;
	if(++p>=piddata->lasterrs+FILTER_TAP_NUM-1) p=piddata->lasterrs;
	for(int i=1;i<FILTER_TAP_NUM;++i) {
		if(++p>=piddata->lasterrs+FILTER_TAP_NUM-1) p=piddata->lasterrs;
		result=fma(filter_taps[i],*p,result);
	}
	*p=newerr;
	piddata->currenterr=p;
	return result;
}

//Callback function for the "/encoder" topic. Prints the encoder value and randomly changes the motor speed.
void receive_encoder(const Encoders::ConstPtr &msg)
{
  //static ros::Time t_start = ros::Time::now();
  /*int right = msg->delta_encoder2;
  int left = msg->delta_encoder1;
  int timestamp = msg->timestamp;
  printf("%d:got encoder L:%d , false R:%d\n",timestamp,left,right);*/
  PWM pwm;
  // pwm.PWM1	= int(255*(0.5f-float(rand()%1000)/1000.0f)*2.0f);//random motor speed[-255,255]
  // pwm.PWM2	= int(255*(0.5f-float(rand()%1000)/1000.0f)*2.0f);//random motor speed[-255,255]

  double left_e=lpfilter(left_data.set-msg->delta_encoder1,&left_data);
  double right_e=lpfilter(right_data.set-msg->delta_encoder2,&right_data);
  left_data.errint+=left_e;
  right_data.errint+=right_e;
  pwm.PWM1=lrint(left_plant.dcgain_inv*(
		  left_e*left_plant.k_p +
		  left_data.errint*left_plant.k_i +
		  (left_e-left_data.lasterr)*left_plant.k_d
		  ));
  pwm.PWM2=lrint(right_plant.dcgain_inv*(
		  right_e*right_plant.k_p +
		  right_data.errint*right_plant.k_i +
		  (right_e-right_data.lasterr)*right_plant.k_d
		  ));
  left_data.lasterr=left_e;
  right_data.lasterr=right_e;
  printf("L: e=%6.3f p=%4i    R: e=%6.3f p=%4i\n",
		  left_e,pwm.PWM1,
		  right_e,pwm.PWM2
		  );

  //pwm.PWM1 = int(255*sin(2*M_PI*1*(ros::Time::now()-t_start).toSec()));
  //pwm.PWM2 = int(255*sin(2*M_PI*1*(ros::Time::now()-t_start).toSec()));

  pwm.header.stamp = ros::Time::now();
  pwm_pub.publish(pwm);
}

void receive_keys(const turtlesim::Velocity::ConstPtr& v) {
	left_data.set =  v->linear - v->angular;
	right_data.set = v->linear + v->angular;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "MotorPID");//Creates a node named "FakeMotorsTest"

	memset(&left_data,0,sizeof(left_data));
	left_data.currenterr=left_data.lasterrs;
	left_data.set=2.7;
	memset(&right_data,0,sizeof(right_data));
	right_data.currenterr=right_data.lasterrs;
	right_data.set=2.7;

	ros::NodeHandle n;
	enc_sub = n.subscribe("motors/encoders", 1000, receive_encoder);//when "/encoder" topic is received, call recive_encoder function
	key_sub=n.subscribe("turtle1/command_velocity",1000, receive_keys );
	pwm_pub = n.advertise<PWM>("motors/pwm", 100000);//used to publish a topic that changes the motor speed

	ros::Rate loop_rate(100);
	//The loop randomly changes the interval with which the "/encoder" topic is published.
	
//	struct timeval start, end;
	while(ros::ok()){

		loop_rate.sleep();
		ros::spinOnce();

	}
	return 0;
}

