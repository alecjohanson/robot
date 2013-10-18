#include <ros/ros.h>
#include <differential_drive/AnalogC.h>
#include <differential_drive/Speed.h>
#include "IRtest.h"
#include <cmath>
#include <iostream>

using namespace differential_drive;

static ros::Publisher cmd_pub;
static ros::Subscriber sharps_sub;

void followWall(const differential_drive::AnalogC &msg){
   
  //this method derives the control of the motors
  Speed spd;//the motors like in fakemotors

  double d1,d2;

  d1 = msg.ch8; // front sensor
  d2 = msg.ch4; // back sensor
  //	msg.ch3;
  //	msg.ch4;
  d1 = alpha_1/(d1+beta_1)-d1_0;
  d2 = alpha_2/(d2+beta_2)-d2_0;
  
  double x = 0;
  double theta = 0;
  theta = atan( (d1-d2)/(2*d) ); 
  x = (d1+d2)*0.5*cos(theta);
  double pi = atan(1)*4; // define pi
  double w = K_x*(x-x_L)*(pi/2 - theta) + k_theta*theta*(exp(-abs(x-x_L) ) );
  
  double v = V_0; 
 
  // Debugg output
  std::cerr<<d1<<' '<<d2<<' '<<std::endl;

  // Publish into the speed topic.
  spd.W1 = -1*(w+v);
  spd.W2 = -1*(v-w);
  spd.header.stamp = ros::Time::now();
  cmd_pub.publish(spd);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    cmd_pub = nh.advertise<Speed>("/motion/Speed", 1);
    //odom_sub = nh.subscribe("milestone0/object",1,followHand);//Christians node
    sharps_sub = nh.subscribe("/sensors/ADC/",1,followWall);
    
    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
