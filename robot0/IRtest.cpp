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
  
  // Distance of sensor 1 and 2.
  double FrontRightSensor,RearRightSensor;

  // Getting the info from the sensors.
  FrontRightSensor = msg.ch8; // front sensor
  RearRightSensor = msg.ch4; // back sensor

  //	msg.ch3;
  //	msg.ch4;

  // Conversion from the ADC to cm.
  FrontRightSensor = alpha_1/(FrontRightSensor+beta_1)-Sharp2Wheel_FrontRightD;
  RearRightSensor = alpha_2/(RearRightSensor+beta_2)-Sharp2Wheel_RearRightD;
  
  double Right_Center_WallD = 0;
  double Robot2Wall_Angle = 0;
  
  Robot2Wall_Angle = -atan( (FrontRightSensor-RearRightSensor)/(IR_rightD) ); // changed to d.
  Right_Center_WallD = (FrontRightSensor+RearRightSensor)*0.5*cos(Robot2Wall_Angle); // distance to the wall.

double turn=1;//active theta-control if near wall only
double forward=1;//active x-control if

// If it is not near a wall, remove turning
if (abs(Right_Center_WallD-Wheel2Wall_D)>10) {turn=0;}
// While it is turning, dont go foward.
if (Robot2Wall_Angle>0.2||-Robot2Wall_Angle>0.2) {forward=0;}

double w = 1*K_x*(Right_Center_WallD-Wheel2Wall_D)*forward - 1*k_theta*Robot2Wall_Angle*turn;

double v = 1*RobotSpeed;
 
double w1=-1*(-w+v);//right wheel
double w2=-1*(v+w); //left wheel

  // Debugg output
  std::cerr<<FrontRightSensor<<' '<<RearRightSensor<<' '<<w<<' '<<w1<<' '<<w2<<' '<<std::endl;

  // Publish into the speed topic.

  spd.W1 = w1;
  spd.W2 = w2;
  spd.header.stamp = ros::Time::now();
  cmd_pub.publish(spd);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    cmd_pub = nh.advertise<Speed>("/motion/Speed", 1);

    sharps_sub = nh.subscribe("/sensors/ADC/",1,followWall);
    
    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
