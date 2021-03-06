#include <ros/ros.h>
#include <robot0/Distance.h>
#include <differential_drive/Speed.h>
#include "IRtest.h"
#include <cmath>
#include <iostream>
#include <stdio.h>

using namespace differential_drive;

static ros::Publisher cmd_pub;
static ros::Subscriber sharps_sub;


void followWall(const robot0::Distance &msg){
   
  //this method derives the control of the motors
  Speed spd;//the motors like in fakemotors
  
  // Distance of sensor 1 and 2.
  double FrontRightSensor,RearRightSensor;

  // Getting the info from the sensors.
  FrontRightSensor = msg.front_r; // front sensor
  RearRightSensor = msg.rear_r; // back sensor

  //	msg.ch3;
  //	msg.ch4;

  double Right_Center_WallD = 0;
  double Robot2Wall_Angle = 0;
  
  Robot2Wall_Angle = -atan( (FrontRightSensor-RearRightSensor)/(IR_rightD) ); // changed to d.
  Right_Center_WallD = (FrontRightSensor+RearRightSensor)*0.5*cos(Robot2Wall_Angle); // distance to the wall.

double turn=1;//active theta-control if near wall only
double forward=1;//active x-control if

// While it is turning, dont go foward.
if (Robot2Wall_Angle>MaxFowardAngle || -Robot2Wall_Angle>MaxFowardAngle)
	{forward=0.2;turn=1;}
// If it is not near a wall, remove turning
else if (abs(Right_Center_WallD-Wheel2Wall_D)>5)
	{forward=1;turn=0;}

// Control
double w = K_forward*(Right_Center_WallD-Wheel2Wall_D)*forward - k_theta*Robot2Wall_Angle*turn;

double v = forward*RobotSpeed;
//speeds
double w1 = -1*(-w+v);//right wheel
double w2 = -1*(v+w); //left wheel



if (false) {
}


  // Debugg output
//std::cerr<<FrontRightSensor<<' '<<RearRightSensor<<' '<<w<<"RIGHT: "<<w1<<" LEFT: "<<w2<<' '<<std::endl;

  // Publish into the speed topic.

  spd.W1 = w1;
  spd.W2 = w2;
  spd.header.stamp = ros::Time::now();
  cmd_pub.publish(spd);
	std::cerr << "Speed R: "<< spd.W1 <<", L:"<<spd.W2 << std::endl; 
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    cmd_pub = nh.advertise<Speed>("/motion/Speed", 1);

    sharps_sub = nh.subscribe("/sharps/Distance/",1,followWall);
    
    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
