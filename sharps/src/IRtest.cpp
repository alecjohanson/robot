#include <ros/ros.h>
#include <sharps/Distance.h>
#include <differential_drive/Speed.h>
#include "IRtest.h"
#include <cmath>
#include <iostream>
#include <stdio.h>

using namespace differential_drive;

static ros::Publisher cmd_pub;
static ros::Subscriber sharps_sub;

void followWall(const sharps::Distance &msg){
   
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

  // If it is not near a wall, remove turning
  if (abs(Right_Center_WallD-Wheel2Wall_D)>10)
    {turn=0;}
  // While it is turning, dont go foward.
  else if (Robot2Wall_Angle>MaxFowardAngle || -Robot2Wall_Angle>MaxFowardAngle)
    {forward=0;turn=1;}

  // Control
  double w = K_forward*(Right_Center_WallD-Wheel2Wall_D)*forward - k_theta*Robot2Wall_Angle*turn;

  double v = forward*RobotSpeed;
   
  double w1= -1*(-w+v);//right wheel
  double w2= -1*(v+w); //left wheel



  if (true) { // change to false if you dont want to run it
  // another kind of control (testing...)
  // only tested on table, don't know how to connect through wifi
  // has some kind of derivate control
  // Keeps the forward speed of the robot constant (Robot_Velocity)
  // Also sets a maximum angular velocity which now depends on Which max speed we want to
  // take a curve with and the curvature radius (now the distance we keep from the wall)
  // larger curvature radius -> more angular velocity to keep same forward velocity, bla bla.
  // using SI-units
  // parameters probably fucked up
  // /alfred 23/10
    double Robot_Wheel_Radius = 0.06; // guess
    // speed 2 dm/s or something
    double Robot_Velocity = 0.2/Robot_Wheel_Radius; // [m/s / m = radians/s]
    double Robot_IR14_Distance = IR_rightD*0.01; // [m] identifying IR's in which quadrant they are in
    double Robot_Base_Length = 0.21; // (educated?) guess
    double Robot_Curve_Velocity = Robot_Velocity; // guess
    double IR_1 = FrontRightSensor*0.01; 
    double IR_4 = RearRightSensor*0.01;  // Taking same sensor measurements as before
    double Wall_Right_Distance = 0.10; // Distance to keep to wall
    double Kt = 3; // Angle
    double Kx = 8;   // distance
    double Kd = 0.1; // derivate(its the angular velocity) sensitivity

    double Robot_Angular_Velocity_Max = Robot_Curve_Velocity/(Wall_Right_Distance+Robot_Base_Length/2);

    double Robot_Angle2Wall = atan((IR_1-IR_4)/Robot_IR14_Distance);
    double Robot_Right_Distance2Wall = (IR_1+IR_4)/2*cos(Robot_Angle2Wall);
    double Robot_Wheel_Left_Velocity = (Robot_Velocity + Kt*Robot_Angle2Wall + Kx* \
            (Robot_Right_Distance2Wall-Wall_Right_Distance) + 2*Kd*Robot_Velocity/ \
            Robot_Base_Length) * 1/(1+2*Kd/Robot_Base_Length);

    double Robot_Wheel_Right_Velocity = 2*Robot_Velocity - Robot_Wheel_Left_Velocity;
    double Robot_Angular_Velocity = (Robot_Wheel_Right_Velocity-Robot_Wheel_Left_Velocity)/Robot_Base_Length;

    // Check if angular velocity is within bounds
    if (Robot_Angular_Velocity > Robot_Angular_Velocity_Max) {
      Robot_Angular_Velocity = Robot_Angular_Velocity_Max;
      Robot_Wheel_Left_Velocity = (2*Robot_Velocity-Robot_Angular_Velocity*Robot_Base_Length)/2;
      Robot_Wheel_Right_Velocity = 2*Robot_Velocity - Robot_Wheel_Left_Velocity;
    }

    // check if IR-distance is close enough for algorithm to be useful
    //if (!(Robot_Right_Distance2Wall > 0.2)) {
      w2 = -Robot_Wheel_Left_Velocity;
      w1 = -Robot_Wheel_Right_Velocity;
    //} else {
      //w2 = 0;
      //w1 = 0;
    //}

    // Debug
    std::cerr<<"XXX - IR1: " << IR_1<<" IR4: "<<IR_4<< \
        "Right: "<<-w1<<" Left: "<<-w2 <<std::endl;
    std::cerr<<"X_error: " << Robot_Right_Distance2Wall-Wall_Right_Distance <<\
      " theta_error: "<<Robot_Angle2Wall <<std::endl;

  }


  // Debugg output
  //std::cerr<<FrontRightSensor<<' '<<RearRightSensor<<' '<<w<<' '<<w1<<' '<<w2<<' '<<std::endl;

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

    sharps_sub = nh.subscribe("/sharps/Distance/",1,followWall);
    
    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
