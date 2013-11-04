#include <ros/ros.h>
#include <differential_drive/AnalogC.h>
#include <robot0/Distance.h>
#include "SensorPublisher.h"
#include <iostream>

using namespace differential_drive;

static ros::Publisher sharps_pub;
static ros::Subscriber arduino_sub;

void getDistance(const differential_drive::AnalogC &msg){
   
  robot0::Distance dst;

  double RearRightSensor,FrontRightSensor;
  double RearLeftSensor, FrontLeftSensor;
  double FrontSensor, RearSensor;

  // Getting the info from the Right sensors.
  FrontRightSensor = msg.ch8; 
  RearRightSensor = msg.ch4; 
  
  // Getting the info from the Left sensors.
  RearLeftSensor = msg.ch5;
  FrontLeftSensor = msg.ch7;

  // Getting the info from the Front and Rear.
  RearSensor = msg.ch3;
  FrontSensor = msg.ch2;

  // Conversion from the ADC to cm.
  FrontRightSensor = alpha_1/(FrontRightSensor+beta_1)-Sharp2Wheel_FrontRightD;
  RearRightSensor = alpha_2/(RearRightSensor+beta_2)-Sharp2Wheel_RearRightD;

  FrontLeftSensor = alpha_3/(FrontLeftSensor+beta_3)-Sharp2Wheel_FrontLeftD;
  RearLeftSensor = alpha_4/(RearLeftSensor+beta_4)-Sharp2Wheel_RearLeftD;
  
  RearSensor = alpha_5 / (RearSensor+beta_5) - Sharp2Edge_RearD;
  FrontSensor = alpha_6 / (FrontSensor+beta_6) - Sharp2Edge_FrontD;


  // Debugg output
  // std::cerr<<'Front Right '<<FrontRightSensor<<std::endl;
  // std::cerr<<'Rear Right  '<<RearRightSensor<<std::endl;
  // std::cerr<<'Front Left  '<<FrontLeftSensor<<std::endl;
  // std::cerr<<'Rear Left   '<<RearLeftSensor<<std::endl;
  // std::cerr<<'Front       '<<FrontSensor<<std::endl;
  // std::cerr<<'Rear        '<<RearSensor<<std::endl;
  
  // Message http://wiki.ros.org/rosbuild/msg
  
  // Publish into the Sharp Distance topic.
  dst.front_r = FrontRightSensor;
  dst.rear_r = RearRightSensor;
  dst.front_l = FrontLeftSensor;
  dst.rear_l = RearLeftSensor;
  dst.rear = RearSensor;
  dst.front = FrontSensor;

  dst.header.stamp = ros::Time::now();
  sharps_pub.publish(dst);
}

// Main
int main(int argc, char** argv)
{
  ros::init(argc, argv, "SensorPublisher");
  ros::NodeHandle nh;
  sharps_pub = nh.advertise<robot0::Distance>("/sharps/Distance", 1);
  arduino_sub = nh.subscribe("/sensors/ADC/",1,getDistance);
  ros::Rate loop_rate(100);
  while(ros::ok())
    {
      loop_rate.sleep();
      ros::spinOnce();
    }
  return 0;
}
