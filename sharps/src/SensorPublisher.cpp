#include <ros/ros.h>
#include <differential_drive/AnalogC.h>
#include <sharps/Distance.h>
#include "SensorPublisher.h"
#include <iostream>

using namespace differential_drive;

static ros::Publisher sharps_pub;
static ros::Subscriber arduino_sub;



double ADCtoCM(double params[4], double SensorData){
	// Receives 4 parameters
	// param[0] = alpha constant
	// param[1] = beta constant
	// param[2] = Distance from sharp to the wheel.
	// param[3] = max distance showed by the sharp.
	// Example of an equation would be: 

	//FrontRightSensor = alpha_1/(FrontRightSensor+beta_1)-Sharp2Wheel_FrontRightD;	
	double SensorCM = params[0]/ ( SensorData - params[1] ) - params[2];
	if (SensorCM < 0)
		SensorCM = 0;
	else if (SensorCM > params[3])
		SensorCM = params[3];
	return SensorCM;
}

void getDistance(const differential_drive::AnalogC &msg){
   
  sharps::Distance dst;

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
  double parameters[] = {alpha_1, beta_1, Sharp2Wheel_FrontRightD, MaxDist};

  //FrontRightSensor = alpha_1/(FrontRightSensor+beta_1)-Sharp2Wheel_FrontRightD;
  FrontRightSensor = ADCtoCM(parameters, FrontRightSensor);

  //RearRightSensor = alpha_2/(RearRightSensor+beta_2)-Sharp2Wheel_RearRightD;
  parameters[2] = Sharp2Wheel_RearRightD;
  RearRightSensor = ADCtoCM(parameters, RearRightSensor);

  //FrontLeftSensor = alpha_3/(FrontLeftSensor+beta_3)-Sharp2Wheel_FrontLeftD;
  parameters[2] = Sharp2Wheel_FrontLeftD;
  FrontLeftSensor = ADCtoCM(parameters, FrontLeftSensor);

  //RearLeftSensor = alpha_4/(RearLeftSensor+beta_4)-Sharp2Wheel_RearLeftD;
  parameters[2] = Sharp2Wheel_RearLeftD;
  RearLeftSensor = ADCtoCM(parameters, RearLeftSensor);

  //RearSensor = alpha_5 / (RearSensor+beta_5) - Sharp2Edge_RearD;
  parameters[2] = Sharp2Edge_RearD;
  RearSensor = ADCtoCM(parameters, RearSensor);

  //FrontSensor = alpha_6 / (FrontSensor+beta_6) - Sharp2Edge_FrontD;
  parameters[2] = Sharp2Edge_FrontD;
  FrontSensor = ADCtoCM(parameters, FrontSensor);

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

  sharps_pub = nh.advertise<sharps::Distance>("/sharps/Distance", 1);
  arduino_sub = nh.subscribe("/sensors/ADC/",1,getDistance);
  ros::Rate loop_rate(100);
  while(ros::ok())
    {
      loop_rate.sleep();
      ros::spinOnce();
    }
  return 0;
}
