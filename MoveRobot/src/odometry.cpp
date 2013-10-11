/*
 * odometry.cpp
 *
 *  Created on: Sep 22, 2013
 *      Author: robo
 */

/*
 * controller.cpp
 *
 *  Created on: Sep 18, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Encoders.h>
#include <differential_drive/Odometry.h>
#include "odometry.h"
#include <visualization_msgs/Marker.h>

using namespace differential_drive;


void receive_encoder(const Encoders::ConstPtr &msg)
{
	static double x,y;
	static double theta;

	int delta_right = msg->delta_encoder2;
	int delta_left = msg->delta_encoder1;

	x += cos(theta)*r/2*(delta_right+delta_left)/ticks_rev*2*M_PI;
	y += sin(theta)*r/2*(delta_right+delta_left)/ticks_rev*2*M_PI;
	theta += -r/2/l*(delta_right-delta_left)/ticks_rev*2*M_PI;

	printf("x = %f, y = %f, theta = %f\n",x,y,theta/M_PI*180);

	while((theta > M_PI) | (theta <= -M_PI))
	{
		if(theta > 0)
		{
			theta -= 2*M_PI;
		}
		else
		{
			theta += 2*M_PI;
		}
	}

	Odometry odometry;
	odometry.x = x;
	odometry.y = y;
	odometry.theta = theta;
	odom_pub.publish(odometry);



}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry");
	ros::NodeHandle nh;
	odom_pub = nh.advertise<Odometry>("/odometry", 100);
	enc_sub = nh.subscribe("/motors/encoders/",1000,receive_encoder);


	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}


