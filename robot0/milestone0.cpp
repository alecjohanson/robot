#include <ros/ros.h>
#include <differential_drive/PWM.h>
#include <differential_drive/Odometry.h>
#include "HandController.h"
#include <cmath>

using namespace differential_drive;

void followHand(const Odometry::ConstPtr &msg){
//this method derives the control of the motors

    double x,y;

    double r = 0;    //distance to the hand
    double alpha = 0;//angle to the vertical plane.

    PWM pwm;//the motors like in fakemotors

    x = msg->y;
    z = msg->x;

    r = pow(x*x + z*z,0.5);
    alpha = atan(x/z);


    if ( (r < r0) && ((alpha < a0)||(alpha > -a0)) ){
	r=r0;
	alpha=0;
    }


    pwm.PWM1 = kr*(r-r0) - ka*alpha;
    pwm.PWM2 = kr*(r-r0) + ka*alpha;
    pwm.header.stamp = ros::Time::now();
    cmd_pub.publish(pwm);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    cmd_pub = nh.advertise<PWM>("/motion/Speed", 100);
    odom_sub = nh.subscribe("/odometry",1,followHand);//Christians node

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
