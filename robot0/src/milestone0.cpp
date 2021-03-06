#include <ros/ros.h>
#include <differential_drive/Speed.h>
#include <geometry_msgs/Point.h>
#include "milestone0.h"
#include <cmath>
#include <iostream>

using namespace differential_drive;

static ros::Publisher cmd_pub;
static ros::Subscriber odom_sub;

void followHand(const geometry_msgs::Point &msg){
//this method derives the control of the motors

    double x,z;

    double r = 0;    //distance to the hand
    double alpha = 0;//angle to the vertical plane.

    Speed spd;//the motors like in fakemotors

    x = msg.y;
    z = msg.x;

    r = hypot(x,z);
    alpha = atan(x/z);


    /*if ( (r < r0) && ((alpha < a0)||(alpha > -a0)) ){
	r=r0;
	alpha=0;
    }*/

    std::cerr<<x<<' '<<z<<' '<<r<<' '<<alpha<<' '<<std::endl;
    if(isnan(alpha)) alpha=0.0;
    if(isnan(x) || isnan(z)) {
        spd.W1 = 0;
        spd.W2 = 0;
    } else {
        spd.W1 = kr*(r-r0) - ka*alpha;
        spd.W2 = kr*(r-r0) + ka*alpha;
    }
    spd.header.stamp = ros::Time::now();
    cmd_pub.publish(spd);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    cmd_pub = nh.advertise<Speed>("/motion/Speed", 1);
    odom_sub = nh.subscribe("milestone0/object",1,followHand);//Christians node

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
