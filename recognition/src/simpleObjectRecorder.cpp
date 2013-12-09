/*
 * simpleObjectRecorder.cpp
 *
 *  Created on: Dec 9, 2013
 *      Author: robo
 */

#include <ros/ros.h>
#include "Contest.h"
#include <string>

Contest *pContest;

static void imgCallback(const sensor_msgs::Image::ConstPtr& msg) {
	pContest->announceObject(std::string(""),msg);
}

int main(int argc, char **argv) {
	ros::init(argc,argv,"objrecord");
	ros::NodeHandle nh;
	Contest c;
	pContest=&c; // Sorry. I know this hurts.
	ros::Subscriber img_sub=nh.subscribe("objdetect/images",1,imgCallback);
	ros::spin();

	return 0;
}


