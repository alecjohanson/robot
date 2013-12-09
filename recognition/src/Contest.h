/*
 * Contest.h
 *
 *  Created on: Dec 9, 2013
 *      Author: robo
 */

#ifndef CONTEST_H_
#define CONTEST_H_

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include <string>

class Contest {
public:
	Contest();
	~Contest();
	void announceObject(const std::string &name,const sensor_msgs::Image::ConstPtr &img);
private:
	ros::NodeHandle nh;
	ros::Publisher evidence_pub;
	ros::Publisher chatter_pub;
};

#endif /* CONTEST_H_ */
