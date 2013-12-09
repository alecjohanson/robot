/*
 * Contest.cpp
 *
 *  Created on: Dec 9, 2013
 *      Author: robo
 */

#include "Contest.h"
#include "contest_msgs/evidence.h"
#include "std_msgs/String.h"

Contest::Contest() :
	nh(),
	evidence_pub(nh.advertise<contest_msgs::evidence>("/contest_evidence",2)),
	chatter_pub(nh.advertise<std_msgs::String>("robot/talk", 1000))
{}

Contest::~Contest() {
}

void Contest::announceObject(const std::string& name,const sensor_msgs::Image::ConstPtr &img) {
	static time_t lastObj;
	time_t now;
	time(&now);
	if(difftime(now,lastObj)>0.5) {
		std_msgs::String str;
		if(name.length()>0)
			str.data="I see a "+name;
		else
			str.data="I see something";
		chatter_pub.publish(str);
	}
	contest_msgs::evidence ev;
	ev.stamp=ros::Time::now();
	ev.group_number=0;
	ev.object_id=name;
	ev.image_evidence=*img.get();
	evidence_pub.publish(ev);
	time(&lastObj);
}
