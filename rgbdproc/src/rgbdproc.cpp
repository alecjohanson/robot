/*
 * rgbdproc.cpp
 *
 *  Created on: Nov 2, 2013
 *      Author: robo
 */

#include <iostream>
#include <ros/ros.h>
#include "RGBDSource.h"

#define RES_X 640
#define RES_Y 480

int main(int argc, char **argv) {
	RGBDSource &rgbd=RGBDSource::getInstance();
	rgbd.setVideoMode(openni::SENSOR_DEPTH,RES_X,RES_Y,30,
			openni::PIXEL_FORMAT_DEPTH_100_UM);
	rgbd.setVideoMode(openni::SENSOR_COLOR,RES_X,RES_Y,30,
			openni::PIXEL_FORMAT_RGB888);
	rgbd.g
	openni::VideoStream& color=rgbd.getVideoStream(openni::SENSOR_COLOR);
	openni::VideoStream& depth=rgbd.getVideoStream(openni::SENSOR_DEPTH);
	while(ros::ok()){
		openni::VideoFrameRef cframe, dframe;
		color.readFrame(&cframe);
		depth.readFrame(&dframe);
		std::cerr << dframe.getFrameIndex()-cframe.getFrameIndex() << " / "
				  << dframe.getTimestamp() -cframe.getTimestamp() << std::endl;
		ros::spinOnce();
	}
	return 0;
}
