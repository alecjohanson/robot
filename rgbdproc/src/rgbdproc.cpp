/*
 * rgbdproc.cpp
 *
 *  Created on: Nov 2, 2013
 *      Author: robo
 */

#include <iostream>
#include <ros/ros.h>
#include "RGBDSource.h"
#include <OpenNI.h>
#include <getopt.h>
#include "DepthToPointcloudConverter.h"

#define RES_X 320
#define RES_Y 240
#define FPS 30

openni::Recorder rec;

static void exitHandler(void);
int have_gui;

static const struct option long_options[] = {
		{"record",   required_argument,         0, 'r'},
		{"playback", required_argument,         0, 'p'},
		{"gui",            no_argument, &have_gui,   1},
		{0, 0, 0, 0}
};

int main(int argc, char **argv) {
	have_gui=0;

	const char *f_record=0, *f_playback=0;
    int option_index = 0;
    while(1) {
    	int c = getopt_long (argc, argv, "abc:d:f:",
    			long_options, &option_index);
    	if(c == -1) break;
    	switch (c) {
    	case 0:
//    		if (long_options[option_index].flag != 0) break;
    		break;
    	case 'r':
    		f_record=optarg;
    		break;
    	case 'p':
    		f_playback=optarg;
    		break;
    	case '?':
    		break;
    	default:
    		abort();
    		break;
    	}
    }
    if(f_record) f_playback=0;
    if(!f_playback) f_playback=openni::ANY_DEVICE;
    ros::init(argc,argv,"rgbdproc");
	RGBDSource rgbd(f_record);
	rgbd.setVideoMode(openni::SENSOR_DEPTH,RES_X,RES_Y,FPS,
			openni::PIXEL_FORMAT_DEPTH_100_UM);
	rgbd.setVideoMode(openni::SENSOR_COLOR,RES_X,RES_Y,FPS,
			openni::PIXEL_FORMAT_RGB888);

	atexit(exitHandler);
	openni::VideoStream& color=rgbd.getVideoStream(openni::SENSOR_COLOR);
	openni::VideoStream& depth=rgbd.getVideoStream(openni::SENSOR_DEPTH);
	if(f_record) {
		rec.create(f_record);
		rec.attach(color,true);
		rec.attach(depth,false);
		rec.start();
	}
	DepthToPointcloudConverter pconv;
	pconv.readStreamInfo(depth);
	rgbd.startStreams();
	while(ros::ok()){
		openni::VideoFrameRef cframe, dframe;
		depth.readFrame(&dframe);
		color.readFrame(&cframe);
		pconv.onNewFrame(dframe);
		while((int64_t)dframe.getTimestamp()-(int64_t)cframe.getTimestamp()>500000L/FPS) {
			cframe.release();
			color.readFrame(&cframe);
		}
		while((int64_t)dframe.getTimestamp()-(int64_t)cframe.getTimestamp()<-500000L/FPS) {
			dframe.release();
			depth.readFrame(&dframe);
		}
		std::cerr << (int64_t)dframe.getTimestamp()-(int64_t)cframe.getTimestamp() << std::endl;

		cframe.release();
		dframe.release();
		ros::spinOnce();
	}
	return 0;
}

static void exitHandler(void) {
	rec.stop();
	rec.destroy();
}
